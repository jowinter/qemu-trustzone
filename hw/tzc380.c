/*
 * ARM TrustZone Address Space Controller TZC-380
 * (As described in ARM DDI 0431C)
 *
 * Copyright (c) 2011 IAIK, Graz University of Technology
 * Written by Johannes Winter
 *
 * This code is licenced under the GPL.
 */
#include "arm_trustzone.h"
#include "sysbus.h"

#define DEBUG_TZC380

#ifdef DEBUG_TZC380
#define tzc380_dbg(...) do {                     \
    fprintf(stderr, "tzc380: " __VA_ARGS__);     \
    fputc('\n', stderr);                         \
  } while(0)
#else
#define tzc380_dbg(...) do {                    \
  } while(0)
#endif

typedef struct {
  uint32_t low;
  uint32_t high;
  uint32_t attrs;
} tzc380_region;

static const uint8_t PERIPHID[5] = { 0x80, 0xB3, 0x1B, 0x00, 0x04 };
static const uint8_t PCELLID[4]  = { 0x0D, 0xF0, 0x05, 0xB1 };

#define TZC380_MAX_REGIONS 16

typedef struct {
  SysBusDevice busdev;  
  uint32_t configuration;
  uint32_t action;
  uint32_t lockdown_range;
  uint32_t lockdown_select;
  uint32_t int_status;
  uint32_t fail_addr_low;
  uint32_t fail_addr_high;
  uint32_t fail_control;
  uint32_t fail_id;
  uint32_t speculation_control;
  uint32_t security_inversion_en;
  tzc380_region region[TZC380_MAX_REGIONS];
} tzc380_state;

/*
 * Gets the total number of regions supported by a TZC380 
 * instance.
 *
 * @return: The total number of supported regions (always
 *  greater or equal than one)
 */
static inline unsigned tzc380_num_regions(tzc380_state *s)
{
  return (s->configuration & 0x0F) + 1;
}

/*
 * Gets the number of the lowest locked region of a TZC380
 * instance.
 *
 * @return: The first locked region of this TZC380 instance or
 *  a value larger than the highest valid region index if no
 *  regions are locked.
 */
static inline unsigned tzc380_first_locked_region(tzc380_state *s)
{
  unsigned region = tzc380_num_regions(s); /* No regions locked */

  if (s->lockdown_range & 0x80000000) {
    unsigned locked = (s->lockdown_range & 0x0000000F) + 1;
    
    if (locked < region) {
      /* regions above no_of_regions-1 are locked */
      region -= locked;
    } else {
      /* all regions are locked */
      region = 0;
    }
  }

  return region;
}

static uint32_t tzc380_read(void *opaque, target_phys_addr_t offset)
{
  tzc380_state *s = opaque;

  if (offset >= 0x100 && offset <= 0x1FC) {
    unsigned region = (offset & 0x0F0) >> 4;
    if (region >= tzc380_num_regions(s)) {
      hw_error("tzc380_read: Invalid region %u\n", region);
      return 0xFFFFFFFF;
    }

    switch (offset & 0x00F) {
    case 0x0: /* Region Setup Low <n> Register */
      return s->region[region].low;

    case 0x4: /* Region Setup High <n> Register */
      return s->region[region].high;

    case 0x8: /* Region Attributes <n> Register */
      return s->region[region].attrs;
      
    default:
      hw_error("tzc380_read: Undefined register 0x%x of region %d\n",
               (unsigned) (offset & 0x00F), region);
      return 0xFFFFFFFF;
    }
  } else {
    switch (offset) {
    case 0x000: /* Configuration */
      return s->configuration;
      
    case 0x004: /* Action register */
      return s->action;
      
    case 0x008: /* Lockdown range register */
      return s->lockdown_range;
      
    case 0x00C: /* Lockdown select register */
      return s->lockdown_select;
      
    case 0x010: /* Interrupt status register */
      return s->int_status;
      
    case 0x020: /* Fail address low register */
      return s->fail_addr_low;
      
    case 0x024: /* Fail address high register */
      return s->fail_addr_high;
      
    case 0x028: /* Fail control register */
      return s->fail_control;
      
    case 0x02C: /* Fail ID register */
      return s->fail_id;
      
    case 0x030: /* Speculation control register */
      return s->speculation_control;
      
    case 0x034: /* Security inversion enable register */
      return s->security_inversion_en;

    case 0xFD0: /* Peripheral Identification Register 4 */
      return PERIPHID[4];

    case 0xFE0: /* Peripheral Identification Register 0 */
    case 0xFE4: /* Peripheral Identification Register 1 */
    case 0xFE8: /* Peripheral Identification Register 2 */
    case 0xFEC: /* Peripheral Identification Register 3 */
      return PERIPHID[(offset - 0xFE0) >> 2];
      
    case 0xFF0: /* Component Identification Register 0 */
    case 0xFF4: /* Component Identification Register 1 */
    case 0xFF8: /* Component Identification Register 2 */
    case 0xFFC: /* Component Identification Register 3 */
      return PCELLID[(offset - 0xFF0) >> 2];

    default:
      hw_error("tzc380_read: Bad offset %x\n", (unsigned) offset);
      break;
    }
  }
}

static void tzc380_write(void *opaque, target_phys_addr_t offset, uint32_t val)
{
  tzc380_state *s = opaque;

  if (offset >= 0x100 && offset <= 0x1FC) {
    unsigned region = (offset & 0x0F0) >> 4;
    if (region >= tzc380_num_regions(s)) {
      hw_error("tzc380_write: Invalid region %u\n", region);
      return;
    } else if (region >= tzc380_first_locked_region(s)) {
      /* todo: raise a security violation instead of hw_error */
      hw_error("tzc380_write: Region %u is locked\n", region);
      return;
    }

    switch (offset & 0x00F) {
    case 0x0: /* Region Setup Low <n> Register */
      s->region[region].low = val & 0xFFFF0000;
      break;

    case 0x4: /* Region Setup High <n> Register */
      s->region[region].high = val;
      break;

    case 0x8: /* Region Attributes <n> Register */
      if (region == 0) {        
        val &= 0xF0000000; /* Allow control of sp0 */
        val |= 0x0000003F; /* 4G region, enabled   */
      } else {
        val &= 0xF000FF7F; /* Mask undefined bits */
      }

      s->region[region].attrs = val;
      break;
      
    default:
      hw_error("tzc380_write: Undefined register 0x%x of region %d\n",
               (unsigned) (offset & 0x00F), region);
      break;
    }

    /* todo: impose now permissions (and flush TLBs and TBs) */

  } else {
    switch (offset) {
    case 0x004: /* Action register */
      s->action = val & 0x00000003;
      break;

    case 0x008: /* Lockdown range register */
      if (s->lockdown_select & 0x00000001) {
        /* todo: raise a security violation instead of hw_error */
        hw_error("tzc380_write: Lockdown range register is locked");
        break;
      }

      s->lockdown_range = val & 0x8000000F;
      break;

    case 0x00C: /* Lockdown select register */
      s->lockdown_select = val & 0x00000007;
      break;

    case 0x014: /* Interrupt clear register */
      /* todo: deassert interrupt */
      s->int_status = 0;
      break;

    case 0x030: /* Speculation control register */
      if (s->lockdown_select & 0x00000004) {
        /* todo: raise a security violation instead of hw_error */
        hw_error("tzc380_write: Speculation control register is locked");
        break;
      }

      s->speculation_control = val & 0x00000003;
      break;

    case 0x034: /* Security inversion enable register */
      if (s->lockdown_select & 0x00000002) {
        /* todo: raise a security violation instead of hw_error */
        hw_error("tzc380_write: Security inversion enable register is locked");
        break;
      }

      s->security_inversion_en = val & 0x00000001;
      break;
      
    default:
      hw_error("tzc380_write: Bad offset %x\n", (unsigned) offset);
      break;
    }
  }
}

static void tzc380_reset(void *opaque)
{
}

static CPUReadMemoryFunc *const tzc380_readfn[] = {
  tzc380_read,
  tzc380_read,
  tzc380_read
};

static CPUWriteMemoryFunc *const tzc380_writefn[] = {
  tzc380_write,
  tzc380_write,
  tzc380_write
};

static int tzc380_init(SysBusDevice *dev)
{
  tzc380_state *s = FROM_SYSBUS(tzc380_state, dev);
  int iomemtype;

  iomemtype = cpu_register_io_memory(tzc380_readfn, tzc380_writefn, s,
                                     DEVICE_NATIVE_ENDIAN);
  sysbus_init_mmio(dev, 0x1000, iomemtype);
  
  /* Configuration: 32-bit AXI bus, 16 regions */
  s->configuration = 0x00001F0F;

  qemu_register_reset(tzc380_reset, s);
  tzc380_reset(s);

  /* todo: vm state ... */
  return 0;
}

static void tzc380_register_devices(void)
{
  sysbus_register_dev("tzc380", sizeof(tzc380_state), tzc380_init);
}

device_init(tzc380_register_devices);
