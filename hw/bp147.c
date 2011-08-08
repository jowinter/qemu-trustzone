/*
 * ARM TrustZone Protection Controller BP147
 *
 * Copyright (c) 2011 IAIK, Graz University of Technology
 * Written by Johannes Winter
 *
 * This code is licenced under the GPL.
 */
#include "arm_trustzone.h"
#include "sysbus.h"

#define DEBUG_BP147

#ifdef DEBUG_BP147
#define bp147_dbg(...) do {                     \
    fprintf(stderr, "bp147: " __VA_ARGS__);     \
    fputc('\n', stderr);                        \
  } while(0)
#else
#define bp147_dbg(...) do {                     \
  } while(0)
#endif


/* Number of decode protection register of the BP147 device */
#define BP147_NUM_DECPROT 3

/* Peripheral Identification Register 0-3 */
static const uint8_t TZPERIPHID[4] = { 0x70, 0x18, 0x04, 0x00 };

/* TZPC Identification Register 0-3 
 *
 * According the ARM DTO 0015A:       0x0D, 0xF0, 0x05, 0x00
 * Observed on CoreTile Express A9x4: 0x0D, 0x0F, 0x05, 0xB1
 */
static const uint8_t TZPCPCELLID[4] = { 0x0D, 0xF0, 0x05, 0xB1 };

typedef struct {
  uint32_t value;
  uint32_t implemented_mask;
  ASCPartition *slave[32];
} bp147_decprot_reg;

typedef struct {
  SysBusDevice busdev;
  uint32_t secure_ram_size;
  bp147_decprot_reg decprot[BP147_NUM_DECPROT];
} bp147_state;

/* Gets the I/O memory type index from a given address */
static inline int bp147_get_ioindex(target_phys_addr_t addr)
{
  return (addr >> IO_MEM_SHIFT) & (IO_MEM_NB_ENTRIES - 1);
}

static void bp147_decprot_update(bp147_state *s, int idx,
                                 uint32_t or_mask,
                                 uint32_t and_mask)
{
  uint32_t old_val = s->decprot[idx].value;
  uint32_t new_val = old_val;
  uint32_t changed;
  int i;

  new_val |= or_mask;
  new_val &= ~and_mask;
  new_val &= s->decprot[idx].implemented_mask;
  changed = old_val ^ new_val;

  if (!changed) {
    /* Nothing changed */
    return;
  }

  bp147_dbg("TZDECPROT%d <- %08x (old: %08x)", idx, new_val, old_val);
  s->decprot[idx].value = new_val;

  /* Update address space partitions */
  for (i = 0; i < 32; ++i) {
    ASCPartition *partition = s->decprot[idx].slave[i];
    uint32_t mask = 1U << i;
    
    if (partition && (changed & mask)) {
      int n_prot = (new_val & mask)? (PAGE_READ|PAGE_WRITE|PAGE_EXEC) : 0;
      int s_prot = PAGE_READ|PAGE_WRITE|PAGE_EXEC;
      arm_trustzone_set_access(partition, n_prot, s_prot);
    }
  }
}
    
static inline void bp147_decprot_set(bp147_state *s, int idx,
                                uint32_t val)
{
  bp147_decprot_update(s, idx, val, 0);
}

static inline void bp147_decprot_clear(bp147_state *s, int idx,
                                uint32_t val)
{
  bp147_decprot_update(s, idx, 0, ~val);
}


static void bp147_write(void *opaque, target_phys_addr_t offset,
                        uint32_t val)
{
  bp147_state *s = opaque;

  switch (offset) {
  case 0x000: /* Secure RAM Region Size Register */
    s->secure_ram_size = val & 0x3FF;
    break;

  case 0x804: /* Decode Protection 0 Set Register */
    bp147_decprot_set(s, 0, val);
    break;

  case 0x808: /* Decode Protection 0 Clear Register */
    bp147_decprot_clear(s, 0, val);
    break;

  case 0x810: /* Decode Protection 1 Set Register */
    bp147_decprot_set(s, 1, val);
    break;
    
  case 0x814: /* Decode Protection 1 Clear Register */
    bp147_decprot_clear(s, 0, val);
    break;

  case 0x81C: /* Decode Protection 2 Set Register */
    bp147_decprot_set(s, 2, val);
    break;

  case 0x820: /* Decode Protection 2 Clear Register */
    bp147_decprot_clear(s, 0, val);
    break;

  default:
    hw_error("bp147_write: Bad offset %x\n", (int)offset);
  }
}

static uint32_t bp147_read(void *opaque, target_phys_addr_t offset)
{
  bp147_state *s = opaque;

  switch (offset) {
  case 0x000: /* Secure RAM Region Size Register */
    return s->secure_ram_size;

  case 0x800: /* Decode Protection 0 Status Register */
    return s->decprot[0].value;

  case 0x80C: /* Decode Protection 1 Status Register */
    return s->decprot[1].value;

  case 0x818: /* Decode Protection 2 Status Register */
    return s->decprot[1].value;

  case 0xFE0: /* Peripheral Identification Register 0 */
  case 0xFE4: /* Peripheral Identification Register 1 */
  case 0xFE8: /* Peripheral Identification Register 2 */
  case 0xFEC: /* Peripheral Identification Register 3 */
    return TZPERIPHID[(offset - 0xFE0) >> 2];

  case 0xFF0: /* TZPC Identification Register 0 */
  case 0xFF4: /* TZPC Identification Register 1 */
  case 0xFF8: /* TZPC Identification Register 2 */
  case 0xFFC: /* TZPC Identification Register 3 */
    return TZPCPCELLID[(offset - 0xFF0) >> 2];

  default:
    hw_error("bp147_read: Bad offset %x\n", (int)offset);
    return 0;
  }
}

static CPUReadMemoryFunc *const bp147_readfn[] = {
  bp147_read,
  bp147_read,
  bp147_read
};

static CPUWriteMemoryFunc *const bp147_writefn[] = {
  bp147_write,
  bp147_write,
  bp147_write
};

void bp147_attach_slave(SysBusDevice *bp147, SysBusDevice *slave, int idx, int bit)
{
  int midx = -1;
  bp147_state *s;
  bp147_decprot_reg *dp;
  int n;

  assert(bp147 != NULL);
  assert(slave != NULL);
  assert(idx >= 0 && idx < BP147_NUM_DECPROT);
  assert(bit >= 0 && bit < 31);

  /* 
   * TODO: We currently only protect the first MMIO region of this device
   * (This could be extended in future)
   */
  for (n = 0; n < QDEV_MAX_MMIO; ++n) {
    if (slave->mmio[n].addr != (target_phys_addr_t)-1 && slave->mmio[n].size > 0) {
      if (midx != -1) {
        hw_error("bp147 model currently only supports sysbus devices with _one_ MMIO region");
        return;
      }

      midx = n;
    }
  }

  if (midx == -1) {
    return;
  }
  
  /* Attach the partition to the BP147 */
  s = FROM_SYSBUS(bp147_state, bp147);
  dp = &s->decprot[idx];
  dp->implemented_mask |= (1U << bit);
  dp->slave[bit] = arm_trustzone_create_partition(slave->mmio[midx].addr, 
                                                  slave->mmio[midx].addr + slave->mmio[midx].size - 1,
                                                  0, PAGE_READ | PAGE_WRITE | PAGE_EXEC);

  bp147_dbg("attached " TARGET_FMT_plx ".." TARGET_FMT_plx " to TZDECPROT%d[%d]",
            dp->slave[bit]->phys_start, dp->slave[bit]->phys_end, idx, bit);
}

static void bp147_save(QEMUFile *f, void *opaque)
{
  bp147_state *s = opaque;
  int i;

  qemu_put_be32(f, s->secure_ram_size);
  
  for (i = 0; i < BP147_NUM_DECPROT; ++i) {
    qemu_put_be32(f, s->decprot[i].value);
    qemu_put_be32(f, s->decprot[i].implemented_mask);
  }
}

static int bp147_load(QEMUFile *f, void *opaque, int version_id)
{
  bp147_state *s = opaque;
  int i;

  if (version_id != 1) {
    return -EINVAL;
  }

  s->secure_ram_size = qemu_get_be32(f);
  
  for (i = 0; i < BP147_NUM_DECPROT; ++i) {
    s->decprot[i].value = qemu_get_be32(f);
    s->decprot[i].implemented_mask = qemu_get_be32(f);
  }

  return 0;
}

static void bp147_reset(void *opaque)
{
  bp147_state *s = opaque;
  int i;

  s->secure_ram_size = 0x200;
  
  for (i = 0; i < BP147_NUM_DECPROT; ++i) {
    /* Force reset to secure world state */
    s->decprot[i].value = 0;
    bp147_decprot_update(s, i, 0, 0);
  }
}

static int bp147_init(SysBusDevice *dev)
{
  bp147_state *s = FROM_SYSBUS(bp147_state, dev);
  int iomemtype;
  
  iomemtype = cpu_register_io_memory(bp147_readfn, bp147_writefn, s,
                                     DEVICE_NATIVE_ENDIAN);
  sysbus_init_mmio(dev, 0x1000, iomemtype);

  
  qemu_register_reset(bp147_reset, s);
  bp147_reset(s);

  register_savevm(&dev->qdev, "bp147_tzpc", -1, 1,
                  bp147_save, bp147_load, s);
  return 0;
}

static void bp147_register_devices(void)
{
  sysbus_register_dev("bp147", sizeof(bp147_state), bp147_init);
}

device_init(bp147_register_devices);
