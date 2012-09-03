/*
 * ARM TrustZone Address Space Controller TZC-380
 * (As described in ARM DDI 0431C)
 *
 * Copyright (c) 2011-2012 IAIK, Graz University of Technology
 * Written by Johannes Winter
 *
 * This code is licenced under the GPL.
 */
#include "sysbus.h"
#include "arm_trustzone.h"

/* TODO: TrustZone: Stub implementation */

typedef struct {
    uint32_t low;
    uint32_t high;
    uint32_t attrs;
} tzc380_region;

static const uint8_t PERIPHID[5] = { 0x80, 0xB3, 0x1B, 0x00, 0x04 };
static const uint8_t PCELLID[4]  = { 0x0D, 0xF0, 0x05, 0xB1 };

#define TZC380_NUM_REGIONS 16

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;

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
    uint32_t region_low[TZC380_NUM_REGIONS];
    uint32_t region_high[TZC380_NUM_REGIONS];
    uint32_t region_attrs[TZC380_NUM_REGIONS];
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

static uint64_t tzc380_read(void *opaque, target_phys_addr_t offset,
                            unsigned size)
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
            return s->region_low[region];

        case 0x4: /* Region Setup High <n> Register */
            return s->region_high[region];

        case 0x8: /* Region Attributes <n> Register */
            return s->region_attrs[region];

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

static void tzc380_write(void *opaque, target_phys_addr_t offset,
                         uint64_t val, unsigned size)
{
    tzc380_state *s = opaque;

    if (offset >= 0x100 && offset <= 0x1FC) {
        unsigned region = (offset & 0x0F0) >> 4;
        if (region >= tzc380_num_regions(s)) {
            hw_error("tzc380_write: Invalid region %u\n", region);
            return;
        } else if (region >= tzc380_first_locked_region(s)) {
            /* TODO: TrustZone: Raise appropriate error */
            return;
        }

        switch (offset & 0x00F) {
        case 0x0: /* Region Setup Low <n> Register */
            s->region_low[region] = val & 0xFFFF0000;
            break;

        case 0x4: /* Region Setup High <n> Register */
            s->region_high[region] = val;
            break;

        case 0x8: /* Region Attributes <n> Register */
            if (region == 0) {
                val &= 0xF0000000; /* Allow control of sp0 */
                val |= 0x0000003F; /* 4G region, enabled   */
            } else {
                val &= 0xF000FF7F; /* Mask undefined bits */
            }

            s->region_attrs[region] = val;
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
                /* TODO: TrustZone: Raise appropriate error */
                break;
            }

            s->speculation_control = val & 0x00000003;
            break;

        case 0x034: /* Security inversion enable register */
            if (s->lockdown_select & 0x00000002) {
                /* TODO: TrustZone: Raise appropriate error */
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

static void tzc380_reset(DeviceState *d)
{
    tzc380_state *s = DO_UPCAST(tzc380_state, busdev.qdev, d);
    int n;

    /* TODO: TrustZone: Check with TZC380 datasheet */
    s->configuration   = 0x00001F0F; /* 32-bit AXI bus, 16 regions */
    s->action          = 0x00000000;
    s->lockdown_range  = 0x00000000;
    s->lockdown_select = 0x00000000;
    s->int_status      = 0x00000000;
    s->fail_addr_low   = 0x00000000;
    s->fail_addr_high  = 0x00000000;
    s->fail_control    = 0x00000000;
    s->fail_id         = 0x00000000;
    s->speculation_control   = 0x00000000;
    s->security_inversion_en = 0x00000000;

    for (n = 0; n < TZC380_NUM_REGIONS; ++n) {
        s->region_low[n]   = 0x00000000;
        s->region_high[n]  = 0x00000000;
        s->region_attrs[n] = 0x00000000;
    }
}

static const MemoryRegionOps tzc380_ops = {
    .read = tzc380_read,
    .write = tzc380_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static int tzc380_init(SysBusDevice *dev)
{
    tzc380_state *s = FROM_SYSBUS(tzc380_state, dev);

    memory_region_init_io(&s->iomem, &tzc380_ops, s, "tzc380", 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}

static const VMStateDescription vmstate_tzc380 = {
    .name = "tzc380",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(configuration,   tzc380_state),
        VMSTATE_UINT32(action,          tzc380_state),
        VMSTATE_UINT32(lockdown_range,  tzc380_state),
        VMSTATE_UINT32(lockdown_select, tzc380_state),
        VMSTATE_UINT32(int_status,      tzc380_state),
        VMSTATE_UINT32(fail_addr_low,   tzc380_state),
        VMSTATE_UINT32(fail_addr_high,  tzc380_state),
        VMSTATE_UINT32(fail_control,    tzc380_state),
        VMSTATE_UINT32(fail_id,         tzc380_state),
        VMSTATE_UINT32(speculation_control,   tzc380_state),
        VMSTATE_UINT32(security_inversion_en, tzc380_state),
        VMSTATE_UINT32_ARRAY(region_low,   tzc380_state, TZC380_NUM_REGIONS),
        VMSTATE_UINT32_ARRAY(region_high,  tzc380_state, TZC380_NUM_REGIONS),
        VMSTATE_UINT32_ARRAY(region_attrs, tzc380_state, TZC380_NUM_REGIONS),
        VMSTATE_END_OF_LIST()
    }
};

static void tzc380_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = tzc380_init;
    dc->no_user = 1;
    dc->reset = tzc380_reset;
    dc->vmsd = &vmstate_tzc380;
}

static TypeInfo tzc380_info = {
    .name          = "tzc380",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(tzc380_state),
    .class_init    = tzc380_class_init,
};

static void tzc380_register_types(void)
{
    type_register_static(&tzc380_info);
}

type_init(tzc380_register_types)
