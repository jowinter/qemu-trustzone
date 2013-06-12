/*
 * ARM TrustZone Address Space Controller TZC-380
 * (As described in ARM DDI 0431C)
 *
 * Copyright (c) 2011-2013 IAIK, Graz University of Technology
 * Written by Johannes Winter <johannes.winter@iaik.tugraz.at>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "hw/hw.h"
#include "hw/sysbus.h"

static const uint8_t PERIPHID[5] = { 0x80, 0xB3, 0x1B, 0x00, 0x04 };
static const uint8_t PCELLID[4]  = { 0x0D, 0xF0, 0x05, 0xB1 };

/*
 * Maximum and minimum number of regions supported by the TZC-380
 * peripheral model. (implied by configuration register)
 */
#define TZC380_MIN_REGIONS  1U
#define TZC380_MAX_REGIONS 16U

typedef struct {
    uint32_t low;
    uint32_t high;
    uint32_t attrs;
} TZC380Region;

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

    uint32_t num_regions;
    TZC380Region *regions;
} TZC380State;


#define TYPE_TZC380 "tzc380"
#define TZC380(obj) OBJECT_CHECK(TZC380State, (obj), TYPE_TZC380)

/*
 * Gets the number of the lowest locked region of a TZC380
 * instance.
 *
 * @return: The first locked region of this TZC380 instance or
 *  a value larger than the highest valid region index if no
 *  regions are locked.
 */
static inline unsigned tzc380_first_locked_region(TZC380State *s)
{
    unsigned region = s->num_regions; /* No regions locked */

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

static uint64_t tzc380_read(void *opaque, hwaddr offset,
                            unsigned size)
{
    TZC380State *s = opaque;

    if (offset >= 0x100 && offset <= 0x1FC) {
        unsigned region = (offset & 0x0F0) >> 4;
        if (region >= s->num_regions) {
            hw_error("tzc380_read: Invalid region %u\n", region);
            return 0xFFFFFFFF;
        }

        switch (offset & 0x00F) {
        case 0x0: /* Region Setup Low <n> Register */
            return s->regions[region].low;

        case 0x4: /* Region Setup High <n> Register */
            return s->regions[region].high;

        case 0x8: /* Region Attributes <n> Register */
            return s->regions[region].attrs;

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

static void tzc380_write(void *opaque, hwaddr offset,
                         uint64_t val, unsigned size)
{
    TZC380State *s = opaque;

    if (offset >= 0x100 && offset <= 0x1FC) {
        unsigned region = (offset & 0x0F0) >> 4;
        if (region >= s->num_regions) {
            hw_error("tzc380_write: Invalid region %u\n", region);
            return;
        } else if (region >= tzc380_first_locked_region(s)) {
            /* TODO: TrustZone: Raise appropriate error */
            return;
        }

        switch (offset & 0x00F) {
        case 0x0: /* Region Setup Low <n> Register */
            s->regions[region].low = val & 0xFFFF0000;
            break;

        case 0x4: /* Region Setup High <n> Register */
            s->regions[region].high = val;
            break;

        case 0x8: /* Region Attributes <n> Register */
            if (region == 0) {
                val &= 0xF0000000; /* Allow control of sp0 */
                val |= 0x0000003F; /* 4G region, enabled   */
            } else {
                val &= 0xF000FF7F; /* Mask undefined bits */
            }

            s->regions[region].attrs = val;
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
    TZC380State *s = TZC380(d);
    int n;

    /* 32-bit AXI bus, configured number of regions */
    s->configuration   = 0x00001F00 | (s->num_regions - 1);
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

    for (n = 0; n < s->num_regions; ++n) {
        s->regions[n].low   = 0x00000000;
        s->regions[n].high  = 0x00000000;
        s->regions[n].attrs = 0x00000000;
    }
}

static const MemoryRegionOps tzc380_ops = {
    .read = tzc380_read,
    .write = tzc380_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void tzc380_realize(DeviceState *dev, Error **errp)
{
    TZC380State *s = TZC380(dev);

    if (s->num_regions < TZC380_MIN_REGIONS ||
        s->num_regions > TZC380_MAX_REGIONS) {
        error_setg(errp,
                   "requested %u regions exceed TZC380 limits (%u..%u)",
                   s->num_regions, TZC380_MIN_REGIONS, TZC380_MAX_REGIONS);
        return;
    }

    s->regions = g_new0(TZC380Region, s->num_regions);

    memory_region_init_io(&s->iomem, &tzc380_ops, s, "arm-tzc380", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void tzc380_finalize(Object *obj)
{
    TZC380State *s = TZC380(obj);
    g_free(s->regions);
}

static Property tzc380_properties[] = {
    /* Number of protection regions */
    DEFINE_PROP_UINT32("num-regions", TZC380State, num_regions,
                       TZC380_MAX_REGIONS),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_tzc380_region = {
    .name = "tzc380_region",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(low,   TZC380Region),
        VMSTATE_UINT32(high,  TZC380Region),
        VMSTATE_UINT32(attrs, TZC380Region),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_tzc380 = {
    .name = "tzc380",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(configuration,   TZC380State),
        VMSTATE_UINT32(action,          TZC380State),
        VMSTATE_UINT32(lockdown_range,  TZC380State),
        VMSTATE_UINT32(lockdown_select, TZC380State),
        VMSTATE_UINT32(int_status,      TZC380State),
        VMSTATE_UINT32(fail_addr_low,   TZC380State),
        VMSTATE_UINT32(fail_addr_high,  TZC380State),
        VMSTATE_UINT32(fail_control,    TZC380State),
        VMSTATE_UINT32(fail_id,         TZC380State),
        VMSTATE_UINT32(speculation_control,   TZC380State),
        VMSTATE_UINT32(security_inversion_en, TZC380State),
        VMSTATE_STRUCT_VARRAY_UINT32(regions, TZC380State, num_regions, 1,
                                     vmstate_tzc380_region, TZC380Region),
        VMSTATE_END_OF_LIST()
    }
};

static void tzc380_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->no_user = 1;
    dc->realize = tzc380_realize;
    dc->reset = tzc380_reset;
    dc->props = tzc380_properties;
    dc->vmsd = &vmstate_tzc380;
}

static TypeInfo tzc380_info = {
    .name          = TYPE_TZC380,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(TZC380State),
    .instance_finalize = tzc380_finalize,
    .class_init    = tzc380_class_init,
};

static void tzc380_register_types(void)
{
    type_register_static(&tzc380_info);
}

type_init(tzc380_register_types)
