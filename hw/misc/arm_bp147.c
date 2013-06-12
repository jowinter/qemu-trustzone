/*
 * ARM TrustZone Protection Controller (BP147)
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

/* Maximum number of decode protection register of the BP147 device
 *
 * Implementations with more than the 3 regions (as discussed
 * in the datahseet) exist in the wild. We assume blatantly assume
 * a maximum of a regions 8 regions for now.
 */
#define BP147_MAX_DECPROTS 8

/* Peripheral Identification Register 0-3 */
static const uint8_t TZPERIPHID[4] = { 0x70, 0x18, 0x04, 0x00 };

/* TZPC Identification Register 0-3
 *
 * According the ARM DTO 0015A:       0x0D, 0xF0, 0x05, 0x00
 * Observed on CoreTile Express A9x4: 0x0D, 0xF0, 0x05, 0xB1
 */
static const uint8_t TZPCPCELLID[4] = { 0x0D, 0xF0, 0x05, 0xB1 };

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    uint32_t secure_ram_size;  /* Secure RAM size register */

    uint32_t decprot_mask;     /* DECPROTx implemented bits mask (all regs) */
    uint32_t num_decprot_regs; /* Number of DECPROTx registers */
    uint32_t *decprot_reset;   /* DECPROTx reset values    */
    uint32_t *decprot_value;   /* DECPROTx register values */
} BP147State;

#define TYPE_BP147 "bp147"
#define BP147(obj) OBJECT_CHECK(BP147State, (obj), TYPE_BP147)


static void bp147_decprot_update(BP147State *s, unsigned idx,
                                 uint32_t or_mask,
                                 uint32_t and_mask)
{
    if (idx >= s->num_decprot_regs) {
        qemu_log_mask(LOG_UNIMP, "bp147: write to unimplemented "
                      " DECPROT#%u register\n", idx);
        return;
    }

    uint32_t old_val = s->decprot_value[idx];
    uint32_t new_val = old_val;

    new_val |= or_mask;
    new_val &= ~and_mask;
    new_val &= s->decprot_mask;
    s->decprot_value[idx] = new_val;
}

static uint32_t bp147_decprot_read(BP147State *s, unsigned idx)
{
    if (idx >= s->num_decprot_regs) {
        qemu_log_mask(LOG_UNIMP, "bp147: read from unimplemented "
                      " DECPROT#%u register\n", idx);
        return 0;
    }

    return s->decprot_value[idx];
}

static void bp147_write(void *opaque, hwaddr offset,
                        uint64_t val, unsigned size)
{
    BP147State *s = (BP147State *) opaque;

    switch (offset) {
    case 0x000: /* Secure RAM Region Size Register */
        s->secure_ram_size = val & 0x3FF;
        break;

    case 0x804: /* Decode Protection 0 Set Register */
    case 0x810: /* Decode Protection 1 Set Register */
    case 0x81C: /* Decode Protection 2 Set Register */
    case 0x828: /* Decode Protection 3 Set Register */
    case 0x834: /* Decode Protection 4 Set Register */
    case 0x840: /* Decode Protection 5 Set Register */
    case 0x84C: /* Decode Protection 6 Set Register */
    case 0x858: /* Decode Protection 7 Set Register */
        bp147_decprot_update(s, (offset - 0x804) / 12, val, 0);
        break;

    case 0x808: /* Decode Protection 0 Clear Register */
    case 0x814: /* Decode Protection 1 Clear Register */
    case 0x820: /* Decode Protection 2 Clear Register */
    case 0x82C: /* Decode Protection 3 Clear Register */
    case 0x838: /* Decode Protection 4 Clear Register */
    case 0x844: /* Decode Protection 5 Clear Register */
    case 0x850: /* Decode Protection 6 Clear Register */
    case 0x85C: /* Decode Protection 7 Clear Register */
        bp147_decprot_update(s, (offset - 0x808) / 12, 0, ~val);
        break;

    default:
        hw_error("bp147_write: Bad offset %x\n", (int)offset);
        break;
    }
}

static uint64_t bp147_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    BP147State *s = (BP147State *) opaque;

    switch (offset) {
    case 0x000: /* Secure RAM Region Size Register */
        return s->secure_ram_size;

    case 0x800: /* Decode Protection 0 Status Register */
    case 0x80C: /* Decode Protection 1 Status Register */
    case 0x818: /* Decode Protection 2 Status Register */
    case 0x824: /* Decode Protection 3 Status Register */
    case 0x830: /* Decode Protection 4 Status Register */
    case 0x83C: /* Decode Protection 5 Status Register */
    case 0x848: /* Decode Protection 6 Status Register */
    case 0x854: /* Decode Protection 7 Status Register */
        return bp147_decprot_read(s, (offset - 0x800) / 12);

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

static const MemoryRegionOps bp147_ops = {
    .read = bp147_read,
    .write = bp147_write,
    .endianness = DEVICE_NATIVE_ENDIAN
};

static void bp147_reset(DeviceState *d)
{
    BP147State *s = BP147(d);
    unsigned i;

    s->secure_ram_size = 0x200;

    for (i = 0; i < s->num_decprot_regs; ++i) {
        s->decprot_value[i] = s->decprot_reset[i];
    }
}

static void bp147_realize(DeviceState *dev, Error **errp)
{
    BP147State *s = BP147(dev);

    if (s->num_decprot_regs > BP147_MAX_DECPROTS) {
        error_setg(errp,
                   "requested %u decprot count exceed BP147 limits (max: %u)",
                   s->num_decprot_regs, BP147_MAX_DECPROTS);
        return;
    }

    s->decprot_value = g_new(uint32_t, s->num_decprot_regs);

    memory_region_init_io(&s->iomem, &bp147_ops, s, "arm-bp147", 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
}

static void bp147_finalize(Object *obj)
{
    BP147State *s = BP147(obj);
    g_free(s->decprot_value);
    g_free(s->decprot_reset);
}

static Property bp147_properties[] = {
    /* Implementation masks for all decode protection registers */
    DEFINE_PROP_UINT32("decprot-mask", BP147State, decprot_mask, 0xFFFFFFFF),
    /* Reset values for the decode protection register */
    DEFINE_PROP_ARRAY("decprot", BP147State, num_decprot_regs,
                      decprot_reset, qdev_prop_uint32, uint32_t),
    DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_bp147 = {
    .name = "arm_bp147",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(secure_ram_size, BP147State),
        VMSTATE_VARRAY_UINT32(decprot_value, BP147State, num_decprot_regs,
                              1, vmstate_info_uint32, uint32_t),
        VMSTATE_END_OF_LIST()
    }
};

static void bp147_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->no_user = 1;
    dc->realize = bp147_realize;
    dc->reset = bp147_reset;
    dc->props = bp147_properties;
    dc->vmsd = &vmstate_bp147;
}

static TypeInfo bp147_info = {
    .name          = TYPE_BP147,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BP147State),
    .instance_finalize = bp147_finalize,
    .class_init    = bp147_class_init,
};

static void bp147_register_types(void)
{
    type_register_static(&bp147_info);
}

type_init(bp147_register_types)
