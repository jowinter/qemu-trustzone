/*
 * ARM TrustZone Protection Controller BP147
 *
 * Copyright (c) 2011-2012 IAIK, Graz University of Technology
 * Written by Johannes Winter <johannes.winter@iaik.tugraz.at>
 *
 * This code is licenced under the GPL.
 */
#include "sysbus.h"
#include "qdev.h"
#include "arm_trustzone.h"

/* TODO: TrustZone: Stub implementation */

/* Number of decode protection register of the BP147 device
 *
 * TODO: TrustZone: Implementations with more then 3 regions
 * (as discussed in the datahseet) exist in the wild. We assume
 * blatantly 8 regions for now.
 */
#define BP147_NUM_DECPROT 8

/* Peripheral Identification Register 0-3 */
static const uint8_t TZPERIPHID[4] = { 0x70, 0x18, 0x04, 0x00 };

/* TZPC Identification Register 0-3
 *
 * According the ARM DTO 0015A:       0x0D, 0xF0, 0x05, 0x00
 * Observed on CoreTile Express A9x4: 0x0D, 0x0F, 0x05, 0xB1
 */
static const uint8_t TZPCPCELLID[4] = { 0x0D, 0xF0, 0x05, 0xB1 };

typedef struct BP147Region BP147Region;

struct BP147Region {
    BP147DecprotInfo info;
    QSLIST_ENTRY(BP147Region) region_link;
};

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    uint32_t secure_ram_size;
    uint32_t decprot[BP147_NUM_DECPROT]; /* Decode protection bits */
    uint32_t impmask[BP147_NUM_DECPROT]; /* Implemented bits mask */
    QSLIST_HEAD(, BP147Region) regions;
} BP147Device;

static void bp147_decprot_update(BP147Device *s, unsigned idx,
                                 uint32_t or_mask,
                                 uint32_t and_mask)
{
    uint32_t old_val = s->decprot[idx];
    uint32_t new_val = old_val;
    uint32_t changed;
    BP147Region *rgn;

    new_val |= or_mask;
    new_val &= ~and_mask;
    new_val &= s->impmask[idx];
    changed = old_val ^ new_val;

    if (!changed) {
        /* Nothing changed */
        return;
    }

    QSLIST_FOREACH(rgn, &s->regions, region_link) {
        uint32_t rgn_mask = 1 << rgn->info.bit;
        if (rgn->info.reg == idx && (changed & rgn_mask)) {
            int value = (new_val & rgn_mask) != 0;
            if (rgn->info.update) {
                /* User-callback */
                (rgn->info.update)(rgn->info.opaque, value);
            }

            if (rgn->info.size != 0) {
                /* TODO: TrustZone: Update the physical protection map */
            }
        }
    }

    s->decprot[idx] = new_val;
}

static void bp147_write(void *opaque, target_phys_addr_t offset,
                        uint64_t val, unsigned size)
{
    BP147Device *s = opaque;

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
    }
}

static uint64_t bp147_read(void *opaque, target_phys_addr_t offset,
                           unsigned size)
{
    BP147Device *s = opaque;

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
        return s->decprot[(offset - 0x800) / 12];

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
    BP147Device *s = DO_UPCAST(BP147Device, busdev.qdev, d);
    int i;

    s->secure_ram_size = 0x200;

    for (i = 0; i < BP147_NUM_DECPROT; ++i) {
        /* TODO: TrustZone: Do we want non-zero reset values here? */
        /* Force reset to secure world state */
        s->decprot[i] = 0x00000000;
    }

    /* TODO: Update settings */
}

void bp147_add_region(DeviceState *d, const BP147DecprotInfo *info)
{
    BP147Device *s = DO_UPCAST(BP147Device, busdev.qdev, d);
    BP147Region *rgn = g_malloc0(sizeof(*rgn));

    if (info->reg >= BP147_NUM_DECPROT || info->bit >= 32) {
        hw_error("Invalid decode protection bit definition");
    }

    rgn->info = *info;

    QSLIST_INSERT_HEAD(&s->regions, rgn, region_link);
    s->impmask[info->reg] |= (1 << info->bit);
}

DeviceState* bp147_create_simple(target_phys_addr_t addr,
                                 const BP147DecprotInfo *info, unsigned count)
{
    DeviceState *dev;
    unsigned n;

    dev = qdev_create(NULL, "bp147");

    for (n = 0; n < count; ++n) {
        bp147_add_region(dev, &info[n]);
    }

    qdev_init_nofail(dev);

    sysbus_mmio_map(sysbus_from_qdev(dev), 0, addr);
    return dev;
}

static int bp147_init(SysBusDevice *dev)
{
    BP147Device *s = FROM_SYSBUS(BP147Device, dev);

    memory_region_init_io(&s->iomem, &bp147_ops, s, "bp147", 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}

static const VMStateDescription vmstate_bp147 = {
    .name = "bp147",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(secure_ram_size, BP147Device),
        VMSTATE_UINT32_ARRAY(decprot, BP147Device, BP147_NUM_DECPROT),
        VMSTATE_END_OF_LIST()
    }
};

static void bp147_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);

    k->init = bp147_init;
    dc->no_user = 1;
    dc->reset = bp147_reset;
}

static void bp147_instance_init(Object *obj)
{
    BP147Device *s = OBJECT_CHECK(BP147Device, obj, "bp147");
    QSLIST_INIT(&s->regions);
}

static void bp147_instance_finalize(Object *obj)
{
    BP147Device *s = OBJECT_CHECK(BP147Device, obj, "bp147");
    BP147Region *rgn;
    BP147Region *tmp;

    QSLIST_FOREACH_SAFE(rgn, &s->regions, region_link, tmp) {
        g_free(rgn);
    }
}

static TypeInfo bp147_info = {
    .name          = "bp147",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(BP147Device),
    .class_init    = bp147_class_init,
    .instance_init = bp147_instance_init,
    .instance_finalize = bp147_instance_finalize
};

static void bp147_register_types(void)
{
    type_register_static(&bp147_info);
}

type_init(bp147_register_types)
