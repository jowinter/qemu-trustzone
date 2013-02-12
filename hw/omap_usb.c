/*
 * TI OMAP3 High-Speed USB Host and OTG Controller emulation.
 *
 * Copyright (C) 2009 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) any later version of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include "qemu-common.h"
#include "qemu/timer.h"
#include "usb.h"
#include "omap.h"
#include "irq.h"
#include "devices.h"
#include "hw.h"
#include "sysbus.h"

//#define OMAP3_HSUSB_DEBUG

#ifdef OMAP3_HSUSB_DEBUG
#define TRACE(fmt,...) fprintf(stderr, "%s: " fmt "\n", __FUNCTION__, ##__VA_ARGS__)
#else
#define TRACE(...)
#endif

/* usb-musb.c */
extern CPUReadMemoryFunc *musb_read[];
extern CPUWriteMemoryFunc *musb_write[];

typedef struct omap3_hsusb_otg_s {
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq mc_irq;
    qemu_irq dma_irq;
    qemu_irq stdby_irq;
    MUSBState *musb;
    
    uint8_t rev;
    uint16_t sysconfig;
    uint8_t interfsel;
    uint8_t simenable;
    uint8_t forcestdby;
} OMAP3HSUSBOTGState;

static const VMStateDescription vmstate_omap3_hsusb_otg = {
    .name = "omap3_hsusb_otg",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT16(sysconfig, OMAP3HSUSBOTGState),
        VMSTATE_UINT8(interfsel, OMAP3HSUSBOTGState),
        VMSTATE_UINT8(simenable, OMAP3HSUSBOTGState),
        VMSTATE_UINT8(forcestdby, OMAP3HSUSBOTGState),
        VMSTATE_END_OF_LIST()
    }
};

static void omap3_hsusb_otg_stdby_update(OMAP3HSUSBOTGState *s)
{
    if (s->stdby_irq) {
        qemu_set_irq(s->stdby_irq, s->forcestdby);
    }
}

static void omap3_hsusb_otg_reset(DeviceState *dev)
{
    OMAP3HSUSBOTGState *s = FROM_SYSBUS(OMAP3HSUSBOTGState,
                                        SYS_BUS_DEVICE(dev));
    s->rev = 0x33;
    s->sysconfig = 0;
    s->interfsel = 0x1;
    s->simenable = 0;
    s->forcestdby = 1;
    musb_reset(s->musb);
    omap3_hsusb_otg_stdby_update(s);
}

static uint32_t omap3_hsusb_otg_read(int access,
                                     void *opaque,
                                     hwaddr addr)
{
    OMAP3HSUSBOTGState *s = opaque;
    
    if (addr < 0x200)
        return musb_read[access](s->musb, addr);
    if (addr < 0x400)
        return musb_read[access](s->musb, 0x20 + ((addr >> 3) & 0x3c));
    switch (addr) {
        case 0x400: /* OTG_REVISION */
            TRACE("OTG_REVISION: 0x%08x", s->rev);
            return s->rev;
        case 0x404: /* OTG_SYSCONFIG */
            TRACE("OTG_SYSCONFIG: 0x%08x", s->sysconfig);
            return s->sysconfig;
        case 0x408: /* OTG_SYSSTATUS */
            TRACE("OTG_SYSSTATUS: 0x00000001");
            return 1; /* reset finished */
        case 0x40c: /* OTG_INTERFSEL */
            TRACE("OTG_INTERFSEL: 0x%08x", s->interfsel);
            return s->interfsel;
        case 0x410: /* OTG_SIMENABLE */
            TRACE("OTG_SIMENABLE: 0x%08x", s->simenable);
            return s->simenable;
        case 0x414: /* OTG_FORCESTDBY */
            TRACE("OTG_FORCESTDBY: 0x%08x", s->forcestdby);
            return s->forcestdby;
        default:
            break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap3_hsusb_otg_write(int access,
                                  void *opaque,
                                  hwaddr addr,
                                  uint32_t value)
{
    OMAP3HSUSBOTGState *s = opaque;
    
    if (addr < 0x200)
        musb_write[access](s->musb, addr, value);
    else if (addr < 0x400)
        musb_write[access](s->musb, 0x20 + ((addr >> 3) & 0x3c), value);
    else switch (addr) {
        case 0x400: /* OTG_REVISION */
        case 0x408: /* OTG_SYSSTATUS */
            OMAP_RO_REG(addr);
            break;
        case 0x404: /* OTG_SYSCONFIG */
            TRACE("OTG_SYSCONFIG = 0x%08x", value);
            if (value & 2) /* SOFTRESET */
                omap3_hsusb_otg_reset(&s->busdev.qdev);
            s->sysconfig = value & 0x301f;
            break;
        case 0x40c: /* OTG_INTERFSEL */
            TRACE("OTG_INTERFSEL = 0x%08x", value);
            s->interfsel = value & 0x3;
            break;
        case 0x410: /* OTG_SIMENABLE */
            TRACE("OTG_SIMENABLE = 0x%08x", value);
            cpu_abort(cpu_single_env,
                      "%s: USB simulation mode not supported\n",
                      __FUNCTION__);
            break;
        case 0x414: /* OTG_FORCESTDBY */
            TRACE("OTG_FORCESTDBY = 0x%08x", value);
            s->forcestdby = value & 1;
            omap3_hsusb_otg_stdby_update(s);
            break;
        default:
            OMAP_BAD_REG(addr);
            break;
    }
}

static uint32_t omap3_hsusb_otg_readb(void *opaque, hwaddr addr)
{
    return omap3_hsusb_otg_read(0, opaque, addr);
}

static uint32_t omap3_hsusb_otg_readh(void *opaque, hwaddr addr)
{
    return omap3_hsusb_otg_read(1, opaque, addr);
}

static uint32_t omap3_hsusb_otg_readw(void *opaque, hwaddr addr)
{
    return omap3_hsusb_otg_read(2, opaque, addr);
}

static void omap3_hsusb_otg_writeb(void *opaque, hwaddr addr,
                                   uint32_t value)
{
    omap3_hsusb_otg_write(0, opaque, addr, value);
}

static void omap3_hsusb_otg_writeh(void *opaque, hwaddr addr,
                                   uint32_t value)
{
    omap3_hsusb_otg_write(1, opaque, addr, value);
}

static void omap3_hsusb_otg_writew(void *opaque, hwaddr addr,
                                   uint32_t value)
{
    omap3_hsusb_otg_write(2, opaque, addr, value);
}

static const MemoryRegionOps omap3_hsusb_otg_ops = {
    .old_mmio = {
        .read = {
            omap3_hsusb_otg_readb,
            omap3_hsusb_otg_readh,
            omap3_hsusb_otg_readw,
        },
        .write = {
            omap3_hsusb_otg_writeb,
            omap3_hsusb_otg_writeh,
            omap3_hsusb_otg_writew,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void omap3_hsusb_musb_core_intr(void *opaque, int source, int level)
{
    OMAP3HSUSBOTGState *s = opaque;
    /*TRACE("intr 0x%08x, 0x%08x, 0x%08x", source, level, musb_core_intr_get(s->musb));*/
    qemu_set_irq(s->mc_irq, level);
}

static int omap3_hsusb_otg_init(SysBusDevice *dev)
{
    OMAP3HSUSBOTGState *s = FROM_SYSBUS(OMAP3HSUSBOTGState, dev);
    sysbus_init_irq(dev, &s->mc_irq);
    sysbus_init_irq(dev, &s->dma_irq);
    sysbus_init_irq(dev, &s->stdby_irq);
    memory_region_init_io(&s->iomem, &omap3_hsusb_otg_ops, s,
                          "omap3_hsusb_otg", 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
    qdev_init_gpio_in(&dev->qdev, omap3_hsusb_musb_core_intr, musb_irq_max);
    s->musb = musb_init(&dev->qdev, 0);
    vmstate_register(&dev->qdev, -1, &vmstate_omap3_hsusb_otg, s);
    return 0;
}

static void omap3_hsusb_otg_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = omap3_hsusb_otg_init;
    dc->reset = omap3_hsusb_otg_reset;
}

static TypeInfo omap3_hsusb_otg_info = {
    .name = "omap3_hsusb_otg",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OMAP3HSUSBOTGState),
    .class_init = omap3_hsusb_otg_class_init,
};

typedef struct omap3_hsusb_host_s {
    SysBusDevice busdev;
    MemoryRegion tll_iomem;
    MemoryRegion host_iomem;
    MemoryRegion ehci_iomem;
    qemu_irq ehci_irq;
    qemu_irq tll_irq;
    
    uint32_t uhh_sysconfig;
    uint32_t uhh_hostconfig;
    uint32_t uhh_debug_csr;
    uint32_t tll_sysconfig;
    uint32_t insnreg05_ulpi;
} OMAP3HSUSBHostState;

static const VMStateDescription vmstate_omap3_hsusb_host = {
    .name = "omap3_hsusb_host",
    .version_id = 1,
    .minimum_version_id = 1,
    .minimum_version_id_old = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(uhh_sysconfig, OMAP3HSUSBHostState),
        VMSTATE_UINT32(uhh_hostconfig, OMAP3HSUSBHostState),
        VMSTATE_UINT32(uhh_debug_csr, OMAP3HSUSBHostState),
        VMSTATE_UINT32(tll_sysconfig, OMAP3HSUSBHostState),
        VMSTATE_UINT32(insnreg05_ulpi, OMAP3HSUSBHostState),
        VMSTATE_END_OF_LIST()
    }
};

static void omap3_hsusb_host_reset(DeviceState *dev)
{
    OMAP3HSUSBHostState *s = FROM_SYSBUS(OMAP3HSUSBHostState,
                                         SYS_BUS_DEVICE(dev));
    s->uhh_sysconfig = 1;
    s->uhh_hostconfig = 0x700;
    s->uhh_debug_csr = 0x20;
    /* TODO: perform OHCI & EHCI reset */
    s->tll_sysconfig = 1;
}

static uint32_t omap3_hsusb_host_read(void *opaque, hwaddr addr)
{
    OMAP3HSUSBHostState *s = opaque;
    TRACE(OMAP_FMT_plx, addr);

    switch (addr) {
        case 0x00: /* UHH_REVISION */
            return 0x10;
        case 0x10: /* UHH_SYSCONFIG */
            return s->uhh_sysconfig;
        case 0x14: /* UHH_SYSSTATUS */
            return 0x7; /* EHCI_RESETDONE | OHCI_RESETDONE | RESETDONE */
        case 0x40: /* UHH_HOSTCONFIG */
            return s->uhh_hostconfig;
        case 0x44: /* UHH_DEBUG_CSR */
            return s->uhh_debug_csr;
        default:
            break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap3_hsusb_host_write(void *opaque, hwaddr addr,
                                   uint32_t value)
{
    OMAP3HSUSBHostState *s = opaque;
    TRACE(OMAP_FMT_plx " = 0x%08x", addr, value);

    switch (addr) {
        case 0x00: /* UHH_REVISION */
        case 0x14: /* UHH_SYSSTATUS */
            OMAP_RO_REG(addr);
            break;
        case 0x10: /* UHH_SYSCONFIG */
            s->uhh_sysconfig = value & 0x311d;
            if (value & 2) { /* SOFTRESET */
                omap3_hsusb_host_reset(&s->busdev.qdev);
            }
            break;
        case 0x40: /* UHH_HOSTCONFIG */
            s->uhh_hostconfig = value & 0x1f3d;
            break;
        case 0x44: /* UHH_DEBUG_CSR */
            s->uhh_debug_csr = value & 0xf00ff;
            break;
        default:
            OMAP_BAD_REG(addr);
            break;
    }
}

static const MemoryRegionOps omap3_hsusb_host_ops = {
    .old_mmio = {
        .read = {
            omap_badwidth_read32,
            omap_badwidth_read32,
            omap3_hsusb_host_read,
        },
        .write = {
            omap_badwidth_write32,
            omap_badwidth_write32,
            omap3_hsusb_host_write,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint32_t omap3_hsusb_ehci_read(void *opaque, hwaddr addr)
{
    OMAP3HSUSBHostState *s = opaque;
    TRACE(OMAP_FMT_plx, addr);
    switch (addr) {
        case 0xa4: /* INSNREG05_ULPI */
            return s->insnreg05_ulpi;
        default:
            break;
    }
    return 0;
}

static void omap3_hsusb_ehci_write(void *opaque, hwaddr addr,
                                   uint32_t value)
{
    OMAP3HSUSBHostState *s = opaque;
    TRACE(OMAP_FMT_plx " = 0x%08x", addr, value);

    switch (addr) {
        case 0xa4: /* INSNREG05_ULPI */
            s->insnreg05_ulpi = value & 0xF0000000;
        default:
            break;
     }
}

static const MemoryRegionOps omap3_hsusb_ehci_ops = {
    .old_mmio = {
        .read = {
            omap_badwidth_read32,
            omap_badwidth_read32,
            omap3_hsusb_ehci_read,
        },
        .write = {
            omap_badwidth_write32,
            omap_badwidth_write32,
            omap3_hsusb_ehci_write,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static uint32_t omap3_hsusb_tll_read(void *opaque, hwaddr addr)
{
    OMAP3HSUSBHostState *s = opaque;
    TRACE(OMAP_FMT_plx, addr);

    switch (addr) {
        case 0x00: /* USBTLL_REVISION */
            return 0x1;
        case 0x10: /* USBTLL_SYSCONFIG */
            return s->tll_sysconfig;
        case 0x14: /* USBTLL_SYSSTATUS */
            return 0x1; /* RESETDONE */
        case 0x18: /* USBTLL_IRQSTATUS */
            return 0;
        case 0x1C: /* USBTLL_IRQENABLE */
            return 0;
        default:
            break;
    }
    return 0;
}

static void omap3_hsusb_tll_write(void *opaque, hwaddr addr,
                                  uint32_t value)
{
    OMAP3HSUSBHostState *s = opaque;
    TRACE(OMAP_FMT_plx " = 0x%08x", addr, value);

    switch (addr) {
        case 0x00: /* USBTLL_REVISION */
        case 0x14: /* USBTLL_SYSSTATUS */
            OMAP_RO_REG(addr);
            break;
        case 0x10: /* USBTLL_SYSCONFIG */
            s->tll_sysconfig = value & 0xFFFFFEE0;;
            break;
        default:
            OMAP_BAD_REG(addr);
            break;
    }
}

static const MemoryRegionOps omap3_hsusb_tll_ops = {
    .old_mmio = {
        .read = {
            omap_badwidth_read32,
            omap_badwidth_read32,
            omap3_hsusb_tll_read,
        },
        .write = {
            omap_badwidth_write32,
            omap_badwidth_write32,
            omap3_hsusb_tll_write,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int omap3_hsusb_host_init(SysBusDevice *dev)
{
    OMAP3HSUSBHostState *s = FROM_SYSBUS(OMAP3HSUSBHostState, dev);
    sysbus_init_irq(dev, &s->ehci_irq);
    sysbus_init_irq(dev, &s->tll_irq);

    memory_region_init_io(&s->tll_iomem, &omap3_hsusb_tll_ops, s,
                              "omap_usb.tll", 0x1000);
    memory_region_init_io(&s->host_iomem, &omap3_hsusb_host_ops, s,
                              "omap_usb.tll", 0x400);
    memory_region_init_io(&s->ehci_iomem, &omap3_hsusb_ehci_ops, s,
                              "omap_usb.tll", 0x400);
    sysbus_init_mmio(dev, &s->tll_iomem);
    sysbus_init_mmio(dev, &s->host_iomem);
    sysbus_init_mmio(dev, &s->ehci_iomem);
    /* OHCI is instantiated by omap3.c */
    vmstate_register(&dev->qdev, -1, &vmstate_omap3_hsusb_host, s);
    return 0;
}

static void omap3_hsusb_host_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = omap3_hsusb_host_init;
    dc->reset = omap3_hsusb_host_reset;
}

static TypeInfo omap3_hsusb_host_info = {
    .name = "omap3_hsusb_host",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OMAP3HSUSBHostState),
    .class_init = omap3_hsusb_host_class_init,
};

static void omap3_hsusb_register_types(void)
{
    type_register_static(&omap3_hsusb_otg_info);
    type_register_static(&omap3_hsusb_host_info);
}

type_init(omap3_hsusb_register_types)
