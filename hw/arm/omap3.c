/*
 * TI OMAP3 processors emulation.
 * Based on the public OMAP34xx and OMAP36xx TRM documents.
 *
 * Copyright (C) 2008 yajin <yajin@vm-kernel.org>
 * Copyright (C) 2009-2010 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "hw/hw.h"
#include "hw/arm.h"
#include "hw/arm/omap.h"
#include "sysemu/sysemu.h"
#include "qemu/timer.h"
#include "char/char.h"
#include "hw/block/flash.h"
#include "hw/arm/soc_dma.h"
#include "hw/sysbus.h"
#include "audio/audio.h"
#include "block/block.h"
#include "hw/qdev-addr.h"

//#define OMAP3_DEBUG
//#define OMAP3_DEBUG_SCM
//#define OMAP3_DEBUG_CM
//#define OMAP3_DEBUG_PRM
//#define OMAP3_DEBUG_SMS

#ifdef OMAP3_DEBUG
#define TRACE(fmt, ...) fprintf(stderr, "%s " fmt "\n", __FUNCTION__, ##__VA_ARGS__)
#else
#define TRACE(...)
#undef OMAP_RO_REG
#undef OMAP_RO_REGV
#undef OMAP_BAD_REG
#undef OMAP_BAD_REGV
#define OMAP_RO_REG(...)
#define OMAP_RO_REGV(...)
#define OMAP_BAD_REG(...)
#define OMAP_BAD_REGV(...)
#endif

#ifdef OMAP3_DEBUG_SCM
#define TRACE_SCM(...) TRACE(__VA_ARGS__)
#else
#define TRACE_SCM(...)
#endif
#ifdef OMAP3_DEBUG_CM
#define TRACE_CM(...) TRACE(__VA_ARGS__)
#else
#define TRACE_CM(...)
#endif
#ifdef OMAP3_DEBUG_PRM
#define TRACE_PRM(...) TRACE(__VA_ARGS__)
#else
#define TRACE_PRM(...)
#endif
#ifdef OMAP3_DEBUG_SMS
#define TRACE_SMS(...) TRACE(__VA_ARGS__)
#else
#define TRACE_SMS(...)
#endif

struct omap3_l3_region_s {
    uint32_t size;
    enum {
        L3TYPE_IA = 0, /* initiator agent */
        L3TYPE_TA,     /* target agent */
        L3TYPE_PM,     /* protection mechanism */
        L3TYPE_UNDEF,  /* every access will emit an error message */
    } type;
};

static const struct omap3_l3_region_s omap3_l3_region[] = {
    {0x0400, L3TYPE_UNDEF},  /* L3RT */
    {0x0400, L3TYPE_UNDEF},  /* L3SI */
    {0x0c00, L3TYPE_UNDEF},  /* reserved */
    {0x0400, L3TYPE_IA},     /* MPUSS_IA */
    {0x0400, L3TYPE_IA},     /* IVASS_IA */
    {0x0400, L3TYPE_IA},     /* SGXSS_IA */
    {0x0400, L3TYPE_TA},     /* SMS_TA */
    {0x0400, L3TYPE_TA},     /* GPMC_TA */
    {0x0400, L3TYPE_TA},     /* OCM_RAM_TA */
    {0x0400, L3TYPE_TA},     /* OCM_ROM_TA */
    {0x0400, L3TYPE_IA},     /* D2D_IA */
    {0x0400, L3TYPE_TA},     /* D2D_TA */
    {0x0800, L3TYPE_UNDEF},  /* reserved */
    {0x0400, L3TYPE_IA},     /* HSUSB_HOST_IA */
    {0x0400, L3TYPE_IA},     /* HSUSB_OTG_IA */
    {0x0400, L3TYPE_UNDEF},  /* reserved */
    {0x0400, L3TYPE_IA},     /* SDMA_RD_IA */
    {0x0400, L3TYPE_IA},     /* SDMA_WR_IA */
    {0x0400, L3TYPE_IA},     /* DSS_IA */
    {0x0400, L3TYPE_IA},     /* CAMISP_IA */
    {0x0400, L3TYPE_IA},     /* DAP_IA */
    {0x0400, L3TYPE_TA},     /* IVASS_TA */
    {0x0400, L3TYPE_TA},     /* SGXSS_TA */
    {0x0400, L3TYPE_TA},     /* L4_CORE_TA */
    {0x0400, L3TYPE_TA},     /* L4_PER_TA */
    {0x0400, L3TYPE_TA},     /* L4_EMU_TA */
    {0x8c00, L3TYPE_UNDEF},  /* reserved */
    {0x0400, L3TYPE_PM},     /* RT_PM */
    {0x2000, L3TYPE_UNDEF},  /* reserved */
    {0x0400, L3TYPE_PM},     /* GPMC_PM */
    {0x0400, L3TYPE_PM},     /* OCM_RAM_PM */
    {0x0400, L3TYPE_PM},     /* OCM_ROM_PM */
    {0x0400, L3TYPE_PM},     /* D2D_PM */
    {0x0c00, L3TYPE_UNDEF},  /* reserved */
    {0x0400, L3TYPE_PM},     /* IVA_PM */
    {0xfebc00, L3TYPE_UNDEF} /* reserved */
};

struct omap3_l3_initiator_agent_s {
    hwaddr base;
    
    uint32_t component;
    uint32_t control;
    uint32_t status;
};

static uint32_t omap3_l3ia_read(void *opaque, hwaddr addr)
{
    struct omap3_l3_initiator_agent_s *s = (struct omap3_l3_initiator_agent_s *)opaque;
    
    switch (addr) {
        case 0x00: /* COMPONENT_L */
            return s->component;
        case 0x04: /* COMPONENT_H */
            return 0;
        case 0x18: /* CORE_L */
            return s->component;
        case 0x1c: /* CORE_H */
            return (s->component >> 16);
        case 0x20: /* AGENT_CONTROL_L */
            return s->control;
        case 0x24: /* AGENT_CONTROL_H */
            return 0;
        case 0x28: /* AGENT_STATUS_L */
            return s->status;
        case 0x2c: /* AGENT_STATUS_H */
            return 0;
        case 0x58: /* ERROR_LOG_L */
            return 0;
        case 0x5c: /* ERROR_LOG_H */
            return 0;
        case 0x60: /* ERROR_LOG_ADDR_L */
            return 0;
        case 0x64: /* ERROR_LOG_ADDR_H */
            return 0;
        default:
            break;
    }
    
    OMAP_BAD_REG(s->base + addr);
    return 0;
}

static void omap3_l3ia_write(void *opaque, hwaddr addr,
                             uint32_t value)
{
    struct omap3_l3_initiator_agent_s *s = (struct omap3_l3_initiator_agent_s *)opaque;
    
    switch (addr) {
        case 0x00: /* COMPONENT_L */
        case 0x04: /* COMPONENT_H */
        case 0x18: /* CORE_L */
        case 0x1c: /* CORE_H */
        case 0x60: /* ERROR_LOG_ADDR_L */
        case 0x64: /* ERROR_LOG_ADDR_H */
            OMAP_RO_REG(s->base + addr);
            break;
        case 0x24: /* AGENT_CONTROL_H */
        case 0x2c: /* AGENT_STATUS_H */
        case 0x5c: /* ERROR_LOG_H */
            /* RW register but all bits are reserved/read-only */
            break;
        case 0x20: /* AGENT_CONTROL_L */
            s->control = value & 0x3e070711;
            /* TODO: some bits are reserved for some IA instances */
            break;
        case 0x28: /* AGENT_STATUS_L */
            s->status &= ~(value & 0x30000000);
            break;
        case 0x58: /* ERROR_LOG_L */
            /* error logging is not implemented, so ignore */
            break;
        default:
            OMAP_BAD_REG(s->base + addr);
            break;
    }
}

static void *omap3_l3ia_init(hwaddr base)
{
    struct omap3_l3_initiator_agent_s *s = g_malloc0(sizeof(*s));
    s->base = base;
    s->component = ('Q' << 24) | ('E' << 16) | ('M' << 8) | ('U' << 0);
    s->control = 0x3e000000;
    s->status = 0;

    return s;
}

static CPUReadMemoryFunc *omap3_l3ia_readfn[] = {
    omap_badwidth_read32,
    omap_badwidth_read32,
    omap3_l3ia_read,
};

static CPUWriteMemoryFunc *omap3_l3ia_writefn[] = {
    omap_badwidth_write32,
    omap_badwidth_write32,
    omap3_l3ia_write,
};

static uint32_t omap3_l3ta_read(void *opaque, hwaddr addr)
{
    struct omap_target_agent_s *s = (struct omap_target_agent_s *)opaque;
    
    switch (addr) {
        case 0x00: /* COMPONENT_L */
            return s->component;
        case 0x04: /* COMPONENT_H */
            return 0;
        case 0x18: /* CORE_L */
            return s->component;
        case 0x1c: /* CORE_H */
            return (s->component >> 16);
        case 0x20: /* AGENT_CONTROL_L */
            return s->control;
        case 0x24: /* AGENT_CONTROL_H */
            return s->control_h;
        case 0x28: /* AGENT_STATUS_L */
            return s->status;
        case 0x2c: /* AGENT_STATUS_H */
            return 0;
        case 0x58: /* ERROR_LOG_L */
            return 0;
        case 0x5c: /* ERROR_LOG_H */
            return 0;
        case 0x60: /* ERROR_LOG_ADDR_L */
            return 0;
        case 0x64: /* ERROR_LOG_ADDR_H */
            return 0;
        default:
            break;
    }
    
    OMAP_BAD_REG(s->base + addr);
    return 0;
}

static void omap3_l3ta_write(void *opaque, hwaddr addr,
                             uint32_t value)
{
    struct omap_target_agent_s *s = (struct omap_target_agent_s *)opaque;
    
    switch (addr) {
        case 0x00: /* COMPONENT_L */
        case 0x04: /* COMPONENT_H */
        case 0x18: /* CORE_L */
        case 0x1c: /* CORE_H */
        case 0x60: /* ERROR_LOG_ADDR_L */
        case 0x64: /* ERROR_LOG_ADDR_H */
            OMAP_RO_REG(s->base + addr);
            break;
        case 0x24: /* AGENT_CONTROL_H */
        case 0x5c: /* ERROR_LOG_H */
            /* RW register but all bits are reserved/read-only */
            break;
        case 0x20: /* AGENT_CONTROL_L */
            s->control = value & 0x03000711;
            break;
        case 0x28: /* AGENT_STATUS_L */
            if ((s->base & 0xff00) == 0x6800       /* L4_CORE */
                || (s->base & 0xff00) == 0x6c00    /* L4_PER */
                || (s->base & 0xff00) == 0x7000) { /* L4_EMU */
                s->status &= ~(value & (1 << 24));
            } else {
                OMAP_RO_REG(s->base + addr);
            }
            break;
        case 0x2c: /* AGENT_STATUS_H */
            if ((s->base & 0xff00) != 0x6800       /* L4_CORE */
                && (s->base & 0xff00) != 0x6c00    /* L4_PER */
                && (s->base & 0xff00) != 0x7000) { /* L4_EMU */
                OMAP_RO_REG(s->base + addr);
            }
            /* upper 32 bits are marked as reserved, don't save the value */
            break;
        case 0x58: /* ERROR_LOG_L */
            /* error logging is not implemented, so ignore */
            break;
        default:
            OMAP_BAD_REG(s->base + addr);
            break;
    }
}

static void *omap3_l3ta_init(hwaddr base)
{
    struct omap_target_agent_s *s = g_malloc0(sizeof(*s));
    s->base = base;
    s->component = ('Q' << 24) | ('E' << 16) | ('M' << 8) | ('U' << 0);
    s->control = 0x03000000;
    s->status = 0;

    return s;
}

static CPUReadMemoryFunc *omap3_l3ta_readfn[] = {
    omap_badwidth_read32,
    omap_badwidth_read32,
    omap3_l3ta_read,
};

static CPUWriteMemoryFunc *omap3_l3ta_writefn[] = {
    omap_badwidth_write32,
    omap_badwidth_write32,
    omap3_l3ta_write,
};

struct omap3_l3pm_s {
    hwaddr base;
    
    uint32_t error_log;
    uint8_t  control;
    uint16_t req_info_permission[8];
    uint16_t read_permission[8];
    uint16_t write_permission[8];
    uint32_t addr_match[7];
};

static uint32_t omap3_l3pm_read8(void *opaque, hwaddr addr)
{
    struct omap3_l3pm_s *s = (struct omap3_l3pm_s *)opaque;
    int i;
    
    switch (addr) {
        case 0x00 ... 0x1f:
        case 0x40 ... 0x47:
            OMAP_BAD_REG(s->base + addr);
            return 0;
        /* ERROR_LOG */
        case 0x20: return s->error_log & 0xff;
        case 0x21: return (s->error_log >> 8) & 0xff;
        case 0x22: return (s->error_log >> 16) & 0xff;
        case 0x23: return (s->error_log >> 24) & 0xff;
        case 0x24 ... 0x27: return 0;
        /* CONTROL */
        case 0x28 ... 0x2a: return 0;
        case 0x2b: return s->control;
        case 0x2c ... 0x2f: return 0;
        /* ERROR_CLEAR_SINGLE */
        case 0x30: return 0; /* TODO: clear single error from log */
        case 0x31 ... 0x37: return 0;
        /* ERROR_CLEAR_MULTI */
        case 0x38: return 0; /* TODO: clear multiple errors from log */
        case 0x39 ... 0x3f: return 0;
        default:
            break;
    }
    
    i = (addr - 0x48) / 0x20;
    addr -= i * 0x20;
    if (i < 7 || (i < 8 && addr < 0x60)) 
        switch (addr) {
            /* REQ_INFO_PERMISSION_i */
            case 0x48: return s->req_info_permission[i] & 0xff;
            case 0x49: return (s->req_info_permission[i] >> 8) & 0xff;
            case 0x4a ... 0x4f: return 0;
            /* READ_PERMISSION_i */
            case 0x50: return s->read_permission[i] & 0xff;
            case 0x51: return (s->read_permission[i] >> 8) & 0xff;
            case 0x52 ... 0x57: return 0;
            /* WRITE_PERMISSION_i */
            case 0x58: return s->write_permission[i] & 0xff;
            case 0x59: return (s->write_permission[i] >> 8) & 0xff;
            case 0x5a ... 0x5f: return 0;
            /* ADDR_MATCH_i */
            case 0x60: return s->addr_match[i] & 0xff;
            case 0x61: return (s->addr_match[i] >> 8) & 0xff;
            case 0x62: return (s->addr_match[i] >> 16) & 0xff;
            case 0x63 ... 0x67: return 0;
            default:
                break;
        }

    OMAP_BAD_REG(s->base + addr);
    return 0;
}

static uint32_t omap3_l3pm_read16(void *opaque, hwaddr addr)
{
    return omap3_l3pm_read8(opaque, addr)
        | (omap3_l3pm_read8(opaque, addr + 1) << 8);
}

static uint32_t omap3_l3pm_read32(void *opaque, hwaddr addr)
{
    return omap3_l3pm_read16(opaque, addr)
        | (omap3_l3pm_read16(opaque, addr + 2) << 16);
}

static void omap3_l3pm_write8(void *opaque, hwaddr addr,
                              uint32_t value)
{
    struct omap3_l3pm_s *s = (struct omap3_l3pm_s *)opaque;
    int i;
    
    switch (addr) {
        case 0x00 ... 0x1f:
        case 0x40 ... 0x47:
            OMAP_BAD_REGV(s->base + addr, value);
            return;
        /* ERROR_LOG */
        case 0x23:
            s->error_log &= ~((value & 0xcf) << 24);
        case 0x20 ... 0x22:
        case 0x24 ... 0x27:
            return;
        /* CONTROL */
        case 0x2b:
            s->control = value & 3;
        case 0x28 ... 0x2a:
        case 0x2c ... 0x2f:
            return;
        /* ERROR_CLEAR_SINGLE / ERROR_CLEAR_MULTI */
        case 0x30 ... 0x3f:
            OMAP_RO_REGV(s->base + addr, value);
            return;
        default:
            break;
    }
    
    i = (addr - 0x48) / 0x20;
    addr -= i * 0x20;
    if (i < 7 || (i < 8 && addr < 0x60)) 
        switch (addr) {
            /* REQ_INFO_PERMISSION_i */
            case 0x48:
                s->req_info_permission[i] =
                    (s->req_info_permission[i] & ~0xff) | (value & 0xff);
                return;
            case 0x49:
                s->req_info_permission[i] =
                    (s->req_info_permission[i] & ~0xff00) | ((value & 0xff) << 8);
                return;
            case 0x4a ... 0x4f:
                return;
            /* READ_PERMISSION_i */
            case 0x50:
                s->read_permission[i] =
                    (s->read_permission[i] & ~0xff) | (value & 0x3e);
                return;
            case 0x51:
                s->read_permission[i] =
                    (s->read_permission[i] & ~0xff00) | ((value & 0x5f) << 8);
                return;
            case 0x52 ... 0x57:
                return;
            /* WRITE_PERMISSION_i */
            case 0x58:
                s->write_permission[i] =
                    (s->write_permission[i] & ~0xff) | (value & 0x3e);
                return;
            case 0x59:
                s->write_permission[i] =
                    (s->write_permission[i] & ~0xff00) | ((value & 0x5f) << 8);
                return;
            case 0x5a ... 0x5f:
                return;
            /* ADDR_MATCH_i */
            case 0x60:
                s->addr_match[i] = (s->addr_match[i] & ~0xff) | (value & 0xff);
                return;
            case 0x61:
                s->addr_match[i] =
                    (s->addr_match[i] & ~0xfe00) | ((value & 0xfe) << 8);
                return;
            case 0x62:
                s->addr_match[i] =
                    (s->addr_match[i] & ~0x0f0000) | ((value & 0x0f) << 16);
                return;
            case 0x63 ... 0x67:
                return;
            default:
                break;
        }
    
    OMAP_BAD_REGV(s->base + addr, value);
}

static void omap3_l3pm_write16(void *opaque, hwaddr addr,
                               uint32_t value)
{
    omap3_l3pm_write8(opaque, addr + 0, value & 0xff);
    omap3_l3pm_write8(opaque, addr + 1, (value >> 8) & 0xff);
}

static void omap3_l3pm_write32(void *opaque, hwaddr addr,
                               uint32_t value)
{
    omap3_l3pm_write16(opaque, addr + 0, value & 0xffff);
    omap3_l3pm_write16(opaque, addr + 2, (value >> 16) & 0xffff);
}

static void *omap3_l3pm_init(hwaddr base)
{
    struct omap3_l3pm_s *s = g_malloc0(sizeof(*s));
    int i;

    s->base = base;
    s->error_log = 0;
    s->control = 0x03;
    switch (s->base) {
        case 0x68010000: /* PM_RT */
            s->req_info_permission[0] = 0xffff;
            s->req_info_permission[1] = 0;
            for (i = 0; i < 2; i++)
                s->read_permission[i] = s->write_permission[i] = 0x1406;
            s->addr_match[0] = 0x10230;
            break;
        case 0x68012400: /* PM_GPMC */
            s->req_info_permission[0] = 0;
            for (i = 3; i < 8; i++)
                s->req_info_permission[i] = 0xffff;
            for (i = 0; i < 8; i++)
                s->read_permission[i] = s->write_permission[i] = 0x563e;
            s->addr_match[0] = 0x00098;
            break;
        case 0x68012800: /* PM_OCM_RAM */
            s->req_info_permission[0] = 0;
            for (i = 1; i < 8; i++)
                s->req_info_permission[i] = 0xffff;
            for (i = 0; i < 8; i++)
                s->read_permission[i] = s->write_permission[i] = 0x5f3e;
            s->addr_match[1] = 0x0f810;
            break;
        case 0x68012C00: /* PM_OCM_ROM */
            s->req_info_permission[1] = 0xffff;
            for (i = 0; i < 2; i++) {
                s->read_permission[i] = 0x1002;
                s->write_permission[i] = 0;
            }
            s->addr_match[0] = 0x14028;
            break;
        case 0x68013000: /* PM_MAD2D */
            s->req_info_permission[0] = 0;
            for (i = 1; i < 8; i++)
                s->req_info_permission[i] = 0xffff;
            for (i = 0; i < 8; i++)
                s->read_permission[i] = s->write_permission[i] = 0x5f1e;
            break;
        case 0x68014000: /* PM_IVA2.2 */
            s->req_info_permission[0] = 0;
            for (i = 1; i < 4; i++)
                s->req_info_permission[i] = 0xffff;
            for (i = 0; i < 4; i++)
                s->read_permission[i] = s->write_permission[i] = 0x140e;
            break;
        default:
            hw_error("%s: unknown PM region " OMAP_FMT_plx, __FUNCTION__,
                     s->base);
            break;
    }

    return s;
}

static CPUReadMemoryFunc *omap3_l3pm_readfn[] = {
    omap3_l3pm_read8,
    omap3_l3pm_read16,
    omap3_l3pm_read32,
};

static CPUWriteMemoryFunc *omap3_l3pm_writefn[] = {
    omap3_l3pm_write8,
    omap3_l3pm_write16,
    omap3_l3pm_write32,
};

struct omap3_l3_s {
    MemoryRegion iomem;
    hwaddr base;
    int region_count;
    void *region[0];
};

static int omap3_l3_findregion(struct omap3_l3_s *l3, hwaddr addr)
{
    hwaddr limit = 0;
    int i;
    for (i = 0; i < l3->region_count; i++) {
        limit += omap3_l3_region[i].size;
        if (addr < limit) {
            return i;
        }
    }
    return -1;
}

static uint64_t omap3_l3_read(void *opaque, hwaddr addr,
                              unsigned size)
{
    struct omap3_l3_s *s = (struct omap3_l3_s *)opaque;
    int i = omap3_l3_findregion(s, addr);
    if (i < 0) {
        hw_error("%s: unknown region addr " OMAP_FMT_plx, __FUNCTION__, addr);
    }
    /* Convert (1,2,4) to (0,1,2) */
    // FIXME better to have the ia/ta/pm provide newstyle read/write fns
    size >>= 1;
    switch (omap3_l3_region[i].type) {
        case L3TYPE_IA:
            return omap3_l3ia_readfn[size](s->region[i], addr);
        case L3TYPE_TA:
            return omap3_l3ta_readfn[size](s->region[i], addr);
        case L3TYPE_PM:
            return omap3_l3pm_readfn[size](s->region[i], addr);
        case L3TYPE_UNDEF:
            TRACE("unsupported register at " OMAP_FMT_plx, addr);
            return 0;
        default:
            break;
    }
    hw_error("%s: unknown region type %d, addr " OMAP_FMT_plx,
             __FUNCTION__, omap3_l3_region[i].type, addr);
}

static void omap3_l3_write(void *opaque, hwaddr addr,
                           uint64_t value, unsigned size)
{
    struct omap3_l3_s *s = (struct omap3_l3_s *)opaque;
    int i = omap3_l3_findregion(s, addr);
    if (i < 0) {
        hw_error("%s: unknown region addr" OMAP_FMT_plx, __FUNCTION__, addr);
    }
    /* Convert (1,2,4) to (0,1,2) */
    // FIXME better to have the ia/ta/pm provide newstyle read/write fns
    size >>= 1;
    switch (omap3_l3_region[i].type) {
        case L3TYPE_IA:
            omap3_l3ia_writefn[size](s->region[i], addr, value);
            break;
        case L3TYPE_TA:
            omap3_l3ta_writefn[size](s->region[i], addr, value);
            break;
        case L3TYPE_PM:
            omap3_l3pm_writefn[size](s->region[i], addr, value);
            break;
        case L3TYPE_UNDEF:
            TRACE("unsupported register at " OMAP_FMT_plx, addr);
            break;
        default:
            hw_error("%s: unknown region type %d, addr " OMAP_FMT_plx,
                     __FUNCTION__, omap3_l3_region[i].type, addr);
            break;
    }
}

static const MemoryRegionOps omap3_l3_ops = {
    .read = omap3_l3_read,
    .write = omap3_l3_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static struct omap3_l3_s *omap3_l3_init(MemoryRegion *sysmem,
                                        hwaddr base)
{
    const int n = sizeof(omap3_l3_region) / sizeof(struct omap3_l3_region_s);
    struct omap3_l3_s *bus = g_malloc0(sizeof(*bus) + n * sizeof(void *));
    bus->region_count = n;
    bus->base = base;
 
    int i;
    for (i = 0; i < n; i++) {
        switch (omap3_l3_region[i].type) {
            case L3TYPE_IA:
                bus->region[i] = omap3_l3ia_init(base);
                break;
            case L3TYPE_TA:
                bus->region[i] = omap3_l3ta_init(base);
                break;
            case L3TYPE_PM:
                bus->region[i] = omap3_l3pm_init(base);
                break;
            case L3TYPE_UNDEF:
                bus->region[i] = 0;
                break;
            default:
                hw_error("%s: unknown region type %d", __FUNCTION__,
                         omap3_l3_region[i].type);
                break;
        }
        base += omap3_l3_region[i].size;
    }

    memory_region_init_io(&bus->iomem, &omap3_l3_ops, bus, "omap3_l3",
                          0x01000000);
    memory_region_add_subregion(sysmem, base, &bus->iomem);
    return bus;
}

typedef enum {
    /* 48000000-48001FFF */
    /* 48002000-48002FFF */ L4ID_SCM = 0,
    /* 48003000-48003FFF */ L4ID_SCM_TA,
    /* 48004000-48005FFF */ L4ID_CM_A,
    /* 48006000-480067FF */ L4ID_CM_B,
    /* 48006800-48006FFF */
    /* 48007000-48007FFF */ L4ID_CM_TA,
    /* 48008000-48023FFF */
    /* 48024000-48024FFF */
    /* 48025000-48025FFF */
    /* 48026000-4803FFFF */
    /* 48040000-480407FF */ L4ID_CORE_AP,
    /* 48040800-48040FFF */ L4ID_CORE_IP,
    /* 48041000-48041FFF */ L4ID_CORE_LA,
    /* 48042000-4804FBFF */
    /* 4804FC00-4804FFFF */ L4ID_DSI,
    /* 48050000-480503FF */ L4ID_DSS,
    /* 48050400-480507FF */ L4ID_DISPC,
    /* 48050800-48050BFF */ L4ID_RFBI,
    /* 48050C00-48050FFF */ L4ID_VENC,
    /* 48051000-48051FFF */ L4ID_DSS_TA,
    /* 48052000-48055FFF */
    /* 48056000-48056FFF */ L4ID_SDMA,
    /* 48057000-48057FFF */ L4ID_SDMA_TA,
    /* 48058000-4805FFFF */
    /* 48060000-48060FFF */ L4ID_I2C3,
    /* 48061000-48061FFF */ L4ID_I2C3_TA,
    /* 48062000-48062FFF */ L4ID_USBTLL,
    /* 48063000-48063FFF */ L4ID_USBTLL_TA,
    /* 48064000-480643FF */ L4ID_USBHOST,
    /* 48064400-480647FF */ L4ID_USBHOST_OHCI,
    /* 48064800-4806BFFF */ L4ID_USBHOST_EHCI,
    /* 48065000-48065FFF */ L4ID_USBHOST_TA,
    /* 48066000-48069FFF */
    /* 4806A000-4806AFFF */ L4ID_UART1,
    /* 4806B000-4806BFFF */ L4ID_UART1_TA,
    /* 4806C000-4806CFFF */ L4ID_UART2,
    /* 4806D000-4806DFFF */ L4ID_UART2_TA,
    /* 4806E000-4806FFFF */
    /* 48070000-48070FFF */ L4ID_I2C1,
    /* 48071000-48071FFF */ L4ID_I2C1_TA,
    /* 48072000-48072FFF */ L4ID_I2C2,
    /* 48073000-48073FFF */ L4ID_I2C2_TA,
    /* 48074000-48074FFF */ L4ID_MCBSP1,
    /* 48075000-48075FFF */ L4ID_MCBSP1_TA,
    /* 48076000-48085FFF */
    /* 48086000-48086FFF */ L4ID_GPTIMER10,
    /* 48087000-48087FFF */ L4ID_GPTIMER10_TA,
    /* 48088000-48088FFF */ L4ID_GPTIMER11,
    /* 48089000-48089FFF */ L4ID_GPTIMER11_TA,
    /* 4808A000-4808AFFF */
    /* 4808B000-4808BFFF */
    /* 4808C000-48093FFF */
    /* 48094000-48094FFF */ L4ID_MAILBOX,
    /* 48095000-48095FFF */ L4ID_MAILBOX_TA,
    /* 48096000-48096FFF */ L4ID_MCBSP5,
    /* 48097000-48097FFF */ L4ID_MCBSP5_TA,
    /* 48098000-48098FFF */ L4ID_MCSPI1,
    /* 48099000-48099FFF */ L4ID_MCSPI1_TA,
    /* 4809A000-4809AFFF */ L4ID_MCSPI2,
    /* 4809B000-4809BFFF */ L4ID_MCSPI2_TA,
    /* 4809C000-4809CFFF */ L4ID_MMCSDIO1,
    /* 4809D000-4809DFFF */ L4ID_MMCSDIO1_TA,
    /* 4809E000-4809EFFF */ L4ID_MSPRO,
    /* 4809F000-4809FFFF */ L4ID_MSPRO_TA,
    /* 480A0000-480AAFFF */
    /* 480AB000-480ABFFF */ L4ID_HSUSBOTG,
    /* 480AC000-480ACFFF */ L4ID_HSUSBOTG_TA,
    /* 480AD000-480ADFFF */ L4ID_MMCSDIO3,
    /* 480AE000-480AEFFF */ L4ID_MMCSDIO3_TA,
    /* 480AF000-480AFFFF */
    /* 480B0000-480B0FFF */
    /* 480B1000-480B1FFF */
    /* 480B2000-480B2FFF */ L4ID_HDQ1WIRE,
    /* 480B3000-480B2FFF */ L4ID_HDQ1WIRE_TA,
    /* 480B4000-480B4FFF */ L4ID_MMCSDIO2,
    /* 480B5000-480B5FFF */ L4ID_MMCSDIO2_TA,
    /* 480B6000-480B6FFF */ L4ID_ICRMPU,
    /* 480B7000-480B7FFF */ L4ID_ICRMPU_TA,
    /* 480B8000-480B8FFF */ L4ID_MCSPI3,
    /* 480B9000-480B9FFF */ L4ID_MCSPI3_TA,
    /* 480BA000-480BAFFF */ L4ID_MCSPI4,
    /* 480BB000-480BBFFF */ L4ID_MCSPI4_TA,
    /* 480BC000-480BFFFF */ L4ID_CAMERAISP,
    /* 480C0000-480C0FFF */ L4ID_CAMERAISP_TA,
    /* 480C1000-480CCFFF */
    /* 480C9000-480C9FFF */ L4ID_SR1,
    /* 480CA000-480CAFFF */ L4ID_SR1_TA,
    /* 480CB000-480CBFFF */ L4ID_SR2,
    /* 480CC000-480CCFFF */ L4ID_SR2_TA,
    /* 480CD000-480CDFFF */ L4ID_ICRMODEM,
    /* 480CE000-480CEFFF */ L4ID_ICRMODEM_TA,
    /* 480CF000-482FFFFF */
    /* 48300000-48303FFF */
    /* 48304000-48304FFF */ L4ID_GPTIMER12,
    /* 48305000-48305FFF */ L4ID_GPTIMER12_TA,
    /* 48306000-48307FFF */ L4ID_PRM_A,
    /* 48308000-483087FF */ L4ID_PRM_B,
    /* 48308800-48308FFF */
    /* 48309000-48309FFF */ L4ID_PRM_TA,
    /* 4830A000-4830AFFF */ L4ID_TAP,
    /* 4830B000-4830BFFF */ L4ID_TAP_TA,
    /* 4830C000-4830FFFF */
    /* 48310000-48310FFF */ L4ID_GPIO1,
    /* 48311000-48311FFF */ L4ID_GPIO1_TA,
    /* 48312000-48313FFF */
    /* 48314000-48314FFF */ L4ID_WDTIMER2,
    /* 48315000-48315FFF */ L4ID_WDTIMER2_TA,
    /* 48316000-48317FFF */
    /* 48318000-48318FFF */ L4ID_GPTIMER1,
    /* 48319000-48319FFF */ L4ID_GPTIMER1_TA,
    /* 4831A000-4831FFFF */
    /* 48320000-48320FFF */ L4ID_32KTIMER,
    /* 48321000-48321FFF */ L4ID_32KTIMER_TA,
    /* 48322000-48327FFF */
    /* 48328000-483287FF */ L4ID_WAKEUP_AP,
    /* 48328800-48328FFF */ L4ID_WAKEUP_C_IP,
    /* 48329000-48329FFF */ L4ID_WAKEUP_LA,
    /* 4832A000-4832A7FF */ L4ID_WAKEUP_E_IP,
    /* 4832A800-4833FFFF */
    /* 48340000-48340FFF */
    /* 48341000-48FFFFFF */
    /* 49000000-490007FF */ L4ID_PER_AP,
    /* 49000800-49000FFF */ L4ID_PER_IP,
    /* 49001000-49001FFF */ L4ID_PER_LA,
    /* 49002000-4901FFFF */
    /* 49020000-49020FFF */ L4ID_UART3,
    /* 49021000-49021FFF */ L4ID_UART3_TA,
    /* 49022000-49022FFF */ L4ID_MCBSP2,
    /* 49023000-49023FFF */ L4ID_MCBSP2_TA,
    /* 49024000-49024FFF */ L4ID_MCBSP3,
    /* 49025000-49025FFF */ L4ID_MCBSP3_TA,
    /* 49026000-49026FFF */ L4ID_MCBSP4,
    /* 49027000-49027FFF */ L4ID_MCBSP4_TA,
    /* 49028000-49028FFF */ L4ID_MCBSP2S,
    /* 49029000-49029FFF */ L4ID_MCBSP2S_TA,
    /* 4902A000-4902AFFF */ L4ID_MCBSP3S,
    /* 4902B000-4902BFFF */ L4ID_MCBSP3S_TA,
    /* 4902C000-4902FFFF */
    /* 49030000-49030FFF */ L4ID_WDTIMER3,
    /* 49031000-49031FFF */ L4ID_WDTIMER3_TA,
    /* 49032000-49032FFF */ L4ID_GPTIMER2,
    /* 49033000-49033FFF */ L4ID_GPTIMER2_TA,
    /* 49034000-49034FFF */ L4ID_GPTIMER3,
    /* 49035000-49035FFF */ L4ID_GPTIMER3_TA,
    /* 49036000-49036FFF */ L4ID_GPTIMER4,
    /* 49037000-49037FFF */ L4ID_GPTIMER4_TA,
    /* 49038000-49038FFF */ L4ID_GPTIMER5,
    /* 49039000-49039FFF */ L4ID_GPTIMER5_TA,
    /* 4903A000-4903AFFF */ L4ID_GPTIMER6,
    /* 4903B000-4903BFFF */ L4ID_GPTIMER6_TA,
    /* 4903C000-4903CFFF */ L4ID_GPTIMER7,
    /* 4903D000-4903DFFF */ L4ID_GPTIMER7_TA,
    /* 4903E000-4903EFFF */ L4ID_GPTIMER8,
    /* 4903F000-4903FFFF */ L4ID_GPTIMER8_TA,
    /* 49040000-49040FFF */ L4ID_GPTIMER9,
    /* 49041000-49041FFF */ L4ID_GPTIMER9_TA,
    /* 49042000-49042FFF */ L4ID_UART4,
    /* 49043000-4903FFFF */ L4ID_UART4_TA,
    /* 49044000-4904FFFF */
    /* 49050000-49050FFF */ L4ID_GPIO2,
    /* 49051000-49051FFF */ L4ID_GPIO2_TA,
    /* 49052000-49052FFF */ L4ID_GPIO3,
    /* 49053000-49053FFF */ L4ID_GPIO3_TA,
    /* 49054000-49054FFF */ L4ID_GPIO4,
    /* 49055000-49055FFF */ L4ID_GPIO4_TA,
    /* 49056000-49056FFF */ L4ID_GPIO5,
    /* 49057000-49057FFF */ L4ID_GPIO5_TA,
    /* 49058000-49058FFF */ L4ID_GPIO6,
    /* 49059000-49059FFF */ L4ID_GPIO6_TA,
    /* 4905A000-490FFFFF */
    /* 54000000-54003FFF */
    /* 54004000-54005FFF */
    /* 54006000-540067FF */ L4ID_EMU_AP,
    /* 54006800-54006FFF */ L4ID_EMU_IP_C,
    /* 54007000-54007FFF */ L4ID_EMU_LA,
    /* 54008000-540087FF */ L4ID_EMU_IP_DAP,
    /* 54008800-5400FFFF */
    /* 54010000-54017FFF */ L4ID_MPUEMU,
    /* 54018000-54018FFF */ L4ID_MPUEMU_TA,
    /* 54019000-54019FFF */ L4ID_TPIU,
    /* 5401A000-5401AFFF */ L4ID_TPIU_TA,
    /* 5401B000-5401BFFF */ L4ID_ETB,
    /* 5401C000-5401CFFF */ L4ID_ETB_TA,
    /* 5401D000-5401DFFF */ L4ID_DAPCTL,
    /* 5401E000-5401EFFF */ L4ID_DAPCTL_TA,
    /* 5401F000-5401FFFF */ L4ID_SDTI_TA,
    /* 54020000-544FFFFF */
    /* 54500000-5450FFFF */ L4ID_SDTI_CFG,
    /* 54510000-545FFFFF */
    /* 54600000-546FFFFF */ L4ID_SDTI,
    /* 54700000-54705FFF */
    /* 54706000-54707FFF */ L4ID_EMU_PRM_A,
    /* 54708000-547087FF */ L4ID_EMU_PRM_B,
    /* 54708800-54708FFF */
    /* 54709000-54709FFF */ L4ID_EMU_PRM_TA,
    /* 5470A000-5470FFFF */
    /* 54710000-54710FFF */ L4ID_EMU_GPIO1,
    /* 54711000-54711FFF */ L4ID_EMU_GPIO1_TA,
    /* 54712000-54713FFF */
    /* 54714000-54714FFF */ L4ID_EMU_WDTM2,
    /* 54715000-54715FFF */ L4ID_EMU_WDTM2_TA,
    /* 54716000-54717FFF */
    /* 54718000-54718FFF */ L4ID_EMU_GPTM1,
    /* 54719000-54719FFF */ L4ID_EMU_GPTM1_TA,
    /* 5471A000-5471FFFF */
    /* 54720000-54720FFF */ L4ID_EMU_32KTM,
    /* 54721000-54721FFF */ L4ID_EMU_32KTM_TA,
    /* 54722000-54727FFF */
    /* 54728000-547287FF */ L4ID_EMU_WKUP_AP,
    /* 54728800-54728FFF */ L4ID_EMU_WKUP_IPC,
    /* 54729000-54729FFF */ L4ID_EMU_WKUP_LA,
    /* 5472A000-5472A7FF */ L4ID_EMU_WKUP_IPE,
    /* 5472A800-547FFFFF */
    L4ID_COUNT
} omap3_l4_region_id_t;

/* we reuse the "access" member for defining region type -- the original
   omap_l4_region_s "access" member is not used anywhere else anyway! */
static struct omap_l4_region_s omap3_l4_region[L4ID_COUNT] = {
    /* L4-Core */
    [L4ID_SCM         ] = {0x00002000, 0x1000, L4TYPE_GENERIC},
    [L4ID_SCM_TA      ] = {0x00003000, 0x1000, L4TYPE_TA},
    [L4ID_CM_A        ] = {0x00004000, 0x2000, L4TYPE_GENERIC},
    [L4ID_CM_B        ] = {0x00006000, 0x0800, L4TYPE_GENERIC},
    [L4ID_CM_TA       ] = {0x00007000, 0x1000, L4TYPE_TA},
    [L4ID_CORE_AP     ] = {0x00040000, 0x0800, L4TYPE_AP},
    [L4ID_CORE_IP     ] = {0x00040800, 0x0800, L4TYPE_IA},
    [L4ID_CORE_LA     ] = {0x00041000, 0x1000, L4TYPE_LA},
    [L4ID_DSI         ] = {0x0004fc00, 0x0400, L4TYPE_GENERIC},
    [L4ID_DSS         ] = {0x00050000, 0x0400, L4TYPE_GENERIC},
    [L4ID_DISPC       ] = {0x00050400, 0x0400, L4TYPE_GENERIC},
    [L4ID_RFBI        ] = {0x00050800, 0x0400, L4TYPE_GENERIC},
    [L4ID_VENC        ] = {0x00050c00, 0x0400, L4TYPE_GENERIC},
    [L4ID_DSS_TA      ] = {0x00051000, 0x1000, L4TYPE_TA},
    [L4ID_SDMA        ] = {0x00056000, 0x1000, L4TYPE_GENERIC},
    [L4ID_SDMA_TA     ] = {0x00057000, 0x1000, L4TYPE_TA},
    [L4ID_I2C3        ] = {0x00060000, 0x1000, L4TYPE_GENERIC},
    [L4ID_I2C3_TA     ] = {0x00061000, 0x1000, L4TYPE_TA},
    [L4ID_USBTLL      ] = {0x00062000, 0x1000, L4TYPE_GENERIC},
    [L4ID_USBTLL_TA   ] = {0x00063000, 0x1000, L4TYPE_TA},
    [L4ID_USBHOST     ] = {0x00064000, 0x0400, L4TYPE_GENERIC},
    [L4ID_USBHOST_OHCI] = {0x00064400, 0x0400, L4TYPE_GENERIC},
    [L4ID_USBHOST_EHCI] = {0x00064800, 0x0400, L4TYPE_GENERIC},
    [L4ID_USBHOST_TA  ] = {0x00065000, 0x1000, L4TYPE_TA},
    [L4ID_UART1       ] = {0x0006a000, 0x1000, L4TYPE_GENERIC},
    [L4ID_UART1_TA    ] = {0x0006b000, 0x1000, L4TYPE_TA},
    [L4ID_UART2       ] = {0x0006c000, 0x1000, L4TYPE_GENERIC},
    [L4ID_UART2_TA    ] = {0x0006d000, 0x1000, L4TYPE_TA},
    [L4ID_I2C1        ] = {0x00070000, 0x1000, L4TYPE_GENERIC},
    [L4ID_I2C1_TA     ] = {0x00071000, 0x1000, L4TYPE_TA},
    [L4ID_I2C2        ] = {0x00072000, 0x1000, L4TYPE_GENERIC},
    [L4ID_I2C2_TA     ] = {0x00073000, 0x1000, L4TYPE_TA},
    [L4ID_MCBSP1      ] = {0x00074000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCBSP1_TA   ] = {0x00075000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER10   ] = {0x00086000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER10_TA] = {0x00087000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER11   ] = {0x00088000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER11_TA] = {0x00089000, 0x1000, L4TYPE_TA},
    [L4ID_MAILBOX     ] = {0x00094000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MAILBOX_TA  ] = {0x00095000, 0x1000, L4TYPE_TA},
    [L4ID_MCBSP5      ] = {0x00096000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCBSP5_TA   ] = {0x00097000, 0x1000, L4TYPE_TA},
    [L4ID_MCSPI1      ] = {0x00098000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCSPI1_TA   ] = {0x00099000, 0x1000, L4TYPE_TA},
    [L4ID_MCSPI2      ] = {0x0009a000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCSPI2_TA   ] = {0x0009b000, 0x1000, L4TYPE_TA},
    [L4ID_MMCSDIO1    ] = {0x0009c000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MMCSDIO1_TA ] = {0x0009d000, 0x1000, L4TYPE_TA},
    [L4ID_MSPRO       ] = {0x0009e000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MSPRO_TA    ] = {0x0009f000, 0x1000, L4TYPE_TA},
    [L4ID_HSUSBOTG    ] = {0x000ab000, 0x1000, L4TYPE_GENERIC},
    [L4ID_HSUSBOTG_TA ] = {0x000ac000, 0x1000, L4TYPE_TA},
    [L4ID_MMCSDIO3    ] = {0x000ad000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MMCSDIO3_TA ] = {0x000ae000, 0x1000, L4TYPE_TA},
    [L4ID_HDQ1WIRE    ] = {0x000b2000, 0x1000, L4TYPE_GENERIC},
    [L4ID_HDQ1WIRE_TA ] = {0x000b3000, 0x1000, L4TYPE_TA},
    [L4ID_MMCSDIO2    ] = {0x000b4000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MMCSDIO2_TA ] = {0x000b5000, 0x1000, L4TYPE_TA},
    [L4ID_ICRMPU      ] = {0x000b6000, 0x1000, L4TYPE_GENERIC},
    [L4ID_ICRMPU_TA   ] = {0x000b7000, 0x1000, L4TYPE_TA},
    [L4ID_MCSPI3      ] = {0x000b8000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCSPI3_TA   ] = {0x000b9000, 0x1000, L4TYPE_TA},
    [L4ID_MCSPI4      ] = {0x000ba000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCSPI4_TA   ] = {0x000bb000, 0x1000, L4TYPE_TA},
    [L4ID_CAMERAISP   ] = {0x000bc000, 0x4000, L4TYPE_GENERIC},
    [L4ID_CAMERAISP_TA] = {0x000c0000, 0x1000, L4TYPE_TA},
    [L4ID_SR1         ] = {0x000c9000, 0x1000, L4TYPE_GENERIC},
    [L4ID_SR1_TA      ] = {0x000ca000, 0x1000, L4TYPE_TA},
    [L4ID_SR2         ] = {0x000cb000, 0x1000, L4TYPE_GENERIC},
    [L4ID_SR2_TA      ] = {0x000cc000, 0x1000, L4TYPE_TA},
    [L4ID_ICRMODEM    ] = {0x000cd000, 0x1000, L4TYPE_GENERIC},
    [L4ID_ICRMODEM_TA ] = {0x000ce000, 0x1000, L4TYPE_TA},
    /* L4-Wakeup interconnect region A */
    [L4ID_GPTIMER12   ] = {0x00304000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER12_TA] = {0x00305000, 0x1000, L4TYPE_TA},
    [L4ID_PRM_A       ] = {0x00306000, 0x2000, L4TYPE_GENERIC},
    [L4ID_PRM_B       ] = {0x00308000, 0x0800, L4TYPE_GENERIC},
    [L4ID_PRM_TA      ] = {0x00309000, 0x1000, L4TYPE_TA},
    /* L4-Core */
    [L4ID_TAP         ] = {0x0030a000, 0x1000, L4TYPE_GENERIC},
    [L4ID_TAP_TA      ] = {0x0030b000, 0x1000, L4TYPE_TA},
    /* L4-Wakeup interconnect region B */
    [L4ID_GPIO1       ] = {0x00310000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPIO1_TA    ] = {0x00311000, 0x1000, L4TYPE_TA},
    [L4ID_WDTIMER2    ] = {0x00314000, 0x1000, L4TYPE_GENERIC},
    [L4ID_WDTIMER2_TA ] = {0x00315000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER1    ] = {0x00318000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER1_TA ] = {0x00319000, 0x1000, L4TYPE_TA},
    [L4ID_32KTIMER    ] = {0x00320000, 0x1000, L4TYPE_GENERIC},
    [L4ID_32KTIMER_TA ] = {0x00321000, 0x1000, L4TYPE_TA},
    [L4ID_WAKEUP_AP   ] = {0x00328000, 0x0800, L4TYPE_AP},
    [L4ID_WAKEUP_C_IP ] = {0x00328800, 0x0800, L4TYPE_IA},
    [L4ID_WAKEUP_LA   ] = {0x00329000, 0x1000, L4TYPE_LA},
    [L4ID_WAKEUP_E_IP ] = {0x0032a000, 0x0800, L4TYPE_IA},
    /* L4-Per */
    [L4ID_PER_AP      ] = {0x01000000, 0x0800, L4TYPE_AP},
    [L4ID_PER_IP      ] = {0x01000800, 0x0800, L4TYPE_IA},
    [L4ID_PER_LA      ] = {0x01001000, 0x1000, L4TYPE_LA},
    [L4ID_UART3       ] = {0x01020000, 0x1000, L4TYPE_GENERIC},
    [L4ID_UART3_TA    ] = {0x01021000, 0x1000, L4TYPE_TA},
    [L4ID_MCBSP2      ] = {0x01022000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCBSP2_TA   ] = {0x01023000, 0x1000, L4TYPE_TA},
    [L4ID_MCBSP3      ] = {0x01024000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCBSP3_TA   ] = {0x01025000, 0x1000, L4TYPE_TA},
    [L4ID_MCBSP4      ] = {0x01026000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCBSP4_TA   ] = {0x01027000, 0x1000, L4TYPE_TA},
    [L4ID_MCBSP2S     ] = {0x01028000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCBSP2S_TA  ] = {0x01029000, 0x1000, L4TYPE_TA},
    [L4ID_MCBSP3S     ] = {0x0102a000, 0x1000, L4TYPE_GENERIC},
    [L4ID_MCBSP3S_TA  ] = {0x0102b000, 0x1000, L4TYPE_TA},
    [L4ID_WDTIMER3    ] = {0x01030000, 0x1000, L4TYPE_GENERIC},
    [L4ID_WDTIMER3_TA ] = {0x01031000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER2    ] = {0x01032000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER2_TA ] = {0x01033000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER3    ] = {0x01034000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER3_TA ] = {0x01035000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER4    ] = {0x01036000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER4_TA ] = {0x01037000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER5    ] = {0x01038000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER5_TA ] = {0x01039000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER6    ] = {0x0103a000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER6_TA ] = {0x0103b000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER7    ] = {0x0103c000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER7_TA ] = {0x0103d000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER8    ] = {0x0103e000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER8_TA ] = {0x0103f000, 0x1000, L4TYPE_TA},
    [L4ID_GPTIMER9    ] = {0x01040000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPTIMER9_TA ] = {0x01041000, 0x1000, L4TYPE_TA},
    [L4ID_UART4       ] = {0x01042000, 0x1000, L4TYPE_GENERIC},
    [L4ID_UART4_TA    ] = {0x01043000, 0x1000, L4TYPE_TA},
    [L4ID_GPIO2       ] = {0x01050000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPIO2_TA    ] = {0x01051000, 0x1000, L4TYPE_TA},
    [L4ID_GPIO3       ] = {0x01052000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPIO3_TA    ] = {0x01053000, 0x1000, L4TYPE_TA},
    [L4ID_GPIO4       ] = {0x01054000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPIO4_TA    ] = {0x01055000, 0x1000, L4TYPE_TA},
    [L4ID_GPIO5       ] = {0x01056000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPIO5_TA    ] = {0x01057000, 0x1000, L4TYPE_TA},
    [L4ID_GPIO6       ] = {0x01058000, 0x1000, L4TYPE_GENERIC},
    [L4ID_GPIO6_TA    ] = {0x01059000, 0x1000, L4TYPE_TA},
    /* L4-Emu */
    [L4ID_EMU_AP      ] = {0x0c006000, 0x0800, L4TYPE_AP},
    [L4ID_EMU_IP_C    ] = {0x0c006800, 0x0800, L4TYPE_IA},
    [L4ID_EMU_LA      ] = {0x0c007000, 0x1000, L4TYPE_LA},
    [L4ID_EMU_IP_DAP  ] = {0x0c008000, 0x0800, L4TYPE_IA},
    [L4ID_MPUEMU      ] = {0x0c010000, 0x8000, L4TYPE_GENERIC},
    [L4ID_MPUEMU_TA   ] = {0x0c018000, 0x1000, L4TYPE_TA},
    [L4ID_TPIU        ] = {0x0c019000, 0x1000, L4TYPE_GENERIC},
    [L4ID_TPIU_TA     ] = {0x0c01a000, 0x1000, L4TYPE_TA},
    [L4ID_ETB         ] = {0x0c01b000, 0x1000, L4TYPE_GENERIC},
    [L4ID_ETB_TA      ] = {0x0c01c000, 0x1000, L4TYPE_TA},
    [L4ID_DAPCTL      ] = {0x0c01d000, 0x1000, L4TYPE_GENERIC},
    [L4ID_DAPCTL_TA   ] = {0x0c01e000, 0x1000, L4TYPE_TA},
    [L4ID_EMU_PRM_A   ] = {0x0c706000, 0x2000, L4TYPE_GENERIC},
    [L4ID_EMU_PRM_B   ] = {0x0c706800, 0x0800, L4TYPE_GENERIC},
    [L4ID_EMU_PRM_TA  ] = {0x0c709000, 0x1000, L4TYPE_TA},
    [L4ID_EMU_GPIO1   ] = {0x0c710000, 0x1000, L4TYPE_GENERIC},
    [L4ID_EMU_GPIO1_TA] = {0x0c711000, 0x1000, L4TYPE_TA},
    [L4ID_EMU_WDTM2   ] = {0x0c714000, 0x1000, L4TYPE_GENERIC},
    [L4ID_EMU_WDTM2_TA] = {0x0c715000, 0x1000, L4TYPE_TA},
    [L4ID_EMU_GPTM1   ] = {0x0c718000, 0x1000, L4TYPE_GENERIC},
    [L4ID_EMU_GPTM1_TA] = {0x0c719000, 0x1000, L4TYPE_TA},
    [L4ID_EMU_32KTM   ] = {0x0c720000, 0x1000, L4TYPE_GENERIC},
    [L4ID_EMU_32KTM_TA] = {0x0c721000, 0x1000, L4TYPE_TA},
    [L4ID_EMU_WKUP_AP ] = {0x0c728000, 0x0800, L4TYPE_AP},
    [L4ID_EMU_WKUP_IPC] = {0x0c728800, 0x0800, L4TYPE_IA},
    [L4ID_EMU_WKUP_LA ] = {0x0c729000, 0x1000, L4TYPE_LA},
    [L4ID_EMU_WKUP_IPE] = {0x0c72a000, 0x0800, L4TYPE_IA},
};

typedef enum {
    L4A_SCM = 0,
    L4A_CM,
    L4A_PRM,
    L4A_GPTIMER1,
    L4A_GPTIMER2,
    L4A_GPTIMER3,
    L4A_GPTIMER4,
    L4A_GPTIMER5,
    L4A_GPTIMER6,
    L4A_GPTIMER7,
    L4A_GPTIMER8,
    L4A_GPTIMER9,
    L4A_GPTIMER10,
    L4A_GPTIMER11,
    L4A_GPTIMER12,
    L4A_WDTIMER2,
    L4A_32KTIMER,
    L4A_UART1,
    L4A_UART2,
    L4A_UART3,
    L4A_UART4,
    L4A_DSS,
    L4A_GPIO1,
    L4A_GPIO2,
    L4A_GPIO3,
    L4A_GPIO4,
    L4A_GPIO5,
    L4A_GPIO6,
    L4A_MMC1,
    L4A_MMC2,
    L4A_MMC3,
    L4A_I2C1,
    L4A_I2C2,
    L4A_I2C3,
    L4A_TAP,
    L4A_USBHS_OTG,
    L4A_USBHS_HOST,
    L4A_USBHS_TLL,
    L4A_MCSPI1,
    L4A_MCSPI2,
    L4A_MCSPI3,
    L4A_MCSPI4,
    L4A_SDMA,
    
    L4A_COUNT
} omap3_l4_agent_info_id_t;

static const struct omap3_l4_agent_info_s omap3_l4_agent_info[L4A_COUNT] = {
    /* L4-Core Agents */
    {L4A_DSS,        L4ID_DSI,       6},
    /* TODO: Camera */
    {L4A_USBHS_OTG,  L4ID_HSUSBOTG,  2},
    {L4A_USBHS_HOST, L4ID_USBHOST,   4},
    {L4A_USBHS_TLL,  L4ID_USBTLL,    2},
    {L4A_UART1,      L4ID_UART1,     2},
    {L4A_UART2,      L4ID_UART2,     2},
    {L4A_I2C1,       L4ID_I2C1,      2},
    {L4A_I2C2,       L4ID_I2C2,      2},
    {L4A_I2C3,       L4ID_I2C3,      2},
    /* TODO: McBSP1 */
    /* TODO: McBSP5 */
    {L4A_GPTIMER10,  L4ID_GPTIMER10, 2},
    {L4A_GPTIMER11,  L4ID_GPTIMER11, 2},
    {L4A_MCSPI1,     L4ID_MCSPI1,    2},
    {L4A_MCSPI2,     L4ID_MCSPI2,    2},
    {L4A_MMC1,       L4ID_MMCSDIO1,  2},
    {L4A_MMC2,       L4ID_MMCSDIO2,  2},
    {L4A_MMC3,       L4ID_MMCSDIO3,  2},
    /* TODO: HDQ/1-Wire */
    /* TODO: Mailbox */
    {L4A_MCSPI3,     L4ID_MCSPI3,    2},
    {L4A_MCSPI4,     L4ID_MCSPI4,    2},
    /* TODO: SR1 */
    /* TODO: SR2 */
    {L4A_SDMA,       L4ID_SDMA,      2},
    {L4A_CM,         L4ID_CM_A,      3},
    {L4A_SCM,        L4ID_SCM,       2},
    {L4A_TAP,        L4ID_TAP,       2},
    /* L4-Wakeup Agents */
    {L4A_GPTIMER12,  L4ID_GPTIMER12, 2},
    {L4A_PRM,        L4ID_PRM_A,     3},
    {L4A_GPIO1,      L4ID_GPIO1,     2},
    {L4A_WDTIMER2,   L4ID_WDTIMER2,  2},
    {L4A_GPTIMER1,   L4ID_GPTIMER1,  2},
    {L4A_32KTIMER,   L4ID_32KTIMER,  2},
    /* L4-Per Agents */
    {L4A_UART3,      L4ID_UART3,     2},
    {L4A_UART4,      L4ID_UART4,     2},
    /* TODO: McBSP2 */
    /* TODO: McBSP3 */
    /* TODO: McBSP4 */
    {L4A_GPTIMER2,   L4ID_GPTIMER2,  2},
    {L4A_GPTIMER3,   L4ID_GPTIMER3,  2},
    {L4A_GPTIMER4,   L4ID_GPTIMER4,  2},
    {L4A_GPTIMER5,   L4ID_GPTIMER5,  2},
    {L4A_GPTIMER6,   L4ID_GPTIMER6,  2},
    {L4A_GPTIMER7,   L4ID_GPTIMER7,  2},
    {L4A_GPTIMER8,   L4ID_GPTIMER8,  2},
    {L4A_GPTIMER9,   L4ID_GPTIMER9,  2},
    {L4A_GPIO2,      L4ID_GPIO2,     2},
    {L4A_GPIO3,      L4ID_GPIO3,     2},
    {L4A_GPIO4,      L4ID_GPIO4,     2},
    {L4A_GPIO5,      L4ID_GPIO5,     2},
    {L4A_GPIO6,      L4ID_GPIO6,     2},
    /* L4-Emu Agents */
    /* TODO: SDTI */
    /* TODO: ETB */
    /* TODO: TPIU */
    /* TODO: MPU */
    /* TODO: DAP */
};

#define omap3_l4ta_init(bus, cs) \
    omap3_l4ta_init(bus, omap3_l4_region, omap3_l4_agent_info, cs)

/* common PRM domain registers */
struct omap3_prm_domain_s {
    uint32_t rm_rstctrl;     /* 50 */
    uint32_t rm_rstst;       /* 58 */
    uint32_t pm_wken;        /* a0 */
    uint32_t pm_mpugrpsel;   /* a4 */
    uint32_t pm_ivagrpsel;   /* a8 */
    uint32_t pm_wkst;        /* b0 */
    uint32_t pm_wkst3;       /* b8 */
    uint32_t pm_wkdep;       /* c8 */
    uint32_t pm_evgenctrl;   /* d4 */
    uint32_t pm_evgenontim;  /* d8 */
    uint32_t pm_evgenofftim; /* dc */
    uint32_t pm_pwstctrl;    /* e0 */
    uint32_t pm_pwstst;      /* e4 */
    uint32_t pm_prepwstst;   /* e8 */
    uint32_t pm_wken3;       /* f0 */
};

struct omap3_prm_s {
    qemu_irq mpu_irq;
    qemu_irq iva_irq;
    MemoryRegion iomem1, iomem2;
    struct omap_mpu_state_s *omap;

    struct omap3_prm_domain_s iva2;
    struct omap3_prm_domain_s mpu;
    struct omap3_prm_domain_s core;
    struct omap3_prm_domain_s sgx;
    struct omap3_prm_domain_s wkup;
    struct omap3_prm_domain_s dss;
    struct omap3_prm_domain_s cam;
    struct omap3_prm_domain_s per;
    struct omap3_prm_domain_s emu;
    struct omap3_prm_domain_s neon;
    struct omap3_prm_domain_s usbhost;

    uint32_t prm_irqstatus_iva2;
    uint32_t prm_irqenable_iva2;
    
    uint32_t pm_iva2grpsel3_core;
    uint32_t pm_mpugrpsel3_core;

    struct {
        uint32_t prm_revision;
        uint32_t prm_sysconfig;
        uint32_t prm_irqstatus_mpu;
        uint32_t prm_irqenable_mpu;
    } ocp;

    struct {
        uint32_t prm_clksel;
        uint32_t prm_clkout_ctrl;
    } ccr; /* clock_control_reg */

    struct {
        uint32_t prm_vc_smps_sa;
        uint32_t prm_vc_smps_vol_ra;
        uint32_t prm_vc_smps_cmd_ra;
        uint32_t prm_vc_cmd_val_0;
        uint32_t prm_vc_cmd_val_1;
        uint32_t prm_vc_hc_conf;
        uint32_t prm_vc_i2c_cfg;
        uint32_t prm_vc_bypass_val;
        uint32_t prm_rstctrl;
        uint32_t prm_rsttimer;
        uint32_t prm_rstst;
        uint32_t prm_voltctrl;
        uint32_t prm_sram_pcharge;
        uint32_t prm_clksrc_ctrl;
        uint32_t prm_obs;
        uint32_t prm_voltsetup1;
        uint32_t prm_voltoffset;
        uint32_t prm_clksetup;
        uint32_t prm_polctrl;
        uint32_t prm_voltsetup2;
        struct {
            /* the following smartreflex control registers taken from
             * OMAP36XX TRM (missing from OMAP34XX TRM) */
            uint32_t config;
            uint32_t vstepmin;
            uint32_t vstepmax;
            uint32_t vlimitto;
            uint32_t voltage;
            uint32_t status;
        } prm_vp[2];
        uint32_t prm_ldo_abb_setup;
        uint32_t prm_ldo_abb_ctrl;
    } gr; /* global_reg */
};

static void omap3_prm_int_update(struct omap3_prm_s *s)
{
    qemu_set_irq(s->mpu_irq, s->ocp.prm_irqstatus_mpu & s->ocp.prm_irqenable_mpu);
    qemu_set_irq(s->iva_irq, s->prm_irqstatus_iva2 & s->prm_irqenable_iva2);
}

static void omap3_prm_reset(struct omap3_prm_s *s)
{
    memset(&s->iva2, 0, sizeof(s->iva2));
    s->iva2.rm_rstctrl    = 0x7;
    s->iva2.rm_rstst      = 0x1;
    s->iva2.pm_wkdep      = 0xb3;
    s->iva2.pm_pwstctrl   = 0xff0f07;
    s->iva2.pm_pwstst     = 0xff7;
    s->prm_irqstatus_iva2 = 0x0;
    s->prm_irqenable_iva2 = 0x0;

    memset(&s->ocp, 0, sizeof(s->ocp));
    s->ocp.prm_revision      = 0x10;
    s->ocp.prm_sysconfig     = 0x1;
    
    memset(&s->mpu, 0, sizeof(s->mpu));
    s->mpu.rm_rstst       = 0x1;
    s->mpu.pm_wkdep       = 0xa5;
    s->mpu.pm_pwstctrl    = 0x30107;
    s->mpu.pm_pwstst      = 0xc7;
    s->mpu.pm_evgenctrl   = 0x12;

    memset(&s->core, 0, sizeof(s->core));
    s->core.rm_rstst       = 0x1;
    s->core.pm_wken        = 0xc33ffe18;
    s->core.pm_mpugrpsel   = 0xc33ffe18;
    s->core.pm_ivagrpsel   = 0xc33ffe18;
    s->core.pm_pwstctrl    = 0xf0307;
    s->core.pm_pwstst      = 0xf7;
    s->core.pm_wken3       = 0x4;
    s->pm_iva2grpsel3_core = 0x4;
    s->pm_mpugrpsel3_core  = 0x4;

    memset(&s->sgx, 0, sizeof(s->sgx));
    s->sgx.rm_rstst     = 0x1;
    s->sgx.pm_wkdep     = 0x16;
    s->sgx.pm_pwstctrl  = 0x30107;
    s->sgx.pm_pwstst    = 0x3;

    memset(&s->wkup, 0, sizeof(s->wkup));
    s->wkup.pm_wken      = 0x3cb;
    s->wkup.pm_mpugrpsel = 0x3cb;
    s->wkup.pm_pwstst    = 0x3; /* TODO: check on real hardware */
    s->wkup.pm_prepwstst = 0x3; /* TODO: check on real hardware */

    memset(&s->ccr, 0, sizeof(s->ccr));
    s->ccr.prm_clksel      = 0x3; /* depends on the hw board, 0x3 for beagle */
    s->ccr.prm_clkout_ctrl = 0x80;

    memset(&s->dss, 0, sizeof(s->dss));
    s->dss.rm_rstst     = 0x1;
    s->dss.pm_wken      = 0x1;
    s->dss.pm_wkdep     = 0x16;
    s->dss.pm_pwstctrl  = 0x30107;
    s->dss.pm_pwstst    = 0x3;

    memset(&s->cam, 0, sizeof(s->cam));
    s->cam.rm_rstst     = 0x1;
    s->cam.pm_wkdep     = 0x16;
    s->cam.pm_pwstctrl  = 0x30107;
    s->cam.pm_pwstst    = 0x3;

    memset(&s->per, 0, sizeof(s->per));
    s->per.rm_rstst     = 0x1;
    s->per.pm_wken      = 0x3efff;
    s->per.pm_mpugrpsel = 0x3efff;
    s->per.pm_ivagrpsel = 0x3efff;
    s->per.pm_wkdep     = 0x17;
    s->per.pm_pwstctrl  = 0x30107;
    s->per.pm_pwstst    = 0x7;

    memset(&s->emu, 0, sizeof(s->emu));
    s->emu.rm_rstst  = 0x1;
    s->emu.pm_pwstst = 0x13;

    memset(&s->gr, 0, sizeof(s->gr));
    s->gr.prm_vc_i2c_cfg     = 0x18;
    s->gr.prm_rsttimer       = 0x1006;
    s->gr.prm_rstst          = 0x1; /* POR */
    s->gr.prm_sram_pcharge   = 0x50;
    s->gr.prm_clksrc_ctrl    = 0x43;
    s->gr.prm_polctrl        = 0xa;
    /* reset values for prm_vp[1,2] registers taken from OMAP36xx TRM */
    s->gr.prm_vp[0].status = s->gr.prm_vp[1].status = 0x1;

    memset(&s->neon, 0, sizeof(s->neon));
    s->neon.rm_rstst     = 0x1;
    s->neon.pm_wkdep     = 0x2;
    s->neon.pm_pwstctrl  = 0x7;
    s->neon.pm_pwstst    = 0x3;

    memset(&s->usbhost, 0, sizeof(s->usbhost));
    s->usbhost.rm_rstst     = 0x1;
    s->usbhost.pm_wken      = 0x1;
    s->usbhost.pm_mpugrpsel = 0x1;
    s->usbhost.pm_ivagrpsel = 0x1;
    s->usbhost.pm_wkdep     = 0x17;
    s->usbhost.pm_pwstctrl  = 0x30107;
    s->usbhost.pm_pwstst    = 0x3;

    omap3_prm_int_update(s);
}

static uint32_t omap3_prm_read(void *opaque, hwaddr addr)
{
    struct omap3_prm_s *s = (struct omap3_prm_s *)opaque;
    struct omap3_prm_domain_s *d = 0;

    TRACE_PRM(OMAP_FMT_plx, addr);
    
    /* handle common domain registers first - all domains may not
       have all common registers though but we're returning zeroes there */
    switch ((addr >> 8) & 0xff) {
        case 0x00: d = &s->iva2; break;
        case 0x09: d = &s->mpu; break;
        case 0x0a: d = &s->core; break;
        case 0x0b: d = &s->sgx; break;
        case 0x0c: d = &s->wkup; break;
        case 0x0e: d = &s->dss; break;
        case 0x0f: d = &s->cam; break;
        case 0x10: d = &s->per; break;
        case 0x11: d = &s->emu; break;
        case 0x13: d = &s->neon; break;
        case 0x14: d = &s->usbhost; break;
        default: break;
    }
    if (d)
        switch (addr & 0xff) {
            case 0x50: return d->rm_rstctrl;
            case 0x58: return d->rm_rstst;
            case 0xa0: return d->pm_wken;
            case 0xa4: return d->pm_mpugrpsel;
            case 0xa8: return d->pm_ivagrpsel;
            case 0xb0: return d->pm_wkst;
            case 0xb8: return d->pm_wkst3;
            case 0xc8: return d->pm_wkdep;
            case 0xd4: return d->pm_evgenctrl;
            case 0xd8: return d->pm_evgenontim;
            case 0xdc: return d->pm_evgenofftim;
            case 0xe0: return d->pm_pwstctrl;
            case 0xe4: return d->pm_pwstst;
            case 0xe8: return d->pm_prepwstst;
            case 0xf0: return d->pm_wken3;
            default: break;
        }

    /* okay, not a common domain register so let's take a closer look */
    switch (addr) {
        case 0x00f8: return s->prm_irqstatus_iva2;
        case 0x00fc: return s->prm_irqenable_iva2;
        case 0x0804: return s->ocp.prm_revision;
        case 0x0814: return s->ocp.prm_sysconfig;
        case 0x0818: return s->ocp.prm_irqstatus_mpu;
        case 0x081c: return s->ocp.prm_irqenable_mpu;
        case 0x0af4: return s->pm_iva2grpsel3_core;
        case 0x0af8: return s->pm_mpugrpsel3_core;
        case 0x0d40: return s->ccr.prm_clksel;
        case 0x0d70: return s->ccr.prm_clkout_ctrl;
        case 0x0de4: return 0x3; /* TODO: check on real hardware */
        case 0x0de8: return 0x3; /* TODO: check on real hardware */
        case 0x1220: return s->gr.prm_vc_smps_sa;
        case 0x1224: return s->gr.prm_vc_smps_vol_ra;
        case 0x1228: return s->gr.prm_vc_smps_cmd_ra;
        case 0x122c: return s->gr.prm_vc_cmd_val_0;
        case 0x1230: return s->gr.prm_vc_cmd_val_1;
        case 0x1234: return s->gr.prm_vc_hc_conf;
        case 0x1238: return s->gr.prm_vc_i2c_cfg;
        case 0x123c: return s->gr.prm_vc_bypass_val;
        case 0x1250: return s->gr.prm_rstctrl;
        case 0x1254: return s->gr.prm_rsttimer;
        case 0x1258: return s->gr.prm_rstst;
        case 0x1260: return s->gr.prm_voltctrl;
        case 0x1264: return s->gr.prm_sram_pcharge;    	
        case 0x1270: return s->gr.prm_clksrc_ctrl;
        case 0x1280: return s->gr.prm_obs;
        case 0x1290: return s->gr.prm_voltsetup1;
        case 0x1294: return s->gr.prm_voltoffset;
        case 0x1298: return s->gr.prm_clksetup;
        case 0x129c: return s->gr.prm_polctrl;
        case 0x12a0: return s->gr.prm_voltsetup2;
        case 0x12b0: return s->gr.prm_vp[0].config;
        case 0x12b4: return s->gr.prm_vp[0].vstepmin;
        case 0x12b8: return s->gr.prm_vp[0].vstepmax;
        case 0x12bc: return s->gr.prm_vp[0].vlimitto;
        case 0x12c0: return s->gr.prm_vp[0].voltage;
        case 0x12c4: return s->gr.prm_vp[0].status;
        case 0x12d0: return s->gr.prm_vp[1].config;
        case 0x12d4: return s->gr.prm_vp[1].vstepmin;
        case 0x12d8: return s->gr.prm_vp[1].vstepmax;
        case 0x12dc: return s->gr.prm_vp[1].vlimitto;
        case 0x12e0: return s->gr.prm_vp[1].voltage;
        case 0x12e4: return s->gr.prm_vp[1].status;
        default: break;
    }

    if (!cpu_is_omap3430(s->omap)) {
        switch (addr) {
            case 0x12f0: return s->gr.prm_ldo_abb_setup;
            case 0x12f4: return s->gr.prm_ldo_abb_ctrl;
            default: break;
        }
    }

    OMAP_BAD_REG(addr);
    return 0;
}

static inline void omap3_prm_clksrc_ctrl_update(struct omap3_prm_s *s)
{
    uint32_t value = s->gr.prm_clksrc_ctrl;
    
    if ((value & 0xd0) == 0x40) {
        omap_clk_setrate(omap_findclk(s->omap, "omap3_sys_clk"), 1, 1);
    } else if ((value & 0xd0) == 0x80) {
        omap_clk_setrate(omap_findclk(s->omap, "omap3_sys_clk"), 2, 1);
    }
    if (cpu_is_omap3630(s->omap)) {
        omap_clk_setrate(omap_findclk(s->omap, "omap3_dpll4_inref"),
                         (value & 0x100) ? 2 : 1,
                         (value & 0x100) ? 13 : 1);
    }
}

static void omap3_prm_clksel_update(struct omap3_prm_s *s)
{
    omap_clk newparent = 0;
    
    switch (s->ccr.prm_clksel & 7) {
        case 0: newparent = omap_findclk(s->omap, "omap3_osc_sys_clk12"); break;
        case 1: newparent = omap_findclk(s->omap, "omap3_osc_sys_clk13"); break;
        case 2: newparent = omap_findclk(s->omap, "omap3_osc_sys_clk192"); break;
        case 3: newparent = omap_findclk(s->omap, "omap3_osc_sys_clk26"); break;
        case 4: newparent = omap_findclk(s->omap, "omap3_osc_sys_clk384"); break;
        case 5: newparent = omap_findclk(s->omap, "omap3_osc_sys_clk168"); break;
        default:
            TRACE_PRM("invalid sys_clk input selection (%d) - ignored",
                      s->ccr.prm_clksel & 7);
            break;
    }
    if (newparent) {
        omap_clk_reparent(omap_findclk(s->omap, "omap3_sys_clk"), newparent);
        omap_clk_reparent(omap_findclk(s->omap, "omap3_sys_clkout1"), newparent);
    }
}

static void omap3_prm_ldo_update(struct omap3_prm_s *s)
{
    s->gr.prm_ldo_abb_setup &= ~0x58; /* default: bypass mode */
    if (s->gr.prm_ldo_abb_ctrl & 1) { /* SR2EN */
        switch (s->gr.prm_ldo_abb_setup & 3) { /* OPP_SEL */
            case 1: /* fast OPP */
                if (s->gr.prm_ldo_abb_ctrl & 4) {
                    s->gr.prm_ldo_abb_setup |= 0x10; /* FBB */
                }
                break;
            case 3: /* slow OPP */
                if (s->gr.prm_ldo_abb_ctrl & 2) {
                    s->gr.prm_ldo_abb_setup |= 0x08; /* RBB */
                }
                break;
            default:
                break;
        }
    }
}

static void omap3_prm_write(void *opaque, hwaddr addr,
                            uint32_t value)
{
    struct omap3_prm_s *s = (struct omap3_prm_s *)opaque;

    TRACE_PRM(OMAP_FMT_plx " = %08x", addr, value);
    switch (addr) {
        /* IVA2_PRM */
        case 0x0050: s->iva2.rm_rstctrl = value & 0x7; break;
        case 0x0058: s->iva2.rm_rstst &= ~(value & 0x3f0f); break;
        case 0x00c8: s->iva2.pm_wkdep = value & 0xb3; break;
        case 0x00e0:
            s->iva2.pm_pwstctrl = 0xcff000 | (value & 0x300f0f);
            /* TODO: support IVA2 wakeup contol. For now let's keep the
             * IVA2 domain always in ON state, never changing */
            s->iva2.pm_pwstst = ((s->iva2.pm_pwstctrl >> 12) & 0xff0) | 0x07;
            s->iva2.pm_prepwstst = s->iva2.pm_pwstst;
            break;
        case 0x00e4: OMAP_RO_REG(addr); break;
        case 0x00e8: /* ignore, we set the value in PWSTCTRL write */ break;
        case 0x00f8:
            s->prm_irqstatus_iva2 &= ~(value & 0x7);
            omap3_prm_int_update(s);
            break;
        case 0x00fc:
            s->prm_irqenable_iva2 = value & 0x7;
            omap3_prm_int_update(s);
            break;
        /* OCP_System_Reg_PRM */
        case 0x0804: OMAP_RO_REG(addr); break;
        case 0x0814: s->ocp.prm_sysconfig = value & 0x1; break;
        case 0x0818:
            if (cpu_is_omap3430(s->omap)) {
                value &= 0x03c003fd;
            } else { /* omap3630 */
                value &= 0x1ffffffd;
            }
            s->ocp.prm_irqstatus_mpu &= ~value;
            omap3_prm_int_update(s);
            break;
        case 0x081c:
            if (cpu_is_omap3430(s->omap)) {
                value &= 0x03c003fd;
            } else { /* omap3630 */
                value &= 0x1ffffffd;
            }
            s->ocp.prm_irqenable_mpu = value;
            omap3_prm_int_update(s);
            break;
        /* MPU_PRM */
        case 0x0958: s->mpu.rm_rstst &= ~(value & 0x080f); break;
        case 0x09c8: s->mpu.pm_wkdep = value & 0xa5; break;
        case 0x09d4: s->mpu.pm_evgenctrl = value & 0x1f; break;
        case 0x09d8: s->mpu.pm_evgenontim = value; break;
        case 0x09dc: s->mpu.pm_evgenofftim = value; break;
        case 0x09e0:
            s->mpu.pm_pwstctrl = value & 0x3010f;
            /* TODO: support MPU wakeup contol. For now let's keep the
             * MPU domain always in ON state, never changing */
            s->mpu.pm_pwstst = ((value >> 10) & 0xc0) | 0x07;
            s->mpu.pm_prepwstst = s->mpu.pm_pwstst;
            break;
        case 0x09e4: OMAP_RO_REG(addr); break;
        case 0x09e8: /* ignore, we set the value in PWSTCTRL write */ break;
        /* CORE_PRM */
        case 0x0a50: s->core.rm_rstctrl = value & 0x3; break; /* TODO: check if available on real hw */
        case 0x0a58: s->core.rm_rstst &= ~(value & 0x7); break;
        case 0x0aa0: s->core.pm_wken = 0x80000008 | (value & 0x433ffe10); break;
        case 0x0aa4: s->core.pm_mpugrpsel = 0x80000008 | (value & 0x433ffe10); break;
        case 0x0aa8: s->core.pm_ivagrpsel = 0x80000008 | (value & 0x433ffe10); break;
        case 0x0ab0: s->core.pm_wkst = value & 0x433ffe10; break;
        case 0x0ab8: s->core.pm_wkst3 &= ~(value & 0x4); break;
        case 0x0ae0:
            s->core.pm_pwstctrl = value & 0x0f031f;
            /* TODO: support CORE wakeup control. For now let's keep the
             * CORE domain always in ON state, never changing */
            s->core.pm_pwstst = ((value >> 12) & 0xf0) | 0x07;
            s->core.pm_prepwstst = s->core.pm_pwstst;
            break;
        case 0x0ae4: OMAP_RO_REG(addr); break;
        case 0x0ae8: /* ignore, we set the value in PWSTCTRL write */; break;
        case 0x0af0: s->core.pm_wken3 = value & 0x4; break;
        case 0x0af4: s->pm_iva2grpsel3_core = value & 0x4; break;
        case 0x0af8: s->pm_mpugrpsel3_core = value & 0x4; break;
        /* SGX_PRM */
        case 0x0b58: s->sgx.rm_rstst &= ~(value & 0xf); break;
        case 0x0bc8: s->sgx.pm_wkdep = value & 0x16; break;
        case 0x0be0:
            s->sgx.pm_pwstctrl = 0x030104 | (value & 0x3);
            /* TODO: support SGX wakeup control. For now let's keep the
             * SGX domain always in ON state, never changing */
            s->sgx.pm_pwstst = 0x3;
            s->sgx.pm_prepwstst = s->sgx.pm_pwstst;
            break;
        case 0x0be4: OMAP_RO_REG(addr); break;
        case 0x0be8: /* ignore, we set the value in PWSTCTRL write */ break;
        /* WKUP_PRM */
        case 0x0ca0:
            s->wkup.pm_wken = 0x2 | (value & 0x0103c9);
            if (s->wkup.pm_wken & (1 << 16)) { /* EN_IO_CHAIN */
                s->wkup.pm_wkst |= (1 << 16);   /* ST_IO_CHAIN */
            } else {
                s->wkup.pm_wkst &= ~(1 << 16);
            }
            break;
        case 0x0ca4: s->wkup.pm_mpugrpsel = 0x0102 | (value & 0x02c9); break;
        case 0x0ca8: s->wkup.pm_ivagrpsel = value & 0x03cb; break;
        case 0x0cb0: s->wkup.pm_wkst &= ~(value & 0x0103cb); break;
        case 0x0ce8: /* ignore */ break;
        /* Clock_Control_Reg_PRM */
        case 0x0d40: 
            s->ccr.prm_clksel = value & 0x7;
            omap3_prm_clksel_update(s);
            break;
        case 0x0d70:
            s->ccr.prm_clkout_ctrl = value & 0x80;
            omap_clk_onoff(omap_findclk(s->omap, "omap3_sys_clkout1"),
                           s->ccr.prm_clkout_ctrl & 0x80);
            break;
        case 0x0de8: /* ignore */ break;
        /* DSS_PRM */
        case 0x0e58: s->dss.rm_rstst &= ~(value & 0xf); break;
        case 0x0ea0: s->dss.pm_wken = value & 1; break;
        case 0x0ec8: s->dss.pm_wkdep = value & 0x16; break;
        case 0x0ee0:
            s->dss.pm_pwstctrl = 0x030104 | (value & 3);
            /* TODO: support DSS wakeup control. For now let's keep the
             * DSS domain always in ON state, never changing */
            s->dss.pm_pwstst = 0x3;
            s->dss.pm_prepwstst = s->dss.pm_pwstst;
            break;
        case 0x0ee4: OMAP_RO_REG(addr); break;
        case 0x0ee8: /* ignore, we set the value in PWSTCTRL write */ break;
        /* CAM_PRM */
        case 0x0f58: s->cam.rm_rstst &= (value & 0xf); break;
        case 0x0fc8: s->cam.pm_wkdep = value & 0x16; break;
        case 0x0fe0:
            s->cam.pm_pwstctrl = 0x030104 | (value & 3);
            /* TODO: support CAM wakeup control. For now let's keep the
             * CAM domain always in ON state, never changing */
            s->cam.pm_pwstst = 0x3;
            s->cam.pm_prepwstst = s->cam.pm_pwstst;
            break;
        case 0x0fe4: OMAP_RO_REG(addr); break;
        case 0x0fe8: /* ignore, we set the value in PWSTCTRL write */ break;
        /* PER_PRM */
        case 0x1058: s->per.rm_rstst &= ~(value & 0xf); break;
        case 0x10a0:
            if (cpu_is_omap3430(s->omap)) {
                value &= 0x03efff;
            } else { /* omap3630 */
                value &= 0x07efff;
            }
            s->per.pm_wken = value;
            break;
        case 0x10a4:
            if (cpu_is_omap3430(s->omap)) {
                value &= 0x03efff;
            } else { /* omap3630 */
                value &= 0x07efff;
            }
            s->per.pm_mpugrpsel = value;
            break;
        case 0x10a8:
            if (cpu_is_omap3430(s->omap)) {
                value &= 0x03efff;
            } else { /* omap3630 */
                value &= 0x07efff;
            }
            s->per.pm_ivagrpsel = value;
            break;
        case 0x10b0:
            if (cpu_is_omap3430(s->omap)) {
                value &= 0x03efff;
            } else { /* omap3630 */
                value &= 0x07efff;
            }
            s->per.pm_wkst &= ~value;
            break;
        case 0x10c8: s->per.pm_wkdep = value & 0x17; break;
        case 0x10e0:
            s->per.pm_pwstctrl = 0x030100 | (value & 7);
            /* TODO: support PER wakeup control. For now let's keep the
             * PER domain always in ON state, never changing */
            s->per.pm_pwstst = 0x07;
            s->per.pm_prepwstst = s->per.pm_pwstst;
            break;
        case 0x10e4: OMAP_RO_REG(addr); break;
        case 0x10e8: /* ignore, we set the value in PWSTCTRL write */ break;
        /* EMU_PRM */
        case 0x1158: s->emu.rm_rstst &= ~(value & 7); break;
        case 0x11e4: OMAP_RO_REG(addr); break;
        case 0x11e8: /* ignore */ break;
        /* Global_Reg_PRM */
        case 0x1220: s->gr.prm_vc_smps_sa = value & 0x7f007f; break;
        case 0x1224: s->gr.prm_vc_smps_vol_ra = value & 0xff00ff; break;
        case 0x1228: s->gr.prm_vc_smps_cmd_ra = value & 0xff00ff; break;
        case 0x122c: s->gr.prm_vc_cmd_val_0 = value; break;
        case 0x1230: s->gr.prm_vc_cmd_val_1 = value; break;
        case 0x1234: s->gr.prm_vc_hc_conf = value & 0x1f001f; break;
        case 0x1238: s->gr.prm_vc_i2c_cfg = value & 0x3f; break;
        case 0x123c: 
            s->gr.prm_vc_bypass_val = value & 0x01ffff7f;
            /* bzzzzzt.... command acknowledged! */
            s->gr.prm_vc_bypass_val &= ~(1 << 24); /* VALID */
            break;
        case 0x1250:
            s->gr.prm_rstctrl = 0;
            if (value & 0x06) { /* RST_DPLL3 | RST_GS */
                qemu_system_reset_request();
            }
            break;
        case 0x1254: s->gr.prm_rsttimer = value & 0x1fff; break;
        case 0x1258: s->gr.prm_rstst &= ~(value & 0x7fb); break;
        case 0x1260:
            if (cpu_is_omap3430(s->omap)) {
                value &= 0x1f;
            } else { /* omap3630 */
                value &= 0x11f;
            }
            s->gr.prm_voltctrl = value;
            break;
        case 0x1264: s->gr.prm_sram_pcharge = value & 0xff; break;
        case 0x1270:
            if (cpu_is_omap3430(s->omap)) {
                value &= 0xd8; /* force osc bypass mode */
            } else { /* omap3630 */
                value &= 0x1d8; /* force osc bypass mode */
            }
            s->gr.prm_clksrc_ctrl = value;
            omap3_prm_clksrc_ctrl_update(s);
            break;
        case 0x1280: OMAP_RO_REG(addr); break;
        case 0x1290: s->gr.prm_voltsetup1 = value; break;
        case 0x1294: s->gr.prm_voltoffset = value & 0xffff; break;
        case 0x1298: s->gr.prm_clksetup = value & 0xffff; break;
        case 0x129c: s->gr.prm_polctrl = value & 0xf; break;
        case 0x12a0: s->gr.prm_voltsetup2 = value & 0xffff; break;
        /* TODO: check if any (more) functionality is needed behind writes
         * to the prm_vp[1,2] registers */
        case 0x12b0:
            s->gr.prm_vp[0].config = value;
            if (value & 4) { /* INITVDD */
                s->gr.prm_vp[0].voltage = (value >> 8) & 0xff;
            }
            if (value & 2) { /* FORCEUPDATE */
                s->ocp.prm_irqstatus_mpu |= (1 << 15); /* VP1_TRANXDONE_ST */
                omap3_prm_int_update(s);
            }
            break;
        case 0x12b4: s->gr.prm_vp[0].vstepmin = value; break;
        case 0x12b8: s->gr.prm_vp[0].vstepmax = value; break;
        case 0x12bc: s->gr.prm_vp[0].vlimitto = value; break;
        case 0x12c0: OMAP_RO_REG(addr); break;
        case 0x12c4: OMAP_RO_REG(addr); break;
        case 0x12d0:
            s->gr.prm_vp[1].config = value;
            if (value & 4) { /* INITVDD */
                s->gr.prm_vp[1].voltage = (value >> 8) & 0xff;
            }
            if (value & 2) { /* FORCEUPDATE */
                s->ocp.prm_irqstatus_mpu |= (1 << 21); /* VP2_TRANXDONE_ST */
                omap3_prm_int_update(s);
            }
            break;
        case 0x12d4: s->gr.prm_vp[1].vstepmin = value; break;
        case 0x12d8: s->gr.prm_vp[1].vstepmax = value; break;
        case 0x12dc: s->gr.prm_vp[1].vlimitto = value; break;
        case 0x12e0: OMAP_RO_REG(addr); break;
        case 0x12e4: OMAP_RO_REG(addr); break;
        case 0x12f0:
            if (cpu_is_omap3430(s->omap)) {
                OMAP_BAD_REGV(addr, value);
            }
            s->gr.prm_ldo_abb_setup &= ~7;
            s->gr.prm_ldo_abb_setup |= value & 3; /* clear OPP_CHANGE */
            if (value & 4) { /* OPP_CHANGE */
                omap3_prm_ldo_update(s);
                s->ocp.prm_irqstatus_mpu |= (1 << 26); /*ABB_LDO_TRANXDONE_ST*/
                omap3_prm_int_update(s);
            }
            break;
        case 0x12f4:
            if (cpu_is_omap3430(s->omap)) {
                OMAP_BAD_REGV(addr, value);
            }
            s->gr.prm_ldo_abb_ctrl = value & 0xff0f;
            omap3_prm_ldo_update(s);
            break;
        /* NEON_PRM */
        case 0x1358: s->neon.rm_rstst &= ~(value & 0xf); break;
        case 0x13c8: s->neon.pm_wkdep = value & 0x2; break;
        case 0x13e0:
            s->neon.pm_pwstctrl = 0x4 | (value & 3);
            /* TODO: support NEON wakeup control. For now let's keep the
             * NEON domain always in ON state, never changing */
            s->neon.pm_pwstst = 0x3;
            s->neon.pm_prepwstst = s->neon.pm_pwstst;
            break;
        case 0x13e4: OMAP_RO_REG(addr); break;
        case 0x13e8: /* ignore, we set the value in PWSTCTRL write */ break;
        /* USBHOST_PRM */
        case 0x1458: s->usbhost.rm_rstst &= ~(value & 0xf); break;
        case 0x14a0: s->usbhost.pm_wken = value & 1; break;
        case 0x14a4: s->usbhost.pm_mpugrpsel = value & 1; break;
        case 0x14a8: s->usbhost.pm_ivagrpsel = value & 1; break;
        case 0x14b0: s->usbhost.pm_wkst &= ~(value & 1); break;
        case 0x14c8: s->usbhost.pm_wkdep = value & 0x17; break;
        case 0x14e0:
            s->usbhost.pm_pwstctrl = 0x030104 | (value & 0x13);
            /* TODO: support USBHOST wakeup control. For now let's keep the
             * USBHOST domain always in ON state, never changing */
            s->usbhost.pm_pwstst = 0x3;
            s->usbhost.pm_prepwstst = s->usbhost.pm_pwstst;
            break;
        case 0x14e4: OMAP_RO_REG(addr); break;
        case 0x14e8: /* ignore, we set the value in PWSTCTRL write */ break;
        default:
            OMAP_BAD_REGV(addr, value);
            break;
    }
}

static const MemoryRegionOps omap3_prm_ops = {
    .old_mmio = {
        .read = {
            omap_badwidth_read32,
            omap_badwidth_read32,
            omap3_prm_read,
        },
        .write = {
            omap_badwidth_write32,
            omap_badwidth_write32,
            omap3_prm_write,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static struct omap3_prm_s *omap3_prm_init(struct omap_target_agent_s *ta,
                                          qemu_irq mpu_int, qemu_irq iva_int,
                                          struct omap_mpu_state_s *mpu)
{
    struct omap3_prm_s *s = (struct omap3_prm_s *) g_malloc0(sizeof(*s));

    s->mpu_irq = mpu_int;
    s->iva_irq = iva_int;
    s->omap = mpu;
    omap3_prm_reset(s);

    memory_region_init_io(&s->iomem1, &omap3_prm_ops, s,
                          "omap3_prm", omap_l4_region_size(ta, 0));
    memory_region_init_alias(&s->iomem2, "omap3_prm2", &s->iomem1, 0,
                             omap_l4_region_size(ta, 1));
    omap_l4_attach(ta, 0, &s->iomem1);
    omap_l4_attach(ta, 1, &s->iomem2);
    return s;
}

struct omap3_cm_s {
    qemu_irq irq[3];
    MemoryRegion iomem1, iomem2;
    struct omap_mpu_state_s *mpu;
    int hsusb_stdby;

    /* IVA2_CM: base + 0x0000 */
    uint32_t cm_fclken_iva2;       /* 00 */
    uint32_t cm_clken_pll_iva2;    /* 04 */
    uint32_t cm_idlest_iva2;       /* 20 */
    uint32_t cm_idlest_pll_iva2;   /* 24 */
    uint32_t cm_autoidle_pll_iva2; /* 34 */
    uint32_t cm_clksel1_pll_iva2;  /* 40 */
    uint32_t cm_clksel2_pll_iva2;  /* 44 */
    uint32_t cm_clkstctrl_iva2;    /* 48 */
    uint32_t cm_clkstst_iva2;      /* 4c */

    /* OCP_System_Reg_CM: base + 0x0800 */
    uint32_t cm_revision;  /* 00 */
    uint32_t cm_sysconfig; /* 10 */

    /* MPU_CM: base + 0x0900 */
    uint32_t cm_clken_pll_mpu;    /* 04 */
    uint32_t cm_idlest_mpu;       /* 20 */
    uint32_t cm_idlest_pll_mpu;   /* 24 */
    uint32_t cm_autoidle_pll_mpu; /* 34 */
    uint32_t cm_clksel1_pll_mpu;  /* 40 */
    uint32_t cm_clksel2_pll_mpu;  /* 44 */
    uint32_t cm_clkstctrl_mpu;    /* 48 */
    uint32_t cm_clkstst_mpu;      /* 4c */

    /* CORE_CM: base + 0x0a00 */
    uint32_t cm_fclken1_core;   /* 0a00 */
    uint32_t cm_fclken2_core;   /* 0a04 */
    uint32_t cm_fclken3_core;   /* 0a08 */
    uint32_t cm_iclken1_core;   /* 0a10 */
    uint32_t cm_iclken2_core;   /* 0a14 */
    uint32_t cm_iclken3_core;   /* 0a18 */
    uint32_t cm_idlest1_core;   /* 0a20 */
    uint32_t cm_idlest2_core;   /* 0a24 */
    uint32_t cm_idlest3_core;   /* 0a28 */
    uint32_t cm_autoidle1_core; /* 0a30 */
    uint32_t cm_autoidle2_core; /* 0a34 */
    uint32_t cm_autoidle3_core; /* 0a38 */
    uint32_t cm_clksel_core;    /* 0a40 */
    uint32_t cm_clkstctrl_core; /* 0a48 */
    uint32_t cm_clkstst_core;   /* 0a4c */

    /* SGX_CM: base + 0x0b00 */
    uint32_t cm_fclken_sgx;    /* 00 */
    uint32_t cm_iclken_sgx;    /* 10 */
    uint32_t cm_idlest_sgx;    /* 20 */
    uint32_t cm_clksel_sgx;    /* 40 */
    uint32_t cm_sleepdep_sgx;  /* 44 */
    uint32_t cm_clkstctrl_sgx; /* 48 */
    uint32_t cm_clkstst_sgx;   /* 4c */

    /* WKUP_CM: base + 0x0c00 */
    uint32_t cm_fclken_wkup;   /* 00 */
    uint32_t cm_iclken_wkup;   /* 10 */
    uint32_t cm_idlest_wkup;   /* 20 */
    uint32_t cm_autoidle_wkup; /* 30 */
    uint32_t cm_clksel_wkup;   /* 40 */
    uint32_t cm_c48;           /* 48 */

    /* Clock_Control_Reg_CM: base + 0x0d00 */
    uint32_t cm_clken_pll;     /* 00 */
    uint32_t cm_clken2_pll;    /* 04 */
    uint32_t cm_idlest_ckgen;  /* 20 */
    uint32_t cm_idlest2_ckgen; /* 24 */
    uint32_t cm_autoidle_pll;  /* 30 */
    uint32_t cm_autoidle2_pll; /* 34 */
    uint32_t cm_clksel1_pll;   /* 40 */
    uint32_t cm_clksel2_pll;   /* 44 */
    uint32_t cm_clksel3_pll;   /* 48 */
    uint32_t cm_clksel4_pll;   /* 4c */
    uint32_t cm_clksel5_pll;   /* 50 */
    uint32_t cm_clkout_ctrl;   /* 70 */

    /* DSS_CM: base + 0x0e00 */
    uint32_t cm_fclken_dss;    /* 00 */
    uint32_t cm_iclken_dss;    /* 10 */
    uint32_t cm_idlest_dss;    /* 20 */
    uint32_t cm_autoidle_dss;  /* 30 */
    uint32_t cm_clksel_dss;    /* 40 */
    uint32_t cm_sleepdep_dss;  /* 44 */
    uint32_t cm_clkstctrl_dss; /* 48 */
    uint32_t cm_clkstst_dss;   /* 4c */

   /* CAM_CM: base + 0x0f00 */
    uint32_t cm_fclken_cam;    /* 00 */
    uint32_t cm_iclken_cam;    /* 10 */
    uint32_t cm_idlest_cam;    /* 20 */
    uint32_t cm_autoidle_cam;  /* 30 */
    uint32_t cm_clksel_cam;    /* 40 */
    uint32_t cm_sleepdep_cam;  /* 44 */
    uint32_t cm_clkstctrl_cam; /* 48 */
    uint32_t cm_clkstst_cam;   /* 4c */

    /* PER_CM: base + 0x1000 */
    uint32_t cm_fclken_per;    /* 00 */
    uint32_t cm_iclken_per;    /* 10 */
    uint32_t cm_idlest_per;    /* 20 */
    uint32_t cm_autoidle_per;  /* 30 */
    uint32_t cm_clksel_per;    /* 40 */
    uint32_t cm_sleepdep_per;  /* 44 */
    uint32_t cm_clkstctrl_per; /* 48 */
    uint32_t cm_clkstst_per;   /* 4c */

    /* EMU_CM: base + 0x1100 */
    uint32_t cm_clksel1_emu;   /* 40 */
    uint32_t cm_clkstctrl_emu; /* 48 */
    uint32_t cm_clkstst_emu;   /* 4c */
    uint32_t cm_clksel2_emu;   /* 50 */
    uint32_t cm_clksel3_emu;   /* 54 */

    /* Global_Reg_CM: base + 0x1200 */
    uint32_t cm_polctrl; /* 9c */

    /* NEON_CM: base + 0x1300 */
    uint32_t cm_idlest_neon;    /* 20 */
    uint32_t cm_clkstctrl_neon; /* 48 */

    /* USBHOST_CM: base + 0x1400 */
    uint32_t cm_fclken_usbhost;    /* 00 */
    uint32_t cm_iclken_usbhost;    /* 10 */
    uint32_t cm_idlest_usbhost;    /* 20 */
    uint32_t cm_autoidle_usbhost;  /* 30 */
    uint32_t cm_sleepdep_usbhost;  /* 44 */
    uint32_t cm_clkstctrl_usbhost; /* 48 */
    uint32_t cm_clkstst_usbhost;   /* 4c */
};

static inline void omap3_cm_clksel_wkup_update(struct omap3_cm_s *s)
{
    omap_clk_reparent(omap_findclk(s->mpu, "omap3_gp1_fclk"),
                      omap_findclk(s->mpu,
                                   (s->cm_clksel_wkup & 1) /* CLKSEL_GPT1 */
                                   ? "omap3_sys_clk"
                                   : "omap3_32k_fclk"));
    omap_clk_setrate(omap_findclk(s->mpu, "omap3_rm_iclk"),
                     (s->cm_clksel_wkup >> 1) & 3, /* CLKSEL_RM */
                     1);

    /* Tell GPTIMER to generate new clk rate */
    omap_gp_timer_change_clk(s->mpu->gptimer[0]);

    TRACE_CM("gptimer1 fclk=%lld",
             omap_clk_getrate(omap_findclk(s->mpu, "omap3_gp1_fclk")));

    /* TODO: CM_USIM_CLK */
}

static inline void omap3_cm_iva2_update(struct omap3_cm_s *s)
{
    uint32_t iva2_dpll_mul = ((s->cm_clksel1_pll_iva2 >> 8) & 0x7ff);
    uint32_t iva2_dpll_div, iva2_dpll_clkout_div, iva2_clk_src;
    omap_clk iva2_clk = omap_findclk(s->mpu, "omap3_iva2_clk");

    omap_clk_onoff(iva2_clk, s->cm_fclken_iva2 & 1);

    switch ((s->cm_clken_pll_iva2 & 0x7)) {
        case 0x01: /* low power stop mode */
        case 0x05: /* low power bypass mode */
            s->cm_idlest_pll_iva2 &= ~1;
            break;
        case 0x07: /* locked */
            if (iva2_dpll_mul < 2)
                s->cm_idlest_pll_iva2 &= ~1;
            else
                s->cm_idlest_pll_iva2 |= 1;
            break;
        default:
            break;
    }
    
    if (s->cm_idlest_pll_iva2 & 1) {
        iva2_dpll_div = s->cm_clksel1_pll_iva2 & 0x7f;
        iva2_dpll_clkout_div = s->cm_clksel2_pll_iva2 & 0x1f;
        omap_clk_reparent(iva2_clk, omap_findclk(s->mpu, "omap3_sys_clk"));
        omap_clk_setrate(iva2_clk,
                         (iva2_dpll_div + 1) * iva2_dpll_clkout_div,
                         iva2_dpll_mul);
    } else {
        /* bypass mode */
        iva2_clk_src = (s->cm_clksel1_pll_iva2 >> 19) & 0x07;
        omap_clk_reparent(iva2_clk, omap_findclk(s->mpu, "omap3_core_clk"));
        omap_clk_setrate(iva2_clk, iva2_clk_src, 1);
    }
}

static inline void omap3_cm_mpu_update(struct omap3_cm_s *s)
{
    uint32_t mpu_dpll_mul = ((s->cm_clksel1_pll_mpu >> 8) & 0x7ff);
    uint32_t mpu_dpll_div, mpu_dpll_clkout_div, mpu_clk_src;
    omap_clk mpu_clk = omap_findclk(s->mpu, "omap3_mpu_clk");
    
    switch ((s->cm_clken_pll_mpu & 0x7)) {
        case 0x05: /* low power bypass mode */
            s->cm_idlest_pll_mpu &= ~1;
            break;
        case 0x07: /* locked */
            if (mpu_dpll_mul < 2)
                s->cm_idlest_pll_mpu &= ~1;
            else
                s->cm_idlest_pll_mpu |= 1;
            break;
        default:
            break;
    }
    
    if (s->cm_idlest_pll_mpu & 1) {
        mpu_dpll_div = s->cm_clksel1_pll_mpu & 0x7f;
        mpu_dpll_clkout_div = s->cm_clksel2_pll_mpu & 0x1f;
        omap_clk_reparent(mpu_clk, omap_findclk(s->mpu, "omap3_sys_clk"));
        omap_clk_setrate(mpu_clk,
                         (mpu_dpll_div + 1) * mpu_dpll_clkout_div,
                         mpu_dpll_mul);
    } else {
        /* bypass mode */
        mpu_clk_src = (s->cm_clksel1_pll_mpu >> 19) & 0x07;
        omap_clk_reparent(mpu_clk, omap_findclk(s->mpu, "omap3_core_clk"));
        omap_clk_setrate(mpu_clk, mpu_clk_src, 1);
    }
}

static inline void omap3_cm_dpll3_update(struct omap3_cm_s *s)
{
    uint32_t core_dpll_mul = ((s->cm_clksel1_pll >> 16) & 0x7ff);
    uint32_t core_dpll_div, core_dpll_clkout_div, div_dpll3;

    switch ((s->cm_clken_pll & 0x7)) {
        case 0x05: /* low power bypass */
        case 0x06: /* fast relock bypass */
            s->cm_idlest_ckgen &= ~1;
            break;
        case 0x07: /* locked */
            if (core_dpll_mul < 2)
                s->cm_idlest_ckgen &= ~1;
            else
                s->cm_idlest_ckgen |= 1;
            break;
        default:
            break;
    }

    if (s->cm_idlest_ckgen & 1) {
        core_dpll_div = (s->cm_clksel1_pll >> 8) & 0x7f;
        core_dpll_clkout_div = (s->cm_clksel1_pll >> 27) & 0x1f;
        div_dpll3 = s->cm_clksel1_emu >> 16;
        if (cpu_is_omap3430(s->mpu)) {
            div_dpll3 &= 0x1f;
        } else { /* omap3630 */
            div_dpll3 &= 0x3f;
        }
        
        if (s->cm_clksel2_emu & 0x80000) { /* OVERRIDE_ENABLE */
        	core_dpll_mul = (s->cm_clksel2_emu >> 8) & 0x7ff;
        	core_dpll_div = s->cm_clksel2_emu & 0x7f;
        }
        
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_core_clk"),
                         (core_dpll_div + 1) * core_dpll_clkout_div,
                         core_dpll_mul);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_core2_clk"),
                         (core_dpll_div + 1) * core_dpll_clkout_div,
                         core_dpll_mul * 2);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_emu_core_alwon_clk"),
                         (core_dpll_div + 1) * div_dpll3,
                         core_dpll_mul * 2);
    } else {
        /* bypass mode */
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_core_clk"), 1, 1);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_core2_clk"), 1, 1);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_emu_core_alwon_clk"), 1, 1);
    }
}

static inline void omap3_cm_dpll4_update(struct omap3_cm_s *s)
{
    uint32_t per_dpll_mul = (s->cm_clksel2_pll >> 8);
    uint32_t per_dpll_div, div_96m, clksel_tv, clksel_dss1, clksel_cam, div_dpll4;

    if (cpu_is_omap3430(s->mpu)) {
        per_dpll_mul &= 0x7ff;
    } else { /* omap3630 */
        per_dpll_mul &= 0xfff;
    }
    switch (((s->cm_clken_pll >> 16) & 0x7)) {
        case 0x01: /* lower power stop mode */
            s->cm_idlest_ckgen &= ~2;
            break;
        case 0x07: /* locked */
            if (per_dpll_mul < 2)
                s->cm_idlest_ckgen &= ~2;
            else
                s->cm_idlest_ckgen |= 2;
            break;
        default:
            break;
    }

    if (s->cm_idlest_ckgen & 2) {
        per_dpll_div = s->cm_clksel2_pll & 0x7f;
        div_96m = s->cm_clksel3_pll & 0x1f;
        clksel_tv = s->cm_clksel_dss >> 8;
        clksel_dss1 = s->cm_clksel_dss;
        clksel_cam = s->cm_clksel_cam;
        div_dpll4 = s->cm_clksel1_emu >> 24;
        if (cpu_is_omap3430(s->mpu)) {
            clksel_tv &= 0x1f;
            clksel_dss1 &= 0x1f;
            clksel_cam &= 0x1f;
            div_dpll4 &= 0x1f;
        } else { /* omap3630 */
            clksel_tv &= 0x3f;
            clksel_dss1 &= 0x3f;
            clksel_cam &= 0x3f;
            div_dpll4 &= 0x3f;
        }
        
        if (s->cm_clksel3_emu & 0x80000) { /* OVERRIDE_ENABLE */
        	per_dpll_mul = (s->cm_clksel3_emu >> 8) & 0x7ff;
        	per_dpll_div =  s->cm_clksel3_emu & 0x7f;
        }
        
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_96m_fclk"),
                         (per_dpll_div + 1) * div_96m,
                         per_dpll_mul * 2);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_54m_fclk"),
                         (per_dpll_div + 1) * clksel_tv,
                         per_dpll_mul * 2);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_dss1_alwon_fclk"),
                         (per_dpll_div + 1) * clksel_dss1,
                         per_dpll_mul * 2);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_cam_mclk"),
                         (per_dpll_div + 1) * clksel_cam,
                         per_dpll_mul * 2);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_per_alwon_clk"),
                         (per_dpll_div + 1) * div_dpll4,
                         per_dpll_mul * 2);
    } else {
        /* bypass mode */
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_96m_fclk"), 1, 1);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_54m_fclk"), 1, 1);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_dss1_alwon_fclk"), 1, 1);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_cam_mclk"), 1, 1);
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_per_alwon_clk"), 1, 1);
    }
}

static inline void omap3_cm_dpll5_update(struct omap3_cm_s *s)
{
    uint32_t per2_dpll_mul = ((s->cm_clksel4_pll >> 8) & 0x7ff);
    uint32_t per2_dpll_div, div_120m;

    switch ((s->cm_clken2_pll & 0x7)) {
        case 0x01: /* low power stop mode */
            s->cm_idlest2_ckgen &= ~1;
            break;
        case 0x07: /* locked */
            if (per2_dpll_mul < 2)
                s->cm_idlest2_ckgen &= ~1;
            else
                s->cm_idlest2_ckgen |= 1;
            break;
        default:
            break;
    }

    if (s->cm_idlest2_ckgen & 1) {
        per2_dpll_div = s->cm_clksel4_pll & 0x7f;
        div_120m = s->cm_clksel5_pll & 0x1f;
        
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_120m_fclk"),
                         (per2_dpll_div + 1) * div_120m,
                         per2_dpll_mul);
    } else {
        /* bypass mode */
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_120m_fclk"), 1, 1);
    }
}

static inline void omap3_cm_48m_update(struct omap3_cm_s *s)
{
    omap_clk pclk = omap_findclk(s->mpu,
                                 (s->cm_clksel1_pll & 0x8) /* SOURCE_48M */
                                 ? "omap3_sys_altclk"
                                 : "omap3_96m_fclk");
    
    omap_clk_reparent(omap_findclk(s->mpu, "omap3_48m_fclk"), pclk);
    omap_clk_reparent(omap_findclk(s->mpu, "omap3_12m_fclk"), pclk);
}

static inline void omap3_cm_gp10gp11_update(struct omap3_cm_s *s)
{
    omap_clk gp10 = omap_findclk(s->mpu, "omap3_gp10_fclk");
    omap_clk gp11 = omap_findclk(s->mpu, "omap3_gp11_fclk");
    omap_clk sys  = omap_findclk(s->mpu, "omap3_sys_clk");
    omap_clk f32k = omap_findclk(s->mpu, "omap3_32k_fclk");

    omap_clk_reparent(gp10, (s->cm_clksel_core & 0x40) ? sys : f32k);
    omap_clk_reparent(gp11, (s->cm_clksel_core & 0x80) ? sys : f32k);
    omap_gp_timer_change_clk(s->mpu->gptimer[9]);
    omap_gp_timer_change_clk(s->mpu->gptimer[10]);
    
    TRACE_CM("gptimer10 fclk = %lld", omap_clk_getrate(gp10));
    TRACE_CM("gptimer11 fclk = %lld", omap_clk_getrate(gp11));
}

static inline void omap3_cm_per_gptimer_update(struct omap3_cm_s *s)
{
    omap_clk sys = omap_findclk(s->mpu, "omap3_sys_clk");
    omap_clk f32k = omap_findclk(s->mpu, "omap3_32k_fclk");
    uint32_t cm_clksel_per = s->cm_clksel_per;
    uint32_t n;
    char clkname[] = "omap3_gp#_fclk";

    for (n = 1; n < 9; n++, cm_clksel_per >>= 1) {
        clkname[8] = '1' + n; /* 2 - 9 */
        omap_clk_reparent(omap_findclk(s->mpu, clkname),
                          (cm_clksel_per & 1) ? sys : f32k);
        omap_gp_timer_change_clk(s->mpu->gptimer[n]);
        TRACE_CM("gptimer%d fclk = %lld", n + 1,
                 omap_clk_getrate(omap_findclk(s->mpu, clkname)));
    }
}

static inline void omap3_cm_clkout2_update(struct omap3_cm_s *s)
{
    omap_clk c = omap_findclk(s->mpu, "omap3_sys_clkout2");
	
    omap_clk_onoff(c, (s->cm_clkout_ctrl >> 7) & 1);
    switch (s->cm_clkout_ctrl & 0x3) {
        case 0:
            omap_clk_reparent(c, omap_findclk(s->mpu, "omap3_core_clk"));
            break;
        case 1:
            omap_clk_reparent(c, omap_findclk(s->mpu, "omap3_sys_clk"));
            break;
        case 2:
            omap_clk_reparent(c, omap_findclk(s->mpu, "omap3_96m_fclk"));
            break;
        case 3:
            omap_clk_reparent(c, omap_findclk(s->mpu, "omap3_54m_fclk"));
            break;
        default:
            break;
    }
    omap_clk_setrate(c, 1 << ((s->cm_clkout_ctrl >> 3) & 7), 1);
}

static inline void omap3_cm_fclken1_core_update(struct omap3_cm_s *s)
{
    uint32_t v = s->cm_fclken1_core;
    
    /* TODO: EN_MCBSP1,5 */
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_gp10_fclk"),  (v >> 11) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_gp11_fclk"),  (v >> 12) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_uart1_fclk"), (v >> 13) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_uart2_fclk"), (v >> 14) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_i2c1_fclk"),  (v >> 15) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_i2c2_fclk"),  (v >> 16) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_i2c3_fclk"),  (v >> 17) & 1);
    /* TODO: EN_HDQ, EN_SPI1-4 */
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_mmc1_fclk"),  (v >> 24) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_mmc2_fclk"),  (v >> 25) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_mmc3_fclk"),  (v >> 30) & 1);
}

static inline void omap3_cm_iclken1_core_update(struct omap3_cm_s *s)
{
    uint32_t v = s->cm_iclken1_core;
    
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_sdrc_iclk"),  (v >> 1) & 1);
    /* TODO: EN_HSOTGUSB, EN_OMAPCTRL, EN_MAILBOXES, EN_MCBSP1,5 */
    /* TODO: EN_GPT10, EN_GPT11 */
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_uart1_iclk"), (v >> 13) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_uart2_iclk"), (v >> 14) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_i2c1_iclk"),  (v >> 15) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_i2c2_iclk"),  (v >> 16) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_i2c3_iclk"),  (v >> 17) & 1);
    /* TODO: EN_HDQ, EN_SPI1-4 */
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_mmc1_iclk"),  (v >> 24) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_mmc2_iclk"),  (v >> 25) & 1);
    omap_clk_onoff(omap_findclk(s->mpu, "omap3_mmc3_iclk"),  (v >> 30) & 1);

    /* if EN_HSOTGUSB==1 then ST_HSOTGUSB_IDLE==0 */
    /* if EN_SDRC==1 then ST_SDMA==0 */
    v = (v & ~0x24) | ((v & 0x12) << 1);
    /* set ST_HSOTGUSB_STDBY */
    if (s->hsusb_stdby) {
        v &= ~0x10;
    } else {
        v |= 0x10;
    }
    
    s->cm_idlest1_core = ~v;
}

static void omap3_cm_hsusb_otg_stdby_callback(void *opaque, int source,
                                              int level)
{
    struct omap3_cm_s *s = opaque;
    s->hsusb_stdby = !!level;
    omap3_cm_iclken1_core_update(s);
}

static inline void omap3_cm_l3l4iclk_update(struct omap3_cm_s *s)
{
    int div = s->cm_clksel_core & 0x3;
    if (div != 1 && div != 2) {
        TRACE_CM("invalid CLKSEL_L3 value (%d)", div);
        div = (div > 2) ? 2 : 1;
        s->cm_clksel_core = (s->cm_clksel_core & ~0x3) | div;
    }
    omap_clk_setrate(omap_findclk(s->mpu, "omap3_l3_iclk"), div, 1);
    
    div = (s->cm_clksel_core >> 2) & 0x3;
    if (div != 1 && div != 2) {
        TRACE_CM("invalid CLKSEL_L4 value (%d)", div);
        div = (div > 2) ? 2 : 1;
        s->cm_clksel_core = (s->cm_clksel_core & ~(0x3 << 2)) | (div << 2);
    }
    omap_clk_setrate(omap_findclk(s->mpu, "omap3_l4_iclk"), div, 1);

    div = (s->cm_clksel_core >> 12) & 0x3;
    if (div != 1 && (!cpu_is_omap3630(s->mpu) || div != 2)) {
        TRACE_CM("invalid CLKSEL_96M value (%d)", div);
        div = 1;
        s->cm_clksel_core = (s->cm_clksel_core & ~(0x3 << 12)) | (div << 2);
    }
    if (cpu_is_omap3630(s->mpu)) {
        omap_clk_setrate(omap_findclk(s->mpu, "omap3_core_96m_fclk"), div, 1);
    }
}

static void omap3_cm_reset(struct omap3_cm_s *s)
{
    s->hsusb_stdby = 1;
    
    s->cm_fclken_iva2 = 0x0;
    s->cm_clken_pll_iva2 = 0x11;
    s->cm_idlest_iva2 = 0x1;
    s->cm_idlest_pll_iva2 = 0;
    s->cm_autoidle_pll_iva2 = 0x0;
    s->cm_clksel1_pll_iva2 = 0x80000;
    s->cm_clksel2_pll_iva2 = 0x1;
    s->cm_clkstctrl_iva2 = 0x0;
    s->cm_clkstst_iva2 = 0x0;

    s->cm_revision = 0x10;
    s->cm_sysconfig = 0x1;

    s->cm_clken_pll_mpu = 0x15;
    s->cm_idlest_mpu = 0x1;
    s->cm_idlest_pll_mpu = 0;
    s->cm_autoidle_pll_mpu = 0x0;
    s->cm_clksel1_pll_mpu = 0x80000;
    s->cm_clksel2_pll_mpu = 0x1;
    s->cm_clkstctrl_mpu = 0x0;
    s->cm_clkstst_mpu = 0x0;

    s->cm_fclken1_core = 0x0;
    s->cm_fclken2_core = 0x0;
    s->cm_fclken3_core = 0x0;
    s->cm_iclken1_core = 0x42;
    s->cm_iclken2_core = 0x0;
    s->cm_iclken3_core = 0x0;
    s->cm_idlest1_core = 0xffffffff;
    s->cm_idlest2_core = 0x0;
    s->cm_idlest3_core = 0xa; 
    s->cm_autoidle1_core = 0x0;
    s->cm_autoidle2_core = 0x0;
    s->cm_autoidle3_core = 0x0;
    s->cm_clksel_core = 0x1105;
    s->cm_clkstctrl_core = 0x0;
    s->cm_clkstst_core = 0x0;

    s->cm_fclken_sgx = 0x0;
    s->cm_iclken_sgx = 0x0;
    s->cm_idlest_sgx = 0x1;
    s->cm_clksel_sgx = 0x0;
    s->cm_sleepdep_sgx = 0x0;
    s->cm_clkstctrl_sgx = 0x0;
    s->cm_clkstst_sgx = 0x0;

    s->cm_fclken_wkup = 0x0;
    s->cm_iclken_wkup = 0x0;
    /*assume all clock can be accessed*/
    s->cm_idlest_wkup = 0x0;
    s->cm_autoidle_wkup = 0x0;
    s->cm_clksel_wkup = 0x12;

    s->cm_clken_pll = 0x110015;
    s->cm_clken2_pll = 0x11;
    s->cm_idlest_ckgen = 0x3f3c; /* FIXME: provide real clock statuses */
    s->cm_idlest2_ckgen = 0xa; /* FIXME: provide real clock statuses */
    s->cm_autoidle_pll = 0x0;
    s->cm_autoidle2_pll = 0x0;
    s->cm_clksel1_pll = 0x8000040;
    s->cm_clksel2_pll = (cpu_is_omap3430(s->mpu) ? 0x0 : 0x02400000);
    s->cm_clksel3_pll = 0x1;
    s->cm_clksel4_pll = 0x0;
    s->cm_clksel5_pll = 0x1;
    s->cm_clkout_ctrl = 0x3;

    s->cm_fclken_dss = 0x0;
    s->cm_iclken_dss = 0x0;
    /*dss can be accessed*/
    s->cm_idlest_dss = 0x0;
    s->cm_autoidle_dss = 0x0;
    s->cm_clksel_dss = (cpu_is_omap3430(s->mpu) ? 0x1010 : 0x410);
    s->cm_sleepdep_dss = 0x0;
    s->cm_clkstctrl_dss = 0x0;
    s->cm_clkstst_dss = 0x0;

    s->cm_fclken_cam = 0x0;
    s->cm_iclken_cam = 0x0;
    s->cm_idlest_cam = 0x1;
    s->cm_autoidle_cam = 0x0;
    s->cm_clksel_cam = (cpu_is_omap3430(s->mpu) ? 0x10 : 0x04);
    s->cm_sleepdep_cam = 0x0;
    s->cm_clkstctrl_cam = 0x0;
    s->cm_clkstst_cam = 0x0;

    s->cm_fclken_per = 0x0;
    s->cm_iclken_per = 0x0;
    //s->cm_idlest_per = (cpu_is_omap3430(s->mpu) ? 0x3ffff : 0x7ffff);
    s->cm_idlest_per = 0x0; //enable all access by default
    s->cm_autoidle_per = 0x0;
    s->cm_clksel_per = 0x0;
    s->cm_sleepdep_per = 0x0;
    s->cm_clkstctrl_per = 0x0;
    s->cm_clkstst_per = 0x0;

    s->cm_clksel1_emu = 0x10100a50;
    s->cm_clkstctrl_emu = 0x2;
    s->cm_clkstst_emu = 0x0;
    s->cm_clksel2_emu = 0x0;
    s->cm_clksel3_emu = 0x0;

    s->cm_polctrl = 0x0;

    s->cm_idlest_neon = 0x1;
    s->cm_clkstctrl_neon = 0x0;

    s->cm_fclken_usbhost = 0x0;
    s->cm_iclken_usbhost = 0x0;
    s->cm_idlest_usbhost = 0x3;
    s->cm_autoidle_usbhost = 0x0;
    s->cm_sleepdep_usbhost = 0x0;
    s->cm_clkstctrl_usbhost = 0x0;
    s->cm_clkstst_usbhost = 0x0;
}

static uint32_t omap3_cm_read(void *opaque, hwaddr addr)
{
    struct omap3_cm_s *s = (struct omap3_cm_s *) opaque;

    TRACE_CM(OMAP_FMT_plx, addr);
    switch (addr) {
        /* IVA2_CM */
        case 0x0000: return s->cm_fclken_iva2;
        case 0x0004: return s->cm_clken_pll_iva2;
        case 0x0020: return s->cm_idlest_iva2;
        case 0x0024: return s->cm_idlest_pll_iva2;
        case 0x0034: return s->cm_autoidle_pll_iva2;
        case 0x0040: return s->cm_clksel1_pll_iva2;
        case 0x0044: return s->cm_clksel2_pll_iva2;
        case 0x0048: return s->cm_clkstctrl_iva2;
        case 0x004c: return s->cm_clkstst_iva2;
        /* OCP_System_Reg_CM */
        case 0x0800: return s->cm_revision;
        case 0x0810: return s->cm_sysconfig;
        /* MPU_CM */
        case 0x0904: return s->cm_clken_pll_mpu;
        case 0x0920: return s->cm_idlest_mpu & 0x0; /*MPU is active*/
        case 0x0924: return s->cm_idlest_pll_mpu;
        case 0x0934: return s->cm_autoidle_pll_mpu;
        case 0x0940: return s->cm_clksel1_pll_mpu;
        case 0x0944: return s->cm_clksel2_pll_mpu;
        case 0x0948: return s->cm_clkstctrl_mpu;
        case 0x094c: return s->cm_clkstst_mpu;
        /* CORE_CM */
        case 0x0a00: return s->cm_fclken1_core;
        case 0x0a04: return s->cm_fclken2_core;
        case 0x0a08: return s->cm_fclken3_core;
        case 0x0a10: return s->cm_iclken1_core;
        case 0x0a14: return s->cm_iclken2_core;
        case 0x0a18: return s->cm_iclken3_core;
        case 0x0a20: return s->cm_idlest1_core;
        case 0x0a24: return s->cm_idlest2_core;
        case 0x0a28: return s->cm_idlest3_core;
        case 0x0a30: return s->cm_autoidle1_core;
        case 0x0a34: return s->cm_autoidle2_core;
        case 0x0a38: return s->cm_autoidle3_core;
        case 0x0a40: return s->cm_clksel_core;
        case 0x0a48: return s->cm_clkstctrl_core;
        case 0x0a4c: return s->cm_clkstst_core;
        /* SGX_CM */
        case 0x0b00: return s->cm_fclken_sgx;
        case 0x0b10: return s->cm_iclken_sgx;
        case 0x0b20: return s->cm_idlest_sgx & 0x0;
        case 0x0b40: return s->cm_clksel_sgx;
        case 0x0b44: return s->cm_sleepdep_sgx;
        case 0x0b48: return s->cm_clkstctrl_sgx;
        case 0x0b4c: return s->cm_clkstst_sgx;
        /* WKUP_CM */
        case 0x0c00: return s->cm_fclken_wkup;
        case 0x0c10: return s->cm_iclken_wkup;
        case 0x0c20: return 0; /* TODO: Check if the timer can be accessed. */
        case 0x0c30: return s->cm_idlest_wkup;
        case 0x0c40: return s->cm_clksel_wkup;
        case 0x0c48: return s->cm_c48;
        /* Clock_Control_Reg_CM */
        case 0x0d00: return s->cm_clken_pll;
        case 0x0d04: return s->cm_clken2_pll;
        case 0x0d20: return s->cm_idlest_ckgen;
        case 0x0d24: return s->cm_idlest2_ckgen;
        case 0x0d30: return s->cm_autoidle_pll;
        case 0x0d34: return s->cm_autoidle2_pll;
        case 0x0d40: return s->cm_clksel1_pll;
        case 0x0d44: return s->cm_clksel2_pll;
        case 0x0d48: return s->cm_clksel3_pll;
        case 0x0d4c: return s->cm_clksel4_pll;
        case 0x0d50: return s->cm_clksel5_pll;
        case 0x0d70: return s->cm_clkout_ctrl;
        /* DSS_CM */
        case 0x0e00: return s->cm_fclken_dss;
        case 0x0e10: return s->cm_iclken_dss;
        case 0x0e20: return s->cm_idlest_dss;
        case 0x0e30: return s->cm_autoidle_dss;
        case 0x0e40: return s->cm_clksel_dss;
        case 0x0e44: return s->cm_sleepdep_dss;
        case 0x0e48: return s->cm_clkstctrl_dss;
        case 0x0e4c: return s->cm_clkstst_dss;
        /* CAM_CM */
        case 0x0f00: return s->cm_fclken_cam;
        case 0x0f10: return s->cm_iclken_cam;
        case 0x0f20: return s->cm_idlest_cam & 0x0;
        case 0x0f30: return s->cm_autoidle_cam;
        case 0x0f40: return s->cm_clksel_cam;
        case 0x0f44: return s->cm_sleepdep_cam;
        case 0x0f48: return s->cm_clkstctrl_cam;
        case 0x0f4c: return s->cm_clkstst_cam;
        /* PER_CM */
        case 0x1000: return s->cm_fclken_per;
        case 0x1010: return s->cm_iclken_per;
        case 0x1020: return s->cm_idlest_per ;
        case 0x1030: return s->cm_autoidle_per;
        case 0x1040: return s->cm_clksel_per;
        case 0x1044: return s->cm_sleepdep_per;
        case 0x1048: return s->cm_clkstctrl_per;
        case 0x104c: return s->cm_clkstst_per;
        /* EMU_CM */
        case 0x1140: return s->cm_clksel1_emu;
        case 0x1148: return s->cm_clkstctrl_emu;
        case 0x114c: return s->cm_clkstst_emu & 0x0;
        case 0x1150: return s->cm_clksel2_emu;
        case 0x1154: return s->cm_clksel3_emu;
        /* Global_Reg_CM */
        case 0x129c: return s->cm_polctrl;
        /* NEON_CM */
        case 0x1320: return s->cm_idlest_neon & 0x0;
        case 0x1348: return s->cm_clkstctrl_neon;
        /* USBHOST_CM */
        case 0x1400: return s->cm_fclken_usbhost;
        case 0x1410: return s->cm_iclken_usbhost;
        case 0x1420: return s->cm_idlest_usbhost & 0x0;
        case 0x1430: return s->cm_autoidle_usbhost;
        case 0x1444: return s->cm_sleepdep_usbhost;
        case 0x1448: return s->cm_clkstctrl_usbhost;
        case 0x144c: return s->cm_clkstst_usbhost;
        /* unknown */
        default: break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap3_cm_write(void *opaque,
                           hwaddr addr,
                           uint32_t value)
{
    struct omap3_cm_s *s = (struct omap3_cm_s *)opaque;

    TRACE_CM(OMAP_FMT_plx " = 0x%08x", addr, value);
    switch (addr) {
        case 0x0020:
        case 0x0024:
        case 0x004c:
        case 0x0800:
        case 0x0920:
        case 0x0924:
        case 0x094c:
        case 0x0a20:
        case 0x0a24:
        case 0x0a28:
        case 0x0a4c:
        case 0x0b20:
        case 0x0b4c:
        case 0x0c20:
        case 0x0d20:
        case 0x0d24:
        case 0x0e20:
        case 0x0e4c:
        case 0x0f20:
        case 0x0f4c:
        case 0x1020:
        case 0x104c:
        case 0x114c:
        case 0x1320:
        case 0x1420:
        case 0x144c:
            OMAP_RO_REGV(addr, value);
            break;
        /* IVA2_CM */
        case 0x0000:
            s->cm_fclken_iva2 = value & 0x1;
            omap3_cm_iva2_update(s);
            break;
        case 0x0004: 
            s->cm_clken_pll_iva2 = value & 0x7ff;
            omap3_cm_iva2_update(s);
            break;
        case 0x0034:
            s->cm_autoidle_pll_iva2 = value & 0x7;
            break;
        case 0x0040:
            s->cm_clksel1_pll_iva2 = value & 0x3fff7f;
            omap3_cm_iva2_update(s);
            break;
        case 0x0044:
            s->cm_clksel2_pll_iva2 = value & 0x1f;
            omap3_cm_iva2_update(s);
            break;
        case 0x0048:
            s->cm_clkstctrl_iva2 = value & 0x3;
            break;
        /* OCP_System_Reg_CM */
        case 0x0810:
            s->cm_sysconfig = value & 0x1;
            break;
        /* MPU_CM */
        case 0x0904:
            s->cm_clken_pll_mpu = value & 0x7ff;
            omap3_cm_mpu_update(s);
            break;
        case 0x0934:
            s->cm_autoidle_pll_mpu = value & 0x7;
            break;
        case 0x0940:
            s->cm_clksel1_pll_mpu = value & 0x3fff7f;
            omap3_cm_mpu_update(s);
            break;
        case 0x0944:
            s->cm_clksel2_pll_mpu = value & 0x1f;
            omap3_cm_mpu_update(s);
            break;
        case 0x0948:
            s->cm_clkstctrl_mpu = value & 0x3;
            break;
        /* CORE_CM */
        case 0xa00:
            s->cm_fclken1_core = value & 0x437ffe00;
            omap3_cm_fclken1_core_update(s);
            break;
        case 0xa04:
            /* TODO: check if modifying this has any effect */
            s->cm_fclken2_core = value;
            break;
        case 0xa08:
            s->cm_fclken3_core = value & 0x7;
            /* TODO: EN_USBTLL, EN_TS */
            break;
        case 0xa10:
            s->cm_iclken1_core = value & 0x637ffed2;
            omap3_cm_iclken1_core_update(s);
            break;
        case 0xa14:
            s->cm_iclken2_core = value & 0x1f;
            break;
        case 0xa18:
            s->cm_iclken3_core = value & 0x4;
            s->cm_idlest3_core = 0xd & ~(s->cm_iclken3_core & 4);
            break;
        case 0xa30:
            s->cm_autoidle1_core = value & 0x7ffffed0;
            break;
        case 0xa34:
            s->cm_autoidle2_core = value & 0x1f;
            break;
        case 0xa38:
            s->cm_autoidle3_core = value & 0x2;
            break;
        case 0xa40:
            if (cpu_is_omap3430(s->mpu)) {
                value = (value & 0xcf) | 0x1100;
            } else { /* omap3630 */
                value = (value & 0x30cf) | 0x100;
            }
            s->cm_clksel_core = value;
            omap3_cm_gp10gp11_update(s);
            omap3_cm_l3l4iclk_update(s);
            break;
        case 0xa48:
            s->cm_clkstctrl_core = value & 0xf;
            break;
        /* SGX_CM */
        case 0xb00: s->cm_fclken_sgx = value & 0x2; break;
        case 0xb10: s->cm_iclken_sgx = value & 0x1; break;
        case 0xb40: s->cm_clksel_sgx = value; break; /* TODO: SGX clock */
        case 0xb44: s->cm_sleepdep_sgx = value & 0x2; break;
        case 0xb48: s->cm_clkstctrl_sgx = value & 0x3; break;
        /* WKUP_CM */
        case 0xc00:
            s->cm_fclken_wkup = value & 0x2e9;
            omap_clk_onoff(omap_findclk(s->mpu, "omap3_gp1_fclk"),
                           s->cm_fclken_wkup & 1);
            /* TODO: EN_GPIO1 */
            /* TODO: EN_WDT2 */
            break;
        case 0xc10:
            s->cm_iclken_wkup = value & 0x23f;
            omap_clk_onoff(omap_findclk(s->mpu, "omap3_wkup_l4_iclk"),
                           s->cm_iclken_wkup ? 1 : 0);
            break;
        case 0xc30: s->cm_autoidle_wkup = value & 0x23f; break;
        case 0xc40:
            s->cm_clksel_wkup = value & 0x7f;
            omap3_cm_clksel_wkup_update(s);
            break;
        /* Clock_Control_Reg_CM */
        case 0xd00:
            s->cm_clken_pll = value & 0xffff17ff;
            omap3_cm_dpll3_update(s);
            omap3_cm_dpll4_update(s);
            break;
        case 0xd04:
            s->cm_clken2_pll = value & 0x7ff;
            omap3_cm_dpll5_update(s);
            break;
        case 0xd30: s->cm_autoidle_pll = value & 0x3f; break;
        case 0xd34: s->cm_autoidle2_pll = value & 0x7; break;
        case 0xd40:
            s->cm_clksel1_pll = value & 0xffffbffc;
            omap3_cm_dpll3_update(s);
            omap3_cm_48m_update(s);
            /* TODO: 96m and 54m update */
            break;
        case 0xd44:
            if (cpu_is_omap3430(s->mpu)) {
                value &= 0x7ff7f;
            } else { /* omap3630 */
                value &= 0xffefff7f;
            }
            s->cm_clksel2_pll = value;
            omap3_cm_dpll4_update(s);
            break;
        case 0xd48:
            s->cm_clksel3_pll = value & 0x1f;
            omap3_cm_dpll4_update(s);
            break;
        case 0xd4c:
            s->cm_clksel4_pll = value & 0x7ff7f;
            omap3_cm_dpll5_update(s);
            break;
        case 0xd50:
            s->cm_clksel5_pll = value & 0x1f;
            omap3_cm_dpll5_update(s);
            break;
        case 0xd70:
            s->cm_clkout_ctrl = value & 0xbb;
            omap3_cm_clkout2_update(s);
            break;
        /* DSS_CM */
        case 0xe00: s->cm_fclken_dss = value & 0x7; break;
        case 0xe10: s->cm_iclken_dss = value & 0x1; break;
        case 0xe30: s->cm_autoidle_dss = value & 0x1; break;
        case 0xe40:
            if (cpu_is_omap3430(s->mpu)) {
                value &= 0x1f1f;
            } else { /* omap3630 */
                value &= 0x3f3f;
            }
            s->cm_clksel_dss = value;
            omap3_cm_dpll4_update(s);
            break;
        case 0xe44: s->cm_sleepdep_dss = value & 0x7; break;
        case 0xe48: s->cm_clkstctrl_dss = value & 0x3; break;
        /* CAM_CM */
        case 0xf00: s->cm_fclken_cam = value & 0x3; break;
        case 0xf10: s->cm_iclken_cam = value & 0x1; break;
        case 0xf30: s->cm_autoidle_cam = value & 0x1; break;
        case 0xf40:
            if (cpu_is_omap3430(s->mpu)) {
                value &= 0x1f;
            } else { /* omap3630 */
                value &= 0x3f;
            }
            s->cm_clksel_cam = value;
            omap3_cm_dpll4_update(s);
            break;
        case 0xf44: s->cm_sleepdep_cam = value & 0x2; break;
        case 0xf48: s->cm_clkstctrl_cam = value & 0x3; break;
        /* PER_CM */
        case 0x1000:
            if (cpu_is_omap3430(s->mpu)) {
                value &= 0x3ffff;
            } else { /* omap3630 */
                value &= 0x7ffff;
            }
            s->cm_fclken_per = value;
            break;
        case 0x1010:
            if (cpu_is_omap3430(s->mpu)) {
                value &= 0x3ffff;
            } else { /* omap3630 */
                value &= 0x7ffff;
            }
            s->cm_iclken_per = value;
            break;
        case 0x1030:
            if (cpu_is_omap3430(s->mpu)) {
                value &= 0x3ffff;
            } else { /* omap3630 */
                value &= 0x7ffff;
            }
            s->cm_autoidle_per = value;
            break;
        case 0x1040:
            s->cm_clksel_per = value & 0xff;
            omap3_cm_per_gptimer_update(s);
            break;
        case 0x1044: s->cm_sleepdep_per = value & 0x6; break;
        case 0x1048: s->cm_clkstctrl_per = value &0x7; break;
        /* EMU_CM */
        case 0x1140:
            if (cpu_is_omap3430(s->mpu)) {
                value &= 0x1f1f3fff;
            } else { /* omap3630 */
                value &= 0x3f3f3fff;
            }
            s->cm_clksel1_emu = value;
            omap3_cm_dpll3_update(s);
            omap3_cm_dpll4_update(s);
            break;
        case 0x1148: s->cm_clkstctrl_emu = value & 0x3; break;
        case 0x1150:
            s->cm_clksel2_emu = value & 0xfff7f;
            omap3_cm_dpll3_update(s);
            break;
        case 0x1154:
            s->cm_clksel3_emu = value & 0xfff7f;
            omap3_cm_dpll4_update(s);
            break;
        /* Global_Reg_CM */
        case 0x129c: s->cm_polctrl = value & 0x1; break;
        /* NEON_CM */
        case 0x1348: s->cm_clkstctrl_neon = value & 0x3; break;
        /* USBHOST_CM */
        case 0x1400: s->cm_fclken_usbhost = value & 0x3; break;
        case 0x1410: s->cm_iclken_usbhost = value & 0x1; break;
        case 0x1430: s->cm_autoidle_usbhost = value & 0x1; break;
        case 0x1444: s->cm_sleepdep_usbhost = value & 0x6; break;
        case 0x1448: s->cm_clkstctrl_usbhost = value & 0x3; break;
        /* unknown */
        default:
            OMAP_BAD_REGV(addr, value);
            break;
    }
}

static const MemoryRegionOps omap3_cm_ops = {
    .old_mmio = {
        .read = {
            omap_badwidth_read32,
            omap_badwidth_read32,
            omap3_cm_read,
        },
        .write = {
            omap_badwidth_write32,
            omap_badwidth_write32,
            omap3_cm_write,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static struct omap3_cm_s *omap3_cm_init(struct omap_target_agent_s *ta,
                                        qemu_irq mpu_int, qemu_irq dsp_int,
                                        qemu_irq iva_int,
                                        struct omap_mpu_state_s *mpu)
{
    struct omap3_cm_s *s = (struct omap3_cm_s *) g_malloc0(sizeof(*s));

    s->irq[0] = mpu_int;
    s->irq[1] = dsp_int;
    s->irq[2] = iva_int;
    s->mpu = mpu;
    omap3_cm_reset(s);

    memory_region_init_io(&s->iomem1, &omap3_cm_ops, s,
                          "omap3_cm", omap_l4_region_size(ta, 0));
    memory_region_init_alias(&s->iomem2, "omap3_cm2", &s->iomem1, 0,
                             omap_l4_region_size(ta, 1));
    omap_l4_attach(ta, 0, &s->iomem1);
    omap_l4_attach(ta, 1, &s->iomem2);

    return s;
}

#define OMAP3_SEC_WDT          1
#define OMAP3_MPU_WDT         2
#define OMAP3_IVA2_WDT        3
/*omap3 watchdog timer*/
struct omap3_wdt_s
{
    qemu_irq irq;               /*IVA2 IRQ */
    MemoryRegion iomem;
    struct omap_mpu_state_s *mpu;
    omap_clk clk;
    QEMUTimer *timer;

    int active;
    int64_t rate;
    int64_t time;
    //int64_t ticks_per_sec;

    uint32_t wd_sysconfig;
    uint32_t wisr;
    uint32_t wier;
    uint32_t wclr;
    uint32_t wcrr;
    uint32_t wldr;
    uint32_t wtgr;
    uint32_t wspr;

    /*pre and ptv in wclr */
    uint32_t pre;
    uint32_t ptv;
    //uint32_t val;

    uint16_t writeh;            /* LSB */
    uint16_t readh;             /* MSB */
};

static inline void omap3_wdt_timer_update(struct omap3_wdt_s *wdt_timer)
{
    int64_t expires;
    if (wdt_timer->active) {
        expires = muldiv64(0xffffffffll - wdt_timer->wcrr,
                           get_ticks_per_sec(), wdt_timer->rate);
        qemu_mod_timer(wdt_timer->timer, wdt_timer->time + expires);
    } else {
        qemu_del_timer(wdt_timer->timer);
    }
}

static void omap3_wdt_clk_setup(struct omap3_wdt_s *timer)
{
    /*TODO: Add irq as user to clk */
}

static inline uint32_t omap3_wdt_timer_read(struct omap3_wdt_s *timer)
{
    uint64_t distance;

    if (timer->active) {
        distance = qemu_get_clock_ns(vm_clock) - timer->time;
        distance = muldiv64(distance, timer->rate, get_ticks_per_sec());

        if (distance >= 0xffffffff - timer->wcrr) {
            return 0xffffffff;
        } else {
            return timer->wcrr + distance;
        }
    } else {
        return timer->wcrr;
    }
}

/*
static inline void omap3_wdt_timer_sync(struct omap3_wdt_s *timer)
{
    if (timer->active) {
        timer->val = omap3_wdt_timer_read(timer);
        timer->time = qemu_get_clock_ns(vm_clock);
    }
}*/

static void omap3_wdt_reset(struct omap3_wdt_s *s, int wdt_index)
{
    s->wd_sysconfig = 0x0;
    s->wisr = 0x0;
    s->wier = 0x0;
    s->wclr = 0x20;
    s->wcrr = 0x0;
    switch (wdt_index) {
        case OMAP3_MPU_WDT:
        case OMAP3_IVA2_WDT:
            s->wldr = 0xfffb0000;
            break;
        case OMAP3_SEC_WDT:
            s->wldr = 0xffa60000;
            break;
        default:
            break;
    }
    s->wtgr = 0x0;
    s->wspr = 0x0;

    switch (wdt_index) {
        case OMAP3_SEC_WDT:
        case OMAP3_MPU_WDT:
            s->active = 1;
            break;
        case OMAP3_IVA2_WDT:
            s->active = 0;
            break;
        default:
            break;
    }
    s->pre = s->wclr & (1 << 5);
    s->ptv = (s->wclr & 0x1c) >> 2;
    s->rate = omap_clk_getrate(s->clk) >> (s->pre ? s->ptv : 0);

    s->active = 1;
    s->time = qemu_get_clock_ns(vm_clock);
    omap3_wdt_timer_update(s);
}

static uint32_t omap3_wdt_read32(void *opaque, hwaddr addr,
                                 int wdt_index)
{
    struct omap3_wdt_s *s = (struct omap3_wdt_s *) opaque;

    switch (addr) {
        case 0x00: return 0x31;
        case 0x10: return s->wd_sysconfig;
        case 0x14: return 0x01; /* RESETDONE */
        case 0x18: return s->wisr;
        case 0x1c: return s->wier;
        case 0x24: return s->wclr;
        case 0x28: /* WCRR */
            s->wcrr = omap3_wdt_timer_read(s);
            s->time = qemu_get_clock_ns(vm_clock);
            return s->wcrr;
        case 0x2c: return s->wldr;
        case 0x30: return s->wtgr;
        case 0x34: return 0;
        case 0x48: return s->wspr;
        default: break;
    }
    hw_error("%s: unknown register offset 0x%08x", __FUNCTION__,
             (uint32_t)addr);
    return 0;
}

static uint32_t omap3_mpu_wdt_read16(void *opaque, hwaddr addr)
{
    struct omap3_wdt_s *s = (struct omap3_wdt_s *) opaque;
    uint32_t ret;

    if (addr & 2)
        return s->readh;

    ret = omap3_wdt_read32(opaque, addr, OMAP3_MPU_WDT);
    s->readh = ret >> 16;
    return ret & 0xffff;
}

static uint32_t omap3_mpu_wdt_read32(void *opaque, hwaddr addr)
{
    return omap3_wdt_read32(opaque, addr, OMAP3_MPU_WDT);
}

static void omap3_wdt_write32(void *opaque, hwaddr addr,
                              uint32_t value, int wdt_index)
{
    struct omap3_wdt_s *s = (struct omap3_wdt_s *) opaque;

    switch (addr) {
    case 0x14: /* WD_SYSSTATUS */
    case 0x34: /* WWPS */
        OMAP_RO_REGV(addr, value);
        break;
    case 0x10: /* WD_SYSCONFIG */
        if (value & 2) { /* SOFTRESET */
            omap3_wdt_reset(s, wdt_index);
        } else {
            s->wd_sysconfig = value & 0x33d;
        }
        break;
    case 0x18: /* WISR */
        s->wisr &= ~(value & 0x1);
        break;
    case 0x1c: /* WIER */
        s->wier = value & 0x1;
        break;
    case 0x24: /* WCLR */
        s->wclr = value & 0x3c;
        break;
    case 0x28: /* WCRR */
        s->wcrr = value;
        s->time = qemu_get_clock_ns(vm_clock);
        omap3_wdt_timer_update(s);
        break;
    case 0x2c: /* WLDR */
        s->wldr = value; /* It will take effect after next overflow */
        break;
    case 0x30: /* WTGR */
        if (value != s->wtgr) {
            s->wtgr = value;
            s->wcrr = s->wldr;
            s->pre = s->wclr & (1 << 5);
            s->ptv = (s->wclr & 0x1c) >> 2;
            s->rate = omap_clk_getrate(s->clk) >> (s->pre ? s->ptv : 0);
            s->time = qemu_get_clock_ns(vm_clock);
            omap3_wdt_timer_update(s);
        }
        break;
    case 0x48: /* WSPR */
        if (((value & 0xffff) == 0x5555) && ((s->wspr & 0xffff) == 0xaaaa)) {
            s->active = 0;
            s->wcrr = omap3_wdt_timer_read(s);
            omap3_wdt_timer_update(s);
        }
        if (((value & 0xffff) == 0x4444) && ((s->wspr & 0xffff) == 0xbbbb)) {
            s->active = 1;
            s->time = qemu_get_clock_ns(vm_clock);
            omap3_wdt_timer_update(s);
        }
        s->wspr = value;
        break;
    default:
        hw_error("%s: unknown register offset 0x%08x", __FUNCTION__,
                 (uint32_t)addr);
        break;
    }
}

static void omap3_mpu_wdt_write16(void *opaque, hwaddr addr,
                                  uint32_t value)
{
    struct omap3_wdt_s *s = (struct omap3_wdt_s *) opaque;

    if (addr & 2)
        return omap3_wdt_write32(opaque, addr, (value << 16) | s->writeh,
                                 OMAP3_MPU_WDT);
    else
        s->writeh = (uint16_t) value;
}

static void omap3_mpu_wdt_write32(void *opaque, hwaddr addr,
                                  uint32_t value)
{
    omap3_wdt_write32(opaque, addr, value, OMAP3_MPU_WDT);
}

static const MemoryRegionOps omap3_mpu_wdt_ops = {
    .old_mmio = {
        .read = {
            omap_badwidth_read32,
            omap3_mpu_wdt_read16,
            omap3_mpu_wdt_read32,
        },
        .write = {
            omap_badwidth_write32,
            omap3_mpu_wdt_write16,
            omap3_mpu_wdt_write32,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void omap3_mpu_wdt_timer_tick(void *opaque)
{
    struct omap3_wdt_s *wdt_timer = (struct omap3_wdt_s *) opaque;

    /*TODO:Sent reset pulse to PRCM */
    wdt_timer->wcrr = wdt_timer->wldr;

    /*after overflow, generate the new wdt_timer->rate */
    wdt_timer->pre = wdt_timer->wclr & (1 << 5);
    wdt_timer->ptv = (wdt_timer->wclr & 0x1c) >> 2;
    wdt_timer->rate =
        omap_clk_getrate(wdt_timer->clk) >> (wdt_timer->pre ? wdt_timer->
                                             ptv : 0);

    wdt_timer->time = qemu_get_clock_ns(vm_clock);
    omap3_wdt_timer_update(wdt_timer);
}

static struct omap3_wdt_s *omap3_mpu_wdt_init(struct omap_target_agent_s *ta,
                                              qemu_irq irq, omap_clk fclk,
                                              omap_clk iclk,
                                              struct omap_mpu_state_s *mpu)
{
    struct omap3_wdt_s *s = (struct omap3_wdt_s *) g_malloc0(sizeof(*s));

    s->irq = irq;
    s->clk = fclk;
    s->timer = qemu_new_timer_ns(vm_clock, omap3_mpu_wdt_timer_tick, s);

    omap3_wdt_reset(s, OMAP3_MPU_WDT);
    if (irq != NULL)
        omap3_wdt_clk_setup(s);

    memory_region_init_io(&s->iomem, &omap3_mpu_wdt_ops, s,
                          "omap3_mpu_wdt", omap_l4_region_size(ta, 0));
    omap_l4_attach(ta, 0, &s->iomem);

    return s;
}

struct omap3_scm_s {
    struct omap_mpu_state_s *mpu;
    MemoryRegion iomem;

	uint8 interface[48];     /*0x4800 2000*/
	uint8 padconfs[576];     /*0x4800 2030*/
	uint32 general[228];     /*0x4800 2270*/
	uint8 mem_wkup[1024];    /*0x4800 2600*/
	uint8 padconfs_wkup[96]; /*0x4800 2a00*/
	uint32 general_wkup[8];  /*0x4800 2a60*/
};

#define PADCONFS_VALUE(wakeup0,wakeup1,offmode0,offmode1, \
						inputenable0,inputenable1,pupd0,pupd1,muxmode0,muxmode1,offset) \
	do { \
		 *(padconfs+offset/4) = (wakeup0 <<14)|(offmode0<<9)|(inputenable0<<8)|(pupd0<<3)|(muxmode0); \
		 *(padconfs+offset/4) |= (wakeup1 <<30)|(offmode1<<25)|(inputenable1<<24)|(pupd1<<19)|(muxmode1<<16); \
} while (0)


static void omap3_scm_reset(struct omap3_scm_s *s)
{
    uint32 *padconfs;
    padconfs = (uint32 *)(s->padconfs);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x0);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x4);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x8);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0xc);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x10);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x14);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x18);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x1c);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x20);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x24);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x28);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x2c);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x30);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x34);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x38);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x3c);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x40);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x44);
    PADCONFS_VALUE(0,0,0,0,1,1,0,1,0,7,0x48);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x4c);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x50);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x54);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x58);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,0,0x5c);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x60);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x64);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x68);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x6c);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x70);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x74);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x78);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x7c);
    PADCONFS_VALUE(0,0,0,0,1,1,0,3,0,7,0x80);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x84);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x88);
    PADCONFS_VALUE(0,0,0,0,1,1,3,0,7,0,0x8c);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x90);
    PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x94);
    PADCONFS_VALUE(0,0,0,0,1,1,1,0,7,0,0x98);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,7,0x9c);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0xa0);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0xa4);
    PADCONFS_VALUE(0,0,0,0,1,1,3,1,7,7,0xa8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xac);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xb0);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xb4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xb8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xbc);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xc0);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xc4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xc8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xcc);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xd0);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xd4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xd8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xdc);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xe0);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xe4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xe8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xec);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xf0);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xf4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xf8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0xfc);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x100);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x104);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x108);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x10c);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x110);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x114);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x118);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x11c);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x120);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x124);
    PADCONFS_VALUE(0,0,0,0,1,1,1,3,7,7,0x128);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x12c);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x130);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x134);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x138);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x13c);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x140);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x144);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x148);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x14c);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x150);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x154);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x158);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x15c);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x160);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x164);
    PADCONFS_VALUE(0,0,0,0,1,1,1,3,7,7,0x168);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x16c);
    PADCONFS_VALUE(0,0,0,0,1,1,3,1,7,7,0x170);
    PADCONFS_VALUE(0,0,0,0,1,1,3,1,7,7,0x174);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x178);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x17c);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x180);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x184);
    PADCONFS_VALUE(0,0,0,0,1,1,1,3,7,7,0x188);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x18c);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x190);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x194);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x198);
    PADCONFS_VALUE(0,0,0,0,1,1,1,3,7,7,0x19c);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x1a0);
    PADCONFS_VALUE(0,0,0,0,1,1,3,1,7,7,0x1a4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x1a8);
    PADCONFS_VALUE(0,0,0,0,1,1,3,1,7,7,0x1ac);
    PADCONFS_VALUE(0,0,0,0,1,1,3,1,7,7,0x1b0);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1b4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1b8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1bc);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1c0);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1c4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1c8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1cc);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1d0);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1d4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1d8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1dc);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1e0);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1e4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1e8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1ec);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1f0);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1f4);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1f8);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1fc);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x200);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x204);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x208);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x20c);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x210);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x214);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x218);
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x21c);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x220);
    PADCONFS_VALUE(0,0,0,0,1,1,3,1,0,0,0x224);
    PADCONFS_VALUE(0,0,0,0,1,1,0,1,0,0,0x228);
    PADCONFS_VALUE(0,0,0,0,1,1,0,1,0,0,0x22c);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x230);
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,7,7,0x234);

	padconfs = (uint32 *)(s->general);
    memset(s->general, 0, sizeof(s->general));
	s->general[0x01] = 0x4000000;  /* CONTROL_DEVCONF_0 */
	s->general[0x1c] = 0x1;        /* 0x480022e0?? */
    s->general[0x20] = 0x30f;      /* CONTROL_STATUS:
                                    * - device type  = GP Device
                                    * - sys_boot:6   = oscillator bypass mode
                                    * - sys_boot:0-5 = NAND, USB, UART3, MMC1*/
	s->general[0x75] = 0x7fc0;     /* CONTROL_PROG_IO0 */
	s->general[0x76] = 0xaa;       /* CONTROL_PROG_IO1 */
	s->general[0x7c] = 0x2700;     /* CONTROL_SDRC_SHARING */
	s->general[0x7d] = 0x300000;   /* CONTROL_SDRC_MCFG0 */
	s->general[0x7e] = 0x300000;   /* CONTROL_SDRC_MCFG1 */
	s->general[0x81] = 0xffff;     /* CONTROL_MODEM_GPMC_DT_FW_REQ_INFO */
	s->general[0x82] = 0xffff;     /* CONTROL_MODEM_GPMC_DT_FW_RD */
	s->general[0x83] = 0xffff;     /* CONTROL_MODEM_GPMC_DT_FW_WR */
	s->general[0x84] = 0x6;        /* CONTROL_MODEM_GPMC_BOOT_CODE */
	s->general[0x85] = 0xffffffff; /* CONTROL_MODEM_SMS_RG_ATT1 */
	s->general[0x86] = 0xffff;     /* CONTROL_MODEM_SMS_RG_RDPERM1 */
	s->general[0x87] = 0xffff;     /* CONTROL_MODEM_SMS_RG_WRPERM1 */
	s->general[0x88] = 0x1;        /* CONTROL_MODEM_D2D_FW_DEBUG_MODE */
	s->general[0x8b] = 0xffffffff; /* CONTROL_DPF_OCM_RAM_FW_REQINFO */
	s->general[0x8c] = 0xffff;     /* CONTROL_DPF_OCM_RAM_FW_WR */
	s->general[0x8e] = 0xffff;     /* CONTROL_DPF_REGION4_GPMC_FW_REQINFO */
	s->general[0x8f] = 0xffff;     /* CONTROL_DPF_REGION4_GPMC_FW_WR */
	s->general[0x91] = 0xffff;     /* CONTROL_DPF_REGION1_IVA2_FW_REQINFO */
	s->general[0x92] = 0xffff;     /* CONTROL_DPF_REGION1_IVA2_FW_WR */
	s->general[0xac] = 0x109;      /* CONTROL_PBIAS_LITE */
	s->general[0xb2] = 0xffff;     /* CONTROL_DPF_MAD2D_FW_ADDR_MATCH */
	s->general[0xb3] = 0xffff;     /* CONTROL_DPF_MAD2D_FW_REQINFO */
	s->general[0xb4] = 0xffff;     /* CONTROL_DPF_MAD2D_FW_WR */
	PADCONFS_VALUE(0,0,0,0,1,1,3,3,4,4,0x368); /* PADCONF_ETK_CLK */
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,4,4,0x36c); /* PADCONF_ETK_D0 */
    PADCONFS_VALUE(0,0,0,0,1,1,3,3,4,4,0x370); /* PADCONF_ETK_D2 */
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,4,4,0x374); /* PADCONF_ETK_D4 */
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,4,4,0x378); /* PADCONF_ETK_D6 */
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,4,4,0x37c); /* PADCONF_ETK_D8 */
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,4,4,0x380); /* PADCONF_ETK_D10 */
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,4,4,0x384); /* PADCONF_ETK_D12 */
    PADCONFS_VALUE(0,0,0,0,1,1,1,1,4,4,0x388); /* PADCONF_ETK_D14 */

	padconfs = (uint32 *)(s->padconfs_wkup);
	PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x0);
	PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x4);
	PADCONFS_VALUE(0,0,0,0,1,1,3,0,0,0,0x8);
	PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0xc);
	PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x10);
	PADCONFS_VALUE(0,0,0,0,1,1,0,0,0,0,0x14);
	PADCONFS_VALUE(0,0,0,0,1,1,1,1,7,7,0x18);
	PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x1c);
	PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x20);
	PADCONFS_VALUE(0,0,0,0,1,1,3,3,0,0,0x24);
	PADCONFS_VALUE(0,0,0,0,1,1,1,1,0,0,0x2c);

	s->general_wkup[0] = 0x66ff; /* 0x48002A60?? */
}

static uint32_t omap3_scm_read8(void *opaque, hwaddr addr)
{
    struct omap3_scm_s *s = (struct omap3_scm_s *) opaque;
    uint8_t* temp;
	
    switch (addr) {
        case 0x000 ... 0x02f: return s->interface[addr];
        case 0x030 ... 0x26f: return s->padconfs[addr - 0x30];
        case 0x270 ... 0x5ff: temp = (uint8_t *)s->general; return temp[addr - 0x270];
        case 0x600 ... 0x9ff: return s->mem_wkup[addr - 0x600];
        case 0xa00 ... 0xa5f: return s->padconfs_wkup[addr - 0xa00];
        case 0xa60 ... 0xa7f: temp = (uint8_t *)s->general_wkup; return temp[addr - 0xa60];
        default: break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static uint32_t omap3_scm_read16(void *opaque, hwaddr addr)
{
    uint32_t v;
    v = omap3_scm_read8(opaque, addr);
    v |= omap3_scm_read8(opaque, addr + 1) << 8;
    return v;
}

static uint32_t omap3_scm_read32(void *opaque, hwaddr addr)
{
    uint32_t v;
    v = omap3_scm_read8(opaque, addr);
    v |= omap3_scm_read8(opaque, addr + 1) << 8;
    v |= omap3_scm_read8(opaque, addr + 2) << 16;
    v |= omap3_scm_read8(opaque, addr + 3) << 24;
    TRACE_SCM(OMAP_FMT_plx " = 0x%08x", addr, v);
    return v;
}

static void omap3_scm_write8(void *opaque, hwaddr addr,
                             uint32_t value)
{
    struct omap3_scm_s *s = (struct omap3_scm_s *) opaque;
    uint8_t* temp;

    switch (addr) {
        case 0x000 ... 0x02f: s->interface[addr] = value; break;
        case 0x030 ... 0x26f: s->padconfs[addr-0x30] = value; break;
        case 0x270 ... 0x5ff: temp = (uint8_t *)s->general; temp[addr-0x270] = value; break;
        case 0x600 ... 0x9ff: s->mem_wkup[addr-0x600] = value; break;
        case 0xa00 ... 0xa5f: s->padconfs_wkup[addr-0xa00] = value; break;
        case 0xa60 ... 0xa7f: temp = (uint8_t *)s->general_wkup; temp[addr-0xa60] = value; break;
        default: OMAP_BAD_REGV(addr, value); break;
    }
}

static void omap3_scm_write16(void *opaque, hwaddr addr,
                              uint32_t value)
{
    omap3_scm_write8(opaque, addr + 0, (value) & 0xff);
    omap3_scm_write8(opaque, addr + 1, (value >> 8) & 0xff);
}

static void omap3_scm_write32(void *opaque, hwaddr addr,
                              uint32_t value)
{
    TRACE_SCM(OMAP_FMT_plx " = 0x%08x", addr, value);
    omap3_scm_write8(opaque, addr + 0, (value) & 0xff);
    omap3_scm_write8(opaque, addr + 1, (value >> 8) & 0xff);
    omap3_scm_write8(opaque, addr + 2, (value >> 16) & 0xff);
    omap3_scm_write8(opaque, addr + 3, (value >> 24) & 0xff);
}

static const MemoryRegionOps omap3_scm_ops = {
    .old_mmio = {
        .read = {
            omap3_scm_read8,
            omap3_scm_read16,
            omap3_scm_read32,
        },
        .write = {
            omap3_scm_write8,
            omap3_scm_write16,
            omap3_scm_write32,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static struct omap3_scm_s *omap3_scm_init(struct omap_target_agent_s *ta,
                                          struct omap_mpu_state_s *mpu)
{
    struct omap3_scm_s *s = (struct omap3_scm_s *) g_malloc0(sizeof(*s));

    s->mpu = mpu;

    omap3_scm_reset(s);

    memory_region_init_io(&s->iomem, &omap3_scm_ops, s,
                          "omap3_scm", omap_l4_region_size(ta, 0));
    omap_l4_attach(ta, 0, &s->iomem);

    return s;
}

/*dummy SDRAM Memory Scheduler emulation*/
struct omap3_sms_s
{
    struct omap_mpu_state_s *mpu;
    MemoryRegion iomem;

    uint32 sms_sysconfig;
    uint32 sms_sysstatus;
    uint32 sms_rg_att[8];
    uint32 sms_rg_rdperm[8];
    uint32 sms_rg_wrperm[8];
    uint32 sms_rg_start[7];
    uint32 sms_rg_end[7];
    uint32 sms_security_control;
    uint32 sms_class_arbiter0;
    uint32 sms_class_arbiter1;
    uint32 sms_class_arbiter2;
    uint32 sms_interclass_arbiter;
    uint32 sms_class_rotation[3];
    uint32 sms_err_addr;
    uint32 sms_err_type;
    uint32 sms_pow_ctrl;
    uint32 sms_rot_control[12];
    uint32 sms_rot_size[12];
    uint32 sms_rot_physical_ba[12];
};

static uint64_t omap3_sms_read(void *opaque, hwaddr addr,
                                 unsigned size)
{
    struct omap3_sms_s *s = (struct omap3_sms_s *) opaque;

    TRACE_SMS("addr = 0x%08x", (uint32_t)addr);
    switch (addr) {
    case 0x10:
    	return s->sms_sysconfig;
    case 0x14:
    	return s->sms_sysstatus;
    case 0x48:
    case 0x68:
    case 0x88:
    case 0xa8:
    case 0xc8:
    case 0xe8:
    case 0x108:
    case 0x128:
    	return s->sms_rg_att[(addr-0x48)/0x20];
    case 0x50:
    case 0x70:
    case 0x90:
    case 0xb0:
    case 0xd0:
    case 0xf0:
    case 0x110:
    case 0x130:
    	return s->sms_rg_rdperm[(addr-0x50)/0x20];
    case 0x58:
    case 0x78:
    case 0x98:
    case 0xb8:
    case 0xd8:
    case 0xf8:
    case 0x118:
    	return s->sms_rg_wrperm[(addr-0x58)/0x20];
    case 0x60:
    case 0x80:
    case 0xa0:
    case 0xc0:
    case 0xe0:
    case 0x100:
    case 0x120:
    	return s->sms_rg_start[(addr-0x60)/0x20];

    case 0x64:
    case 0x84:
    case 0xa4:
    case 0xc4:
    case 0xe4:
    case 0x104:
    case 0x124:
    	return s->sms_rg_end[(addr-0x64)/0x20];
    case 0x140:
    	return s->sms_security_control;
    case 0x150:
    	return s->sms_class_arbiter0;
	case 0x154:
		return s->sms_class_arbiter1;
	case 0x158:
		return s->sms_class_arbiter2;
	case 0x160:
		return s->sms_interclass_arbiter;
	case 0x164:
	case 0x168:
	case 0x16c:
		return s->sms_class_rotation[(addr-0x164)/4];
	case 0x170:
		return s->sms_err_addr;
	case 0x174:
		return s->sms_err_type;
	case 0x178:
		return s->sms_pow_ctrl;
	case 0x180:
	case 0x190:
	case 0x1a0:
	case 0x1b0:
	case 0x1c0:
	case 0x1d0:
	case 0x1e0:
	case 0x1f0:
	case 0x200:
	case 0x210:
	case 0x220:
	case 0x230:
		return s->sms_rot_control[(addr-0x180)/0x10];
	case 0x184:
	case 0x194:
	case 0x1a4:
	case 0x1b4:
	case 0x1c4:
	case 0x1d4:
	case 0x1e4:
	case 0x1f4:
	case 0x204:
	case 0x214:
	case 0x224:
	case 0x234:
		return s->sms_rot_size[(addr-0x184)/0x10];

	case 0x188:
	case 0x198:
	case 0x1a8:
	case 0x1b8:
	case 0x1c8:
	case 0x1d8:
	case 0x1e8:
	case 0x1f8:
	case 0x208:
	case 0x218:
	case 0x228:
	case 0x238:
		return s->sms_rot_size[(addr-0x188)/0x10];

    default:
        break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap3_sms_write(void *opaque, hwaddr addr,
                            uint64_t value, unsigned size)
{
    struct omap3_sms_s *s = (struct omap3_sms_s *) opaque;
    //int i;

    TRACE_SMS("addr = 0x%08x, value = 0x%08x", (uint32_t)addr, value);
    switch (addr) {
    case 0x14:
    	OMAP_RO_REG(addr);
        return;
    case 0x10:
    	s->sms_sysconfig = value & 0x1f;
    	break;
    
    case 0x48:
    case 0x68:
    case 0x88:
    case 0xa8:
    case 0xc8:
    case 0xe8:
    case 0x108:
    case 0x128:
    	s->sms_rg_att[(addr-0x48)/0x20] = value;
    	break;
    case 0x50:
    case 0x70:
    case 0x90:
    case 0xb0:
    case 0xd0:
    case 0xf0:
    case 0x110:
    case 0x130:
    	s->sms_rg_rdperm[(addr-0x50)/0x20] = value&0xffff;
    	break;
    case 0x58:
    case 0x78:
    case 0x98:
    case 0xb8:
    case 0xd8:
    case 0xf8:
    case 0x118:
    	s->sms_rg_wrperm[(addr-0x58)/0x20] = value&0xffff;
    	break;    	
    case 0x60:
    case 0x80:
    case 0xa0:
    case 0xc0:
    case 0xe0:
    case 0x100:
    case 0x120:
    	s->sms_rg_start[(addr-0x60)/0x20] = value;
    	break;
    case 0x64:
    case 0x84:
    case 0xa4:
    case 0xc4:
    case 0xe4:
    case 0x104:
    case 0x124:
    	s->sms_rg_end[(addr-0x64)/0x20] = value;
    	break;
    case 0x140:
    	s->sms_security_control = value &0xfffffff;
    	break;
    case 0x150:
    	s->sms_class_arbiter0 = value;
    	break;
	case 0x154:
		s->sms_class_arbiter1 = value;
		break;
	case 0x158:
		s->sms_class_arbiter2 = value;
		break;
	case 0x160:
		s->sms_interclass_arbiter = value;
		break;
	case 0x164:
	case 0x168:
	case 0x16c:
		s->sms_class_rotation[(addr-0x164)/4] = value;
		break;
	case 0x170:
		s->sms_err_addr = value;
		break;
	case 0x174:
		s->sms_err_type = value;
		break;
	case 0x178:
		s->sms_pow_ctrl = value;
		break;
	case 0x180:
	case 0x190:
	case 0x1a0:
	case 0x1b0:
	case 0x1c0:
	case 0x1d0:
	case 0x1e0:
	case 0x1f0:
	case 0x200:
	case 0x210:
	case 0x220:
	case 0x230:
		s->sms_rot_control[(addr-0x180)/0x10] = value;
		break;
	case 0x184:
	case 0x194:
	case 0x1a4:
	case 0x1b4:
	case 0x1c4:
	case 0x1d4:
	case 0x1e4:
	case 0x1f4:
	case 0x204:
	case 0x214:
	case 0x224:
	case 0x234:
		s->sms_rot_size[(addr-0x184)/0x10] = value;
		break;

	case 0x188:
	case 0x198:
	case 0x1a8:
	case 0x1b8:
	case 0x1c8:
	case 0x1d8:
	case 0x1e8:
	case 0x1f8:
	case 0x208:
	case 0x218:
	case 0x228:
	case 0x238:
		s->sms_rot_size[(addr-0x188)/0x10] = value;   
		break;
	default:
        OMAP_BAD_REGV(addr, value);
        break;
    }
}

static const MemoryRegionOps omap3_sms_ops = {
    .read = omap3_sms_read,
    .write = omap3_sms_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .valid.min_access_size = 4,
};

static void omap3_sms_reset(struct omap3_sms_s *s)
{
	s->sms_sysconfig = 0x1;
	s->sms_class_arbiter0 = 0x500000;
	s->sms_class_arbiter1 = 0x500;
	s->sms_class_arbiter2 = 0x55000;
	s->sms_interclass_arbiter = 0x400040;
	s->sms_class_rotation[0] = 0x1;
	s->sms_class_rotation[1] = 0x1;
	s->sms_class_rotation[2] = 0x1;
	s->sms_pow_ctrl = 0x80;
}

static struct omap3_sms_s *omap3_sms_init(MemoryRegion *sysmem,
                                          struct omap_mpu_state_s *mpu)
{
    struct omap3_sms_s *s = g_malloc0(sizeof(*s));

    s->mpu = mpu;

    omap3_sms_reset(s);

    memory_region_init_io(&s->iomem, &omap3_sms_ops, s, "omap3_sms", 0x10000);
    memory_region_add_subregion(sysmem, 0x6c000000, &s->iomem);
    return s;
}

static void omap3_reset(void *opaque)
{
    struct omap_mpu_state_s *s = opaque;
    int i;

    cpu_reset(CPU(s->cpu));
    omap_dma_reset(s->dma);
    omap3_cm_reset(s->omap3_cm);
    omap3_prm_reset(s->omap3_prm);
    omap3_wdt_reset(s->omap3_mpu_wdt, OMAP3_MPU_WDT);
    omap3_scm_reset(s->omap3_scm);
    omap3_sms_reset(s->omap3_sms);
    for (i = 0; i < 12; i++) {
        omap_gp_timer_reset(s->gptimer[i]);
    }
    omap_synctimer_reset(s->synctimer);
    omap_sdrc_reset(s->sdrc);
    omap_gpmc_reset(s->gpmc);

    omap3_boot_rom_emu(s);
}

static const struct dma_irq_map omap3_dma_irq_map[] = {
    {0, OMAP_INT_3XXX_SDMA_IRQ0},
    {0, OMAP_INT_3XXX_SDMA_IRQ1},
    {0, OMAP_INT_3XXX_SDMA_IRQ2},
    {0, OMAP_INT_3XXX_SDMA_IRQ3},
};

static int omap3_validate_addr(struct omap_mpu_state_s *s,
                               hwaddr addr)
{
    return 1;
}

struct omap_mpu_state_s *omap3_mpu_init(MemoryRegion *sysmem,
                                        int model,
                                        unsigned long sdram_size,
                                        CharDriverState *chr_uart1,
                                        CharDriverState *chr_uart2,
                                        CharDriverState *chr_uart3,
                                        CharDriverState *chr_uart4)
{
    struct omap_mpu_state_s *s = g_malloc0(sizeof(*s));
    qemu_irq *cpu_irq;
    qemu_irq drqs[4];
    int i;
    SysBusDevice *busdev;
    /* values reported by beagleboard(-xm) hw */
    const unsigned uart_revision = model == omap3430 ? 0x46 : 0x52;

    if (model != omap3430 && model != omap3630) {
        hw_error("%s: invalid cpu model (%d)", __FUNCTION__, model);
    }
    s->mpu_model = model;
    s->cpu = cpu_arm_init("cortex-a8-r2");
    if (!s->cpu) {
        hw_error("%s: Unable to find CPU definition", __FUNCTION__);
    }
    s->sdram_size = sdram_size;
    s->sram_size = OMAP3XXX_SRAM_SIZE;

    /* Clocks */
    omap_clk_init(s);

    /* Memory-mapped stuff */
    memory_region_init_ram(&s->sdram, "omap3_dram", s->sdram_size);
    memory_region_add_subregion(sysmem, OMAP3_Q2_BASE, &s->sdram);
    memory_region_init_ram(&s->sram, "omap3_sram", s->sram_size);
    memory_region_add_subregion(sysmem, OMAP3_SRAM_BASE, &s->sram);

    s->l4 = omap_l4_init(sysmem, OMAP3_L4_BASE, L4A_COUNT, L4ID_COUNT);

    cpu_irq = arm_pic_init_cpu(s->cpu);
    s->ih[0] = qdev_create(NULL, "omap2-intc");
    qdev_prop_set_uint8(s->ih[0], "revision", 0x40);
    qdev_prop_set_ptr(s->ih[0], "fclk", omap_findclk(s, "omap3_mpu_intc_fclk"));
    qdev_prop_set_ptr(s->ih[0], "iclk", omap_findclk(s, "omap3_mpu_intc_iclk"));
    qdev_init_nofail(s->ih[0]);
    busdev = SYS_BUS_DEVICE(s->ih[0]);
    sysbus_connect_irq(busdev, 0, cpu_irq[ARM_PIC_CPU_IRQ]);
    sysbus_connect_irq(busdev, 1, cpu_irq[ARM_PIC_CPU_FIQ]);
    sysbus_mmio_map(busdev, 0, 0x48200000);
    for (i = 0; i < 4; i++) {
        drqs[i] = qdev_get_gpio_in(s->ih[omap3_dma_irq_map[i].ih],
                                   omap3_dma_irq_map[i].intr);
    }
    s->dma = omap3_dma4_init(omap3_l4ta_init(s->l4, L4A_SDMA), s, drqs, 32,
                             omap_findclk(s, "omap3_sdma_fclk"),
                             omap_findclk(s, "omap3_sdma_iclk"));
    s->port->addr_valid = omap3_validate_addr;
    soc_dma_port_add_mem(s->dma, memory_region_get_ram_ptr(&s->sdram),
                         OMAP2_Q2_BASE, s->sdram_size);
    soc_dma_port_add_mem(s->dma, memory_region_get_ram_ptr(&s->sram),
                         OMAP2_SRAM_BASE, s->sram_size);

    s->omap3_cm = omap3_cm_init(omap3_l4ta_init(s->l4, L4A_CM),
                                NULL, NULL, NULL, s);

    s->omap3_prm = omap3_prm_init(omap3_l4ta_init(s->l4, L4A_PRM),
                                  qdev_get_gpio_in(s->ih[0],
                                                   OMAP_INT_3XXX_PRCM_MPU_IRQ),
                                  NULL, s);

    s->omap3_mpu_wdt = omap3_mpu_wdt_init(omap3_l4ta_init(s->l4, L4A_WDTIMER2),
                                          NULL,
                                          omap_findclk(s, "omap3_wkup_32k_fclk"),
                                          omap_findclk(s, "omap3_wkup_l4_iclk"),
                                          s);

    s->omap3_l3 = omap3_l3_init(sysmem, OMAP3_L3_BASE);
    s->omap3_scm = omap3_scm_init(omap3_l4ta_init(s->l4, L4A_SCM), s);

    s->omap3_sms = omap3_sms_init(sysmem, s);

    s->gptimer[0] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER1),
                                       qdev_get_gpio_in(s->ih[0],
                                                        OMAP_INT_3XXX_GPT1_IRQ),
                                       omap_findclk(s, "omap3_gp1_fclk"),
                                       omap_findclk(s, "omap3_wkup_l4_iclk"));
    s->gptimer[1] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER2),
                                       qdev_get_gpio_in(s->ih[0],
                                                        OMAP_INT_3XXX_GPT2_IRQ),
                                       omap_findclk(s, "omap3_gp2_fclk"),
                                       omap_findclk(s, "omap3_per_l4_iclk"));
    s->gptimer[2] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER3),
                                       qdev_get_gpio_in(s->ih[0],
                                                        OMAP_INT_3XXX_GPT3_IRQ),
                                       omap_findclk(s, "omap3_gp3_fclk"),
                                       omap_findclk(s, "omap3_per_l4_iclk"));
    s->gptimer[3] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER4),
                                       qdev_get_gpio_in(s->ih[0],
                                                        OMAP_INT_3XXX_GPT4_IRQ),
                                       omap_findclk(s, "omap3_gp4_fclk"),
                                       omap_findclk(s, "omap3_per_l4_iclk"));
    s->gptimer[4] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER5),
                                       qdev_get_gpio_in(s->ih[0],
                                                        OMAP_INT_3XXX_GPT5_IRQ),
                                       omap_findclk(s, "omap3_gp5_fclk"),
                                       omap_findclk(s, "omap3_per_l4_iclk"));
    s->gptimer[5] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER6),
                                       qdev_get_gpio_in(s->ih[0],
                                                        OMAP_INT_3XXX_GPT6_IRQ),
                                       omap_findclk(s, "omap3_gp6_fclk"),
                                       omap_findclk(s, "omap3_per_l4_iclk"));
    s->gptimer[6] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER7),
                                       qdev_get_gpio_in(s->ih[0],
                                                        OMAP_INT_3XXX_GPT7_IRQ),
                                       omap_findclk(s, "omap3_gp7_fclk"),
                                       omap_findclk(s, "omap3_per_l4_iclk"));
    s->gptimer[7] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER8),
                                       qdev_get_gpio_in(s->ih[0],
                                                        OMAP_INT_3XXX_GPT8_IRQ),
                                       omap_findclk(s, "omap3_gp8_fclk"),
                                       omap_findclk(s, "omap3_per_l4_iclk"));
    s->gptimer[8] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER9),
                                       qdev_get_gpio_in(s->ih[0],
                                                        OMAP_INT_3XXX_GPT9_IRQ),
                                       omap_findclk(s, "omap3_gp9_fclk"),
                                       omap_findclk(s, "omap3_per_l4_iclk"));
    s->gptimer[9] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER10),
                                       qdev_get_gpio_in(s->ih[0],
                                                       OMAP_INT_3XXX_GPT10_IRQ),
                                       omap_findclk(s, "omap3_gp10_fclk"),
                                       omap_findclk(s, "omap3_core_l4_iclk"));
    s->gptimer[10] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER11),
                                       qdev_get_gpio_in(s->ih[0],
                                                       OMAP_INT_3XXX_GPT11_IRQ),
                                       omap_findclk(s, "omap3_gp12_fclk"),
                                       omap_findclk(s, "omap3_core_l4_iclk"));
    s->gptimer[11] = omap_gp_timer_init(omap3_l4ta_init(s->l4, L4A_GPTIMER12),
                                        qdev_get_gpio_in(s->ih[0],
                                                       OMAP_INT_3XXX_GPT12_IRQ),
                                        omap_findclk(s, "omap3_gp12_fclk"),
                                        omap_findclk(s, "omap3_wkup_l4_iclk"));

    s->synctimer = omap_synctimer_init(omap3_l4ta_init(s->l4, L4A_32KTIMER), s,
                                       omap_findclk(s, "omap3_sys_32k"), NULL);

    s->sdrc = omap_sdrc_init(sysmem, 0x6d000000);

    s->gpmc = omap_gpmc_init(s, 0x6e000000,
                             qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_GPMC_IRQ),
                             s->drq[OMAP3XXX_DMA_GPMC]);

    s->uart[0] = qdev_create(NULL, "omap_uart");
    s->uart[0]->id = "uart1";
    qdev_prop_set_uint32(s->uart[0], "mmio_size", 0x1000);
    qdev_prop_set_uint32(s->uart[0], "baudrate",
                         omap_clk_getrate(omap_findclk(s, "omap3_uart1_fclk"))
                         / 16);
    qdev_prop_set_chr(s->uart[0], "chardev", chr_uart1);
    qdev_prop_set_uint32(s->uart[0], "revision", uart_revision);
    qdev_init_nofail(s->uart[0]);
    busdev = SYS_BUS_DEVICE(s->uart[0]);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_UART1_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_UART1_TX]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_UART1_RX]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4,L4A_UART1), 0));

    s->uart[1] = qdev_create(NULL, "omap_uart");
    s->uart[1]->id = "uart2";
    qdev_prop_set_uint32(s->uart[1], "mmio_size", 0x1000);
    qdev_prop_set_uint32(s->uart[1], "baudrate",
                         omap_clk_getrate(omap_findclk(s, "omap3_uart2_fclk"))
                         / 16);
    qdev_prop_set_chr(s->uart[1], "chardev", chr_uart2);
    qdev_prop_set_uint32(s->uart[1], "revision", uart_revision);
    qdev_init_nofail(s->uart[1]);
    busdev = SYS_BUS_DEVICE(s->uart[1]);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_UART2_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_UART2_TX]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_UART2_RX]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4,L4A_UART2), 0));

    s->uart[2] = qdev_create(NULL, "omap_uart");
    s->uart[2]->id = "uart3";
    qdev_prop_set_uint32(s->uart[2], "mmio_size", 0x1000);
    qdev_prop_set_uint32(s->uart[2], "baudrate",
                         omap_clk_getrate(omap_findclk(s, "omap3_uart3_fclk"))
                         / 16);
    qdev_prop_set_chr(s->uart[2], "chardev", chr_uart3);
    qdev_prop_set_uint32(s->uart[2], "revision", uart_revision);
    qdev_init_nofail(s->uart[2]);
    busdev = SYS_BUS_DEVICE(s->uart[2]);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_UART3_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_UART3_TX]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_UART3_RX]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4,L4A_UART3), 0));

    if (model == omap3630) {
        s->uart[3] = qdev_create(NULL, "omap_uart");
        s->uart[3]->id = "uart4";
        qdev_prop_set_uint32(s->uart[3], "mmio_size", 0x1000);
        qdev_prop_set_uint32(s->uart[3], "baudrate",
                             omap_clk_getrate(omap_findclk(s,
                                                           "omap3_uart4_fclk"))
                             / 16);
        qdev_prop_set_chr(s->uart[3], "chardev", chr_uart4);
        qdev_prop_set_uint32(s->uart[3], "revision", uart_revision);
        qdev_init_nofail(s->uart[3]);
        busdev = SYS_BUS_DEVICE(s->uart[3]);
        sysbus_connect_irq(busdev, 0,
                           qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_UART4_IRQ));
        sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_UART4_TX]);
        sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_UART4_RX]);
        sysbus_mmio_map(busdev, 0, omap_l4_region_base(omap3_l4ta_init(s->l4,
                                                                L4A_UART4),0));
    }

    s->dss = qdev_create(NULL, "omap_dss");
    qdev_prop_set_int32(s->dss, "mpu_model", s->mpu_model);
    qdev_init_nofail(s->dss);
    busdev = SYS_BUS_DEVICE(s->dss);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_DSS_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_DSS_LINETRIGGER]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_DSS0]);
    sysbus_connect_irq(busdev, 3, s->drq[OMAP3XXX_DMA_DSS1]);
    sysbus_connect_irq(busdev, 4, s->drq[OMAP3XXX_DMA_DSS2]);
    sysbus_connect_irq(busdev, 5, s->drq[OMAP3XXX_DMA_DSS3]);
    struct omap_target_agent_s *ta = omap3_l4ta_init(s->l4, L4A_DSS);
    sysbus_mmio_map(busdev, 0, omap_l4_region_base(ta, 1));
    sysbus_mmio_map(busdev, 1, omap_l4_region_base(ta, 2));
    sysbus_mmio_map(busdev, 2, omap_l4_region_base(ta, 3));
    sysbus_mmio_map(busdev, 3, omap_l4_region_base(ta, 4));
    sysbus_mmio_map(busdev, 4, omap_l4_region_base(ta, 0));

    s->gpio = qdev_create(NULL, "omap2-gpio");
    qdev_prop_set_int32(s->gpio, "mpu_model", s->mpu_model);
    qdev_prop_set_ptr(s->gpio, "iclk", omap_findclk(s, "omap3_wkup_l4_iclk"));
    qdev_prop_set_ptr(s->gpio, "fclk0", omap_findclk(s, "omap3_wkup_32k_fclk"));
    qdev_prop_set_ptr(s->gpio, "fclk1", omap_findclk(s, "omap3_per_32k_fclk"));
    qdev_prop_set_ptr(s->gpio, "fclk2", omap_findclk(s, "omap3_per_32k_fclk"));
    qdev_prop_set_ptr(s->gpio, "fclk3", omap_findclk(s, "omap3_per_32k_fclk"));
    qdev_prop_set_ptr(s->gpio, "fclk4", omap_findclk(s, "omap3_per_32k_fclk"));
    qdev_prop_set_ptr(s->gpio, "fclk5", omap_findclk(s, "omap3_per_32k_fclk"));
    qdev_init_nofail(s->gpio);
    busdev = SYS_BUS_DEVICE(s->gpio);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_GPIO1_MPU_IRQ));
    sysbus_connect_irq(busdev, 3,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_GPIO2_MPU_IRQ));
    sysbus_connect_irq(busdev, 6,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_GPIO3_MPU_IRQ));
    sysbus_connect_irq(busdev, 9,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_GPIO4_MPU_IRQ));
    sysbus_connect_irq(busdev, 12,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_GPIO5_MPU_IRQ));
    sysbus_connect_irq(busdev, 15,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_GPIO6_MPU_IRQ));
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_GPIO1), 0));
    sysbus_mmio_map(busdev, 1,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_GPIO2), 0));
    sysbus_mmio_map(busdev, 2,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_GPIO3), 0));
    sysbus_mmio_map(busdev, 3,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_GPIO4), 0));
    sysbus_mmio_map(busdev, 4,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_GPIO5), 0));
    sysbus_mmio_map(busdev, 5,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_GPIO6), 0));

    omap_tap_init(omap3_l4ta_init(s->l4, L4A_TAP), s);

    s->omap3_mmc[0] = qdev_create(NULL, "omap3_mmc");
    s->omap3_mmc[0]->id = "mmc1";
    qdev_init_nofail(s->omap3_mmc[0]);
    busdev = SYS_BUS_DEVICE(s->omap3_mmc[0]);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_MMC1_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_MMC1_TX]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_MMC1_RX]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_MMC1), 0));
    s->omap3_mmc[1] = qdev_create(NULL, "omap3_mmc");
    s->omap3_mmc[1]->id = "mmc2";
    qdev_init_nofail(s->omap3_mmc[1]);
    busdev = SYS_BUS_DEVICE(s->omap3_mmc[1]);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_MMC2_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_MMC2_TX]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_MMC2_RX]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_MMC2), 0));
    s->omap3_mmc[2] = qdev_create(NULL, "omap3_mmc");
    s->omap3_mmc[2]->id = "mmc3";
    qdev_init_nofail(s->omap3_mmc[2]);
    busdev = SYS_BUS_DEVICE(s->omap3_mmc[2]);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_MMC3_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_MMC3_TX]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_MMC3_RX]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_MMC3), 0));

    /* Later OMAP3 models have a different I2C controller rev */
    i = (s->mpu_model < omap3630) ? 0x3c : 0x40;

    s->i2c[0] = qdev_create(NULL, "omap_i2c");
    qdev_prop_set_uint8(s->i2c[0], "revision", i);
    qdev_prop_set_uint32(s->i2c[0], "fifo-size", 8);
    qdev_prop_set_ptr(s->i2c[0], "iclk", omap_findclk(s, "omap3_i2c1_iclk"));
    qdev_prop_set_ptr(s->i2c[0], "fclk", omap_findclk(s, "omap3_i2c1_fclk"));
    qdev_init_nofail(s->i2c[0]);
    busdev = SYS_BUS_DEVICE(s->i2c[0]);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_I2C1_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_I2C1_TX]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_I2C1_RX]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4,L4A_I2C1), 0));

    s->i2c[1] = qdev_create(NULL, "omap_i2c");
    qdev_prop_set_uint8(s->i2c[1], "revision", i);
    qdev_prop_set_uint32(s->i2c[1], "fifo-size", 8);
    qdev_prop_set_ptr(s->i2c[1], "iclk", omap_findclk(s, "omap3_i2c2_iclk"));
    qdev_prop_set_ptr(s->i2c[1], "fclk", omap_findclk(s, "omap3_i2c2_fclk"));
    qdev_init_nofail(s->i2c[1]);
    busdev = SYS_BUS_DEVICE(s->i2c[1]);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_I2C2_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_I2C2_TX]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_I2C2_RX]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4,L4A_I2C2), 0));

    s->i2c[2] = qdev_create(NULL, "omap_i2c");
    qdev_prop_set_uint8(s->i2c[2], "revision", i);
    qdev_prop_set_uint32(s->i2c[2], "fifo-size", 64);
    qdev_prop_set_ptr(s->i2c[2], "iclk", omap_findclk(s, "omap3_i2c3_iclk"));
    qdev_prop_set_ptr(s->i2c[2], "fclk", omap_findclk(s, "omap3_i2c3_fclk"));
    qdev_init_nofail(s->i2c[2]);
    busdev = SYS_BUS_DEVICE(s->i2c[2]);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_I2C3_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_I2C3_TX]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_I2C3_RX]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4,L4A_I2C3), 0));

    s->omap3_usb_otg = qdev_create(NULL, "omap3_hsusb_otg");
    qdev_init_nofail(s->omap3_usb_otg);
    busdev = SYS_BUS_DEVICE(s->omap3_usb_otg);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_HSUSB_MC));
    sysbus_connect_irq(busdev, 1,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_HSUSB_DMA));
    sysbus_connect_irq(busdev, 2,
                       qemu_allocate_irqs(omap3_cm_hsusb_otg_stdby_callback,
                                          s->omap3_cm, 1)[0]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_USBHS_OTG),
                                        0));

    s->omap3_usb_host = qdev_create(NULL, "omap3_hsusb_host");
    qdev_init_nofail(s->omap3_usb_host);
    busdev =  SYS_BUS_DEVICE(s->omap3_usb_host);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_EHCI_IRQ));
    sysbus_connect_irq(busdev, 1,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_TLL_IRQ));
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_USBHS_TLL),
                                        0));
    struct omap_target_agent_s *usbhost_ta =
        omap3_l4ta_init(s->l4, L4A_USBHS_HOST);
    sysbus_mmio_map(busdev, 1, omap_l4_region_base(usbhost_ta, 0));
    sysbus_mmio_map(busdev, 2, omap_l4_region_base(usbhost_ta, 2));

    s->omap3_usb_ohci = qdev_create(NULL, "sysbus-ohci");
    qdev_prop_set_uint32(s->omap3_usb_ohci, "num-ports", 3);
    qdev_prop_set_taddr(s->omap3_usb_ohci, "dma-offset", 0);
    qdev_init_nofail(s->omap3_usb_ohci);
    busdev = SYS_BUS_DEVICE(s->omap3_usb_ohci);
    sysbus_mmio_map(busdev, 0, omap_l4_region_base(usbhost_ta, 1));
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_OHCI_IRQ));

    s->mcspi = qdev_create(NULL, "omap_mcspi");
    qdev_prop_set_int32(s->mcspi, "mpu_model", s->mpu_model);
    qdev_init_nofail(s->mcspi);
    busdev = SYS_BUS_DEVICE(s->mcspi);
    sysbus_connect_irq(busdev, 0,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_MCSPI1_IRQ));
    sysbus_connect_irq(busdev, 1, s->drq[OMAP3XXX_DMA_SPI1_TX0]);
    sysbus_connect_irq(busdev, 2, s->drq[OMAP3XXX_DMA_SPI1_RX0]);
    sysbus_connect_irq(busdev, 3, s->drq[OMAP3XXX_DMA_SPI1_TX1]);
    sysbus_connect_irq(busdev, 4, s->drq[OMAP3XXX_DMA_SPI1_RX1]);
    sysbus_connect_irq(busdev, 5, s->drq[OMAP3XXX_DMA_SPI1_TX2]);
    sysbus_connect_irq(busdev, 6, s->drq[OMAP3XXX_DMA_SPI1_RX2]);
    sysbus_connect_irq(busdev, 7, s->drq[OMAP3XXX_DMA_SPI1_TX3]);
    sysbus_connect_irq(busdev, 8, s->drq[OMAP3XXX_DMA_SPI1_RX3]);
    sysbus_connect_irq(busdev, 9,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_MCSPI2_IRQ));
    sysbus_connect_irq(busdev, 10, s->drq[OMAP3XXX_DMA_SPI2_TX0]);
    sysbus_connect_irq(busdev, 11, s->drq[OMAP3XXX_DMA_SPI2_RX0]);
    sysbus_connect_irq(busdev, 12, s->drq[OMAP3XXX_DMA_SPI2_TX1]);
    sysbus_connect_irq(busdev, 13, s->drq[OMAP3XXX_DMA_SPI2_RX1]);
    sysbus_connect_irq(busdev, 14,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_MCSPI3_IRQ));
    sysbus_connect_irq(busdev, 15, s->drq[OMAP3XXX_DMA_SPI3_TX0]);
    sysbus_connect_irq(busdev, 16, s->drq[OMAP3XXX_DMA_SPI3_RX0]);
    sysbus_connect_irq(busdev, 17, s->drq[OMAP3XXX_DMA_SPI3_TX1]);
    sysbus_connect_irq(busdev, 18, s->drq[OMAP3XXX_DMA_SPI3_RX1]);
    sysbus_connect_irq(busdev, 19,
                       qdev_get_gpio_in(s->ih[0], OMAP_INT_3XXX_MCSPI4_IRQ));
    sysbus_connect_irq(busdev, 20, s->drq[OMAP3XXX_DMA_SPI4_TX0]);
    sysbus_connect_irq(busdev, 21, s->drq[OMAP3XXX_DMA_SPI4_RX0]);
    sysbus_mmio_map(busdev, 0,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_MCSPI1), 0));
    sysbus_mmio_map(busdev, 1,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_MCSPI2), 0));
    sysbus_mmio_map(busdev, 2,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_MCSPI3), 0));
    sysbus_mmio_map(busdev, 3,
                    omap_l4_region_base(omap3_l4ta_init(s->l4, L4A_MCSPI4), 0));

    omap3_boot_rom_init(s);

    qemu_register_reset(omap3_reset, s);
    return s;
}
