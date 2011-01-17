/*
 * OMAP2/3 Display Subsystem.
 *
 * Copyright (C) 2008,2009 Nokia Corporation
 * Original OMAP2 support written by Andrzej Zaborowski <andrew@openedhand.com>
 * Enhancements and OMAP3 support written by Juha Riihimäki
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
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#include "hw.h"
#include "console.h"
#include "omap.h"
#include "qemu-common.h"
#include "qemu-timer.h"
#include "sysemu.h"
#include "devices.h"
#include "omap_dss.h"

//#define OMAP_DSS_DEBUG
#define OMAP_DSS_DEBUG_DISPC
#define OMAP_DSS_DEBUG_DISS
#define OMAP_DSS_DEBUG_DSI
#define OMAP_DSS_DEBUG_RFBI
//#define OMAP_DSS_DEBUG_VENC

#ifdef OMAP_DSS_DEBUG
#define TRACE(fmt,...) fprintf(stderr, "%s@%d: " fmt "\n", __FUNCTION__, \
                               __LINE__, ##__VA_ARGS__)
#define LAYERNAME(n) ((!(n)) ? "GFX" : ((n)==1) ? "VID1" : "VID2")
#ifdef OMAP_DSS_DEBUG_DISPC
#define TRACEDISPC(fmt,...) TRACE(fmt, ##__VA_ARGS__)
#else
#define TRACEDISPC(...)
#endif
#ifdef OMAP_DSS_DEBUG_DISS
#define TRACEDISS(fmt,...) TRACE(fmt, ##__VA_ARGS__)
#else
#define TRACEDISS(...)
#endif
#ifdef OMAP_DSS_DEBUG_DSI
#define TRACEDSI(fmt,...) TRACE(fmt, ##__VA_ARGS__)
#else
#define TRACEDSI(...)
#endif
#ifdef OMAP_DSS_DEBUG_RFBI
#define TRACERFBI(fmt,...) TRACE(fmt, ##__VA_ARGS__)
#else
#define TRACERFBI(...)
#endif
#ifdef OMAP_DSS_DEBUG_VENC
#define TRACEVENC(fmt,...) TRACE(fmt, ##__VA_ARGS__)
#else
#define TRACEVENC(...)
#endif
#else
#define TRACE(...)
#define TRACEDISPC(...)
#define TRACEDISS(...)
#define TRACEDSI(...)
#define TRACERFBI(...)
#define TRACEVENC(...)
#undef OMAP_RO_REG
#undef OMAP_RO_REGV
#undef OMAP_BAD_REG
#undef OMAP_BAD_REGV
#define OMAP_RO_REG(...)
#define OMAP_RO_REGV(...)
#define OMAP_BAD_REG(...)
#define OMAP_BAD_REGV(...)
#endif

#define OMAP_DSI_RX_FIFO_SIZE 32

struct omap_dss_s {
    qemu_irq irq;
    qemu_irq drq;
    
    struct omap_mpu_state_s *mpu;

    uint32_t autoidle;
    uint32_t control;
    uint32_t sdi_control;
    uint32_t pll_control;
    uint32_t dss_status;
    
    struct omap_dss_dispc_s dispc;
    const struct omap_dss_panel_s *dig, *lcd;
    
    struct {
        int idlemode;
        uint32_t control;
        int enable;
        int pixels;
        int busy;
        int skiplines;
        uint16_t rxbuf;
        uint32_t config[2];
        uint32_t time[4];
        uint32_t data[6];
        uint16_t vsync;
        uint16_t hsync;
        const struct rfbi_chip_s *chip[2];
    } rfbi;
    
    struct {
        qemu_irq drq[4];
        /* protocol engine registers */
        uint32_t sysconfig;
        uint32_t irqst;
        uint32_t irqen;
        uint32_t ctrl;
        uint32_t complexio_cfg1;
        uint32_t complexio_cfg2;
        uint32_t complexio_irqst;
        uint32_t complexio_irqen;
        uint32_t clk_ctrl;
        uint32_t timing1;
        uint32_t timing2;
        uint32_t vm_timing1;
        uint32_t vm_timing2;
        uint32_t vm_timing3;
        uint32_t vm_timing4;
        uint32_t vm_timing5;
        uint32_t vm_timing6;
        uint32_t vm_timing7;
        uint32_t clk_timing;
        uint32_t stopclk_timing;
        uint32_t tx_fifo_vc_size;
        uint32_t rx_fifo_vc_size;
        struct {
            uint32_t ctrl;
            uint32_t te;
            uint32_t lp_header;
            uint32_t lp_payload;
            int lp_counter;
            uint32_t sp_header;
            uint32_t irqst;
            uint32_t irqen;
            uint32_t rx_fifo[OMAP_DSI_RX_FIFO_SIZE];
            int rx_fifo_pos;
            int rx_fifo_len;
            struct dsi_chip_s *chip;
        } vc[4];
        /* phy registers */
        uint32_t phy_cfg0;
        uint32_t phy_cfg1;
        uint32_t phy_cfg2;
        /* pll controller registers */
        uint32_t pll_control;
        uint32_t pll_go;
        uint32_t pll_config1;
        uint32_t pll_config2;
    } dsi;
};

/* Bytes(!) per pixel */
const int omap_lcd_Bpp[0x10] = {
    0,  /* 0x0: BITMAP1 (CLUT) */
    0,  /* 0x1: BITMAP2 (CLUT) */
    0,  /* 0x2: BITMAP4 (CLUT) */
    1,  /* 0x3: BITMAP8 (CLUT) */
    2,  /* 0x4: RGB12 (unpacked 16-bit container)*/
    2,  /* 0x5: ARGB16 */
    2,  /* 0x6: RGB16 */
    0,  /* 0x7: reserved */
    4,  /* 0x8: RGB24 (unpacked in 32-bit container) */
    3,  /* 0x9: RGB24 (packed in 24-bit container) */
    2,  /* 0xa: YUV2 422 */
    2,  /* 0xb: UYVY 422 */
    4,  /* 0xc: ARGB32 */
    4,  /* 0xd: RGBA32 */
    4,  /* 0xe: RGBx32 (24-bit RGB aligned on MSB of the 32-bit container) */
    0,  /* 0xf: reserved */
};

static void omap_dss_interrupt_update(struct omap_dss_s *s)
{
    qemu_set_irq(s->irq, 
                 (s->dsi.irqst & s->dsi.irqen)
                 | (s->dsi.complexio_irqst & s->dsi.complexio_irqen)
                 | (s->dsi.vc[0].irqst & s->dsi.vc[0].irqen)
                 | (s->dsi.vc[1].irqst & s->dsi.vc[1].irqen)
                 | (s->dsi.vc[2].irqst & s->dsi.vc[2].irqen)
                 | (s->dsi.vc[3].irqst & s->dsi.vc[3].irqen)
                 | (s->dispc.irqst & s->dispc.irqen));
}

void omap_dss_lcd_framedone(void *opaque)
{
    struct omap_dss_s *s = (struct omap_dss_s *)opaque;
    if (s->dispc.control & 1) { /* LCDENABLE */
        if ((s->dispc.control & (1 << 11))) { /* STALLMODE */
            s->dispc.control &= ~1; /* LCDENABLE */
            if ((s->dsi.ctrl & 1)) { /* IF_EN */
                int n = 0;
                for (; n < 4; n++) {
                    if ((s->dsi.vc[n].ctrl & 1) &&       /* VC_EN */
                        (s->dsi.vc[n].te & (3 << 30))) { /* TE_START | TE_EN */
                        s->dsi.vc[n].te = 0;   /* TE_START,TE_EN,TE_SIZE = 0 */
                        s->dsi.irqst |= 1 << 16; /* TE_TRIGGER_IRQ */
                    }
                }
            }
            if ((s->rfbi.control & 1)) { /* ENABLE */
                s->rfbi.pixels = 0;
                s->rfbi.busy = 0;
            }
            if (s->dispc.lcdframer) {
                qemu_del_timer(s->dispc.lcdframer);
            }
        } else {
            if (s->dispc.lcdframer) {
                qemu_mod_timer(s->dispc.lcdframer,
                               qemu_get_clock(vm_clock)
                               + get_ticks_per_sec() / 10);
            }
        }
        s->dispc.irqst |= 1 | 2;    /* FRAMEDONE | VSYNC */
        omap_dss_interrupt_update(s);
    }
}

static void omap_dsi_te_trigger(struct omap_dss_s *s, struct dsi_chip_s *chip)
{
    int i;
    if ((s->dsi.ctrl & 1)) { /* IF_EN */
        for (i = 0; i < 4; i++) {
            if ((s->dsi.vc[i].ctrl & 1) && /* VC_EN */
                s->dsi.vc[i].chip == chip) {
                if ((s->dsi.vc[i].te & (3 << 30))) { /* TE_START | TE_EN */
                    s->dsi.vc[i].te = 0;
                    s->dispc.control &= ~1; /* LCDENABLE */
                    s->dispc.irqst |= 1; /* FRAMEDONE */
                }
                s->dsi.irqst |= 1 << 16; /* TE_TRIGGER_IRQ */
                omap_dss_interrupt_update(s);
                break;
            }
        }
    }
}

static void omap_rfbi_transfer_stop(struct omap_dss_s *s)
{
    if (!s->rfbi.busy)
        return;
    
    /* TODO: in non-Bypass mode we probably need to just deassert the DRQ.  */
    
    s->rfbi.busy = 0;
    s->rfbi.control &= ~0x10; /* ITE */
}

static void omap_rfbi_transfer_start(struct omap_dss_s *s)
{
    void *data;
    target_phys_addr_t len;
    target_phys_addr_t data_addr;
    int pitch;
    static void *bounce_buffer;
    static target_phys_addr_t bounce_len;
    
    if (!s->rfbi.enable || s->rfbi.busy)
        return;
    
    if (s->rfbi.control & (1 << 1)) {				/* BYPASS */
        /* TODO: in non-Bypass mode we probably need to just assert the
         * DRQ and wait for DMA to write the pixels.  */
        hw_error("%s: Bypass mode unimplemented", __FUNCTION__);
    }
    
    if (!(s->dispc.control & (1 << 11))) /* STALLMODE */
        return;
    
    s->rfbi.busy = 1;
    
    len = s->rfbi.pixels * 2;
    
    data_addr = s->dispc.plane[0].addr[0];
    data = cpu_physical_memory_map(data_addr, &len, 0);
    if (data && len != s->rfbi.pixels * 2) {
        cpu_physical_memory_unmap(data, len, 0, 0);
        data = NULL;
        len = s->rfbi.pixels * 2;
    }
    if (!data) {
        if (len > bounce_len) {
            bounce_buffer = qemu_realloc(bounce_buffer, len);
        }
        data = bounce_buffer;
        cpu_physical_memory_read(data_addr, data, len);
    }
    
    /* TODO: negative values */
    pitch = s->dispc.plane[0].nx + (s->dispc.plane[0].rowinc - 1) / 2;
    
    if ((s->rfbi.control & (1 << 2)) && s->rfbi.chip[0])
        s->rfbi.chip[0]->block(s->rfbi.chip[0]->opaque, 1, data, len, pitch);
    if ((s->rfbi.control & (1 << 3)) && s->rfbi.chip[1])
        s->rfbi.chip[1]->block(s->rfbi.chip[1]->opaque, 1, data, len, pitch);
    
    if (data != bounce_buffer) {
        cpu_physical_memory_unmap(data, len, 0, len);
    }
    
    omap_rfbi_transfer_stop(s);
    
    omap_dss_lcd_framedone(s);
}

static void omap_dsi_transfer_start(struct omap_dss_s *s, int ch)
{
    if (((s->dispc.control >> 11) & 1) && /* STALLMODE */
        (s->dsi.ctrl & 1) &&              /* IF_EN */
        (s->dsi.vc[ch].ctrl & 1) &&       /* VC_EN */
        (s->dsi.vc[ch].te >> 30) & 3) {   /* TE_START | TE_EN */
        TRACEDSI("start TE data transfer on channel %d for %d bytes",
                 ch, s->dsi.vc[ch].te & 0xffffff);
        TRACEDSI("vc%d   irqenable=0x%08x", ch, s->dsi.vc[ch].irqen);
        TRACEDSI("dsi   irqenable=0x%08x", s->dsi.irqen);
        TRACEDSI("dispc irqenable=0x%08x", s->dispc.irqen);
        if (!s->dsi.vc[ch].chip) {
            TRACEDSI("ERROR - no DSI chip attached on channel %d", ch);
        } else {
            int tx_dma = (s->dsi.vc[ch].ctrl >> 21) & 7; /* DMA_TX_REQ_NB */
            if (tx_dma < 4) {
                qemu_irq_raise(s->dsi.drq[tx_dma]);
            } else {
                if (s->dsi.vc[ch].chip->blt(s->dsi.vc[ch].chip->opaque,
                                            &s->dispc)) {
                    omap_dss_lcd_framedone(s);
                }
            }
        }
    }
}

static void omap_dsi_reset(struct omap_dss_s *s)
{
    int i;
    
    s->dsi.sysconfig = 0x11;
    s->dsi.irqst = 0;
    s->dsi.irqen = 0;
    s->dsi.ctrl = 0x100;
    s->dsi.complexio_cfg1 = 0x20000000;
    s->dsi.complexio_cfg2 = 0;
    s->dsi.complexio_irqst = 0;
    s->dsi.complexio_irqen = 0;
    s->dsi.clk_ctrl = 1;
    s->dsi.timing1 = 0x7fff7fff;
    s->dsi.timing2 = 0x7fff7fff;
    s->dsi.vm_timing1 = 0;
    s->dsi.vm_timing2 = 0;
    s->dsi.vm_timing3 = 0;
    s->dsi.vm_timing4 = 0;
    s->dsi.vm_timing5 = 0;
    s->dsi.vm_timing6 = 0;
    s->dsi.vm_timing7 = 0;
    s->dsi.clk_timing = 0x0101;
    s->dsi.stopclk_timing = 0x80;
    s->dsi.tx_fifo_vc_size = 0;
    s->dsi.rx_fifo_vc_size = 0;
    for (i = 0; i < 4; i++) {
        s->dsi.vc[i].ctrl = 0;
        s->dsi.vc[i].te = 0;
        s->dsi.vc[i].lp_header = 0;
        s->dsi.vc[i].lp_payload = 0;
        s->dsi.vc[i].lp_counter = 0;
        s->dsi.vc[i].sp_header = 0;
        s->dsi.vc[i].irqst = 0;
        s->dsi.vc[i].irqen = 0;
        s->dsi.vc[i].rx_fifo_pos = 0;
        s->dsi.vc[i].rx_fifo_len = 0;
    }
    if (cpu_is_omap3630(s->mpu)) {
        s->dsi.phy_cfg0 = 0x1e481d3a;
        s->dsi.phy_cfg1 = 0x420a1a6a;
        s->dsi.phy_cfg2 = 0xb800001a;
    } else {
        s->dsi.phy_cfg0 = 0x1a3c1a28;
        s->dsi.phy_cfg1 = 0x420a1875;
        s->dsi.phy_cfg2 = 0xb800001b;
    }
    s->dsi.pll_control = 0;
    s->dsi.pll_go = 0;
    s->dsi.pll_config1 = 0;
    s->dsi.pll_config2 = 0;
    omap_dss_interrupt_update(s);
}

static void omap_rfbi_reset(struct omap_dss_s *s)
{
    s->rfbi.idlemode = 0;
    s->rfbi.control = 2;
    s->rfbi.enable = 0;
    s->rfbi.pixels = 0;
    s->rfbi.skiplines = 0;
    s->rfbi.busy = 0;
    s->rfbi.config[0] = 0x00310000;
    s->rfbi.config[1] = 0x00310000;
    s->rfbi.time[0] = 0;
    s->rfbi.time[1] = 0;
    s->rfbi.time[2] = 0;
    s->rfbi.time[3] = 0;
    s->rfbi.data[0] = 0;
    s->rfbi.data[1] = 0;
    s->rfbi.data[2] = 0;
    s->rfbi.data[3] = 0;
    s->rfbi.data[4] = 0;
    s->rfbi.data[5] = 0;
    s->rfbi.vsync = 0;
    s->rfbi.hsync = 0;
}

void omap_dss_reset(struct omap_dss_s *s)
{
    int i, j;
    
    s->autoidle = 0x10; /* was 0 for OMAP2 but bit4 must be set for OMAP3 */
    s->control = 0;
    if (cpu_is_omap3430(s->mpu)) {
        s->sdi_control = 0;
        s->pll_control = 0;
        s->dss_status = 0x81; /* bit 7 is not present prior to OMAP3 */
    } else { /* omap2, omap3630 */
        s->dss_status = 0x01;
    }

    s->dispc.idlemode = 0;
    s->dispc.irqst = 0;
    s->dispc.irqen = 0;
    s->dispc.control = 0;
    s->dispc.config = 0;
    s->dispc.capable = 0x161;
    s->dispc.timing[0] = 0;
    s->dispc.timing[1] = 0;
    s->dispc.timing[2] = 0;
    s->dispc.timing[3] = 0x00010002;
    s->dispc.line = 0;
    s->dispc.bg[0] = 0;
    s->dispc.bg[1] = 0;
    s->dispc.trans[0] = 0;
    s->dispc.trans[1] = 0;
    s->dispc.size_dig = 0;
    s->dispc.size_lcd = 0;
    s->dispc.global_alpha = 0;
    s->dispc.cpr_coef_r = 0;
    s->dispc.cpr_coef_g = 0;
    s->dispc.cpr_coef_b = 0;

    for (i = 0; i < 3; i++) {
        s->dispc.plane[i].enable = 0;
        s->dispc.plane[i].bpp = 0;
        s->dispc.plane[i].addr[0] = 0;
        s->dispc.plane[i].addr[1] = 0;
        s->dispc.plane[i].addr[2] = 0;
        s->dispc.plane[i].posx = 0;
        s->dispc.plane[i].posy = 0;
        s->dispc.plane[i].nx = 1;
        s->dispc.plane[i].ny = 1;
        s->dispc.plane[i].attr = 0;
        s->dispc.plane[i].tresh = (s->dispc.rev < 0x30) ? 0 : 0x03ff03c0;
        s->dispc.plane[i].rowinc = 1;
        s->dispc.plane[i].colinc = 1;
        s->dispc.plane[i].wininc = 0;
        s->dispc.plane[i].preload = 0x100;
        s->dispc.plane[i].fir = 0;
        s->dispc.plane[i].picture_size = 0;
        s->dispc.plane[i].accu[0] = 0;
        s->dispc.plane[i].accu[1] = 0;
        for (j = 0; j < 5; j++)
            s->dispc.plane[i].conv_coef[j] = 0;
        for (j = 0; j < 8; j++) {
            s->dispc.plane[i].fir_coef_h[j] = 0;
            s->dispc.plane[i].fir_coef_hv[j] = 0;
            s->dispc.plane[i].fir_coef_v[j] = 0;
        }
    }
        
    omap_dsi_reset(s);
    omap_rfbi_reset(s);
    omap_dss_interrupt_update(s);
}

static uint32_t omap_diss_read(void *opaque, target_phys_addr_t addr)
{
    struct omap_dss_s *s = (struct omap_dss_s *) opaque;
    uint32_t x;

    switch (addr) {
    case 0x00:	/* DSS_REVISIONNUMBER */
        TRACEDISS("DSS_REVISIONNUMBER: 0x20");
        return 0x20;

    case 0x10:	/* DSS_SYSCONFIG */
        TRACEDISS("DSS_SYSCONFIG: 0x%08x", s->autoidle);
        return s->autoidle;

    case 0x14:	/* DSS_SYSSTATUS */
        TRACEDISS("DSS_SYSSTATUS: 0x1");
        return 1; /* RESETDONE */
            
    case 0x18:  /* DSS_IRQSTATUS */
        x = (s->dispc.irqst & s->dispc.irqen) ? 1 : 0;
        if ((s->dsi.irqst & s->dsi.irqen)
            | (s->dsi.complexio_irqst & s->dsi.complexio_irqen)
            | (s->dsi.vc[0].irqst & s->dsi.vc[0].irqen)
            | (s->dsi.vc[1].irqst & s->dsi.vc[1].irqen)
            | (s->dsi.vc[2].irqst & s->dsi.vc[2].irqen)
            | (s->dsi.vc[3].irqst & s->dsi.vc[3].irqen))
            x |= 2;
        TRACEDISS("DSS_IRQSTATUS: 0x%08x", x);
        return x;
            
    case 0x40:	/* DSS_CONTROL */
        TRACEDISS("DSS_CONTROL: 0x%08x", s->control);
        return s->control;

    case 0x44:  /* DSS_SDI_CONTROL */
        if (cpu_is_omap3430(s->mpu)) {
            TRACEDISS("DSS_SDI_CONTROL: 0x%08x", s->sdi_control);
            return s->sdi_control;
        }
        break;

    case 0x48: /* DSS_PLL_CONTROL */
        if (cpu_is_omap3430(s->mpu)) {
            TRACEDISS("DSS_PLL_CONTROL: 0x%08x", s->pll_control);
            return s->pll_control;
        }
        break;

    case 0x50:	/* DSS_PSA_LCD_REG_1 */
    case 0x54:	/* DSS_PSA_LCD_REG_2 */
    case 0x58:	/* DSS_PSA_VIDEO_REG */
        if (cpu_class_omap2(s->mpu)) {
            TRACEDISS("DSS_PSA_xxx: 0");
            /* TODO: fake some values according to s->control bits */
            return 0;
        }
        break;

    case 0x5c:
        x = s->dss_status;
        if (cpu_class_omap2(s->mpu)) {
            TRACEDISS("DSS_STATUS: 0x%08x", x);
        } else {
            if (cpu_is_omap3430(s->mpu)) {
                s->dss_status &= ~(1 << 6); /* SDI_PLL_BUSYFLAG */
                TRACEDISS("DSS_SDI_STATUS: 0x%08x", x);
            } else { /* omap3630 */
                TRACEDISS("DSS_CLK_STATUS: 0x%08x", x);
            }
        }
        return x;
    default:
        break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap_diss_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct omap_dss_s *s = (struct omap_dss_s *) opaque;

    switch (addr) {
    case 0x00:	/* DSS_REVISIONNUMBER */
    case 0x14:	/* DSS_SYSSTATUS */
    case 0x18:  /* DSS_IRQSTATUS */
    case 0x50:	/* DSS_PSA_LCD_REG_1 */
    case 0x54:	/* DSS_PSA_LCD_REG_2 */
    case 0x58:	/* DSS_PSA_VIDEO_REG */
    case 0x5c:	/* DSS_STATUS */
        /* quietly ignore */
        /*OMAP_RO_REGV(addr, value);*/
        break;

    case 0x10:	/* DSS_SYSCONFIG */
        TRACEDISS("DSS_SYSCONFIG = 0x%08x", value);
        if (value & 2) { /* SOFTRESET */
            omap_dss_reset(s);
        }
        if (cpu_class_omap2(s->mpu)) {
            value &= 0x01;
        } else {
            value &= 0x19;
        }
        s->autoidle = value;
        break;

    case 0x40:	/* DSS_CONTROL */
        TRACEDISS("DSS_CONTROL = 0x%08x", value);
        if (cpu_class_omap2(s->mpu)) {
            value &= 0x3dd;
        } else {
            value &= 0x3ff;
        }
        s->control = value;
        s->dss_status &= ~0x3;
        s->dss_status |= 1 + (s->control & 1);
        break;

    case 0x44: /* DSS_SDI_CONTROL */
        if (cpu_is_omap3430(s->mpu)) {
            TRACEDISS("DSS_SDI_CONTROL = 0x%08x", value);
            s->sdi_control = value & 0x000ff80f;
        } else {
            OMAP_BAD_REGV(addr, value);
        }
        break;

    case 0x48: /* DSS_PLL_CONTROL */
        if (cpu_is_omap3430(s->mpu)) {
            TRACEDISS("DSS_PLL_CONTROL = 0x%08x", value);
            if (value & (1 << 18)) { /* SDI_PLL_SYSRESET */
                s->dss_status |= 1 << 2;    /* SDI_PLL_RESETDONE */
            } else {
                s->dss_status &= ~(1 << 2); /* SDI_PLL_RESETDONE */
            }
            if (value & (1 << 28)) { /* SDI_PLL_GOBIT */
                s->dss_status |= 1 << 6;    /* SDI_PLL_BUSYFLAG */
                s->dss_status &= ~(1 << 5); /* SDI_PLL_LOCK */
            } else {
                if (s->pll_control & (1 << 28)) { /* SDI_PLL_GOBIT */
                    s->dss_status &= ~(1 << 6); /* SDI_PLL_BUSYFLAG */
                    s->dss_status |= 1 << 5;    /* SDI_PLL_LOCK */
                }
            }
            s->pll_control = value;
        } else {
            OMAP_BAD_REGV(addr, value);
        }
        break;
            
    default:
        OMAP_BAD_REGV(addr, value);
        break;
    }
}

static CPUReadMemoryFunc * const omap_diss1_readfn[] = {
    omap_badwidth_read32,
    omap_badwidth_read32,
    omap_diss_read,
};

static CPUWriteMemoryFunc * const omap_diss1_writefn[] = {
    omap_badwidth_write32,
    omap_badwidth_write32,
    omap_diss_write,
};

static uint32_t omap_disc_read(void *opaque, target_phys_addr_t addr)
{
    struct omap_dss_s *s = (struct omap_dss_s *) opaque;
    int n = 0;

    switch (addr) {
    case 0x000:	/* DISPC_REVISION */
        TRACEDISPC("DISPC_REVISION: 0x%08x", s->dispc.rev);
        return s->dispc.rev;
    case 0x010:	/* DISPC_SYSCONFIG */
        TRACEDISPC("DISPC_SYSCONFIG: 0x%08x", s->dispc.idlemode);
        return s->dispc.idlemode;
    case 0x014:	/* DISPC_SYSSTATUS */
        TRACEDISPC("DISPC_SYSSTATUS: 1");
        return 1;						/* RESETDONE */
    case 0x018:	/* DISPC_IRQSTATUS */
        TRACEDISPC("DISPC_IRQSTATUS: 0x%08x", s->dispc.irqst);
        return s->dispc.irqst;
    case 0x01c:	/* DISPC_IRQENABLE */
        TRACEDISPC("DISPC_IRQENABLE: 0x%08x", s->dispc.irqen);
        return s->dispc.irqen;
    case 0x040:	/* DISPC_CONTROL */
        TRACEDISPC("DISPC_CONTROL: 0x%08x", s->dispc.control);
        return s->dispc.control;
    case 0x044:	/* DISPC_CONFIG */
        TRACEDISPC("DISPC_CONFIG: 0x%08x", s->dispc.config);
        return s->dispc.config;
    case 0x048:	/* DISPC_CAPABLE */
        TRACEDISPC("DISPC_CAPABLE: 0x%08x", s->dispc.capable);
        return s->dispc.capable;
    case 0x04c:	/* DISPC_DEFAULT_COLOR0 */
        TRACEDISPC("DISPC_DEFAULT_COLOR0: 0x%08x", s->dispc.bg[0]);
        return s->dispc.bg[0];
    case 0x050:	/* DISPC_DEFAULT_COLOR1 */
        TRACEDISPC("DISPC_DEFAULT_COLOR0: 0x%08x", s->dispc.bg[1]);
        return s->dispc.bg[1];
    case 0x054:	/* DISPC_TRANS_COLOR0 */
        TRACEDISPC("DISPC_TRANS_COLOR0: 0x%08x", s->dispc.trans[0]);
        return s->dispc.trans[0];
    case 0x058:	/* DISPC_TRANS_COLOR1 */
        TRACEDISPC("DISPC_TRANS_COLOR0: 0x%08x", s->dispc.trans[1]);
        return s->dispc.trans[1];
    case 0x05c:	/* DISPC_LINE_STATUS */
        TRACEDISPC("DISPC_LINE_STATUS: 0x7ff");
        return 0x7ff;
    case 0x060:	/* DISPC_LINE_NUMBER */
        TRACEDISPC("DISPC_LINE_NUMBER: 0x%08x", s->dispc.line);
        return s->dispc.line;
    case 0x064:	/* DISPC_TIMING_H */
        TRACEDISPC("DISPC_TIMING_H: 0x%08x", s->dispc.timing[0]);
        return s->dispc.timing[0];
    case 0x068:	/* DISPC_TIMING_V */
        TRACEDISPC("DISPC_TIMING_H: 0x%08x", s->dispc.timing[1]);
        return s->dispc.timing[1];
    case 0x06c:	/* DISPC_POL_FREQ */
        TRACEDISPC("DISPC_POL_FREQ: 0x%08x", s->dispc.timing[2]);
        return s->dispc.timing[2];
    case 0x070:	/* DISPC_DIVISOR */
        TRACEDISPC("DISPC_DIVISOR: 0x%08x", s->dispc.timing[3]);
        return s->dispc.timing[3];
    case 0x074: /* DISPC_GLOBAL_ALPHA */
        TRACEDISPC("DISPC_GLOBAL_ALPHA: 0x%08x", s->dispc.global_alpha);
        return s->dispc.global_alpha;
    case 0x078:	/* DISPC_SIZE_DIG */
        TRACEDISPC("DISPC_SIZE_DIG: 0x%08x", s->dispc.size_dig);
        return s->dispc.size_dig;
    case 0x07c:	/* DISPC_SIZE_LCD */
        TRACEDISPC("DISPC_SIZE_LCD: 0x%08x", s->dispc.size_lcd);
        return s->dispc.size_lcd;
    case 0x14c:	/* DISPC_VID2_BA0 */
        n++;
    case 0x0bc:	/* DISPC_VID1_BA0 */
        n++;
    case 0x080:	/* DISPC_GFX_BA0 */
        TRACEDISPC("DISPC_%s_BA0: " OMAP_FMT_plx, LAYERNAME(n),
                   s->dispc.plane[n].addr[0]);
        return s->dispc.plane[n].addr[0];
    case 0x150:	/* DISPC_VID2_BA1 */
        n++;
    case 0x0c0:	/* DISPC_VID1_BA1 */
        n++;
    case 0x084:	/* DISPC_GFX_BA1 */
        TRACEDISPC("DISPC_%s_BA1: " OMAP_FMT_plx, LAYERNAME(n),
                   s->dispc.plane[n].addr[1]);
        return s->dispc.plane[n].addr[1];
    case 0x154:	/* DISPC_VID2_POSITION */
        n++;
    case 0x0c4:	/* DISPC_VID1_POSITION */
        n++;
    case 0x088:	/* DISPC_GFX_POSITION */
        TRACEDISPC("DISPC_%s_POSITION: 0x%08x", LAYERNAME(n),
                   (s->dispc.plane[n].posy << 16) | s->dispc.plane[n].posx);
        return (s->dispc.plane[n].posy << 16) | s->dispc.plane[n].posx;
    case 0x158:	/* DISPC_VID2_SIZE */
        n++;
    case 0x0c8:	/* DISPC_VID1_SIZE */
        n++;
    case 0x08c:	/* DISPC_GFX_SIZE */
        TRACEDISPC("DISPC_%s_SIZE: 0x%08x", LAYERNAME(n),
                   ((s->dispc.plane[n].ny - 1) << 16)
                   | (s->dispc.plane[n].nx - 1));
        return ((s->dispc.plane[n].ny - 1) << 16) | (s->dispc.plane[n].nx - 1);
    case 0x15c:	/* DISPC_VID2_ATTRIBUTES */
        n++;
    case 0x0cc:	/* DISPC_VID1_ATTRIBUTES */
        n++;
    case 0x0a0:	/* DISPC_GFX_ATTRIBUTES */
        TRACEDISPC("DISPC_%s_ATTRIBUTES: 0x%08x", LAYERNAME(n),
                   s->dispc.plane[n].attr);
        return s->dispc.plane[n].attr;
    case 0x160:	/* DISPC_VID2_FIFO_THRESHOLD */
        n++;
    case 0x0d0:	/* DISPC_VID1_FIFO_THRESHOLD */
        n++;
    case 0x0a4:	/* DISPC_GFX_FIFO_TRESHOLD */
        TRACEDISPC("DISPC_%s_THRESHOLD: 0x%08x", LAYERNAME(n),
                   s->dispc.plane[n].tresh);
        return s->dispc.plane[n].tresh;
    case 0x164:	/* DISPC_VID2_FIFO_SIZE_STATUS */
        n++;
    case 0x0d4:	/* DISPC_VID1_FIFO_SIZE_STATUS */
        n++;
    case 0x0a8:	/* DISPC_GFX_FIFO_SIZE_STATUS */
        TRACEDISPC("DISPC_%s_FIFO_SIZE_STATUS: 0x%08x", LAYERNAME(n),
                   s->dispc.rev < 0x30 ? 256 : 1024);
        return s->dispc.rev < 0x30 ? 256 : 1024;
    case 0x168:	/* DISPC_VID2_ROW_INC */
        n++;
    case 0x0d8:	/* DISPC_VID1_ROW_INC */
        n++;
    case 0x0ac:	/* DISPC_GFX_ROW_INC */
        TRACEDISPC("DISPC_%s_ROW_INC: 0x%08x", LAYERNAME(n),
                   s->dispc.plane[n].rowinc);
        return s->dispc.plane[n].rowinc;
    case 0x16c:	/* DISPC_VID2_PIXEL_INC */
        n++;
    case 0x0dc:	/* DISPC_VID1_PIXEL_INC */
        n++;
    case 0x0b0:	/* DISPC_GFX_PIXEL_INC */
        TRACEDISPC("DISPC_%s_PIXEL_INC: 0x%08x", LAYERNAME(n),
                   s->dispc.plane[n].colinc);
        return s->dispc.plane[n].colinc;
    case 0x0b4:	/* DISPC_GFX_WINDOW_SKIP */
        TRACEDISPC("DISPC_GFX_WINDOW_SKIP: 0x%08x", s->dispc.plane[0].wininc);
        return s->dispc.plane[0].wininc;
    case 0x0b8:	/* DISPC_GFX_TABLE_BA */
        TRACEDISPC("DISPC_GFX_TABLE_BA: " OMAP_FMT_plx,
                   s->dispc.plane[0].addr[2]);
        return s->dispc.plane[0].addr[2];
    case 0x170:	/* DISPC_VID2_FIR */
        n++;
    case 0x0e0:	/* DISPC_VID1_FIR */
        n++;
        TRACEDISPC("DISPC_%s_FIR: 0x%08x", LAYERNAME(n),
                   s->dispc.plane[n].fir);
        return s->dispc.plane[n].fir;
    case 0x174:	/* DISPC_VID2_PICTURE_SIZE */
        n++;
    case 0x0e4:	/* DISPC_VID1_PICTURE_SIZE */
        n++;
        TRACEDISPC("DISPC_%s_PICTURE_SIZE: 0x%08x", LAYERNAME(n),
                   s->dispc.plane[n].picture_size);
        return s->dispc.plane[n].picture_size;
    case 0x178:	/* DISPC_VID2_ACCU0 */
    case 0x17c:	/* DISPC_VID2_ACCU1 */
        n++;
    case 0x0e8:	/* DISPC_VID1_ACCU0 */
    case 0x0ec:	/* DISPC_VID1_ACCU1 */
        n++;
        TRACEDISPC("DISPC_%s_ACCU%d: 0x%08x", LAYERNAME(n),
                   (int)((addr >> 1) & 1),
                   s->dispc.plane[n].accu[(addr >> 1 ) & 1]);
        return s->dispc.plane[n].accu[(addr >> 1) & 1];
    case 0x180 ... 0x1bc:	/* DISPC_VID2_FIR_COEF */
        n++;
    case 0x0f0 ... 0x12c:	/* DISPC_VID1_FIR_COEF */
        n++;
        if (addr & 4) {
            TRACEDISPC("DISPC_%s_FIR_COEF_HV%d: 0x%08x", LAYERNAME(n),
                       (int)((addr - ((n > 1) ? 0x180 : 0xf0)) / 8),
                       s->dispc.plane[n].fir_coef_hv[
                           (addr - ((n > 1) ? 0x180 : 0xf0)) / 8]);
            return s->dispc.plane[n].fir_coef_hv[
                (addr - ((n > 1) ? 0x180 : 0xf0)) / 8];
        }
        TRACEDISPC("DISPC_%s_FIR_COEF_H%d: 0x%08x", LAYERNAME(n),
                   (int)((addr - ((n > 1) ? 0x180 : 0xf0)) / 8),
                   s->dispc.plane[n].fir_coef_h[
                       (addr - ((n > 1) ? 0x180 : 0xf0)) / 8]);
        return s->dispc.plane[n].fir_coef_h[
            (addr - ((n > 1) ? 0x180 : 0xf0)) / 8];
    case 0x1c0 ... 0x1d0: /* DISPC_VID2_CONV_COEFi */
        n++;
    case 0x130 ... 0x140: /* DISPC_VID1_CONV_COEFi */
        n++;
        TRACEDISPC("DISPC_%s_CONV_COEF%d: 0x%08x", LAYERNAME(n),
                   (int)((addr - ((n > 1) ? 0x1c0 : 0x130)) / 4),
                   s->dispc.plane[n].conv_coef[
                       (addr - ((n > 1) ? 0x1c0 : 0x130)) / 4]);
        return s->dispc.plane[n].conv_coef[
            (addr - ((n > 1) ? 0x1c0 : 0x130)) / 4];
    case 0x1d4:	/* DISPC_DATA_CYCLE1 */
    case 0x1d8:	/* DISPC_DATA_CYCLE2 */
    case 0x1dc:	/* DISPC_DATA_CYCLE3 */
        TRACEDISPC("DISPC_DATA_CYCLE%d: 0", (int)((addr - 0x1d4) / 4));
        return 0;
    case 0x200 ... 0x21c: /* DISPC_VID2_FIR_COEF_Vi */
        n++;
    case 0x1e0 ... 0x1fc: /* DISPC_VID1_FIR_COEF_Vi */
        n++;
        TRACEDISPC("DISPC_%s_FIR_COEF_V%d: 0x%08x", LAYERNAME(n),
                   (int)((addr & 0x01f) / 4),
                   s->dispc.plane[n].fir_coef_v[(addr & 0x01f) / 4]);
        return s->dispc.plane[n].fir_coef_v[(addr & 0x01f) / 4];
    case 0x220: /* DISPC_CPR_COEF_R */
        TRACEDISPC("DISPC_CPR_COEF_R: 0x%08x", s->dispc.cpr_coef_r);
        return s->dispc.cpr_coef_r;
    case 0x224: /* DISPC_CPR_COEF_G */
        TRACEDISPC("DISPC_CPR_COEF_G: 0x%08x", s->dispc.cpr_coef_g);
        return s->dispc.cpr_coef_g;
    case 0x228: /* DISPC_CPR_COEF_B */
        TRACEDISPC("DISPC_CPR_COEF_B: 0x%08x", s->dispc.cpr_coef_b);
        return s->dispc.cpr_coef_b;
    case 0x234: /* DISPC_VID2_PRELOAD */
        n++;
    case 0x230: /* DISPC_VID1_PRELOAD */
        n++;
    case 0x22c: /* DISPC_GFX_PRELOAD */
        TRACEDISPC("DISPC_%s_PRELOAD: 0x%08x", LAYERNAME(n),
                   s->dispc.plane[n].preload);
        return s->dispc.plane[n].preload;
    default:
        break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap_disc_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct omap_dss_s *s = (struct omap_dss_s *) opaque;
    uint32_t n = 0;

    switch (addr) {
    case 0x000: /* DISPC_REVISION */
    case 0x014: /* DISPC_SYSSTATUS */
    case 0x05c: /* DISPC_LINE_STATUS */
    case 0x0a8: /* DISPC_GFX_FIFO_SIZE_STATUS */
        /* quietly ignore */
        /*OMAP_RO_REGV(addr, value);*/
        break;
    case 0x010:	/* DISPC_SYSCONFIG */
        TRACEDISPC("DISPC_SYSCONFIG = 0x%08x", value);
        if (value & 2) { /* SOFTRESET */
            omap_dss_reset(s);
        }
        s->dispc.idlemode = value & ((s->dispc.rev < 0x30) ? 0x301b : 0x331f);
        break;
    case 0x018:	/* DISPC_IRQSTATUS */
        TRACEDISPC("DISPC_IRQSTATUS = 0x%08x", value);
        s->dispc.irqst &= ~value;
        omap_dss_interrupt_update(s);
        break;
    case 0x01c:	/* DISPC_IRQENABLE */
        TRACEDISPC("DISPC_IRQENABLE = 0x%08x", value);
        s->dispc.irqen = value & ((s->dispc.rev < 0x30) ? 0xffff : 0x1ffff);
        omap_dss_interrupt_update(s);
        break;
    case 0x040:	/* DISPC_CONTROL */
        TRACEDISPC("DISPC_CONTROL = 0x%08x", value);
        n = s->dispc.control; /* cache old value */
        /* always clear GODIGITAL and GOLCD to signal completed shadowing */
        if (s->dispc.rev < 0x30) {
            s->dispc.control = value & 0x07ff9f9f;
        } else {
            s->dispc.control = (value & 0xffff9b9f)
                | (s->dispc.control & 0x6000);
        }
        if (value & (1 << 12))			/* OVERLAY_OPTIMIZATION */
            if (~((s->dispc.plane[1].attr | s->dispc.plane[2].attr) & 1)) {
                 TRACEDISPC("Overlay Optimization when no overlay "
                            "region effectively exists leads to "
                            "unpredictable behaviour!");
            }
        if ((value & 0x21) && s->lcd) { /* GOLCD | LCDENABLE */
            s->lcd->controlupdate(s->lcd->opaque, &s->dispc);
        }
        if ((value & 0x42) && s->dig) { /* GODIGITAL | DIGITALENABLE */
            s->dig->controlupdate(s->dig->opaque, &s->dispc);
        }
        if (value & 1) { /* LCDENABLE */
            if ((value & (1 << 11))) { /* STALLMODE */
                if ((s->rfbi.control & 0x11) && /* ITE | ENABLE */
                    !(s->rfbi.config[0] & s->rfbi.config[1] & 0xc)) { /* TRIGGERMODE */
                    omap_rfbi_transfer_start(s);
                }
                if (s->dsi.ctrl & 1) { /* IF_EN */
                    int ch;
                    for (ch = 0; ch < 4; ch++) {
                        if ((s->dsi.vc[ch].ctrl & 1) &&     /* VC_EN */
                            (s->dsi.vc[ch].te >> 30) & 3) { /* TE_START | TE_EN */
                            omap_dsi_transfer_start(s, ch);
                        }
                    }
                }
            } else if (s->dispc.lcdframer) {
                qemu_mod_timer(s->dispc.lcdframer,
                               qemu_get_clock(vm_clock)
                               + get_ticks_per_sec() / 10);
            }
        } else if (n & 1) { /* enable -> disable, signal wip frame done */
            s->dispc.control |= 1;
            omap_dss_lcd_framedone(s);
            s->dispc.control &= ~1;
        }
        break;
    case 0x044:	/* DISPC_CONFIG */
        TRACEDISPC("DISPC_CONFIG = 0x%08x", value);
        s->dispc.config = value & 0x3fff;
        /* XXX:
         * bits 2:1 (LOADMODE) reset to 0 after set to 1 and palette loaded
         * bits 2:1 (LOADMODE) reset to 2 after set to 3 and palette loaded
         */
        break;
    case 0x048:	/* DISPC_CAPABLE */
        TRACEDISPC("DISPC_CAPABLE = 0x%08x", value);
        s->dispc.capable = value & 0x3ff;
        break;
    case 0x04c:	/* DISPC_DEFAULT_COLOR0 */
        TRACEDISPC("DISPC_DEFAULT_COLOR0 = 0x%08x", value);
        s->dispc.bg[0] = value & 0xffffff;
        break;
    case 0x050:	/* DISPC_DEFAULT_COLOR1 */
        TRACEDISPC("DISPC_DEFAULT_COLOR1 = 0x%08x", value);
        s->dispc.bg[1] = value & 0xffffff;
        break;
    case 0x054:	/* DISPC_TRANS_COLOR0 */
        TRACEDISPC("DISPC_TRANS_COLOR0 = 0x%08x", value);
        s->dispc.trans[0] = value & 0xffffff;
        break;
    case 0x058:	/* DISPC_TRANS_COLOR1 */
        TRACEDISPC("DISPC_TRANS_COLOR1 = 0x%08x", value);
        s->dispc.trans[1] = value & 0xffffff;
        break;
    case 0x060:	/* DISPC_LINE_NUMBER */
        TRACEDISPC("DISPC_LINE_NUMBER = 0x%08x", value);
        s->dispc.line = value & 0x7ff;
        break;
    case 0x064:	/* DISPC_TIMING_H */
        TRACEDISPC("DISPC_TIMING_H = 0x%08x", value);
        s->dispc.timing[0] = value & 0x0ff0ff3f;
        break;
    case 0x068:	/* DISPC_TIMING_V */
        TRACEDISPC("DISPC_TIMING_V = 0x%08x", value);
        s->dispc.timing[1] = value & 0x0ff0ff3f;
        break;
    case 0x06c:	/* DISPC_POL_FREQ */
        TRACEDISPC("DISPC_POL_FREQ = 0x%08x", value);
        s->dispc.timing[2] = value & 0x0003ffff;
        break;
    case 0x070:	/* DISPC_DIVISOR */
        TRACEDISPC("DISPC_DIVISOR = 0x%08x", value);
        s->dispc.timing[3] = value & 0x00ff00ff;
        break;
    case 0x074: /* DISPC_GLOBAL_ALPHA */
        TRACEDISPC("DISPC_GLOBAL_ALPHA = 0x%08x", value);
        s->dispc.global_alpha = value & 0x00ff00ff;
        break;
    case 0x078:	/* DISPC_SIZE_DIG */
        TRACEDISPC("DISPC_SIZE_DIG = 0x%08x (%dx%d)",
                   value, (value & 0x7ff) + 1, ((value >> 16) & 0x7ff) + 1);
        s->dispc.size_dig = value;
        break;
    case 0x07c:	/* DISPC_SIZE_LCD */
        TRACEDISPC("DISPC_SIZE_LCD = 0x%08x (%dx%d)",
                   value, (value & 0x7ff) + 1, ((value >> 16) & 0x7ff) + 1);
        s->dispc.size_lcd = value;
        break;
    case 0x14c:	/* DISPC_VID2_BA0 */
        n++;
    case 0x0bc: /* DISPC_VID1_BA0 */
        n++;
    case 0x080:	/* DISPC_GFX_BA0 */
        TRACEDISPC("DISPC_%s_BA0 = 0x%08x", LAYERNAME(n), value);
        s->dispc.plane[n].addr[0] = (target_phys_addr_t) value;
        break;
    case 0x150:	/* DISPC_VID2_BA1 */
        n++;
    case 0x0c0:	/* DISPC_VID1_BA1 */
        n++;
    case 0x084:	/* DISPC_GFX_BA1 */
        TRACEDISPC("DISPC_%s_BA1 = 0x%08x", LAYERNAME(n), value);
        s->dispc.plane[n].addr[1] = (target_phys_addr_t) value;
        break;
    case 0x154:	/* DISPC_VID2_POSITION */
        n++;
    case 0x0c4:	/* DISPC_VID1_POSITION */
        n++;
    case 0x088:	/* DISPC_GFX_POSITION */
        s->dispc.plane[n].posx = ((value >>  0) & 0x7ff);		/* GFXPOSX */
        s->dispc.plane[n].posy = ((value >> 16) & 0x7ff);		/* GFXPOSY */
        TRACEDISPC("DISPC_%s_POSITION = 0x%08x (%d,%d)", LAYERNAME(n),
                 value, s->dispc.plane[n].posx, s->dispc.plane[n].posy);
        break;
    case 0x158:	/* DISPC_VID2_SIZE */
        n++;
    case 0x0c8:	/* DISPC_VID1_SIZE */
        n++;
    case 0x08c:	/* DISPC_GFX_SIZE */
        s->dispc.plane[n].nx = ((value >>  0) & 0x7ff) + 1;		/* GFXSIZEX */
        s->dispc.plane[n].ny = ((value >> 16) & 0x7ff) + 1;		/* GFXSIZEY */
        TRACEDISPC("DISPC_%s_SIZE = 0x%08x (%dx%d)", LAYERNAME(n),
                 value, s->dispc.plane[n].nx, s->dispc.plane[n].ny);
        break;
    case 0x0a0:	/* DISPC_GFX_ATTRIBUTES */
        TRACEDISPC("DISPC_GFX_ATTRIBUTES = 0x%08x", value);
        if (cpu_is_omap3630(s->mpu)) {
            value &= 0x1000ffff;
        } else {
            value &= 0xffff;
        }
        s->dispc.plane[0].attr = value;
        if (value & (3 << 9)) {
            hw_error("%s: Big-endian pixel format not supported",
                     __FUNCTION__);
        }
        s->dispc.plane[0].enable = value & 1;
        s->dispc.plane[0].bpp = (value >> 1) & 0xf;
        s->dispc.plane[0].rotation_flag = (value >> 12) & 0x3;
        s->dispc.plane[0].gfx_format = (value >> 1) & 0xf;
        s->dispc.plane[0].gfx_channel = (value >> 8) & 0x1;
        break;
    case 0x160:	/* DISPC_VID2_FIFO_TRESHOLD */
        n++;
    case 0x0d0:	/* DISPC_VID1_FIFO_TRESHOLD */
        n++;
    case 0x0a4:	/* DISPC_GFX_FIFO_THRESHOLD */
        TRACEDISPC("DISPC_%s_FIFO_THRESHOLD = 0x%08x", LAYERNAME(n), value);
        s->dispc.plane[n].tresh = value & ((s->dispc.rev < 0x30) 
                                       ? 0x01ff01ff : 0x0fff0fff);
        break;
    case 0x168:	/* DISPC_VID2_ROW_INC */
        n++;
    case 0x0d8:	/* DISPC_VID1_ROW_INC */
        n++;
    case 0x0ac:	/* DISPC_GFX_ROW_INC */
        TRACEDISPC("DISPC_%s_ROW_INC = 0x%08x", LAYERNAME(n), value);
        s->dispc.plane[n].rowinc = value;
        break;
    case 0x16c:	/* DISPC_VID2_PIXEL_INC */
        n++;
    case 0x0dc:	/* DISPC_VID1_PIXEL_INC */
        n++;
    case 0x0b0:	/* DISPC_GFX_PIXEL_INC */
        TRACEDISPC("DISPC_%s_PIXEL_INC = 0x%08x", LAYERNAME(n), value);
        s->dispc.plane[n].colinc = value;
        break;
    case 0x0b4:	/* DISPC_GFX_WINDOW_SKIP */
        TRACEDISPC("DISPC_GFX_WINDOW_SKIP = 0x%08x", value);
        s->dispc.plane[0].wininc = value;
        break;
    case 0x0b8:	/* DISPC_GFX_TABLE_BA */
        TRACEDISPC("DISPC_GFX_TABLE_BA = 0x%08x", value);
        s->dispc.plane[0].addr[2] = (target_phys_addr_t) value;
        break;
    case 0x15c:	/* DISPC_VID2_ATTRIBUTES */
        n++;
    case 0x0cc:	/* DISPC_VID1_ATTRIBUTES */
        n++;
        TRACEDISPC("DISPC_%s_ATTRIBUTES = 0x%08x", LAYERNAME(n), value);
        if (cpu_is_omap3630(s->mpu)) {
            value &= 0x11ffffff;
        } else {
            value &= 0x01ffffff;
        }
        s->dispc.plane[n].attr = value;
        break;
    case 0x170:	/* DISPC_VID2_FIR */
        n++;
    case 0x0e0:	/* DISPC_VID1_FIR */
        n++;
        TRACEDISPC("DISPC_%s_FIR = 0x%08x", LAYERNAME(n), value);
        s->dispc.plane[n].fir = value & 0x1fff1fff;
        break;
    case 0x174:	/* DISPC_VID2_PICTURE_SIZE */
        n++;
    case 0x0e4:	/* DISPC_VID1_PICTURE_SIZE */
        n++;
        TRACEDISPC("DISPC_%s_PICTURE_SIZE = 0x%08x", LAYERNAME(n), value);
        s->dispc.plane[n].picture_size = value & 0x07ff07ff;
        break;
    case 0x178:	/* DISPC_VID2_ACCU0 */
    case 0x17c:	/* DISPC_VID2_ACCU1 */
        n++;
    case 0x0e8:	/* DISPC_VID1_ACCU0 */
    case 0x0ec:	/* DISPC_VID1_ACCU1 */
        n++;
        TRACEDISPC("DISPC_%s_ACCU%d = 0x%08x", LAYERNAME(n),
                 (int)((addr >> 1) & 1), value);
        s->dispc.plane[n].accu[(addr >> 1) & 1] = value & 0x03ff03ff;
        break;
    case 0x180 ... 0x1bc:	/* DISPC_VID2_FIR_COEF */
        n++;
    case 0x0f0 ... 0x12c:	/* DISPC_VID1_FIR_COEF */
        n++;
        if (addr & 4) {
            TRACEDISPC("DISPC_%s_FIR_COEF_HV%d = 0x%08x", LAYERNAME(n),
                     (int)((addr - ((n > 1) ? 0x180 : 0xf0)) / 8), value);
            s->dispc.plane[n].fir_coef_hv[(addr - ((n > 1) ? 0x180 : 0xf0)) / 8] = value;
        } else {
            TRACEDISPC("DISPC_%s_FIR_COEF_H%d = 0x%08x", LAYERNAME(n),
                     (int)((addr - ((n > 1) ? 0x180 : 0xf0)) / 8), value);
            s->dispc.plane[n].fir_coef_h[(addr - ((n > 1) ? 0x180 : 0xf0)) / 8] = value;
        }
        break;
    case 0x1c0 ... 0x1d0: /* DISPC_VID2_CONV_COEFi */
        n++;
    case 0x130 ... 0x140: /* DISPC_VID1_CONV_COEFi */
        n++;
        TRACEDISPC("DISPC_%s_CONV_COEF%d = 0x%08x", LAYERNAME(n),
                 (int)((addr - ((n > 1) ? 0x1c0 : 0x130)) / 4), value);
        s->dispc.plane[n].conv_coef[(addr - ((n > 1) ? 0x1c0 : 0x130)) / 4] = value;
        break;
    case 0x1d4:	/* DISPC_DATA_CYCLE1 */
    case 0x1d8:	/* DISPC_DATA_CYCLE2 */
    case 0x1dc:	/* DISPC_DATA_CYCLE3 */
        TRACEDISPC("DISPC_DATA_CYCLE%d = 0x%08x (ignored)",
                 (int)((addr - 0x1d4) / 4), value);
        break;
    case 0x200 ... 0x21c: /* DISPC_VID2_FIR_COEF_Vi */
        n++;
    case 0x1e0 ... 0x1fc: /* DISPC_VID1_FIR_COEF_Vi */
        n++;
        TRACEDISPC("DISPC_%s_FIR_COEF_V%d = 0x%08x", LAYERNAME(n),
                 (int)((addr & 0x01f) / 4), value);
        s->dispc.plane[n].fir_coef_v[(addr & 0x01f) / 4] = value & 0x0000ffff;
        break;
    case 0x220: /* DISPC_CPR_COEF_R */
        TRACEDISPC("DISPC_CPR_COEF_R = 0x%08x", value);
        s->dispc.cpr_coef_r = value & 0xffbffbff;
        break;
    case 0x224: /* DISPC_CPR_COEF_G */
        TRACEDISPC("DISPC_CPR_COEF_G = 0x%08x", value);
        s->dispc.cpr_coef_g = value & 0xffbffbff;
        break;
    case 0x228: /* DISPC_CPR_COEF_B */
        TRACEDISPC("DISPC_CPR_COEF_B = 0x%08x", value);
        s->dispc.cpr_coef_b = value & 0xffbffbff;
        break;
    case 0x234: /* DISPC_VID2_PRELOAD */
        n++;
    case 0x230: /* DISPC_VID1_PRELOAD */
        n++;
    case 0x22c: /* DISPC_GFX_PRELOAD */
        TRACEDISPC("DISPC_%s_PRELOAD = 0x%08x", LAYERNAME(n), value);
        s->dispc.plane[n].preload = value & 0x0fff;
        break;
    default:
        OMAP_BAD_REGV(addr, value);
        break;
    }
}

static CPUReadMemoryFunc * const omap_disc1_readfn[] = {
    omap_badwidth_read32,
    omap_badwidth_read32,
    omap_disc_read,
};

static CPUWriteMemoryFunc * const omap_disc1_writefn[] = {
    omap_badwidth_write32,
    omap_badwidth_write32,
    omap_disc_write,
};

static uint32_t omap_rfbi_read(void *opaque, target_phys_addr_t addr)
{
    struct omap_dss_s *s = (struct omap_dss_s *) opaque;

    switch (addr) {
    case 0x00:	/* RFBI_REVISION */
        TRACERFBI("RFBI_REVISION: 0x10");
        return 0x10;

    case 0x10:	/* RFBI_SYSCONFIG */
        TRACERFBI("RFBI_SYSCONFIG: 0x%08x", s->rfbi.idlemode);
        return s->rfbi.idlemode;

    case 0x14:	/* RFBI_SYSSTATUS */
        TRACERFBI("RFBI_SYSSTATUS: 0x%08x", 1 | (s->rfbi.busy << 8));
        return 1 | (s->rfbi.busy << 8); /* RESETDONE */

    case 0x40:	/* RFBI_CONTROL */
        TRACERFBI("RFBI_CONTROL: 0x%08x", s->rfbi.control);
        return s->rfbi.control;

    case 0x44:	/* RFBI_PIXELCNT */
        TRACERFBI("RFBI_PIXELCNT: 0x%08x", s->rfbi.pixels);
        return s->rfbi.pixels;

    case 0x48:	/* RFBI_LINE_NUMBER */
        TRACERFBI("RFBI_LINE_NUMBER: 0x%08x", s->rfbi.skiplines);
        return s->rfbi.skiplines;

    case 0x58:	/* RFBI_READ */
    case 0x5c:	/* RFBI_STATUS */
        TRACERFBI("RFBI_READ/STATUS: 0x%08x", s->rfbi.rxbuf);
        return s->rfbi.rxbuf;

    case 0x60:	/* RFBI_CONFIG0 */
        TRACERFBI("RFBI_CONFIG0: 0x%08x", s->rfbi.config[0]);
        return s->rfbi.config[0];
    case 0x64:	/* RFBI_ONOFF_TIME0 */
        TRACERFBI("RFBI_ONOFF_TIME0: 0x%08x", s->rfbi.time[0]);
        return s->rfbi.time[0];
    case 0x68:	/* RFBI_CYCLE_TIME0 */
        TRACERFBI("RFBI_CYCLE_TIME0: 0x%08x", s->rfbi.time[1]);
        return s->rfbi.time[1];
    case 0x6c:	/* RFBI_DATA_CYCLE1_0 */
        TRACERFBI("RFBI_DATA_CYCLE1_0: 0x%08x", s->rfbi.data[0]);
        return s->rfbi.data[0];
    case 0x70:	/* RFBI_DATA_CYCLE2_0 */
        TRACERFBI("RFBI_DATA_CYCLE2_0: 0x%08x", s->rfbi.data[1]);
        return s->rfbi.data[1];
    case 0x74:	/* RFBI_DATA_CYCLE3_0 */
        TRACERFBI("RFBI_DATA_CYCLE3_0: 0x%08x", s->rfbi.data[2]);
        return s->rfbi.data[2];

    case 0x78:	/* RFBI_CONFIG1 */
        TRACERFBI("RFBI_CONFIG1: 0x%08x", s->rfbi.config[1]);
        return s->rfbi.config[1];
    case 0x7c:	/* RFBI_ONOFF_TIME1 */
        TRACERFBI("RFBI_ONOFF_TIME1: 0x%08x", s->rfbi.time[2]);
        return s->rfbi.time[2];
    case 0x80:	/* RFBI_CYCLE_TIME1 */
        TRACERFBI("RFBI_CYCLE_TIME1: 0x%08x", s->rfbi.time[3]);
        return s->rfbi.time[3];
    case 0x84:	/* RFBI_DATA_CYCLE1_1 */
        TRACERFBI("RFBI_DATA_CYCLE1_1: 0x%08x", s->rfbi.data[3]);
        return s->rfbi.data[3];
    case 0x88:	/* RFBI_DATA_CYCLE2_1 */
        TRACERFBI("RFBI_DATA_CYCLE2_1: 0x%08x", s->rfbi.data[4]);
        return s->rfbi.data[4];
    case 0x8c:	/* RFBI_DATA_CYCLE3_1 */
        TRACERFBI("RFBI_DATA_CYCLE3_1: 0x%08x", s->rfbi.data[5]);
        return s->rfbi.data[5];

    case 0x90:	/* RFBI_VSYNC_WIDTH */
        TRACERFBI("RFBI_VSYNC_WIDTH: 0x%08x", s->rfbi.vsync);
        return s->rfbi.vsync;
    case 0x94:	/* RFBI_HSYNC_WIDTH */
        TRACERFBI("RFBI_HSYNC_WIDTH: 0x%08x", s->rfbi.hsync);
        return s->rfbi.hsync;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap_rfbi_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    struct omap_dss_s *s = (struct omap_dss_s *) opaque;

    switch (addr) {
    case 0x10:	/* RFBI_SYSCONFIG */
        TRACERFBI("RFBI_SYSCONFIG = 0x%08x", value);
        if (value & 2)						/* SOFTRESET */
            omap_rfbi_reset(s);
        s->rfbi.idlemode = value & 0x19;
        break;

    case 0x40:	/* RFBI_CONTROL */
        TRACERFBI("RFBI_CONTROL = 0x%08x", value);
        if (s->dispc.rev < 0x30) 
            s->rfbi.control = value & 0x1f;
        else
            s->rfbi.control = value & 0x1ff; 
        s->rfbi.enable = value & 1;
        if ((s->dispc.control & 1) && /* LCDENABLE */
            (value & 0x10) &&         /* ITE */
            !(s->rfbi.config[0] & s->rfbi.config[1] & 0xc)) /* TRIGGERMODE */
            omap_rfbi_transfer_start(s);
        break;

    case 0x44:	/* RFBI_PIXELCNT */
        TRACERFBI("RFBI_PIXELCNT = 0x%08x", value);
        s->rfbi.pixels = value;
        break;

    case 0x48:	/* RFBI_LINE_NUMBER */
        TRACERFBI("RFBI_LINE_NUMBER = 0x%08x", value);
        s->rfbi.skiplines = value & 0x7ff;
        break;

    case 0x4c:	/* RFBI_CMD */
        TRACERFBI("RFBI_CMD = 0x%08x", value);
        if ((s->rfbi.control & (1 << 2)) && s->rfbi.chip[0])
            s->rfbi.chip[0]->write(s->rfbi.chip[0]->opaque, 0, value & 0xffff);
        if ((s->rfbi.control & (1 << 3)) && s->rfbi.chip[1])
            s->rfbi.chip[1]->write(s->rfbi.chip[1]->opaque, 0, value & 0xffff);
        break;
    case 0x50:	/* RFBI_PARAM */
        TRACERFBI("RFBI_PARAM = 0x%08x", value);
        if ((s->rfbi.control & (1 << 2)) && s->rfbi.chip[0])
            s->rfbi.chip[0]->write(s->rfbi.chip[0]->opaque, 1, value & 0xffff);
        if ((s->rfbi.control & (1 << 3)) && s->rfbi.chip[1])
            s->rfbi.chip[1]->write(s->rfbi.chip[1]->opaque, 1, value & 0xffff);
        break;
    case 0x54:	/* RFBI_DATA */
        TRACERFBI("RFBI_DATA = 0x%08x", value);
        /* TODO: take into account the format set up in s->rfbi.config[?] and
         * s->rfbi.data[?], but special-case the most usual scenario so that
         * speed doesn't suffer.  */
        if ((s->rfbi.control & (1 << 2)) && s->rfbi.chip[0]) {
            s->rfbi.chip[0]->write(s->rfbi.chip[0]->opaque, 1, value & 0xffff);
            s->rfbi.chip[0]->write(s->rfbi.chip[0]->opaque, 1, value >> 16);
        }
        if ((s->rfbi.control & (1 << 3)) && s->rfbi.chip[1]) {
            s->rfbi.chip[1]->write(s->rfbi.chip[1]->opaque, 1, value & 0xffff);
            s->rfbi.chip[1]->write(s->rfbi.chip[1]->opaque, 1, value >> 16);
        }
        if (!-- s->rfbi.pixels)
            omap_rfbi_transfer_stop(s);
        break;
    case 0x58:	/* RFBI_READ */
        TRACERFBI("RFBI_READ = 0x%08x", value);
        if ((s->rfbi.control & (1 << 2)) && s->rfbi.chip[0])
            s->rfbi.rxbuf = s->rfbi.chip[0]->read(s->rfbi.chip[0]->opaque, 1);
        else if ((s->rfbi.control & (1 << 3)) && s->rfbi.chip[1])
            s->rfbi.rxbuf = s->rfbi.chip[0]->read(s->rfbi.chip[0]->opaque, 1);
        if (!-- s->rfbi.pixels)
            omap_rfbi_transfer_stop(s);
        break;

    case 0x5c:	/* RFBI_STATUS */
        TRACERFBI("RFBI_STATUS = 0x%08x", value);
        if ((s->rfbi.control & (1 << 2)) && s->rfbi.chip[0])
            s->rfbi.rxbuf = s->rfbi.chip[0]->read(s->rfbi.chip[0]->opaque, 0);
        else if ((s->rfbi.control & (1 << 3)) && s->rfbi.chip[1])
            s->rfbi.rxbuf = s->rfbi.chip[0]->read(s->rfbi.chip[0]->opaque, 0);
        if (!-- s->rfbi.pixels)
            omap_rfbi_transfer_stop(s);
        break;

    case 0x60:	/* RFBI_CONFIG0 */
        TRACERFBI("RFBI_CONFIG0 = 0x%08x", value);
        s->rfbi.config[0] = value & 0x003f1fff;
        break;

    case 0x64:	/* RFBI_ONOFF_TIME0 */
        TRACERFBI("RFBI_ONOFF_TIME0 = 0x%08x", value);
        s->rfbi.time[0] = value & 0x3fffffff;
        break;
    case 0x68:	/* RFBI_CYCLE_TIME0 */
        TRACERFBI("RFBI_CYCLE_TIME0 = 0x%08x", value);
        s->rfbi.time[1] = value & 0x0fffffff;
        break;
    case 0x6c:	/* RFBI_DATA_CYCLE1_0 */
        TRACERFBI("RFBI_DATA_CYCLE1_0 = 0x%08x", value);
        s->rfbi.data[0] = value & 0x0f1f0f1f;
        break;
    case 0x70:	/* RFBI_DATA_CYCLE2_0 */
        TRACERFBI("RFBI_DATA_CYCLE2_0 = 0x%08x", value);
        s->rfbi.data[1] = value & 0x0f1f0f1f;
        break;
    case 0x74:	/* RFBI_DATA_CYCLE3_0 */
        TRACERFBI("RFBI_DATA_CYCLE3_0 = 0x%08x", value);
        s->rfbi.data[2] = value & 0x0f1f0f1f;
        break;
    case 0x78:	/* RFBI_CONFIG1 */
        TRACERFBI("RFBI_CONFIG1 = 0x%08x", value);
        s->rfbi.config[1] = value & 0x003f1fff;
        break;

    case 0x7c:	/* RFBI_ONOFF_TIME1 */
        TRACERFBI("RFBI_ONOFF_TIME1 = 0x%08x", value);
        s->rfbi.time[2] = value & 0x3fffffff;
        break;
    case 0x80:	/* RFBI_CYCLE_TIME1 */
        TRACERFBI("RFBI_CYCLE_TIME1 = 0x%08x", value);
        s->rfbi.time[3] = value & 0x0fffffff;
        break;
    case 0x84:	/* RFBI_DATA_CYCLE1_1 */
        TRACERFBI("RFBI_DATA_CYCLE1_1 = 0x%08x", value);
        s->rfbi.data[3] = value & 0x0f1f0f1f;
        break;
    case 0x88:	/* RFBI_DATA_CYCLE2_1 */
        TRACERFBI("RFBI_DATA_CYCLE2_1 = 0x%08x", value);
        s->rfbi.data[4] = value & 0x0f1f0f1f;
        break;
    case 0x8c:	/* RFBI_DATA_CYCLE3_1 */
        TRACERFBI("RFBI_DATA_CYCLE3_1 = 0x%08x", value);
        s->rfbi.data[5] = value & 0x0f1f0f1f;
        break;

    case 0x90:	/* RFBI_VSYNC_WIDTH */
        TRACERFBI("RFBI_VSYNC_WIDTH = 0x%08x", value);
        s->rfbi.vsync = value & 0xffff;
        break;
    case 0x94:	/* RFBI_HSYNC_WIDTH */
        TRACERFBI("RFBI_HSYNC_WIDTH = 0x%08x", value);
        s->rfbi.hsync = value & 0xffff;
        break;

    default:
        OMAP_BAD_REGV(addr, value);
        break;
    }
}

static CPUReadMemoryFunc * const omap_rfbi1_readfn[] = {
    omap_badwidth_read32,
    omap_badwidth_read32,
    omap_rfbi_read,
};

static CPUWriteMemoryFunc * const omap_rfbi1_writefn[] = {
    omap_badwidth_write32,
    omap_badwidth_write32,
    omap_rfbi_write,
};

static uint32_t omap_venc_read(void *opaque, target_phys_addr_t addr)
{
    switch (addr) {
        case 0x00:	/* REV_ID */
            return 0x2;
        case 0x04:	/* STATUS */
        case 0x08:	/* F_CONTROL */
        case 0x10:	/* VIDOUT_CTRL */
        case 0x14:	/* SYNC_CTRL */
        case 0x1c:	/* LLEN */
        case 0x20:	/* FLENS */
        case 0x24:	/* HFLTR_CTRL */
        case 0x28:	/* CC_CARR_WSS_CARR */
        case 0x2c:	/* C_PHASE */
        case 0x30:	/* GAIN_U */
        case 0x34:	/* GAIN_V */
        case 0x38:	/* GAIN_Y */
        case 0x3c:	/* BLACK_LEVEL */
        case 0x40:	/* BLANK_LEVEL */
        case 0x44:	/* X_COLOR */
        case 0x48:	/* M_CONTROL */
        case 0x4c:	/* BSTAMP_WSS_DATA */
        case 0x50:	/* S_CARR */
        case 0x54:	/* LINE21 */
        case 0x58:	/* LN_SEL */
        case 0x5c:	/* L21__WC_CTL */
        case 0x60:	/* HTRIGGER_VTRIGGER */
        case 0x64:	/* SAVID__EAVID */
        case 0x68:	/* FLEN__FAL */
        case 0x6c:	/* LAL__PHASE_RESET */
        case 0x70:	/* HS_INT_START_STOP_X */
        case 0x74:	/* HS_EXT_START_STOP_X */
        case 0x78:	/* VS_INT_START_X */
        case 0x7c:	/* VS_INT_STOP_X__VS_INT_START_Y */
        case 0x80:	/* VS_INT_STOP_Y__VS_INT_START_X */
        case 0x84:	/* VS_EXT_STOP_X__VS_EXT_START_Y */
        case 0x88:	/* VS_EXT_STOP_Y */
        case 0x90:	/* AVID_START_STOP_X */
        case 0x94:	/* AVID_START_STOP_Y */
        case 0xa0:	/* FID_INT_START_X__FID_INT_START_Y */
        case 0xa4:	/* FID_INT_OFFSET_Y__FID_EXT_START_X */
        case 0xa8:	/* FID_EXT_START_Y__FID_EXT_OFFSET_Y */
        case 0xb0:	/* TVDETGP_INT_START_STOP_X */
        case 0xb4:	/* TVDETGP_INT_START_STOP_Y */
        case 0xb8:	/* GEN_CTRL */
        case 0xc4:	/* DAC_TST__DAC_A */
        case 0xc8:	/* DAC_B__DAC_C */
            return 0;
            
        default:
            break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap_venc_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    switch (addr) {
        case 0x00: /* REV_ID */
        case 0x04: /* STATUS */
            /* read-only, ignore */
            break;
        case 0x08:	/* F_CONTROL */
        case 0x10:	/* VIDOUT_CTRL */
        case 0x14:	/* SYNC_CTRL */
        case 0x1c:	/* LLEN */
        case 0x20:	/* FLENS */
        case 0x24:	/* HFLTR_CTRL */
        case 0x28:	/* CC_CARR_WSS_CARR */
        case 0x2c:	/* C_PHASE */
        case 0x30:	/* GAIN_U */
        case 0x34:	/* GAIN_V */
        case 0x38:	/* GAIN_Y */
        case 0x3c:	/* BLACK_LEVEL */
        case 0x40:	/* BLANK_LEVEL */
        case 0x44:	/* X_COLOR */
        case 0x48:	/* M_CONTROL */
        case 0x4c:	/* BSTAMP_WSS_DATA */
        case 0x50:	/* S_CARR */
        case 0x54:	/* LINE21 */
        case 0x58:	/* LN_SEL */
        case 0x5c:	/* L21__WC_CTL */
        case 0x60:	/* HTRIGGER_VTRIGGER */
        case 0x64:	/* SAVID__EAVID */
        case 0x68:	/* FLEN__FAL */
        case 0x6c:	/* LAL__PHASE_RESET */
        case 0x70:	/* HS_INT_START_STOP_X */
        case 0x74:	/* HS_EXT_START_STOP_X */
        case 0x78:	/* VS_INT_START_X */
        case 0x7c:	/* VS_INT_STOP_X__VS_INT_START_Y */
        case 0x80:	/* VS_INT_STOP_Y__VS_INT_START_X */
        case 0x84:	/* VS_EXT_STOP_X__VS_EXT_START_Y */
        case 0x88:	/* VS_EXT_STOP_Y */
        case 0x90:	/* AVID_START_STOP_X */
        case 0x94:	/* AVID_START_STOP_Y */
        case 0xa0:	/* FID_INT_START_X__FID_INT_START_Y */
        case 0xa4:	/* FID_INT_OFFSET_Y__FID_EXT_START_X */
        case 0xa8:	/* FID_EXT_START_Y__FID_EXT_OFFSET_Y */
        case 0xb0:	/* TVDETGP_INT_START_STOP_X */
        case 0xb4:	/* TVDETGP_INT_START_STOP_Y */
        case 0xb8:	/* GEN_CTRL */
        case 0xc4:	/* DAC_TST__DAC_A */
        case 0xc8:	/* DAC_B__DAC_C */
            break;
            
        default:
            OMAP_BAD_REGV(addr, value);
            break;
    }
}

static CPUReadMemoryFunc * const omap_venc1_readfn[] = {
    omap_badwidth_read32,
    omap_badwidth_read32,
    omap_venc_read,
};

static CPUWriteMemoryFunc * const omap_venc1_writefn[] = {
    omap_badwidth_write32,
    omap_badwidth_write32,
    omap_venc_write,
};

static uint32_t omap_im3_read(void *opaque, target_phys_addr_t addr)
{
    switch (addr) {
    case 0x0a8:	/* SBIMERRLOGA */
    case 0x0b0:	/* SBIMERRLOG */
    case 0x190:	/* SBIMSTATE */
    case 0x198:	/* SBTMSTATE_L */
    case 0x19c:	/* SBTMSTATE_H */
    case 0x1a8:	/* SBIMCONFIG_L */
    case 0x1ac:	/* SBIMCONFIG_H */
    case 0x1f8:	/* SBID_L */
    case 0x1fc:	/* SBID_H */
        return 0;

    default:
        break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap_im3_write(void *opaque, target_phys_addr_t addr,
                uint32_t value)
{
    switch (addr) {
    case 0x0b0:	/* SBIMERRLOG */
    case 0x190:	/* SBIMSTATE */
    case 0x198:	/* SBTMSTATE_L */
    case 0x19c:	/* SBTMSTATE_H */
    case 0x1a8:	/* SBIMCONFIG_L */
    case 0x1ac:	/* SBIMCONFIG_H */
        break;

    default:
        OMAP_BAD_REGV(addr, value);
        break;
    }
}

static CPUReadMemoryFunc * const omap_im3_readfn[] = {
    omap_badwidth_read32,
    omap_badwidth_read32,
    omap_im3_read,
};

static CPUWriteMemoryFunc * const omap_im3_writefn[] = {
    omap_badwidth_write32,
    omap_badwidth_write32,
    omap_im3_write,
};

static void omap_dsi_push_rx_fifo(struct omap_dss_s *s, int ch, uint32_t value)
{
    int p;
    
    if (s->dsi.vc[ch].rx_fifo_len < OMAP_DSI_RX_FIFO_SIZE) {
        p = s->dsi.vc[ch].rx_fifo_pos + s->dsi.vc[ch].rx_fifo_len;
        if (p >= OMAP_DSI_RX_FIFO_SIZE)
            p -= OMAP_DSI_RX_FIFO_SIZE;
        s->dsi.vc[ch].rx_fifo[p] = value;
        s->dsi.vc[ch].rx_fifo_len++;
    } else {
        TRACEDSI("vc%d rx fifo overflow!", ch);
    }
}

static uint32_t omap_dsi_pull_rx_fifo(struct omap_dss_s *s, int ch)
{
    int v = 0;
    
    if (!s->dsi.vc[ch].rx_fifo_len) {
        TRACEDSI("vc%d rx fifo underflow!", ch);
    } else {
        v = s->dsi.vc[ch].rx_fifo[s->dsi.vc[ch].rx_fifo_pos++];
        s->dsi.vc[ch].rx_fifo_len--;
        if (s->dsi.vc[ch].rx_fifo_pos >= OMAP_DSI_RX_FIFO_SIZE)
            s->dsi.vc[ch].rx_fifo_pos = 0;
    }
    
    return v;
}

static uint32_t omap_dsi_read(void *opaque, target_phys_addr_t addr)
{
    struct omap_dss_s *s = (struct omap_dss_s *)opaque;
    uint32_t x, y;
    
    switch (addr) {
        case 0x000: /* DSI_REVISION */
            TRACEDSI("DSI_REVISION = 0x10");
            return 0x10;
        case 0x010: /* DSI_SYSCONFIG */
            TRACEDSI("DSI_SYSCONFIG = 0x%04x", s->dsi.sysconfig);
            return s->dsi.sysconfig;
        case 0x014: /* DSI_SYSSTATUS */
            TRACEDSI("DSI_SYSSTATUS = 0x01");
            return 1; /* RESET_DONE */
        case 0x018: /* DSI_IRQSTATUS */
            TRACEDSI("DSI_IRQSTATUS = 0x%08x", s->dsi.irqst);
            return s->dsi.irqst;
        case 0x01c: /* DSI_IRQENABLE */
            TRACEDSI("DSI_IRQENABLE = 0x%08x", s->dsi.irqen);
            return s->dsi.irqen;
        case 0x040: /* DSI_CTRL */
            TRACEDSI("DSI_CTRL = 0x%08x", s->dsi.ctrl);
            return s->dsi.ctrl;
        case 0x048: /* DSI_COMPLEXIO_CFG1 */
            TRACEDSI("DSI_COMPLEXIO_CFG1 = 0x%08x", s->dsi.complexio_cfg1);
            return s->dsi.complexio_cfg1;
        case 0x04c: /* DSI_COMPLEXIO_IRQSTATUS */
            TRACEDSI("DSI_COMPLEXIO_IRQSTATUS = 0x%08x", s->dsi.complexio_irqst);
            return s->dsi.complexio_irqst;
        case 0x050: /* DSI_COMPLEXIO_IRQENABLE */
            TRACEDSI("DSI_COMPLEXIO_IRQENABLE = 0x%08x", s->dsi.complexio_irqen);
            return s->dsi.complexio_irqen;
        case 0x054: /* DSI_CLK_CTRL */
            TRACEDSI("DSI_CLK_CTRL = 0x%08x", s->dsi.clk_ctrl);
            return s->dsi.clk_ctrl;
        case 0x058: /* DSI_TIMING1 */
            TRACEDSI("DSI_TIMING1 = 0x%08x", s->dsi.timing1);
            return s->dsi.timing1;
        case 0x05c: /* DSI_TIMING2 */
            TRACEDSI("DSI_TIMING2 = 0x%08x", s->dsi.timing2);
            return s->dsi.timing2;
        case 0x060: /* DSI_VM_TIMING1 */
            TRACEDSI("DSI_VM_TIMING1 = 0x%08x", s->dsi.vm_timing1);
            return s->dsi.vm_timing1;
        case 0x064: /* DSI_VM_TIMING2 */
            TRACEDSI("DSI_VM_TIMING2 = 0x%08x", s->dsi.vm_timing2);
            return s->dsi.vm_timing2;
        case 0x068: /* DSI_VM_TIMING3 */
            TRACEDSI("DSI_VM_TIMING3 = 0x%08x", s->dsi.vm_timing3);
            return s->dsi.vm_timing3;
        case 0x06c: /* DSI_CLK_TIMING */
            TRACEDSI("DSI_CLK_TIMING = 0x%08x", s->dsi.clk_timing);
            return s->dsi.clk_timing;
        case 0x070: /* DSI_TX_FIFO_VC_SIZE */
            TRACEDSI("DSI_TX_FIFO_VC_SIZE = 0x%08x", s->dsi.tx_fifo_vc_size);
            return s->dsi.tx_fifo_vc_size;
        case 0x074: /* DSI_RX_FIFO_VC_SIZE */
            TRACEDSI("DSI_RX_FIFO_VC_SIZE = 0x%08x", s->dsi.rx_fifo_vc_size);
            return s->dsi.rx_fifo_vc_size;
        case 0x078: /* DSI_COMPLEXIO_CFG_2 */
            TRACEDSI("DSI_COMPLEXIO_CFG_2 = 0x%08x", s->dsi.complexio_cfg2);
            return s->dsi.complexio_cfg2;
        case 0x07c: /* DSI_RX_FIFO_VC_FULLNESS */
            TRACEDSI("DSI_RX_FIFO_VC_FULLNESS = 0x00");
            return 0;
        case 0x080: /* DSI_VM_TIMING4 */
            TRACEDSI("DSI_VM_TIMING4 = 0x%08x", s->dsi.vm_timing4);
            return s->dsi.vm_timing4;
        case 0x084: /* DSI_TX_FIFO_VC_EMPTINESS */
            TRACEDSI("DSI_TX_FIFO_VC_EMPTINESS = 0x7f7f7f7f");
            return 0x7f7f7f7f;
        case 0x088: /* DSI_VM_TIMING5 */
            TRACEDSI("DSI_VM_TIMING5 = 0x%08x", s->dsi.vm_timing5);
            return s->dsi.vm_timing5;
        case 0x08c: /* DSI_VM_TIMING6 */
            TRACEDSI("DSI_VM_TIMING6 = 0x%08x", s->dsi.vm_timing6);
            return s->dsi.vm_timing6;
        case 0x090: /* DSI_VM_TIMING7 */
            TRACEDSI("DSI_VM_TIMING7 = 0x%08x", s->dsi.vm_timing7);
            return s->dsi.vm_timing7;
        case 0x094: /* DSI_STOPCLK_TIMING */
            TRACEDSI("DSI_STOPCLK_TIMING = 0x%08x", s->dsi.stopclk_timing);
            return s->dsi.stopclk_timing;
        case 0x100 ... 0x17c: /* DSI_VCx_xxx */
            x = (addr >> 5) & 3;
            switch (addr & 0x1f) {
                case 0x00: /* DSI_VCx_CTRL */
                    TRACEDSI("DSI_VC%d_CTRL = 0x%08x", x, s->dsi.vc[x].ctrl);
                    return s->dsi.vc[x].ctrl;
                case 0x04: /* DSI_VCx_TE */
                    TRACEDSI("DSI_VC%d_TE = 0x%08x", x, s->dsi.vc[x].te);
                    return s->dsi.vc[x].te;
                case 0x08: /* DSI_VCx_LONG_PACKET_HEADER */
                    /* write-only */
                    TRACEDSI("DSI_VC%d_LONG_PACKET_HEADER = 0", x);
                    return 0;
                case 0x0c: /* DSI_VCx_LONG_PACKET_PAYLOAD */
                    /* write-only */
                    TRACEDSI("DSI_VC%d_LONG_PACKET_PAYLOAD = 0", x);
                    return 0;
                case 0x10: /* DSI_VCx_SHORT_PACKET_HEADER */
                    if (s->dsi.vc[x].ctrl & (1 << 20)) { /* RX_FIFO_NOT_EMPTY */
                        y = omap_dsi_pull_rx_fifo(s, x);
                        TRACEDSI("DSI_VC%d_SHORT_PACKET_HEADER = 0x%08x", x, y);
                        if (!s->dsi.vc[x].rx_fifo_len)
                            s->dsi.vc[x].ctrl &= ~(1 << 20); /* RX_FIFO_NOT_EMPTY */
                        return y;
                    }
                    TRACEDSI("vc%d rx fifo underflow!", x);
                    return 0;
                case 0x18: /* DSI_VCx_IRQSTATUS */
                    TRACEDSI("DSI_VC%d_IRQSTATUS = 0x%08x", x, s->dsi.vc[x].irqst);
                    return s->dsi.vc[x].irqst;
                case 0x1c: /* DSI_VCx_IRQENABLE */
                    TRACEDSI("DSI_VC%d_IRQENABLE = 0x%08x", x, s->dsi.vc[x].irqen);
                    return s->dsi.vc[x].irqen;
                default:
                    OMAP_BAD_REG(addr);
            }
            break;
        
        case 0x200: /* DSI_PHY_CFG0 */
            TRACEDSI("DSI_PHY_CFG0 = 0x%08x", s->dsi.phy_cfg0);
            return s->dsi.phy_cfg0;
        case 0x204: /* DSI_PHY_CFG1 */
            TRACEDSI("DSI_PHY_CFG1 = 0x%08x", s->dsi.phy_cfg1);
            return s->dsi.phy_cfg1;
        case 0x208: /* DSI_PHY_CFG2 */
            TRACEDSI("DSI_PHY_CFG2 = 0x%08x", s->dsi.phy_cfg2);
            return s->dsi.phy_cfg2;
        case 0x214: /* DSI_PHY_CFG5 */
            TRACEDSI("DSI_PHY_CFG5 = 0xfc000000");
            return 0xfc000000; /* all resets done */
            
        case 0x300: /* DSI_PLL_CONTROL */
            TRACEDSI("DSI_PLL_CONTROL = 0x%08x", s->dsi.pll_control);
            return s->dsi.pll_control;
        case 0x304: /* DSI_PLL_STATUS */
            x = 1; /* DSI_PLLCTRL_RESET_DONE */
            if ((s->dsi.clk_ctrl >> 28) & 3) { /* DSI PLL control powered? */
                if (((s->dsi.pll_config1 >> 1) & 0x7f) &&  /* DSI_PLL_REGN */
                    ((s->dsi.pll_config1 >> 8) & 0x7ff)) { /* DSI_PLL_REGM */
                    x |= 2; /* DSI_PLL_LOCK */
                }
            }
            if ((s->dsi.pll_config2 >> 20) & 1) /* DSI_HSDIVBYPASS */
                x |= (1 << 9);                  /* DSI_BYPASSACKZ */
            if ((s->dsi.pll_config2 >> 18) & 1) /* DSI_PROTO_CLOCK_EN */
                x |= (1 << 8);                  /* DSIPROTO_CLOCK_ACK */
            if ((s->dsi.pll_config2 >> 16) & 1) /* DSS_CLOCK_EN */
                x |= (1 << 7);                  /* DSS_CLOCK_ACK */
            if (!((s->dsi.pll_config2 >> 13) & 1)) /* DSI_PLL_REFEN */
                x |= (1 << 3);                     /* DSI_PLL_LOSSREF */
            TRACEDSI("DSI_PLL_STATUS = 0x%08x", x);
            return x;
        case 0x308: /* DSI_PLL_GO */
            TRACEDSI("DSI_PLL_GO = 0x%08x", s->dsi.pll_go);
            return s->dsi.pll_go;
        case 0x30c: /* DSI_PLL_CONFIGURATION1 */
            TRACEDSI("DSI_PLL_CONFIGURATION1 = 0x%08x", s->dsi.pll_config1);
            return s->dsi.pll_config1;
        case 0x310: /* DSI_PLL_CONFIGURATION2 */
            TRACEDSI("DSI_PLL_CONFIGURATION2 = 0x%08x", s->dsi.pll_config2);
            return s->dsi.pll_config2;
            
        default:
            break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap_dsi_txdone(struct omap_dss_s *s, int ch, int bta)
{
    if (bta) {
        s->dsi.vc[ch].irqst |= 0x20;       /* BTA_IRQ */
        if (s->dsi.vc[ch].rx_fifo_len)
            s->dsi.vc[ch].ctrl |= 1 << 20; /* RX_FIFO_NOT_EMPTY */
    } else {
        s->dsi.vc[ch].irqst |= 0x04;       /* PACKET_SENT_IRQ */
    }
    s->dsi.irqst |= 1 << ch;               /* VIRTUAL_CHANNELx_IRQ */
    omap_dss_interrupt_update(s);
}

static void omap_dsi_short_write(struct omap_dss_s *s, int ch)
{
    uint32_t data = s->dsi.vc[ch].sp_header;
    
    if (((data >> 6) & 0x03) != ch) {
        TRACEDSI("error - vc%d != %d", ch, (data >> 6) & 0x03);
    } else {
        if (s->dsi.vc[ch].chip) {
            switch (data & 0x3f) { /* id */
                case 0x05: /* short_write_0 */
                    s->dsi.vc[ch].chip->write(s->dsi.vc[ch].chip->opaque,
                                              (data >> 8) & 0xff, 1);
                    break;
                case 0x06: /* read */
                    data = s->dsi.vc[ch].chip->read(s->dsi.vc[ch].chip->opaque,
                                                    (data >> 8) & 0xff, 1);
                    /* responses cannot be all-zero so it is safe to use that
                     * as a no-reply value */
                    if (data) {
                        omap_dsi_push_rx_fifo(s, ch, data);
                    }
                    break;
                case 0x15: /* short_write_1 */
                    s->dsi.vc[ch].chip->write(s->dsi.vc[ch].chip->opaque,
                                              (data >> 8) & 0xffff, 2);
                    break;
                case 0x37: /* set maximum return packet size */
                    s->dsi.vc[ch].chip->max_return_size = (data >> 8) & 0xffff;
                    break;
                default:
                    hw_error("%s: unknown DSI id (0x%02x)",
                            __FUNCTION__, data & 0x3f);
                    break;
            }
        } else {
            TRACEDSI("ERROR - no DSI chip attached on channel %d", ch);
        }
        omap_dsi_txdone(s, ch, (s->dsi.vc[ch].ctrl & 0x04)); /* BTA_SHORT_EN */
    }
}

static void omap_dsi_long_write(struct omap_dss_s *s, int ch)
{
    uint32_t hdr = s->dsi.vc[ch].lp_header;

    /* TODO: implement packet footer sending (16bit checksum).
     * Currently none is sent and receiver is supposed to not expect one */
    if (((hdr >> 6) & 0x03) != ch) {
        TRACEDSI("error - vc%d != %d", ch, (hdr >> 6) & 0x03);
    } else {
        if (s->dsi.vc[ch].chip) {
            switch (hdr & 0x3f) { /* id */
                case 0x09: /* null packet */
                    TRACEDSI("null packet");
                    break;
                case 0x39: /* long write */
                    s->dsi.vc[ch].chip->write(s->dsi.vc[ch].chip->opaque,
                                              s->dsi.vc[ch].lp_payload,
                                              s->dsi.vc[ch].lp_counter > 4
                                              ? 4 : s->dsi.vc[ch].lp_counter);
                    break;
                default:
                    hw_error("%s: unknown DSI id (0x%02x)",
                            __FUNCTION__, hdr & 0x3f);
                    break;
            }
        } else {
            TRACEDSI("ERROR - no DSI chip attached on channel %d", ch);
        }
        if ((s->dsi.vc[ch].te >> 30) & 3) {     /* TE_START | TE_EN */
            /* TODO: do we really need to implement something for this?
             * Should writes decrease the TE_SIZE counter, for example?
             * For now, the TE transfers are completed immediately */
        } else {
            if (s->dsi.vc[ch].lp_counter > 0)
                s->dsi.vc[ch].lp_counter -= 4;
            if (s->dsi.vc[ch].lp_counter <= 0)
                omap_dsi_txdone(s, ch, (s->dsi.vc[ch].ctrl & 0x08)); /* BTA_LONG_EN */
        }
    }
}

static void omap_dsi_write(void *opaque, target_phys_addr_t addr,
                           uint32_t value)
{
    struct omap_dss_s *s = (struct omap_dss_s *)opaque;
    uint32_t x;
    
    switch (addr) {
        case 0x000: /* DSI_REVISION */
        case 0x014: /* DSI_SYSSTATUS */
        case 0x07c: /* DSI_RX_FIFO_VC_FULLNESS */
        case 0x084: /* DSI_RX_FIFO_VC_EMPTINESS */
        case 0x214: /* DSI_PHY_CFG5 */
        case 0x304: /* DSI_PLL_STATUS */
            /* read-only, ignore */
            break;
        case 0x010: /* DSI_SYSCONFIG */
            TRACEDSI("DSI_SYSCONFIG = 0x%08x", value);
            if (value & 2) /* SOFT_RESET */
                omap_dsi_reset(s);
            else
                s->dsi.sysconfig = value;
            break;
        case 0x018: /* DSI_IRQSTATUS */
            TRACEDSI("DSI_IRQSTATUS = 0x%08x", value);
            s->dsi.irqst &= ~(value & 0x1fc3b0);
            omap_dss_interrupt_update(s);
            break;
        case 0x01c: /* DSI_IRQENABLE */
            TRACEDSI("DSI_IRQENABLE = 0x%08x", value);
            s->dsi.irqen = value & 0x1fc3b0;
            omap_dss_interrupt_update(s);
            break;
        case 0x040: /* DSI_CTRL */
            TRACEDSI("DSI_CTRL = 0x%08x", value);
            s->dsi.ctrl = value & 0x7ffffff;
            break;
        case 0x048: /* DSI_COMPLEXIO_CFG_1 */
            TRACEDSI("DSI_COMPLEXIO_CFG1 = 0x%08x", value);
            value |= 1 << 29;    /* RESET_DONE */
            value |= 1 << 21;    /* LDO_POWER_GOOD_STATE */
            value &= ~(1 << 30); /* GOBIT */
            /* copy PWR_CMD directly to PWR_STATUS */
            value &= ~(3 << 25);
            value |= (value >> 2) & (3 << 25);
            /* TODO: notify screen refresh control about PWR_STATUS */
            s->dsi.complexio_cfg1 = value;
            break;
        case 0x04c: /* DSI_COMPLEXIO_IRQSTATUS */
            TRACEDSI("DSI_COMPLEXIO_IRQSTATUS = 0x%08x", value);
            s->dsi.complexio_irqst &= ~(value & 0xc3f39ce7);
            if (s->dsi.complexio_irqst & s->dsi.complexio_irqen)
                s->dsi.irqst |= (1 << 10);  /* COMPLEXIO_ERR_IRQ */
            else
                s->dsi.irqst &= ~(1 << 10); /* COMPLEXIO_ERR_IRQ */
            omap_dss_interrupt_update(s);
            break;
        case 0x050: /* DSI_COMPLEXIO_IRQENABLE */
            TRACEDSI("DSI_COMPLEXIO_IRQENABLE = 0x%08x", value);
            s->dsi.complexio_irqen = value & 0xc3f39ce7;
            omap_dss_interrupt_update(s);
            break;
        case 0x054: /* DSI_CLK_CTRL */
            TRACEDSI("DSI_CLK_CTRL = 0x%08x", value);
            value &= 0xc03fffff;
            /* copy PLL_PWR_CMD directly to PLL_PWR_STATUS */
            value |= (value >> 2) & (3 << 28);
            s->dsi.clk_ctrl = value;
            break;
        case 0x058: /* DSI_TIMING1 */
            TRACEDSI("DSI_TIMING1 = 0x%08x", value);
            value &= ~(1 << 15); /* deassert ForceTxStopMode signal */
            s->dsi.timing1 = value;
            break;
        case 0x05c: /* DSI_TIMING2 */
            TRACEDSI("DSI_TIMING2 = 0x%08x", value);
            s->dsi.timing2 = value;
            break;
        case 0x060: /* DSI_VM_TIMING1 */
            TRACEDSI("DSI_VM_TIMING1 = 0x%08x", value);
            s->dsi.vm_timing1 = value;
            break;
        case 0x064: /* DSI_VM_TIMING2 */
            TRACEDSI("DSI_VM_TIMING2 = 0x%08x", value);
            s->dsi.vm_timing2 = value & 0x0fffffff;
            break;
        case 0x068: /* DSI_VM_TIMING3 */
            TRACEDSI("DSI_VM_TIMING3 = 0x%08x", value);
            s->dsi.vm_timing3 = value;
            break;
        case 0x06c: /* DSI_CLK_TIMING */
            TRACEDSI("DSI_CLK_TIMING = 0x%08x", value);
            s->dsi.clk_timing = value & 0xffff;
            break;
        case 0x070: /* DSI_TX_FIFO_VC_SIZE */
            TRACEDSI("DSI_TX_FIFO_VC_SIZE = 0x%08x", value);
            s->dsi.tx_fifo_vc_size = value & 0xf7f7f7f7;
            break;
        case 0x074: /* DSI_RX_FIFO_VC_SIZE */
            TRACEDSI("DSI_RX_FIFO_VC_SIZE = 0x%08x", value);
            s->dsi.rx_fifo_vc_size = value & 0xf7f7f7f7;
            break;
        case 0x078: /* DSI_COMPLEXIO_CFG_2 */
            TRACEDSI("DSI_COMPLEXIO_CFG_2 = 0x%08x", value);
            s->dsi.complexio_cfg2 = (value & 0xfffcffff)
                                    | (s->dsi.complexio_cfg2 & (3 << 16));
            break;
        case 0x080: /* DSI_VM_TIMING4 */
            TRACEDSI("DSI_VM_TIMING4 = 0x%08x", value);
            s->dsi.vm_timing4 = value;
            break;
        case 0x088: /* DSI_VM_TIMING5 */
            TRACEDSI("DSI_VM_TIMING5 = 0x%08x", value);
            s->dsi.vm_timing5 = value;
            break;
        case 0x08c: /* DSI_VM_TIMING6 */
            TRACEDSI("DSI_VM_TIMING6 = 0x%08x", value);
            s->dsi.vm_timing6 = value;
            break;
        case 0x090: /* DSI_VM_TIMING7 */
            TRACEDSI("DSI_VM_TIMING7 = 0x%08x", value);
            s->dsi.vm_timing7 = value;
            break;
        case 0x094: /* DSI_STOPCLK_TIMING */
            TRACEDSI("DSI_STOPCLK_TIMING = 0x%08x", value);
            s->dsi.stopclk_timing = value & 0xff;
            break;
        case 0x100 ... 0x17c: /* DSI_VCx_xxx */
            x = (addr >> 5) & 3;
            switch (addr & 0x1f) {
                case 0x00: /* DSI_VCx_CTRL */
                    TRACEDSI("DSI_VC%d_CTRL = 0x%08x", x, value);
                    if (((value >> 27) & 7) != 4) /* DMA_RX_REQ_NB */
                        hw_error("%s: RX DMA mode not implemented", __FUNCTION__);
                    if (((value >> 21) & 7) != 4) /* DMA_TX_REQ_NB */
                        hw_error("%s: TX DMA mode not implemented", __FUNCTION__);
                    if (value & 1) { /* VC_EN */
                        s->dsi.vc[x].ctrl &= ~0x40;  /* BTA_EN */
                        s->dsi.vc[x].ctrl |= 0x8001; /* VC_BUSY | VC_EN */
                    } else {
                        /* clear VC_BUSY and VC_EN, assign writable bits */
                        s->dsi.vc[x].ctrl = (s->dsi.vc[x].ctrl & 0x114020) |
                                            (value & 0x3fee039f);
                    }
                    if (value & 0x40) /* BTA_EN */
                        omap_dsi_txdone(s, x, 1);
                    break;
                case 0x04: /* DSI_VCx_TE */
                    TRACEDSI("DSI_VC%d_TE = 0x%08x", x, value);
                    value &= 0xc0ffffff;
                    /* according to the OMAP3 TRM the TE_EN bit in this
                     * register is protected by VCx_CTRL VC_EN bit but
                     * let's forget that */
                    s->dsi.vc[x].te = value;
                    if (s->dispc.control & 1) /* LCDENABLE */
                        omap_dsi_transfer_start(s, x);
                    break;
                case 0x08: /* DSI_VCx_LONG_PACKET_HEADER */
                    TRACEDSI("DSI_VC%d_LONG_PACKET_HEADER id=0x%02x, len=0x%04x, ecc=0x%02x",
                             x, value & 0xff, (value >> 8) & 0xffff, (value >> 24) & 0xff);
                    s->dsi.vc[x].lp_header = value;
                    s->dsi.vc[x].lp_counter = (value >> 8) & 0xffff;
                    break;
                case 0x0c: /* DSI_VCx_LONG_PACKET_PAYLOAD */
                    TRACEDSI("DSI_VC%d_LONG_PACKET_PAYLOAD = 0x%08x", x, value);
                    s->dsi.vc[x].lp_payload = value;
                    if ((s->dsi.vc[x].te >> 30) & 3) { /* TE_START | TE_EN */
                        int tx_dma = (s->dsi.vc[x].ctrl >> 21) & 7; /* DMA_TX_REQ_NB */
                        if (tx_dma < 4)
                            qemu_irq_lower(s->dsi.drq[tx_dma]);
                    }
                    omap_dsi_long_write(s, x);
                    break;
                case 0x10: /* DSI_VCx_SHORT_PACKET_HEADER */
                    TRACEDSI("DSI_VC%d_SHORT_PACKET_HEADER = 0x%08x", x, value);
                    s->dsi.vc[x].sp_header = value;
                    omap_dsi_short_write(s, x);
                    break;
                case 0x18: /* DSI_VCx_IRQSTATUS */
                    TRACEDSI("DSI_VC%d_IRQSTATUS = 0x%08x", x, value);
                    s->dsi.vc[x].irqst &= ~(value & 0x1ff);
                    if (s->dsi.vc[x].irqst & s->dsi.vc[x].irqen)
                        s->dsi.irqst |= 1 << x;    /* VIRTUAL_CHANNELx_IRQ */
                    else
                        s->dsi.irqst &= ~(1 << x); /* VIRTUAL_CHANNELx_IRQ */
                    omap_dss_interrupt_update(s);
                    break;
                case 0x1c: /* DSI_VCx_IRQENABLE */
                    TRACEDSI("DSI_VC%d_IRQENABLE = 0x%08x", x, value);
                    s->dsi.vc[x].irqen = value & 0x1ff;
                    omap_dss_interrupt_update(s);
                    break;
                default:
                    OMAP_BAD_REGV(addr, value);
                    break;
            }
            break;
            
        case 0x200: /* DSI_PHY_CFG0 */
            TRACEDSI("DSI_PHY_CFG0 = 0x%08x", value);
            s->dsi.phy_cfg0 = value;
            break;
        case 0x204: /* DSI_PHY_CFG1 */
            TRACEDSI("DSI_PHY_CFG1 = 0x%08x", value);
            s->dsi.phy_cfg1 = value;
            break;
        case 0x208: /* DSI_PHY_CFG2 */
            TRACEDSI("DSI_PHY_CFG2 = 0x%08x", value);
            if (cpu_is_omap3630(s->mpu)) {
                value &= 0xff0000ff;
            }
            s->dsi.phy_cfg2 = value;
            break;
            
        case 0x300: /* DSI_PLL_CONTROL */
            TRACEDSI("DSI_PLL_CONTROL = 0x%08x", value);
            s->dsi.pll_control = value & 0x1f;
            break;
        case 0x308: /* DSI_PLL_GO */
            TRACEDSI("DSI_PLL_GO = 0x%08x", value);
            /* TODO: check if we need to update something here */
            value &= ~1; /* mark it done */
            s->dsi.pll_go = value & 1;
            break;
        case 0x30c: /* DSI_PLL_CONFIGURATION1 */
            TRACEDSI("DSI_PLL_CONFIGURATION1 = 0x%08x", value);
            s->dsi.pll_config1 = value & 0x7ffffff;
            break;
        case 0x310: /* DSI_PLL_CONFIGURATION2 */
            TRACEDSI("DSI_PLL_CONFIGURATION2 = 0x%08x", value);
            if (cpu_is_omap3630(s->mpu)) {
                value &= 0x1fffe1;
            } else {
                value &= 0x1fffff;
            }
            s->dsi.pll_config2 = value;
            break;
            
        default:
            OMAP_BAD_REGV(addr, value);
            break;
    }
}

static CPUReadMemoryFunc *omap_dsi_readfn[] = {
    omap_badwidth_read32,
    omap_badwidth_read32,
    omap_dsi_read,
};

static CPUWriteMemoryFunc *omap_dsi_writefn[] = {
    omap_badwidth_write32,
    omap_badwidth_write32,
    omap_dsi_write,
};

static struct omap_dss_s *omap_dss_init_internal(struct omap_target_agent_s *ta,
                                                 struct omap_mpu_state_s *mpu,
                                                 qemu_irq irq, qemu_irq drq)
{
    int iomemtype[5];
    int region_base = cpu_class_omap3(mpu) ? 1 : 0;
    struct omap_dss_s *s = (struct omap_dss_s *)
            qemu_mallocz(sizeof(struct omap_dss_s));

    s->mpu = mpu;
    s->irq = irq;
    s->drq = drq;

    iomemtype[0] = l4_register_io_memory(omap_diss1_readfn,
                                         omap_diss1_writefn, s);
    iomemtype[1] = l4_register_io_memory(omap_disc1_readfn,
                                         omap_disc1_writefn, s);
    iomemtype[2] = l4_register_io_memory(omap_rfbi1_readfn,
                                         omap_rfbi1_writefn, s);
    iomemtype[3] = l4_register_io_memory(omap_venc1_readfn,
                                         omap_venc1_writefn, s);

    omap_l4_attach(ta, region_base + 0, iomemtype[0]); /* DISS */
    omap_l4_attach(ta, region_base + 1, iomemtype[1]); /* DISC */
    omap_l4_attach(ta, region_base + 2, iomemtype[2]); /* RFBI */
    omap_l4_attach(ta, region_base + 3, iomemtype[3]); /* VENC */

    if (cpu_class_omap2(mpu)) {
        s->dispc.rev = 0x20;
        iomemtype[4] = cpu_register_io_memory(omap_im3_readfn,
                                              omap_im3_writefn, s,
                                              DEVICE_NATIVE_ENDIAN);
        cpu_register_physical_memory(0x68000800, 0x1000, iomemtype[4]);
    } else if (cpu_class_omap3(mpu)) {
        s->dispc.rev = 0x30;
        iomemtype[4] = l4_register_io_memory(omap_dsi_readfn,
                                             omap_dsi_writefn, s);
        omap_l4_attach(ta, 0, iomemtype[4]);
    } else {
        hw_error("%s: unsupported cpu type", __FUNCTION__);
    }

    return s;
}

struct omap_dss_s *omap_dss_init(struct omap_target_agent_s *ta,
                                 struct omap_mpu_state_s *mpu,
                                 qemu_irq irq, qemu_irq drq,
                                 omap_clk fck1, omap_clk fck2, omap_clk ck54m,
                                 omap_clk ick1, omap_clk ick2)
{
    struct omap_dss_s *s = omap_dss_init_internal(ta, mpu, irq, drq);
    omap_dss_reset(s);
    return s;
}    

struct omap_dss_s *omap3_dss_init(struct omap_target_agent_s *ta,
                                  struct omap_mpu_state_s *mpu,
                                  qemu_irq irq, qemu_irq line_trigger,
                                  qemu_irq dma0, qemu_irq dma1,
                                  qemu_irq dma2, qemu_irq dma3)
{
    struct omap_dss_s *s = omap_dss_init_internal(ta, mpu, irq, line_trigger);
    
    s->dsi.drq[0] = dma0;
    s->dsi.drq[1] = dma1;
    s->dsi.drq[2] = dma2;
    s->dsi.drq[3] = dma3;
    
    s->dispc.lcdframer = qemu_new_timer(vm_clock, omap_dss_lcd_framedone, s);

    omap_dss_reset(s);
    return s;
}    

void omap_rfbi_attach(struct omap_dss_s *s, int cs,
                      const struct rfbi_chip_s *chip)
{
    if (cs < 0 || cs > 1) {
        hw_error("%s: wrong CS %i\n", __FUNCTION__, cs);
    }
    if (s->rfbi.chip[cs]) {
        TRACERFBI("warning - replacing previously attached "
                  "RFBI chip on CS%d", cs);
    }
    s->rfbi.chip[cs] = chip;
}

void omap_dsi_attach(struct omap_dss_s *s, int vc,
                     struct dsi_chip_s *chip)
{
    if (vc < 0 || vc > 3) {
        hw_error("%s: invalid vc %d\n", __FUNCTION__, vc);
    }
    if (s->dsi.vc[vc].chip) {
        TRACEDSI("warning - replacing previously attached "
                 "DSI chip on VC%d", vc);
    }
    chip->te_trigger = omap_dsi_te_trigger;
    s->dsi.vc[vc].chip = chip;
}

void omap_lcd_panel_attach(struct omap_dss_s *s,
                           const struct omap_dss_panel_s *p)
{
    if (s->lcd) {
        TRACE("warning - replacing previously attached LCD panel");
    }
    s->lcd = p;
}
