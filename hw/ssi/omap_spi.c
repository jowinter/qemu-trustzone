/*
 * TI OMAP processor's Multichannel SPI emulation.
 *
 * Copyright (C) 2007-2009 Nokia Corporation
 *
 * Original code for OMAP2 by Andrzej Zaborowski <andrew@openedhand.com>
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
#include "hw/hw.h"
#include "hw/arm/omap.h"
#include "hw/sysbus.h"
#include "hw/spi.h"

//#define SPI_DEBUG

#ifdef SPI_DEBUG
#define TRACE(fmt,...) fprintf(stderr, "%s@%d: " fmt "\n", __FUNCTION__, \
                               __LINE__, ##__VA_ARGS__);
#else
#define TRACE(...)
#endif

#define SPI_FIFOSIZE 64
#define SPI_REV_OMAP2420 0x14
#define SPI_REV_OMAP3430 0x21
#define IS_OMAP3_SPI(s) ((s)->revision >= SPI_REV_OMAP3430)

typedef struct omap_mcspi_bus_s {
    SPIBus *bus;
    MemoryRegion iomem;
    qemu_irq irq;
    int chnum;
    uint8_t revision;

    uint32_t sysconfig;
    uint32_t systest;
    uint32_t irqst;
    uint32_t irqen;
    uint32_t wken;
    uint32_t control;

    uint32_t xferlevel;
    struct omap_mcspi_fifo_s {
        int start;
        int len;
        int size;
        uint8_t buf[SPI_FIFOSIZE];
    } tx_fifo, rx_fifo;
    int fifo_ch;
    int fifo_wcnt;

    struct omap_mcspi_ch_s {
        qemu_irq txdrq;
        qemu_irq rxdrq;

        uint32_t tx;
        uint32_t rx;

        uint32_t config;
        uint32_t status;
        uint32_t control;
    } *ch;
} OMAPSPIBusState;

typedef struct omap_mcspi_s {
    SysBusDevice busdev;
    int mpu_model;
    int buscount;
    OMAPSPIBusState *bus;
} OMAPSPIState;

static inline void omap_mcspi_interrupt_update(OMAPSPIBusState *s)
{
    qemu_set_irq(s->irq, s->irqst & s->irqen);
}

static inline void omap_mcspi_dmarequest_update(OMAPSPIBusState *s,
                                                int chnum)
{
    struct omap_mcspi_ch_s *ch = &s->ch[chnum];
    if ((ch->control & 1) &&                         /* EN */
        (ch->config & (1 << 14)) &&                  /* DMAW */
        (ch->status & (1 << 1)) &&                   /* TXS */
        ((ch->config >> 12) & 3) != 1) {             /* TRM */
        if (!IS_OMAP3_SPI(s) ||
            !(ch->config & (1 << 27)) ||             /* FFEW */
            s->tx_fifo.len <= (s->xferlevel & 0x3f)) /* AEL */
            qemu_irq_raise(ch->txdrq);
        else
            qemu_irq_lower(ch->txdrq);
    }
    if ((ch->control & 1) &&                                /* EN */
        (ch->config & (1 << 15)) &&                         /* DMAW */
        (ch->status & (1 << 0)) &&                          /* RXS */
        ((ch->config >> 12) & 3) != 2) {                    /* TRM */
        if (!IS_OMAP3_SPI(s) ||
            !(ch->config & (1 << 28)) ||                    /* FFER */
            s->rx_fifo.len >= ((s->xferlevel >> 8) & 0x3f)) /* AFL */
            qemu_irq_raise(ch->rxdrq);
        else
            qemu_irq_lower(ch->rxdrq);
    }
}

static void omap_mcspi_fifo_reset(OMAPSPIBusState *s)
{
    struct omap_mcspi_ch_s *ch;

    s->tx_fifo.len = 0;
    s->rx_fifo.len = 0;
    s->tx_fifo.start = 0;
    s->rx_fifo.start = 0;
    if (s->fifo_ch < 0) {
        s->tx_fifo.size  = s->rx_fifo.size  = 0;
    } else {
        ch = &s->ch[s->fifo_ch];
        s->tx_fifo.size = ((ch->config >> 27) & 1) ? SPI_FIFOSIZE : 0;
        s->rx_fifo.size = ((ch->config >> 28) & 1) ? SPI_FIFOSIZE : 0;
        if (((ch->config >> 27) & 3) == 3) {
            s->tx_fifo.size >>= 1;
            s->rx_fifo.size >>= 1;
        }
    }
}

/* returns next word in FIFO or the n first bytes if there is not
 * enough data in FIFO */
static uint32_t omap_mcspi_fifo_get(struct omap_mcspi_fifo_s *s, int wl)
{
    uint32_t v, sh;

    for (v = 0, sh = 0; wl > 0 && s->len; wl -= 8, s->len--, sh += 8) {
        v |= ((uint32_t)s->buf[s->start++]) << sh;
        if (s->start >= s->size)
            s->start = 0;
    }
    return v;
}

/* pushes a word to FIFO or the first n bytes of the word if the FIFO
 * is too full to hold the full word */
static void omap_mcspi_fifo_put(struct omap_mcspi_fifo_s *s, int wl,
                                uint32_t v)
{
    int p = s->start + s->len;

    for (; wl > 0 && s->len < s->size; wl -=8, v >>= 8, s->len++) {
        if (p >= s->size)
            p -= s->size;
        s->buf[p++] = (uint8_t)(v & 0xff);
    }
}

static void omap_mcspi_transfer_run(OMAPSPIBusState *s, int chnum)
{
    struct omap_mcspi_ch_s *ch = s->ch + chnum;
    int trm = (ch->config >> 12) & 3;
    int wl;

    if (!(ch->control & 1))                  /* EN */
        return;
    if ((ch->status & 1) && trm != 2 &&      /* RXS */
        !(ch->config & (1 << 19)))           /* TURBO */
        goto intr_update;
    if ((ch->status & (1 << 1)) && trm != 1) /* TXS */
        goto intr_update;

    if (!(s->control & 1) ||        /* SINGLE */
        (ch->config & (1 << 20))) { /* FORCE */
        wl = 1 + (0x1f & (ch->config >> 7)); /* WL */
        if (!IS_OMAP3_SPI(s) || s->fifo_ch != chnum ||
            !((ch->config >> 27) & 3)) {     /* FFER | FFEW */
            ch->rx = spi_txrx(s->bus, chnum, ch->tx, wl);
        } else {
            switch ((ch->config >> 27) & 3) {
            case 1: /* !FFER, FFEW */
                if (trm != 1)
                    ch->tx = omap_mcspi_fifo_get(&s->tx_fifo, wl);
                ch->rx = spi_txrx(s->bus, chnum, ch->tx, wl);
                s->fifo_wcnt--;
                break;
            case 2: /* FFER, !FFEW */
                ch->rx = spi_txrx(s->bus, chnum, ch->tx, wl);
                if (trm != 2)
                    omap_mcspi_fifo_put(&s->rx_fifo, wl, ch->rx);
                s->fifo_wcnt--;
                break;
            case 3: /* FFER, FFEW */
                while (s->rx_fifo.len < s->rx_fifo.size &&
                       s->tx_fifo.len && s->fifo_wcnt) {
                    if (trm != 1)
                        ch->tx = omap_mcspi_fifo_get(&s->tx_fifo, wl);
                    ch->rx = spi_txrx(s->bus, chnum, ch->tx, wl);
                    if (trm != 2)
                        omap_mcspi_fifo_put(&s->rx_fifo, wl, ch->rx);
                    s->fifo_wcnt--;
                }
                break;
            default:
                break;
            }
            if ((ch->config & (1 << 28)) &&        /* FFER */
                s->rx_fifo.len >= s->rx_fifo.size)
                ch->status |= 1 << 6;              /* RXFFF */
            ch->status &= ~(1 << 5);               /* RXFFE */
            ch->status &= ~(1 << 4);               /* TXFFF */
            if ((ch->config & (1 << 27)) &&        /* FFEW */
                !s->tx_fifo.len)
                ch->status |= 1 << 3;              /* TXFFE */
            if (!s->fifo_wcnt &&
                ((s->xferlevel >> 16) & 0xffff))   /* WCNT */
                s->irqst |= 1 << 17;               /* EOW */
        }
    }

    ch->tx = 0;
    ch->status |= 1 << 2;               /* EOT */
    ch->status |= 1 << 1;               /* TXS */
    if (trm != 2) {
        ch->status |= 1;                /* RXS */
    } else {
        ch->status &= ~1;               /* RXS */
    }

intr_update:
    if ((ch->status & 1) &&	trm != 2 &&                     /* RXS */
        !(ch->config & (1 << 19)))                          /* TURBO */
        if (!IS_OMAP3_SPI(s) || s->fifo_ch != chnum ||
            !((ch->config >> 28) & 1) ||                    /* FFER */
            s->rx_fifo.len >= ((s->xferlevel >> 8) & 0x3f)) /* AFL */
            s->irqst |= 1 << (2 + 4 * chnum);               /* RX_FULL */
    if ((ch->status & (1 << 1)) && trm != 1)                /* TXS */
        if (!IS_OMAP3_SPI(s) || s->fifo_ch != chnum ||
            !((ch->config >> 27) & 1) ||                    /* FFEW */
            s->tx_fifo.len <= (s->xferlevel & 0x3f))        /* AEL */
            s->irqst |= 1 << (4 * chnum);                   /* TX_EMPTY */
    omap_mcspi_interrupt_update(s);
    omap_mcspi_dmarequest_update(s, chnum);
}

static void omap_mcspi_bus_reset(OMAPSPIBusState *s)
{
    int ch;

    s->sysconfig = 0;
    s->systest = 0;
    s->irqst = 0;
    s->irqen = 0;
    s->wken = 0;
    s->control = 4;

    s->fifo_ch = -1;
    omap_mcspi_fifo_reset(s);

    for (ch = 0; ch < s->chnum; ch ++) {
        s->ch[ch].config = 0x060000;
        s->ch[ch].status = 2;				/* TXS */
        s->ch[ch].control = 0;

        omap_mcspi_dmarequest_update(s, ch);
    }

    omap_mcspi_interrupt_update(s);
}

static uint64_t omap_mcspi_read(void *opaque, hwaddr addr,
                                unsigned size)
{
    OMAPSPIBusState *s = (OMAPSPIBusState *) opaque;
    int ch = 0;
    uint32_t ret;

    if (size != 4) {
        return omap_badwidth_read32(opaque, addr);
    }

    switch (addr) {
    case 0x00:	/* MCSPI_REVISION */
        TRACE("REVISION = 0x%08x", s->revision);
        return s->revision;

    case 0x10:	/* MCSPI_SYSCONFIG */
        TRACE("SYSCONFIG = 0x%08x", s->sysconfig);
        return s->sysconfig;

    case 0x14:	/* MCSPI_SYSSTATUS */
        TRACE("SYSSTATUS = 0x00000001");
        return 1;					/* RESETDONE */

    case 0x18:	/* MCSPI_IRQSTATUS */
        TRACE("IRQSTATUS = 0x%08x", s->irqst);
        return s->irqst;

    case 0x1c:	/* MCSPI_IRQENABLE */
        TRACE("IRQENABLE = 0x%08x", s->irqen);
        return s->irqen;

    case 0x20:	/* MCSPI_WAKEUPENABLE */
        TRACE("WAKEUPENABLE = 0x%08x", s->wken);
        return s->wken;

    case 0x24:	/* MCSPI_SYST */
        TRACE("SYST = 0x%08x", s->systest);
        return s->systest;

    case 0x28:	/* MCSPI_MODULCTRL */
        TRACE("MODULCTRL = 0x%08x", s->control);
        return s->control;

    case 0x68: ch ++;
        /* fall through */
    case 0x54: ch ++;
        /* fall through */
    case 0x40: ch ++;
        /* fall through */
    case 0x2c:	/* MCSPI_CHCONF */
        TRACE("CHCONF%d = 0x%08x", ch,
              (ch < s->chnum) ? s->ch[ch].config : 0);
        return (ch < s->chnum) ? s->ch[ch].config : 0;

    case 0x6c: ch ++;
        /* fall through */
    case 0x58: ch ++;
        /* fall through */
    case 0x44: ch ++;
        /* fall through */
    case 0x30:	/* MCSPI_CHSTAT */
        TRACE("CHSTAT%d = 0x%08x", ch,
              (ch < s->chnum) ? s->ch[ch].status : 0);
        return (ch < s->chnum) ? s->ch[ch].status : 0;

    case 0x70: ch ++;
        /* fall through */
    case 0x5c: ch ++;
        /* fall through */
    case 0x48: ch ++;
        /* fall through */
    case 0x34:	/* MCSPI_CHCTRL */
        TRACE("CHCTRL%d = 0x%08x", ch,
              (ch < s->chnum) ? s->ch[ch].control : 0);
        return (ch < s->chnum) ? s->ch[ch].control : 0;

    case 0x74: ch ++;
        /* fall through */
    case 0x60: ch ++;
        /* fall through */
    case 0x4c: ch ++;
        /* fall through */
    case 0x38:	/* MCSPI_TX */
        TRACE("TX%d = 0x%08x", ch,
              (ch < s->chnum) ? s->ch[ch].tx : 0);
        return (ch < s->chnum) ? s->ch[ch].tx : 0;

    case 0x78: ch ++;
        /* fall through */
    case 0x64: ch ++;
        /* fall through */
    case 0x50: ch ++;
        /* fall through */
    case 0x3c:	/* MCSPI_RX */
        if (ch < s->chnum) {
            if (!IS_OMAP3_SPI(s) || ch != s->fifo_ch ||
                !(s->ch[ch].config & (1 << 28))) { /* FFER */
                s->ch[ch].status &= ~1;            /* RXS */
                ret = s->ch[ch].rx;
                TRACE("RX%d = 0x%08x", ch, ret);
                omap_mcspi_transfer_run(s, ch);
                return ret;
            }
            if (!s->rx_fifo.len) {
                TRACE("rxfifo underflow!");
            } else {
                qemu_irq_lower(s->ch[ch].rxdrq);
                s->ch[ch].status &= ~(1 << 6);                 /* RXFFF */
                if (((s->ch[ch].config >> 12) & 3) != 2)        /* TRM */
                    ret = omap_mcspi_fifo_get(&s->rx_fifo,
                        1 + ((s->ch[ch].config >> 7) & 0x1f)); /* WL */
                else
                    ret = s->ch[ch].rx;
                TRACE("RX%d = 0x%08x", ch, ret);
                if (!s->rx_fifo.len) {
                    s->ch[ch].status &= ~1;     /* RXS */
                    s->ch[ch].status |= 1 << 5; /* RXFFE */
                    omap_mcspi_transfer_run(s, ch);
                }
                return ret;
            }
        }
        TRACE("RX%d = 0x00000000", ch);
        return 0;

    case 0x7c: /* MCSPI_XFERLEVEL */
        if (IS_OMAP3_SPI(s)) {
            if ((s->xferlevel >> 16) & 0xffff) /* WCNT */
                ret = ((s->xferlevel & 0xffff0000) - (s->fifo_wcnt << 16));
            else
                ret = ((-s->fifo_wcnt) & 0xffff) << 16;
            TRACE("XFERLEVEL = 0x%08x", (s->xferlevel & 0xffff) | ret);
            return (s->xferlevel & 0xffff) | ret;
        }
        break;

    default:
        break;
    }

    OMAP_BAD_REG(addr);
    return 0;
}

static void omap_mcspi_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned size)
{
    OMAPSPIBusState *s = (OMAPSPIBusState *) opaque;
    uint32_t old;
    int ch = 0;

    if (size != 4) {
        return omap_badwidth_write32(opaque, addr, value);
    }

    switch (addr) {
    case 0x00:	/* MCSPI_REVISION */
    case 0x14:	/* MCSPI_SYSSTATUS */
    case 0x30:	/* MCSPI_CHSTAT0 */
    case 0x3c:	/* MCSPI_RX0 */
    case 0x44:	/* MCSPI_CHSTAT1 */
    case 0x50:	/* MCSPI_RX1 */
    case 0x58:	/* MCSPI_CHSTAT2 */
    case 0x64:	/* MCSPI_RX2 */
    case 0x6c:	/* MCSPI_CHSTAT3 */
    case 0x78:	/* MCSPI_RX3 */
        /* silently ignore */
        //OMAP_RO_REGV(addr, value);
        return;

    case 0x10:	/* MCSPI_SYSCONFIG */
        TRACE("SYSCONFIG = 0x%08x", value);
        if (value & (1 << 1))				/* SOFTRESET */
            omap_mcspi_bus_reset(s);
        s->sysconfig = value & 0x31d;
        break;

    case 0x18:	/* MCSPI_IRQSTATUS */
        TRACE("IRQSTATUS = 0x%08x", value);
        if (!((s->control & (1 << 3)) && (s->systest & (1 << 11)))) {
            s->irqst &= ~value;
            omap_mcspi_interrupt_update(s);
        }
        break;

    case 0x1c:	/* MCSPI_IRQENABLE */
        TRACE("IRQENABLE = 0x%08x", value);
        s->irqen = value & (IS_OMAP3_SPI(s) ? 0x3777f : 0x1777f);
        omap_mcspi_interrupt_update(s);
        break;

    case 0x20:	/* MCSPI_WAKEUPENABLE */
        TRACE("WAKEUPENABLE = 0x%08x", value);
        s->wken = value & 1;
        break;

    case 0x24:	/* MCSPI_SYST */
        TRACE("SYST = 0x%08x", value);
        if (s->control & (1 << 3))			/* SYSTEM_TEST */
            if (value & (1 << 11)) {			/* SSB */
                s->irqst |= 0x1777f;
                omap_mcspi_interrupt_update(s);
            }
        s->systest = value & 0xfff;
        break;

    case 0x28:	/* MCSPI_MODULCTRL */
        TRACE("MODULCTRL = 0x%08x", value);
        if (value & (1 << 3))				/* SYSTEM_TEST */
            if (s->systest & (1 << 11)) {		/* SSB */
                s->irqst |= IS_OMAP3_SPI(s) ? 0x3777f : 0x1777f;
                omap_mcspi_interrupt_update(s);
            }
        s->control = value & 0xf;
        break;

    case 0x68: ch ++;
        /* fall through */
    case 0x54: ch ++;
        /* fall through */
    case 0x40: ch ++;
        /* fall through */
    case 0x2c:	/* MCSPI_CHCONF */
        TRACE("CHCONF%d = 0x%08x", ch, value);
        if (ch < s->chnum) {
            old = s->ch[ch].config;
            s->ch[ch].config = value & (IS_OMAP3_SPI(s)
                                        ? 0x3fffffff : 0x7fffff);
            if (IS_OMAP3_SPI(s) &&
                ((value ^ old) & (3 << 27))) { /* FFER | FFEW */
                s->fifo_ch = ((value & (3 << 27))) ? ch : -1;
                omap_mcspi_fifo_reset(s);
            }
            if (((value ^ old) & (3 << 14)) || /* DMAR | DMAW */
                (IS_OMAP3_SPI(s) &&
                 ((value ^ old) & (3 << 27)))) /* FFER | FFEW */
                omap_mcspi_dmarequest_update(s, ch);
            if (((value >> 12) & 3) == 3) {   /* TRM */
                TRACE("invalid TRM value (3)");
            }
                if (((value >> 7) & 0x1f) < 3) {  /* WL */
                TRACE("invalid WL value (%" PRIx64 ")", (value >> 7) & 0x1f);
                }
            if (IS_OMAP3_SPI(s) && ((value >> 23) & 1)) { /* SBE */
                TRACE("start-bit mode is not supported");
            }
        }
        break;

    case 0x70: ch ++;
        /* fall through */
    case 0x5c: ch ++;
        /* fall through */
    case 0x48: ch ++;
        /* fall through */
    case 0x34:	/* MCSPI_CHCTRL */
        TRACE("CHCTRL%d = 0x%08x", ch, value);
        if (ch < s->chnum) {
            old = s->ch[ch].control;
            s->ch[ch].control = value & (IS_OMAP3_SPI(s) ? 0xff01 : 1);
            if (value & ~old & 1) { /* EN */
                if (IS_OMAP3_SPI(s) && s->fifo_ch == ch)
                    omap_mcspi_fifo_reset(s);
                omap_mcspi_transfer_run(s, ch);
            }
        }
        break;

    case 0x74: ch ++;
        /* fall through */
    case 0x60: ch ++;
        /* fall through */
    case 0x4c: ch ++;
        /* fall through */
    case 0x38:	/* MCSPI_TX */
        TRACE("TX%d = 0x%08x", ch, value);
        if (ch < s->chnum) {
            if (!IS_OMAP3_SPI(s) || s->fifo_ch != ch ||
                !(s->ch[ch].config & (1 << 27))) { /* FFEW */
                s->ch[ch].tx = value;
                s->ch[ch].status &= ~0x06;         /* EOT | TXS */
                omap_mcspi_transfer_run(s, ch);
            } else {
                if (s->tx_fifo.len >= s->tx_fifo.size) {
                    TRACE("txfifo overflow!");
                } else {
                    qemu_irq_lower(s->ch[ch].txdrq);
                    s->ch[ch].status &= ~0x0e;      /* TXFFE | EOT | TXS */
                    if (((s->ch[ch].config >> 12) & 3) != 1) {    /* TRM */
                        omap_mcspi_fifo_put(
                            &s->tx_fifo,
                            1 + ((s->ch[ch].config >> 7) & 0x1f), /* WL */
                            value);
                        if (s->tx_fifo.len >= s->tx_fifo.size)
                            s->ch[ch].status |= 1 << 4;        /* TXFFF */
                        if (s->tx_fifo.len >= (s->xferlevel & 0x3f))
                            omap_mcspi_transfer_run(s, ch);
                    } else {
                        s->ch[ch].tx = value;
                        omap_mcspi_transfer_run(s, ch);
                    }
                }
            }
        }
        break;

    case 0x7c: /* MCSPI_XFERLEVEL */
        TRACE("XFERLEVEL = 0x%08x", value);
        if (IS_OMAP3_SPI(s)) {
            if (value != s->xferlevel) {
                s->fifo_wcnt = (value >> 16) & 0xffff;
                s->xferlevel = value & 0xffff3f3f;
                omap_mcspi_fifo_reset(s);
            }
        } else
            OMAP_BAD_REG(addr);
        break;

    default:
        OMAP_BAD_REG(addr);
        return;
    }
}

static const MemoryRegionOps omap_mcspi_ops = {
    .read = omap_mcspi_read,
    .write = omap_mcspi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void omap_mcspi_reset(DeviceState *qdev)
{
    int i;
    OMAPSPIState *s = FROM_SYSBUS(OMAPSPIState, SYS_BUS_DEVICE(qdev));
    for (i = 0; i < s->buscount; i++) {
        omap_mcspi_bus_reset(&s->bus[i]);
    }
}

static int omap_mcspi_init(SysBusDevice *busdev)
{
    int i, j;
    OMAPSPIBusState *bs;
    OMAPSPIState *s = FROM_SYSBUS(OMAPSPIState, busdev);
    
    s->buscount = (s->mpu_model < omap3430) ? 2 : 4;
    s->bus = g_new0(OMAPSPIBusState, s->buscount);
    for (i = 0; i < s->buscount; i++) {
        bs = &s->bus[i];
        if (s->mpu_model < omap3430) {
            bs->revision = SPI_REV_OMAP2420;
            bs->chnum = i ? 2 : 4;
        } else {
            bs->revision = SPI_REV_OMAP3430;
            bs->chnum = (i > 2) ? 1 : (i ? 2 : 4);
        }
        sysbus_init_irq(busdev, &bs->irq);
        bs->bus = spi_init_bus(&busdev->qdev, NULL, bs->chnum);
        bs->ch = g_new0(struct omap_mcspi_ch_s, bs->chnum);
        for (j = 0; j < bs->chnum; j++) {
            sysbus_init_irq(busdev, &bs->ch[j].txdrq);
            sysbus_init_irq(busdev, &bs->ch[j].rxdrq);
        }
        memory_region_init_io(&bs->iomem, &omap_mcspi_ops, bs, "omap.mcspi",
                              0x1000);
        sysbus_init_mmio(busdev, &bs->iomem);
    }
    return 0;
}

SPIBus *omap_mcspi_bus(DeviceState *qdev, int bus_number)
{
    OMAPSPIState *s = FROM_SYSBUS(OMAPSPIState, SYS_BUS_DEVICE(qdev));
    if (bus_number < s->buscount) {
        return s->bus[bus_number].bus;
    }
    hw_error("%s: invalid bus number %d\n", __FUNCTION__, bus_number);
}

static Property omap_mcspi_properties[] = {
    DEFINE_PROP_INT32("mpu_model", OMAPSPIState, mpu_model, 0),
    DEFINE_PROP_END_OF_LIST()
};

static void omap_mcspi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = omap_mcspi_init;
    dc->props = omap_mcspi_properties;
    dc->reset = omap_mcspi_reset;
}

static TypeInfo omap_mcspi_info = {
    .name = "omap_mcspi",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(OMAPSPIState),
    .class_init = omap_mcspi_class_init,
};

static void omap_mcspi_register_types(void)
{
    type_register_static(&omap_mcspi_info);
}

type_init(omap_mcspi_register_types)
