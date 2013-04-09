/*
 * OMAP3 MMC/SD/SDIO interface emulation
 *
 * Copyright (C) 2008 yajin  <yajin@vm-kernel.org>
 * Copyright (C) 2009 Nokia Corporation
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
#include "hw/arm/omap.h"
#include "hw/sd.h"
#include "hw/sysbus.h"

/* debug levels:
   0 - no debug
   1 - print non-fatal errors
   2 - print out all commands in processing order
   3 - dump all register accesses and buffer management */
#define MMC_DEBUG_LEVEL 0

#if MMC_DEBUG_LEVEL>0
#define TRACE(fmt,...) fprintf(stderr, "%s: " fmt "\n", \
                               __FUNCTION__, ##__VA_ARGS__)
#else
#define TRACE(...)
#endif

#if MMC_DEBUG_LEVEL>1
#define TRACE1(...) TRACE(__VA_ARGS__)
#else
#define TRACE1(...)
#endif

#if MMC_DEBUG_LEVEL>2
#define TRACE2(...) TRACE(__VA_ARGS__)
#else
#define TRACE2(...)
#endif

struct omap3_mmc_s
{
    SysBusDevice busdev;
    MemoryRegion iomem;
    qemu_irq irq;
    qemu_irq dma[2];
    qemu_irq coverswitch;
    omap_clk clk;
    SDState *card;

    uint32_t sysconfig;
    uint32_t sysstatus;
    uint32_t csre;
    uint32_t systest;
    uint32_t con;
    uint32_t pwcnt;
    uint32_t blk;
    uint32_t arg;
    uint32_t cmd;
    uint32_t rsp10;
    uint32_t rsp32;
    uint32_t rsp54;
    uint32_t rsp76;
    uint32_t data;
    uint32_t pstate;
    uint32_t hctl;
    uint32_t sysctl;
    uint32_t stat;
    uint32_t ie;
    uint32_t ise;
    uint32_t ac12;
    uint32_t capa;
    uint32_t cur_capa;
    uint32_t rev;

    uint16_t blen_counter;
    uint16_t nblk_counter;

    uint32_t fifo[256];
    int fifo_start;
    int fifo_len;

    int ddir;
    int transfer;
    int stop;
};

/* Bit names for STAT/IC/IE registers */
#define STAT_CC (1 << 0)
#define STAT_TC (1 << 1)
#define STAT_BGE (1 << 2)
#define STAT_BWR (1 << 4)
#define STAT_BRR (1 << 5)
#define STAT_CIRQ (1 << 8)
#define STAT_OBI (1 << 9)
#define STAT_ERRI (1 << 15)
#define STAT_CTO (1 << 16)
#define STAT_CCRC (1 << 17)
#define STAT_CEB (1 << 18)
#define STAT_CIE (1 << 19)
#define STAT_DTO (1 << 20)
#define STAT_DCRC (1 << 21)
#define STAT_DEB (1 << 22)
#define STAT_ACE (1 << 24)
#define STAT_CERR (1 << 28)
#define STAT_BADA (1 << 29)

#define STAT_MASK \
    (STAT_CC|STAT_TC|STAT_BGE| \
    STAT_BWR|STAT_BRR| \
    STAT_CIRQ|STAT_OBI| \
    STAT_ERRI| \
    STAT_CTO|STAT_CCRC|STAT_CEB|STAT_CIE| \
    STAT_DTO|STAT_DCRC|STAT_DEB| \
    STAT_ACE|STAT_CERR|STAT_BADA)

static void omap3_mmc_reset(DeviceState *dev)
{
    struct omap3_mmc_s *s = FROM_SYSBUS(struct omap3_mmc_s,
                                        SYS_BUS_DEVICE(dev));
    s->sysconfig = 0x00000015;
    s->sysstatus = 0;
    s->csre      = 0;
    s->systest   = 0;
    s->con       = 0x00000500;
    s->pwcnt     = 0;
    s->blk       = 0;
    s->arg       = 0;
    s->cmd       = 0;
    s->rsp10     = 0;
    s->rsp32     = 0;
    s->rsp54     = 0;
    s->rsp76     = 0;
    s->data      = 0;
    s->pstate    = 0x00040000;
    s->hctl      = 0;
    s->sysctl    = 0;
    s->stat      = 0;
    s->ie        = 0;
    s->ise       = 0;
    s->ac12      = 0;
    s->capa      = 0x00e10080;
    s->cur_capa  = 0;
    s->rev       = 0x26000000;
    
    s->blen_counter = 0;
    s->nblk_counter = 0;
    
    memset(s->fifo, 0, sizeof(s->fifo));
    s->fifo_start = 0;
    s->fifo_len   = 0;
    
    s->ddir       = 0;
    s->transfer   = 0;
    s->stop       = 0;

    if (s->card) {
        sd_reset(s->card);
    }
}

typedef enum
{
    sd_nore = 0,     /* no response */
    sd_136_bits = 1, /* response length 136 bits */
    sd_48_bits = 2,  /* response length 48 bits */
    sd_48b_bits = 3, /* response length 48 bits with busy after response */
} omap3_sd_rsp_type_t;

static void omap3_mmc_command(struct omap3_mmc_s *host);

static void omap3_mmc_interrupts_update(struct omap3_mmc_s *s)
{
    qemu_set_irq(s->irq, !!(s->stat & s->ie & s->ise));
}

static void omap3_mmc_fifolevel_update(struct omap3_mmc_s *host)
{
    enum { ongoing, ready, aborted } state = ongoing;
    
    if ((host->cmd & (1 << 21))) { /* DP */
        if (host->ddir) {
            TRACE2("receive, dma=%d, fifo_len=%d bytes",
                   host->cmd & 1, host->fifo_len * 4);
            
            /* omap3_mmc_transfer ensures we always have data in FIFO
               during receive as long as all data has not been transferred -
               NOTE that the actual transfer may be finished already (i.e.
               host->transfer is cleared) but not all data has been read out
               from FIFO yet */
            if (host->fifo_len) {
                if (host->cmd & 1) { /* DE */
                    if (host->fifo_len * 4 == (host->blk & 0x7ff)) { /* BLEN */
                        if (host->stop)
                            state = aborted;
                        else
                            qemu_irq_raise(host->dma[1]);
                    } else
                        qemu_irq_lower(host->dma[1]);
                } else {
                    if (host->stop 
                        && host->fifo_len * 4 == (host->blk & 0x7ff))
                        state = aborted;
                    else {
                        host->pstate |= 0x0800; /* BRE */
                        host->stat   |= STAT_BRR;
                    }
                }
            }
            else
                state = host->stop ? aborted : ready;
        } else {
            /* omap3_mmc_transfer keeps FIFO empty during transmit so
               we just check all blocks have been transferred or not */
            if (host->transfer) {
                if (host->cmd & 1) { /* DE */
                    if (host->blen_counter == (host->blk & 0x7ff)) { /* BLEN */
                        if (host->stop)
                            state = aborted;
                        else
                            qemu_irq_raise(host->dma[0]);
                    } else
                        qemu_irq_lower(host->dma[0]);
                } else {
                    if (host->stop
                        && host->blen_counter == (host->blk & 0x7ff))
                        state = aborted;
                    else {
                        host->pstate |= 0x0400; /* BWE */
                        host->stat   |= STAT_BWR;
                    }
                }
            } else
                state = host->stop ? aborted : ready;
        }

        if ((host->cmd & 1) || state != ongoing) { /* DE */
            host->pstate &= ~0x0c00;               /* BRE | BWE */
            host->stat &= ~(STAT_BRR | STAT_BWR);
            if (state != ongoing) {
                TRACE2("transfer %s", 
                       state == ready
                       ? "complete"
                       : "aborted --> complete");
                host->stat |= STAT_TC;
                if (host->cmd & 0x04) {            /* ACEN */
                    host->stop = 0x0cc30000;
                    state = aborted;
                }
                if (state == aborted) {
                    host->cmd = host->stop;
                    host->stop = 0;
                    omap3_mmc_command(host);
                }
            }
        }
    }
}

static void omap3_mmc_transfer(struct omap3_mmc_s *host)
{
    int i;
    uint32_t x;
#if MMC_DEBUG_LEVEL>1
    int j;
    uint8_t c, sym[17];
#endif

    /* IF data transfer is inactive
       OR block count enabled with zero block count
       OR in receive mode and we have unread data in FIFO
       OR in transmit mode and we have no data in FIFO,
       THEN don't do anything */
    if (!host->transfer
        || ((host->cmd & 2) && !host->nblk_counter)
        || (host->ddir && host->fifo_len)
        || (!host->ddir && !host->fifo_len))
        return;
    
    if (host->ddir) {
        TRACE2("begin, %d blocks (%d bytes/block) left to receive, %d bytes in FIFO",
               (host->cmd & 2) ? host->nblk_counter : 1,
               host->blk & 0x7ff, 
               host->fifo_len * 4);
        while (host->blen_counter && host->fifo_len < 255) {
            for (i = 0, x = 0; i < 32 && host->blen_counter; i += 8, host->blen_counter--)
                x |= sd_read_data(host->card) << i;
            host->fifo[(host->fifo_start + host->fifo_len) & 0xff] = x;
            host->fifo_len++;
        }
        TRACE2("end, %d bytes in FIFO:", host->fifo_len * 4);
#if MMC_DEBUG_LEVEL>1
        for (i = 0; i < host->fifo_len; ) {
            fprintf(stderr, "%s: [0x%03x] ", __FUNCTION__, i * 4);
            do {
                x = host->fifo[(host->fifo_start + i) & 0xff];
                for (j = 0; j < 4; j++) {
                    c = (x >> (j * 8)) & 0xff;
                    fprintf(stderr, "%02x ", c);
                    sym[(i & 3) * 4 + j] = (c < 32 || c > 126) ? '.' : c;
                }
            } while (((++i) & 3));
            sym[16] = 0;
            fprintf(stderr, "%s\n", sym);
        }
#endif
    } else {
        TRACE2("%d bytes left to transmit in current block", host->blen_counter);
        while (host->blen_counter && host->fifo_len) {
            for (i = 0; i < 32 && host->blen_counter; i += 8, host->blen_counter--)
                sd_write_data(host->card, (host->fifo[host->fifo_start] >> i) & 0xff);
            host->fifo_start++;
            host->fifo_len--;
            host->fifo_start &= 0xff;
        }
    }

    if (!host->blen_counter) {
        if (host->cmd & 2) /* BCE */
            host->nblk_counter--;
        TRACE2("block done, %d blocks left",
               (host->cmd & (1 << 5)) ? host->nblk_counter : 0);
        host->blen_counter = host->blk & 0x7ff;
        if (!(host->cmd & (1 << 5)) /* MSBS */
            || !host->nblk_counter) {
            host->nblk_counter = (host->blk >> 16) & 0xffff;
            host->transfer = 0;
            host->pstate &= ~0x0306; /* RTA | WTA | DLA | DATI */
        }
    }
}

static void omap3_mmc_command(struct omap3_mmc_s *s)
{
    uint32_t rspstatus, mask;
    int rsplen, timeout;
    SDRequest request;
    uint8_t response[16];
    int cmd = (s->cmd >> 24) & 0x3f; /* INDX */
    int rsptype = (s->cmd >> 16) & 3;
    int dp = s->cmd & (1 << 21);
    
    TRACE1("%d type=%d rsp=%d arg=0x%08x blk=0x%08x, fifo=%d/%d",
           cmd, (s->cmd >> 22) & 3, (s->cmd >> 16) & 3, s->arg,
           s->blk, s->fifo_start, s->fifo_len);

    if ((s->con & 2) && !cmd) { /* INIT and CMD0 */
        s->stat   |= STAT_CC;
        s->pstate &= 0xfffffffe;
        return;
    }
    
    if (dp) {
        s->fifo_start = 0;
        s->fifo_len = 0;
        s->transfer = 1;
        s->ddir = (s->cmd >> 4) & 1;
        /* DLA | DATI | (RTA/WTA) */
        s->pstate |= 0x6 | (s->ddir ? 0x200 : 0x100);
    } else {
        s->transfer = 0;
        s->pstate &= ~0x306; /* RTA | WTA | DLA | DATI */
    }
    
    timeout = 0;
    mask = 0;
    rspstatus = 0;
    
    request.cmd = cmd;
    request.arg = s->arg;
    request.crc = 0; /* FIXME */
    
    rsplen = sd_do_command(s->card, &request, response);
    
    switch (rsptype) {
        case sd_nore:
            rsplen = 0;
            break;
        case sd_136_bits:
            if (rsplen < 16) {
                timeout = 1;
                break;
            }
            rsplen = 16;
            s->rsp76 = (response[0] << 24) | (response[1] << 16) |
                       (response[2] << 8) | (response[3] << 0);
            s->rsp54 = (response[4] << 24) | (response[5] << 16) |
                       (response[6] << 8) | (response[7] << 0);
            s->rsp32 = (response[8] << 24) | (response[9] << 16) |
                       (response[10] << 8) | (response[11] << 0);
            s->rsp10 = (response[12] << 24) | (response[13] << 16) |
                       (response[14] << 8) | (response[15] << 0);
            break;
        case sd_48_bits:
        case sd_48b_bits:
            if (rsplen < 4) {
                timeout = 1;
                break;
            }
            rsplen = 4;
            s->rsp10 = (response[0] << 24) | (response[1] << 16) |
                       (response[2] << 8) | (response[3] << 0);
            switch (cmd) {
                case 41: /* r3 */
                    break;
                case 3:  /* r6 */
                    mask = 0xe00;
                    rspstatus = (response[2] << 8) | response[3];
                    break;
                default:
                    if (cmd == 8 && !sd_is_mmc(s->card)) {
                        /* r7 */
                        break;
                    }
                    mask = OUT_OF_RANGE | ADDRESS_ERROR | BLOCK_LEN_ERROR |
                        ERASE_SEQ_ERROR | ERASE_PARAM | WP_VIOLATION |
                        LOCK_UNLOCK_FAILED | COM_CRC_ERROR | ILLEGAL_COMMAND |
                        CARD_ECC_FAILED | CC_ERROR | SD_ERROR |
                        CID_CSD_OVERWRITE | WP_ERASE_SKIP;
                    rspstatus = (response[0] << 24) | (response[1] << 16) |
                                (response[2] << 8) | (response[3] << 0);
                    break;
            }
        default:
            break;
    }
    
    if (cmd == 12 || cmd == 52) { /* stop transfer commands */
        /*s->fifo_start = 0;*/
        /*s->fifo_len = 0;*/
        s->transfer = 0;
        s->pstate &= ~0x0f06;     /* BRE | BWE | RTA | WTA | DLA | DATI */
        s->stat &= ~(STAT_BRR | STAT_BWR);
        s->stat |= STAT_TC;
        qemu_irq_lower(s->dma[0]);
        qemu_irq_lower(s->dma[1]);
    }
    
    if (rspstatus & mask & s->csre) {
        s->stat |= STAT_CERR;
        s->pstate &= ~0x306; /* RTA | WTA | DLA | DATI */
        s->transfer = 0;
    } else {
        s->stat &= ~STAT_CERR;
        /* If this is an R1b command with no data transfer and
         * there wasn't an error, then we have effectively
         * emulated the command as having a zero length "busy"
         * response. Set TC to tell the driver the "busy" period
         * is over.
         */
        if (!timeout && !dp && rsptype == sd_48b_bits) {
            s->stat |= STAT_TC;
        }
    }
    s->stat |= timeout ? STAT_CTO : STAT_CC;
}

static uint32_t omap3_mmc_read(void *opaque, hwaddr addr)
{
    struct omap3_mmc_s *s = (struct omap3_mmc_s *) opaque;
    uint32_t i ;

    switch (addr) {
        case 0x10:
            TRACE2("SYSCONFIG = %08x", s->sysconfig);
            return s->sysconfig;
        case 0x14:
            TRACE2("SYSSTATUS = %08x", s->sysstatus | 0x1);
            return s->sysstatus | 0x1; /*reset completed */
        case 0x24:
            TRACE2("CSRE = %08x", s->csre);
            return s->csre;
        case 0x28:
            TRACE2("SYSTEST = %08x", s->systest);
            return s->systest;
        case 0x2c: /* MMCHS_CON */
            TRACE2("CON = %08x", s->con);
            return s->con;
        case 0x30:
            TRACE2("PWCNT = %08x", s->pwcnt);
            return s->pwcnt;
        case 0x104: /* MMCHS_BLK */
            TRACE2("BLK = %08x", s->blk);
            return s->blk;
        case 0x108: /* MMCHS_ARG */
            TRACE2("ARG = %08x", s->arg);
            return s->arg;
        case 0x10c:
            TRACE2("CMD = %08x", s->cmd);
            return s->cmd;
        case 0x110:
            TRACE2("RSP10 = %08x", s->rsp10);
            return s->rsp10;
        case 0x114:
            TRACE2("RSP32 = %08x", s->rsp32);
            return s->rsp32;
        case 0x118:
            TRACE2("RSP54 = %08x", s->rsp54);
            return s->rsp54;
        case 0x11c:
            TRACE2("RSP76 = %08x", s->rsp76);
            return s->rsp76;
        case 0x120:
            /* in PIO mode, access allowed only when BRE is set */
            if (!(s->cmd & 1) && !(s->pstate & 0x0800)) {
                s->stat |= STAT_BADA;
                i = 0;
            } else {
                i = s->fifo[s->fifo_start];
                s->fifo[s->fifo_start] = 0;
                if (s->fifo_len == 0) {
                    TRACE("FIFO underrun");
                    return i;
                }
                s->fifo_start++;
                s->fifo_len--;
                s->fifo_start &= 255;
                omap3_mmc_transfer(s);
                omap3_mmc_fifolevel_update(s);
            }
            omap3_mmc_interrupts_update(s);
            return i;
        case 0x124: /* MMCHS_PSTATE */
            TRACE2("PSTATE = %08x", s->pstate);
            return s->pstate;
        case 0x128:
            TRACE2("HCTL = %08x", s->hctl);
            return s->hctl;
        case 0x12c: /* MMCHS_SYSCTL */
            TRACE2("SYSCTL = %08x", s->sysctl);
            return s->sysctl;
        case 0x130: /* MMCHS_STAT */
            if (s->stat & 0xffff0000)
                s->stat |= STAT_ERRI;
            else
                s->stat &= ~STAT_ERRI;
            TRACE2("STAT = %08x", s->stat);
            return s->stat;
        case 0x134:
            TRACE2("IE = %08x", s->ie);
            return s->ie;
        case 0x138:
            TRACE2("ISE = %08x", s->ise);
            return s->ise;
        case 0x13c:
            TRACE2("AC12 = %08x", s->ac12);
            return s->ac12;
        case 0x140: /* MMCHS_CAPA */
            TRACE2("CAPA = %08x", s->capa);
            return s->capa;
        case 0x148:
            TRACE2("CUR_CAPA = %08x", s->cur_capa);
            return s->cur_capa;
        case 0x1fc:
            TRACE2("REV = %08x", s->rev);
            return s->rev;
        default:
            OMAP_BAD_REG(addr);
            exit(-1);
            return 0;
    }
}

static void omap3_mmc_write(void *opaque, hwaddr addr,
                            uint32_t value)
{
    struct omap3_mmc_s *s = (struct omap3_mmc_s *) opaque;
    
    switch (addr) {
        case 0x014:
        case 0x110:
        case 0x114:
        case 0x118:
        case 0x11c:
        case 0x124:
        case 0x13c:
        case 0x1fc:
            OMAP_RO_REG(addr);
            break;
        case 0x010:
            TRACE2("SYSCONFIG = %08x", value);
            if (value & 2)
                omap3_mmc_reset(&s->busdev.qdev);
            s->sysconfig = value & 0x31d;
            break;
        case 0x024:
            TRACE2("CSRE = %08x", value);
            s->csre = value;
            break;
        case 0x028:
            TRACE2("SYSTEST = %08x", value);
            s->systest = value;
            break;
        case 0x02c: /* MMCHS_CON */
            TRACE2("CON = %08x", value);
            if (value & 0x10) {   /* MODE */
                TRACE("SYSTEST mode is not supported");
            }
            if ((value & 0x20) && !sd_is_mmc(s->card)) { /* DW8 */
                TRACE("8-bit data width is not supported for SD cards");
            }
            if (value & 0x1000) { /* CEATA */
                TRACE("CE-ATA control mode not supported");
            }
            s->con = value & 0x1ffff;
            break;
        case 0x030:
            TRACE2("PWCNT = %08x", value);
            s->pwcnt = value;
            break;
        case 0x104: /* MMCHS_BLK */
            TRACE2("BLK = %08x", value);
            s->blk = value & 0xffff07ff;
            s->blen_counter = value & 0x7ff;
            s->nblk_counter = (value >> 16) & 0xffff;
            break;
        case 0x108: /* MMCHS_ARG */
            TRACE2("ARG = %08x", value);
            s->arg = value;
            break;
        case 0x10c: /* MMCHS_CMD */
            TRACE2("CMD = %08x", value);
            if (!s->card) {
                s->stat |= STAT_CTO;
            } else {
                /* TODO: writing to bits 0-15 should have no effect during
                   an active data transfer */
                if (s->transfer && !s->stop
                    && (((value >> 24) & 0x3f) == 12
                        || ((value >> 24) & 0x3f) == 52)) {
                    s->stop = value & 0x3ffb0037;
                } else {
                    s->cmd = value & 0x3ffb0037;
                    omap3_mmc_command(s);
                }
                omap3_mmc_transfer(s);
                omap3_mmc_fifolevel_update(s);
            }
            omap3_mmc_interrupts_update(s);
            break;
        case 0x120:
            /* in PIO mode, access allowed only when BWE is set */
            if (!(s->cmd & 1) && !(s->pstate & 0x0400)) {
                s->stat |= STAT_BADA;
            } else {
                if (s->fifo_len == 256) {
                    TRACE("FIFO overrun");
                    break;
                }
                s->fifo[(s->fifo_start + s->fifo_len) & 255] = value;
                s->fifo_len++;
                omap3_mmc_transfer(s);
                omap3_mmc_fifolevel_update(s);
            }
            omap3_mmc_interrupts_update(s);
            break;
        case 0x128: /* MMCHS_HCTL */
            TRACE2("HCTL = %08x", value);
            s->hctl = value & 0xf0f0f02;
            if (s->hctl & (1 << 16)) { /* SBGR */
                TRACE("Stop at block gap feature not implemented!");
            }
            break;
        case 0x12c: /* MMCHS_SYSCTL */
            TRACE2("SYSCTL = %08x", value);
            if (value & 0x04000000) { /* SRD */
                s->data    = 0;
                s->pstate &= ~0x00000f06; /* BRE, BWE, RTA, WTA, DLA, DATI */
                s->hctl   &= ~0x00030000; /* SGBR, CR */
                s->stat   &= ~(STAT_BRR|STAT_BWR|STAT_BGE);
                s->fifo_start = 0;
                s->fifo_len = 0;
            }
            if (value & 0x02000000) { /* SRC */
                s->pstate &= ~0x00000001; /* CMDI */
            }
            if (value & 0x01000000) { /* SRA */
                uint32_t capa = s->capa;
                uint32_t cur_capa = s->cur_capa;
                omap3_mmc_reset(&s->busdev.qdev);
                s->capa = capa;
                s->cur_capa = cur_capa;
            }
            value = (value & ~2) | ((value & 1) << 1); /* copy ICE directly to ICS */
            s->sysctl = value & 0x000fffc7;
            break;
        case 0x130:
            TRACE2("STAT = %08x", value);
            /* STAT_CIRQ and STAT_ERRI are write-ignored */
            value = value & (STAT_MASK & ~(STAT_CIRQ|STAT_ERRI));
            s->stat &= ~value;
            omap3_mmc_interrupts_update(s);
            break;
        case 0x134: /* MMCHS_IE */
            TRACE2("IE = %08x", value);
            if (!(s->con & 0x4000)) {
                /* if CON:OBIE is clear, ignore write to OBI_ENABLE */
                value = (value & ~STAT_OBI) | (s->ie & STAT_OBI);
            }
            s->ie = value & (STAT_MASK & ~STAT_ERRI);
            if (!(s->ie & STAT_CIRQ)) {
                s->stat &= ~STAT_CIRQ;
            }
            omap3_mmc_interrupts_update(s);
            break;
        case 0x138:
            TRACE2("ISE = %08x", value);
            s->ise = value & (STAT_MASK & ~STAT_ERRI);
            omap3_mmc_interrupts_update(s);
            break;
        case 0x140: /* MMCHS_CAPA */
            TRACE2("CAPA = %08x", value);
            s->capa &= ~0x07000000;
            s->capa |= value & 0x07000000;
            break;
        case 0x148:
            TRACE2("CUR_CAPA = %08x", value);
            s->cur_capa = value & 0xffffff;
            break;
        default:
            OMAP_BAD_REG(addr);
            exit(-1);
    }
}

static const MemoryRegionOps omap3_mmc_ops = {
    .old_mmio = {
        .read = {
            omap_badwidth_read32,
            omap_badwidth_read32,
            omap3_mmc_read,
        },
        .write = {
            omap_badwidth_write32,
            omap_badwidth_write32,
            omap3_mmc_write,
        },
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static int omap3_mmc_init(SysBusDevice *dev)
{
    struct omap3_mmc_s *s = FROM_SYSBUS(struct omap3_mmc_s, dev);
    sysbus_init_irq(dev, &s->irq);
    sysbus_init_irq(dev, &s->dma[0]);
    sysbus_init_irq(dev, &s->dma[1]);
    memory_region_init_io(&s->iomem, &omap3_mmc_ops, s,
                          "omap3_mmc", 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
    return 0;
}

static void omap3_mmc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SysBusDeviceClass *k = SYS_BUS_DEVICE_CLASS(klass);
    k->init = omap3_mmc_init;
    dc->reset = omap3_mmc_reset;
}

static TypeInfo omap3_mmc_info = {
    .name = "omap3_mmc",
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(struct omap3_mmc_s),
    .class_init = omap3_mmc_class_init,
};

static void omap3_mmc_register_types(void)
{
    type_register_static(&omap3_mmc_info);
}

void omap3_mmc_attach(DeviceState *dev, BlockDriverState *bs,
                      int is_spi, int is_mmc)
{
    struct omap3_mmc_s *s = FROM_SYSBUS(struct omap3_mmc_s,
                                        SYS_BUS_DEVICE(dev));
    if (s->card) {
        hw_error("%s: card already attached!", __FUNCTION__);
    }
    s->card = sd_init(bs, is_spi, is_mmc);
}

type_init(omap3_mmc_register_types)
