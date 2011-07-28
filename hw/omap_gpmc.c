/*
 * TI OMAP general purpose memory controller emulation.
 *
 * Copyright (C) 2007-2009 Nokia Corporation
 * Original code written by Andrzej Zaborowski <andrew@openedhand.com>
 * Enhancements for OMAP3 and NAND support written by Juha Riihimäki
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
#include "flash.h"
#include "omap.h"
#include "sysbus.h"

/* General-Purpose Memory Controller */
struct omap_gpmc_s {
    qemu_irq irq;
    qemu_irq drq;
    int accept_256;

    uint8_t revision;
    uint8_t sysconfig;
    uint16_t irqst;
    uint16_t irqen;
    uint16_t lastirq;
    uint16_t timeout;
    uint16_t config;
    struct omap_gpmc_cs_file_s {
        uint32_t config[7];
        DeviceState *dev;
        int mmio_index;
        int iomemtype;
    } cs_file[8];
    int ecc_cs;
    int ecc_ptr;
    uint32_t ecc_cfg;
    ECCState ecc[9];
    struct prefetch {
        uint32_t config1; /* GPMC_PREFETCH_CONFIG1 */
        uint32_t transfercount; /* GPMC_PREFETCH_CONFIG2:TRANSFERCOUNT */
        int startengine; /* GPMC_PREFETCH_CONTROL:STARTENGINE */
        int fifopointer; /* GPMC_PREFETCH_STATUS:FIFOPOINTER */
        int count; /* GPMC_PREFETCH_STATUS:COUNTVALUE */
        int iomemtype;
        uint8_t fifo[64];
    } prefetch;
};

#define OMAP_GPMC_8BIT 0
#define OMAP_GPMC_16BIT 1

static int omap_gpmc_devtype(struct omap_gpmc_cs_file_s *f)
{
    return (f->config[0] >> 10) & 3;
}

static int omap_gpmc_devsize(struct omap_gpmc_cs_file_s *f)
{
    return (f->config[0] >> 12) & 3;
}

/* Extract the chip-select value from the prefetch config1 register */
static int prefetch_cs(uint32_t config1)
{
    return (config1 >> 24) & 7;
}

static int prefetch_threshold(uint32_t config1)
{
    return (config1 >> 8) & 0x7f;
}

static void omap_gpmc_int_update(struct omap_gpmc_s *s)
{
    /* The TRM is a bit unclear, but it seems to say that
     * the TERMINALCOUNTSTATUS bit is set only on the
     * transition when the prefetch engine goes from
     * active to inactive, whereas the FIFOEVENTSTATUS
     * bit is held high as long as the fifo has at
     * least THRESHOLD bytes available.
     * So we do the latter here, but TERMINALCOUNTSTATUS
     * is set elsewhere.
     */
    if (s->prefetch.fifopointer >= prefetch_threshold(s->prefetch.config1)) {
        s->irqst |= 1;
    }
    if ((s->irqen & s->irqst) != s->lastirq) {
        s->lastirq = s->irqen & s->irqst;
        qemu_set_irq(s->irq, s->lastirq);
    }
}

static void omap_gpmc_dma_update(struct omap_gpmc_s *s, int value)
{
    if (s->prefetch.config1 & 4) {
        qemu_set_irq(s->drq, value);
    }
}

/* Access functions for when a NAND-like device is mapped into memory:
 * all addresses in the region behave like accesses to the relevant
 * GPMC_NAND_DATA_i register (which is actually implemented to call these)
 */
static uint32_t omap_nand_read8(void *opaque, target_phys_addr_t addr)
{
    struct omap_gpmc_cs_file_s *f = (struct omap_gpmc_cs_file_s *)opaque;
    nand_setpins(f->dev, 0, 0, 0, 1, 0);
    switch (omap_gpmc_devsize(f)) {
    case OMAP_GPMC_8BIT:
        return nand_getio(f->dev);
    case OMAP_GPMC_16BIT:
        /* reading 8bits from a 16bit device?! */
        return nand_getio(f->dev);
    default:
        return 0;
    }
}

static uint32_t omap_nand_read16(void *opaque, target_phys_addr_t addr)
{
    struct omap_gpmc_cs_file_s *f = (struct omap_gpmc_cs_file_s *)opaque;
    uint32_t x1, x2;
    nand_setpins(f->dev, 0, 0, 0, 1, 0);
    switch (omap_gpmc_devsize(f)) {
    case OMAP_GPMC_8BIT:
        x1 = nand_getio(f->dev);
        x2 = nand_getio(f->dev);
        return (x2 << 8) | x1;
    case OMAP_GPMC_16BIT:
        return nand_getio(f->dev);
    default:
        return 0;
    }
}

static uint32_t omap_nand_read32(void *opaque, target_phys_addr_t addr)
{
    struct omap_gpmc_cs_file_s *f = (struct omap_gpmc_cs_file_s *)opaque;
    uint32_t x1, x2, x3, x4;
    nand_setpins(f->dev, 0, 0, 0, 1, 0);
    switch (omap_gpmc_devsize(f)) {
    case OMAP_GPMC_8BIT:
        x1 = nand_getio(f->dev);
        x2 = nand_getio(f->dev);
        x3 = nand_getio(f->dev);
        x4 = nand_getio(f->dev);
        return (x4 << 24) | (x3 << 16) | (x2 << 8) | x1;
    case OMAP_GPMC_16BIT:
        x1 = nand_getio(f->dev);
        x2 = nand_getio(f->dev);
        return (x2 << 16) | x1;
    default:
        return 0;
    }
}

static void omap_nand_write8(void *opaque, target_phys_addr_t addr,
                             uint32_t value)
{
    struct omap_gpmc_cs_file_s *f = (struct omap_gpmc_cs_file_s *)opaque;
    nand_setpins(f->dev, 0, 0, 0, 1, 0);
    switch (omap_gpmc_devsize(f)) {
    case OMAP_GPMC_8BIT:
        nand_setio(f->dev, value & 0xff);
        break;
    case OMAP_GPMC_16BIT:
        /* writing to a 16bit device with 8bit access!? */
        nand_setio(f->dev, value & 0xffff);
        break;
    default:
        break;
    }
}

static void omap_nand_write16(void *opaque, target_phys_addr_t addr,
                              uint32_t value)
{
    struct omap_gpmc_cs_file_s *f = (struct omap_gpmc_cs_file_s *)opaque;
    nand_setpins(f->dev, 0, 0, 0, 1, 0);
    switch (omap_gpmc_devsize(f)) {
    case OMAP_GPMC_8BIT:
        nand_setio(f->dev, value & 0xff);
        nand_setio(f->dev, (value >> 8) & 0xff);
        break;
    case OMAP_GPMC_16BIT:
        nand_setio(f->dev, value & 0xffff);
        break;
    default:
        break;
    }
}

static void omap_nand_write32(void *opaque, target_phys_addr_t addr,
                              uint32_t value)
{
    struct omap_gpmc_cs_file_s *f = (struct omap_gpmc_cs_file_s *)opaque;
    nand_setpins(f->dev, 0, 0, 0, 1, 0);
    switch (omap_gpmc_devsize(f)) {
    case OMAP_GPMC_8BIT:
        nand_setio(f->dev, value & 0xff);
        nand_setio(f->dev, (value >> 8) & 0xff);
        nand_setio(f->dev, (value >> 16) & 0xff);
        nand_setio(f->dev, (value >> 24) & 0xff);
        break;
    case OMAP_GPMC_16BIT:
        nand_setio(f->dev, value & 0xffff);
        nand_setio(f->dev, (value >> 16) & 0xffff);
        break;
    default:
        break;
    }
}

static CPUReadMemoryFunc *omap_nand_readfn[] = {
    omap_nand_read8,
    omap_nand_read16,
    omap_nand_read32,
};

static CPUWriteMemoryFunc *omap_nand_writefn[] = {
    omap_nand_write8,
    omap_nand_write16,
    omap_nand_write32,
};

static void fill_prefetch_fifo(struct omap_gpmc_s *s)
{
    /* Fill the prefetch FIFO by reading data from NAND.
     * We do this synchronously, unlike the hardware which
     * will do this asynchronously. We refill when the
     * FIFO has THRESHOLD bytes free, and we always refill
     * as much data as possible starting at the top end
     * of the FIFO.
     * (We have to refill at THRESHOLD rather than waiting
     * for the FIFO to empty to allow for the case where
     * the FIFO size isn't an exact multiple of THRESHOLD
     * and we're doing DMA transfers.)
     * This means we never need to handle wrap-around in
     * the fifo-reading code, and the next byte of data
     * to read is always fifo[63 - fifopointer].
     */
    int fptr;
    int cs = prefetch_cs(s->prefetch.config1);
    int is16bit = (((s->cs_file[cs].config[0] >> 12) & 3) != 0);
    int bytes;
    /* Don't believe the bit of the OMAP TRM that says that COUNTVALUE
     * and TRANSFERCOUNT are in units of 16 bit words for 16 bit NAND.
     * Instead believe the bit that says it is always a byte count.
     */
    bytes = 64 - s->prefetch.fifopointer;
    if (bytes > s->prefetch.count) {
        bytes = s->prefetch.count;
    }
    s->prefetch.count -= bytes;
    s->prefetch.fifopointer += bytes;
    fptr = 64 - s->prefetch.fifopointer;
    /* Move the existing data in the FIFO so it sits just
     * before what we're about to read in
     */
    while (fptr < (64 - bytes)) {
        s->prefetch.fifo[fptr] = s->prefetch.fifo[fptr + bytes];
        fptr++;
    }
    while (fptr < 64) {
        if (is16bit) {
            uint32_t v = omap_nand_read16(&s->cs_file[cs], 0);
            s->prefetch.fifo[fptr++] = v & 0xff;
            s->prefetch.fifo[fptr++] = (v >> 8) & 0xff;
        } else {
            s->prefetch.fifo[fptr++] = omap_nand_read8(&s->cs_file[cs], 0);
        }
    }
    if (s->prefetch.startengine && (s->prefetch.count == 0)) {
        /* This was the final transfer: raise TERMINALCOUNTSTATUS */
        s->irqst |= 2;
        s->prefetch.startengine = 0;
    }
    /* If there are any bytes in the FIFO at this point then
     * we must raise a DMA request (either this is a final part
     * transfer, or we filled the FIFO in which case we certainly
     * have THRESHOLD bytes available)
     */
    if (s->prefetch.fifopointer != 0) {
        omap_gpmc_dma_update(s, 1);
    }
    omap_gpmc_int_update(s);
}

/* Access functions for a NAND-like device when the prefetch/postwrite
 * engine is enabled -- all addresses in the region behave alike:
 * data is read or written to the FIFO.
 */
static uint32_t omap_gpmc_prefetch_read8(void *opaque, target_phys_addr_t addr)
{
    struct omap_gpmc_s *s = (struct omap_gpmc_s *) opaque;
    uint32_t data;
    if (s->prefetch.config1 & 1) {
        /* The TRM doesn't define the behaviour if you read from the
         * FIFO when the prefetch engine is in write mode. We choose
         * to always return zero.
         */
        return 0;
    }
    /* Note that trying to read an empty fifo repeats the last byte */
    if (s->prefetch.fifopointer) {
        s->prefetch.fifopointer--;
    }
    data = s->prefetch.fifo[63 - s->prefetch.fifopointer];
    if (s->prefetch.fifopointer ==
        (64 - prefetch_threshold(s->prefetch.config1))) {
        /* We've drained THRESHOLD bytes now. So deassert the
         * DMA request, then refill the FIFO (which will probably
         * assert it again.)
         */
        omap_gpmc_dma_update(s, 0);
        fill_prefetch_fifo(s);
    }
    omap_gpmc_int_update(s);
    return data;
}
static uint32_t omap_gpmc_prefetch_read16(void *opaque, target_phys_addr_t addr)
{
    uint32_t r = omap_gpmc_prefetch_read8(opaque, addr);
    r |= (omap_gpmc_prefetch_read8(opaque, addr) << 8);
    return r;
}
static uint32_t omap_gpmc_prefetch_read32(void *opaque, target_phys_addr_t addr)
{
    uint32_t r = omap_gpmc_prefetch_read8(opaque, addr);
    r |= (omap_gpmc_prefetch_read8(opaque, addr) << 8);
    r |= (omap_gpmc_prefetch_read8(opaque, addr) << 16);
    r |= (omap_gpmc_prefetch_read8(opaque, addr) << 24);
    return r;
}

static void omap_gpmc_prefetch_write8(void *opaque, target_phys_addr_t addr,
                                      uint32_t value)
{
    struct omap_gpmc_s *s = (struct omap_gpmc_s *) opaque;
    int cs = prefetch_cs(s->prefetch.config1);
    if ((s->prefetch.config1 & 1) == 0) {
        /* The TRM doesn't define the behaviour of writing to the
         * FIFO when the prefetch engine is in read mode. We
         * choose to ignore the write.
         */
        return;
    }
    if (s->prefetch.count == 0) {
        /* The TRM doesn't define the behaviour of writing to the
         * FIFO if the transfer is complete. We choose to ignore.
         */
        return;
    }
    /* The only reason we do any data buffering in postwrite
     * mode is if we are talking to a 16 bit NAND device, in
     * which case we need to buffer the first byte of the
     * 16 bit word until the other byte arrives.
     */
    int is16bit = (((s->cs_file[cs].config[0] >> 12) & 3) != 0);
    if (is16bit) {
        /* fifopointer alternates between 64 (waiting for first
         * byte of word) and 63 (waiting for second byte)
         */
        if (s->prefetch.fifopointer == 64) {
            s->prefetch.fifo[0] = value;
            s->prefetch.fifopointer--;
        } else {
            value = (value << 8) | s->prefetch.fifo[0];
            omap_nand_write16(&s->cs_file[cs], 0, value);
            s->prefetch.count--;
            s->prefetch.fifopointer = 64;
        }
    } else {
        /* Just write the byte : fifopointer remains 64 at all times */
        omap_nand_write8(&s->cs_file[cs], 0, value);
        s->prefetch.count--;
    }
    if (s->prefetch.count == 0) {
        /* Final transfer: raise TERMINALCOUNTSTATUS */
        s->irqst |= 2;
        s->prefetch.startengine = 0;
    }
    omap_gpmc_int_update(s);
}
static void omap_gpmc_prefetch_write16(void *opaque, target_phys_addr_t addr,
                                       uint32_t value)
{
    omap_gpmc_prefetch_write8(opaque, addr, value & 0xff);
    omap_gpmc_prefetch_write8(opaque, addr, (value >> 8) & 0xff);
}
static void omap_gpmc_prefetch_write32(void *opaque, target_phys_addr_t addr,
                                       uint32_t value)
{
    omap_gpmc_prefetch_write8(opaque, addr, value & 0xff);
    omap_gpmc_prefetch_write8(opaque, addr, (value >> 8) & 0xff);
    omap_gpmc_prefetch_write8(opaque, addr, (value >> 16) & 0xff);
    omap_gpmc_prefetch_write8(opaque, addr, (value >> 24) & 0xff);
}

static CPUReadMemoryFunc *omap_gpmc_prefetch_readfn[] = {
    omap_gpmc_prefetch_read8,
    omap_gpmc_prefetch_read16,
    omap_gpmc_prefetch_read32,
};

static CPUWriteMemoryFunc *omap_gpmc_prefetch_writefn[] = {
    omap_gpmc_prefetch_write8,
    omap_gpmc_prefetch_write16,
    omap_gpmc_prefetch_write32,
};

static void omap_gpmc_cs_map(struct omap_gpmc_s *s, int cs)
{
    struct omap_gpmc_cs_file_s *f = &s->cs_file[cs];
    uint32_t mask = (f->config[6] >> 8) & 0xf;
    uint32_t base = f->config[6] & 0x3f;
    uint32_t size;
    if (!(f->config[6] & (1 << 6))) {
        /* Do nothing unless CSVALID */
        return;
    }
    /* TODO: check for overlapping regions and report access errors */
    if (mask != 0x8 && mask != 0xc && mask != 0xe && mask != 0xf
         && !(s->accept_256 && !mask)) {
        fprintf(stderr, "%s: invalid chip-select mask address (0x%x)\n",
                 __func__, mask);
    }
    base <<= 24;
    size = (0x0fffffff & ~(mask << 24)) + 1;
    switch (omap_gpmc_devtype(f)) {
    case OMAP_GPMC_NOR:
        /* TODO: rather than setting the size of the mapping (which should be
         * constant), the mask should cause wrapping of the address space, so
         * that the same memory becomes accessible at every <i>size</i> bytes
         * starting from <i>base</i>.  */
        if (f->dev && f->mmio_index >= 0) {
            sysbus_mmio_resize(sysbus_from_qdev(f->dev), f->mmio_index, size);
            sysbus_mmio_map(sysbus_from_qdev(f->dev), f->mmio_index, base);
        }
        break;
    case OMAP_GPMC_NAND:
        if ((s->prefetch.config1 & 0x80) &&
            (prefetch_cs(s->prefetch.config1) == cs)) {
            /* The prefetch engine is enabled for this CS: map the FIFO */
            cpu_register_physical_memory(base, size, s->prefetch.iomemtype);
        } else {
            cpu_register_physical_memory(base, size, f->iomemtype);
        }
        break;
    default:
        break;
    }
}

static void omap_gpmc_cs_unmap(struct omap_gpmc_s *s, int cs)
{
    struct omap_gpmc_cs_file_s *f = &s->cs_file[cs];
    if (!(f->config[6] & (1 << 6))) {
        /* Do nothing unless CSVALID */
        return;
    }

    switch (omap_gpmc_devtype(f)) {
    case OMAP_GPMC_NOR:
        if (f->dev && f->mmio_index >= 0) {
            sysbus_mmio_unmap(sysbus_from_qdev(f->dev), f->mmio_index);
        }
        break;
    case OMAP_GPMC_NAND:
        {
            uint32_t mask = (f->config[6] >> 8) & 0xf;
            uint32_t base = (f->config[6] & 0x3f) << 24;
            uint32_t size = (0x0fffffff & ~(mask << 24)) + 1;
            cpu_register_physical_memory(base, size, IO_MEM_UNASSIGNED);
        }
        break;
    default:
        break;
    }
}

void omap_gpmc_reset(struct omap_gpmc_s *s)
{
    int i;

    s->sysconfig = 0;
    s->irqst = 0;
    s->irqen = 0;
    omap_gpmc_int_update(s);
    s->timeout = 0;
    s->config = 0xa00;
    s->prefetch.config1 = 0x00004000;
    s->prefetch.transfercount = 0x00000000;
    s->prefetch.startengine = 0;
    s->prefetch.fifopointer = 0;
    s->prefetch.count = 0;
    for (i = 0; i < 8; i ++) {
        omap_gpmc_cs_unmap(s, i);
        s->cs_file[i].config[1] = 0x101001;
        s->cs_file[i].config[2] = 0x020201;
        s->cs_file[i].config[3] = 0x10031003;
        s->cs_file[i].config[4] = 0x10f1111;
        s->cs_file[i].config[5] = 0;
        s->cs_file[i].config[6] = 0xf00;
        /* FIXME: attached devices should be probed for some of the CFG1 bits
         * for now we just keep those bits intact over resets as they are set
         * initially with omap_gpmc_attach function */
        if (i == 0) {
            s->cs_file[i].config[0] &= 0x00433e00;
            s->cs_file[i].config[6] |= 1 << 6; /* CSVALID */
            omap_gpmc_cs_map(s, i);
        } else {
            /* FIXME: again, this should force device size to 16bit but
             * here instead we keep what omap_gpmc_attach has done */
            s->cs_file[i].config[0] &= 0x00403c00;
        }
    }
    s->ecc_cs = 0;
    s->ecc_ptr = 0;
    s->ecc_cfg = 0x3fcff000;
    for (i = 0; i < 9; i ++)
        ecc_reset(&s->ecc[i]);
}

static uint32_t omap_gpmc_read32(void *opaque, target_phys_addr_t addr)
{
    struct omap_gpmc_s *s = (struct omap_gpmc_s *) opaque;
    int cs;
    struct omap_gpmc_cs_file_s *f;

    switch (addr) {
    case 0x000:	/* GPMC_REVISION */
        return s->revision;

    case 0x010:	/* GPMC_SYSCONFIG */
        return s->sysconfig;

    case 0x014:	/* GPMC_SYSSTATUS */
        return 1;						/* RESETDONE */

    case 0x018:	/* GPMC_IRQSTATUS */
        return s->irqst;

    case 0x01c:	/* GPMC_IRQENABLE */
        return s->irqen;

    case 0x040:	/* GPMC_TIMEOUT_CONTROL */
        return s->timeout;

    case 0x044:	/* GPMC_ERR_ADDRESS */
    case 0x048:	/* GPMC_ERR_TYPE */
        return 0;

    case 0x050:	/* GPMC_CONFIG */
        return s->config;

    case 0x054:	/* GPMC_STATUS */
        return 0x001;

    case 0x060 ... 0x1d4:
        cs = (addr - 0x060) / 0x30;
        addr -= cs * 0x30;
        f = s->cs_file + cs;
        switch (addr) {
        case 0x60:      /* GPMC_CONFIG1 */
            return f->config[0];
        case 0x64:      /* GPMC_CONFIG2 */
            return f->config[1];
        case 0x68:      /* GPMC_CONFIG3 */
            return f->config[2];
        case 0x6c:      /* GPMC_CONFIG4 */
            return f->config[3];
        case 0x70:      /* GPMC_CONFIG5 */
            return f->config[4];
        case 0x74:      /* GPMC_CONFIG6 */
            return f->config[5];
        case 0x78:      /* GPMC_CONFIG7 */
            return f->config[6];
        case 0x84:      /* GPMC_NAND_DATA */
            if (omap_gpmc_devtype(f) == OMAP_GPMC_NAND) {
                return omap_nand_read32(f, 0);
            }
            return 0;
        }
        break;

    case 0x1e0:	/* GPMC_PREFETCH_CONFIG1 */
        return s->prefetch.config1;
    case 0x1e4:	/* GPMC_PREFETCH_CONFIG2 */
        return s->prefetch.transfercount;
    case 0x1ec:	/* GPMC_PREFETCH_CONTROL */
        return s->prefetch.startengine;
    case 0x1f0:	/* GPMC_PREFETCH_STATUS */
        return (s->prefetch.fifopointer << 24) |
                ((s->prefetch.fifopointer >=
                  ((s->prefetch.config1 >> 8) & 0x7f) ? 1 : 0) << 16) |
                s->prefetch.count;

    case 0x1f4:	/* GPMC_ECC_CONFIG */
        return s->ecc_cs;
    case 0x1f8:	/* GPMC_ECC_CONTROL */
        return s->ecc_ptr;
    case 0x1fc:	/* GPMC_ECC_SIZE_CONFIG */
        return s->ecc_cfg;
    case 0x200 ... 0x220:	/* GPMC_ECC_RESULT */
        cs = (addr & 0x1f) >> 2;
        /* TODO: check correctness */
        return
                ((s->ecc[cs].cp    &  0x07) <<  0) |
                ((s->ecc[cs].cp    &  0x38) << 13) |
                ((s->ecc[cs].lp[0] & 0x1ff) <<  3) |
                ((s->ecc[cs].lp[1] & 0x1ff) << 19);

    case 0x230:	/* GPMC_TESTMODE_CTRL */
        return 0;
    case 0x234:	/* GPMC_PSA_LSB */
    case 0x238:	/* GPMC_PSA_MSB */
        return 0x00000000;
    }

    OMAP_BAD_REG(addr);
    return 0;
}

static uint32_t omap_gpmc_read8(void *opaque, target_phys_addr_t addr)
{
    struct omap_gpmc_s *s = (struct omap_gpmc_s *) opaque;
    int cs;
    struct omap_gpmc_cs_file_s *f;

    switch (addr) {
    case 0x060 ... 0x1d4:
        cs = (addr - 0x060) / 0x30;
        addr -= cs * 0x30;
        f = s->cs_file + cs;
        switch (addr) {
        case 0x84 ... 0x87: /* GPMC_NAND_DATA */
            if (omap_gpmc_devtype(f) == OMAP_GPMC_NAND) {
                return omap_nand_read8(f, 0);
            }
            return 0;
        default:
            break;
        }
        break;
    default:
        break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static uint32_t omap_gpmc_read16(void *opaque, target_phys_addr_t addr)
{
    struct omap_gpmc_s *s = (struct omap_gpmc_s *) opaque;
    int cs;
    struct omap_gpmc_cs_file_s *f;

    switch (addr) {
    case 0x060 ... 0x1d4:
        cs = (addr - 0x060) / 0x30;
        addr -= cs * 0x30;
        f = s->cs_file + cs;
        switch (addr) {
        case 0x84: /* GPMC_NAND_DATA */
        case 0x86:
            if (omap_gpmc_devtype(f) == OMAP_GPMC_NAND) {
                return omap_nand_read16(f, 0);
            }
            return 0;
        default:
            break;
        }
        break;
    default:
        break;
    }
    OMAP_BAD_REG(addr);
    return 0;
}

static void omap_gpmc_write32(void *opaque, target_phys_addr_t addr,
                              uint32_t value)
{
    struct omap_gpmc_s *s = (struct omap_gpmc_s *) opaque;
    int cs;
    struct omap_gpmc_cs_file_s *f;

    switch (addr) {
    case 0x000:	/* GPMC_REVISION */
    case 0x014:	/* GPMC_SYSSTATUS */
    case 0x054:	/* GPMC_STATUS */
    case 0x1f0:	/* GPMC_PREFETCH_STATUS */
    case 0x200 ... 0x220:	/* GPMC_ECC_RESULT */
    case 0x234:	/* GPMC_PSA_LSB */
    case 0x238:	/* GPMC_PSA_MSB */
        OMAP_RO_REGV(addr, value);
        break;

    case 0x010:	/* GPMC_SYSCONFIG */
        if ((value >> 3) == 0x3)
            fprintf(stderr, "%s: bad SDRAM idle mode %i\n",
                            __FUNCTION__, value >> 3);
        if (value & 2)
            omap_gpmc_reset(s);
        s->sysconfig = value & 0x19;
        break;

    case 0x018:	/* GPMC_IRQSTATUS */
        s->irqen &= ~value;
        omap_gpmc_int_update(s);
        break;

    case 0x01c:	/* GPMC_IRQENABLE */
        s->irqen = value & 0xf03;
        omap_gpmc_int_update(s);
        break;

    case 0x040:	/* GPMC_TIMEOUT_CONTROL */
        s->timeout = value & 0x1ff1;
        break;

    case 0x044:	/* GPMC_ERR_ADDRESS */
    case 0x048:	/* GPMC_ERR_TYPE */
        break;

    case 0x050:	/* GPMC_CONFIG */
        s->config = value & 0xf13;
        break;

    case 0x060 ... 0x1d4:
        cs = (addr - 0x060) / 0x30;
        addr -= cs * 0x30;
        f = s->cs_file + cs;
        switch (addr) {
        case 0x60:      /* GPMC_CONFIG1 */
            f->config[0] = value & 0xffef3e13;
            break;
        case 0x64:      /* GPMC_CONFIG2 */
            f->config[1] = value & 0x001f1f8f;
            break;
        case 0x68:      /* GPMC_CONFIG3 */
            f->config[2] = value & 0x001f1f8f;
            break;
        case 0x6c:      /* GPMC_CONFIG4 */
            f->config[3] = value & 0x1f8f1f8f;
            break;
        case 0x70:      /* GPMC_CONFIG5 */
            f->config[4] = value & 0x0f1f1f1f;
            break;
        case 0x74:      /* GPMC_CONFIG6 */
            f->config[5] = value & 0x00000fcf;
            break;
        case 0x78:      /* GPMC_CONFIG7 */
            if ((f->config[6] ^ value) & 0xf7f) {
                omap_gpmc_cs_unmap(s, cs);
                f->config[6] = value & 0x00000f7f;
                omap_gpmc_cs_map(s, cs);
            }
            break;
        case 0x7c:      /* GPMC_NAND_COMMAND */
        case 0x80:      /* GPMC_NAND_ADDRESS */
            if (omap_gpmc_devtype(f) == OMAP_GPMC_NAND) {
                switch (addr) {
                case 0x7c: /* CLE */
                    nand_setpins(f->dev, 1, 0, 0, 1, 0);
                    break;
                case 0x80: /* ALE */
                    nand_setpins(f->dev, 0, 1, 0, 1, 0);
                    break;
                default:
                    break;
                }
                switch (omap_gpmc_devsize(f)) {
                case OMAP_GPMC_8BIT:
                    nand_setio(f->dev, value & 0xff);
                    nand_setio(f->dev, (value >> 8) & 0xff);
                    nand_setio(f->dev, (value >> 16) & 0xff);
                    nand_setio(f->dev, (value >> 24) & 0xff);
                    break;
                case OMAP_GPMC_16BIT:
                    nand_setio(f->dev, value & 0xffff);
                    nand_setio(f->dev, (value >> 16) & 0xffff);
                    break;
                default:
                    break;
                }
            }
            break;
        case 0x84:  /* GPMC_NAND_DATA */
            if (omap_gpmc_devtype(f) == OMAP_GPMC_NAND) {
                omap_nand_write32(f, 0, value);
            }
            break;
        default:
            goto bad_reg;
        }
        break;

    case 0x1e0:	/* GPMC_PREFETCH_CONFIG1 */
        if (!s->prefetch.startengine) {
            uint32_t oldconfig1 = s->prefetch.config1;
            uint32_t changed;
            s->prefetch.config1 = value & 0x7f8f7fbf;
            changed = oldconfig1 ^ s->prefetch.config1;
            if (changed & (0x80 | 0x7000000)) {
                /* Turning the engine on or off, or mapping it somewhere else.
                 * cs_map() and cs_unmap() check the prefetch config and
                 * overall CSVALID bits, so it is sufficient to unmap-and-map
                 * both the old cs and the new one.
                 */
                int oldcs = prefetch_cs(oldconfig1);
                int newcs = prefetch_cs(s->prefetch.config1);
                omap_gpmc_cs_unmap(s, oldcs);
                omap_gpmc_cs_map(s, oldcs);
                if (newcs != oldcs) {
                    omap_gpmc_cs_unmap(s, newcs);
                    omap_gpmc_cs_map(s, newcs);
                }
            }
        }
        break;

    case 0x1e4:	/* GPMC_PREFETCH_CONFIG2 */
        if (!s->prefetch.startengine) {
            s->prefetch.transfercount = value & 0x3fff;
        }
        break;

    case 0x1ec:	/* GPMC_PREFETCH_CONTROL */
        if (s->prefetch.startengine != (value & 1)) {
            s->prefetch.startengine = value & 1;
            if (s->prefetch.startengine) {
                /* Prefetch engine start */
                s->prefetch.count = s->prefetch.transfercount;
                if (s->prefetch.config1 & 1) {
                    /* Write */
                    s->prefetch.fifopointer = 64;
                } else {
                    /* Read */
                    s->prefetch.fifopointer = 0;
                    fill_prefetch_fifo(s);
                }
            } else {
                /* Prefetch engine forcibly stopped. The TRM
                 * doesn't define the behaviour if you do this.
                 * We clear the prefetch count, which means that
                 * we permit no more writes, and don't read any
                 * more data from NAND. The CPU can still drain
                 * the FIFO of unread data.
                 */
                s->prefetch.count = 0;
            }
            omap_gpmc_int_update(s);
        }
        break;

    case 0x1f4:	/* GPMC_ECC_CONFIG */
        s->ecc_cs = 0x8f;
        break;
    case 0x1f8:	/* GPMC_ECC_CONTROL */
        if (value & (1 << 8))
            for (cs = 0; cs < 9; cs ++)
                ecc_reset(&s->ecc[cs]);
        s->ecc_ptr = value & 0xf;
        if (s->ecc_ptr == 0 || s->ecc_ptr > 9) {
            s->ecc_ptr = 0;
            s->ecc_cs &= ~1;
        }
        break;
    case 0x1fc:	/* GPMC_ECC_SIZE_CONFIG */
        s->ecc_cfg = value & 0x3fcff1ff;
        break;
    case 0x230:	/* GPMC_TESTMODE_CTRL */
        if (value & 7)
            fprintf(stderr, "%s: test mode enable attempt\n", __FUNCTION__);
        break;

    default:
    bad_reg:
        OMAP_BAD_REGV(addr, value);
        return;
    }
}

static void omap_gpmc_write8(void *opaque, target_phys_addr_t addr,
                             uint32_t value)
{
    struct omap_gpmc_s *s = (struct omap_gpmc_s *) opaque;
    int cs;
    struct omap_gpmc_cs_file_s *f;

    switch (addr) {
    case 0x060 ... 0x1d4:
        cs = (addr - 0x060) / 0x30;
        addr -= cs * 0x30;
        f = s->cs_file + cs;
        switch (addr) {
        case 0x7c ... 0x7f:     /* GPMC_NAND_COMMAND */
        case 0x80 ... 0x83:     /* GPMC_NAND_ADDRESS */
            if (omap_gpmc_devtype(f) == OMAP_GPMC_NAND) {
                switch (addr) {
                case 0x7c ... 0x7f:
                    nand_setpins(f->dev, 1, 0, 0, 1, 0); /* CLE */
                    break;
                case 0x80 ... 0x83:
                    nand_setpins(f->dev, 0, 1, 0, 1, 0); /* ALE */
                    break;
                default:
                    break;
                }
                switch (omap_gpmc_devsize(f)) {
                case OMAP_GPMC_8BIT:
                    nand_setio(f->dev, value & 0xff);
                    break;
                case OMAP_GPMC_16BIT:
                    /* writing to a 16bit device with 8bit access!? */
                    nand_setio(f->dev, value & 0xffff);
                    break;
                default:
                    break;
                }
            }
            break;
        case 0x84 ... 0x87: /* GPMC_NAND_DATA */
            if (omap_gpmc_devtype(f) == OMAP_GPMC_NAND) {
                omap_nand_write8(f, 0, value);
            }
            break;
        default:
            goto bad_reg;
        }
        break;
    default:
    bad_reg:
        OMAP_BAD_REGV(addr, value);
        return;
    }
}

static void omap_gpmc_write16(void *opaque, target_phys_addr_t addr,
                              uint32_t value)
{
    struct omap_gpmc_s *s = (struct omap_gpmc_s *) opaque;
    int cs;
    struct omap_gpmc_cs_file_s *f;

    switch (addr) {
    case 0x060 ... 0x1d4:
        cs = (addr - 0x060) / 0x30;
        addr -= cs * 0x30;
        f = s->cs_file + cs;
        switch (addr) {
        case 0x7c:      /* GPMC_NAND_COMMAND */
        case 0x7e:
        case 0x80:      /* GPMC_NAND_ADDRESS */
        case 0x82:
            if (omap_gpmc_devtype(f) == OMAP_GPMC_NAND) {
                switch (addr) {
                case 0x7c:
                case 0x7e:
                    nand_setpins(f->dev, 1, 0, 0, 1, 0); /* CLE */
                    break;
                case 0x80:
                case 0x82:
                    nand_setpins(f->dev, 0, 1, 0, 1, 0); /* ALE */
                    break;
                case 0x84:
                case 0x86:
                    nand_setpins(f->dev, 0, 0, 0, 1, 0);
                    break;
                default:
                    break;
                }
                switch (omap_gpmc_devsize(f)) {
                case OMAP_GPMC_8BIT:
                    nand_setio(f->dev, value & 0xff);
                    nand_setio(f->dev, (value >> 8) & 0xff);
                    break;
                case OMAP_GPMC_16BIT:
                    nand_setio(f->dev, value & 0xffff);
                    break;
                default:
                    break;
                }
            }
            break;
        case 0x84:      /* GPMC_NAND_DATA */
        case 0x86:
            if (omap_gpmc_devtype(f) == OMAP_GPMC_NAND) {
                omap_nand_write16(f, 0, value);
            }
            break;
        default:
            goto bad_reg;
        }
        break;
    default:
    bad_reg:
        OMAP_BAD_REGV(addr, value);
        return;
    }
}

static CPUReadMemoryFunc * const omap_gpmc_readfn[] = {
    omap_gpmc_read8,
    omap_gpmc_read16,
    omap_gpmc_read32,
};

static CPUWriteMemoryFunc * const omap_gpmc_writefn[] = {
    omap_gpmc_write8,
    omap_gpmc_write16,
    omap_gpmc_write32,
};

struct omap_gpmc_s *omap_gpmc_init(struct omap_mpu_state_s *mpu,
                                   target_phys_addr_t base,
                                   qemu_irq irq, qemu_irq drq)
{
    int iomemtype, cs;
    struct omap_gpmc_s *s = (struct omap_gpmc_s *)
            qemu_mallocz(sizeof(struct omap_gpmc_s));

    s->irq = irq;
    s->drq = drq;
    s->accept_256 = cpu_is_omap3630(mpu);
    s->revision = cpu_class_omap3(mpu) ? 0x50 : 0x20;
    s->lastirq = 0;
    omap_gpmc_reset(s);

    iomemtype = cpu_register_io_memory(omap_gpmc_readfn,
                    omap_gpmc_writefn, s, DEVICE_NATIVE_ENDIAN);
    cpu_register_physical_memory(base, 0x1000, iomemtype);

    /* We have to register a different IO memory handler for each
     * chip select region in case a NAND device is mapped there.  */
    for (cs = 0; cs < 8; cs++) {
        s->cs_file[cs].iomemtype = cpu_register_io_memory(omap_nand_readfn,
                                                          omap_nand_writefn,
                                                          &s->cs_file[cs],
                                                          DEVICE_NATIVE_ENDIAN);
    }

    s->prefetch.iomemtype = cpu_register_io_memory(omap_gpmc_prefetch_readfn,
                                                   omap_gpmc_prefetch_writefn,
                                                   s, DEVICE_NATIVE_ENDIAN);
    return s;
}

void omap_gpmc_attach(struct omap_gpmc_s *s, int cs, DeviceState *dev,
                      int mmio_index, int devicetype)
{
    struct omap_gpmc_cs_file_s *f;

    if (cs < 0 || cs >= 8) {
        fprintf(stderr, "%s: bad chip-select %i\n", __FUNCTION__, cs);
        exit(-1);
    }
    f = &s->cs_file[cs];
    if (dev != f->dev || mmio_index != f->mmio_index ||
        devicetype != omap_gpmc_devtype(f)) {
        omap_gpmc_cs_unmap(s, cs);
        f->dev = dev;
        f->mmio_index = mmio_index;
        f->config[0] &= ~(0x3 << 10);
        f->config[0] |= (devicetype & 3) << 10;
        f->config[0] &= ~(0x3 << 12);
        if (omap_gpmc_devtype(f) == OMAP_GPMC_NAND) {
            if (nand_getbuswidth(f->dev) == 16) {
                f->config[0] |= OMAP_GPMC_16BIT << 12;
            }
        }
        omap_gpmc_cs_map(s, cs);
    }
}
