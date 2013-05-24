/*
 * TI OMAP3 boot ROM emulation. Based on information in the OMAP34xx 3.1
 * Technical Reference Manual from Texas Instruments.
 *
 * Copyright (C) 2009 Nokia Corporation
 *
 * The OMAP3 boot ROM service routines accessed via ARM SMC instruction
 * are not available in this emulation due to the limited availability
 * of public documentation on the ARM TrustZone functionality. However
 * it seems executing the SMC instruction as a NOP causes no harm.
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
#include "sysemu/sysemu.h"
#include "sysemu/char.h"
#include "hw/block/flash.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"

//#define OMAP3_BOOT_DEBUG

#ifdef OMAP3_BOOT_DEBUG
#define TRACE(fmt,...) fprintf(stderr, "%s@%d: " fmt "\n", __FUNCTION__, \
                               __LINE__, ##__VA_ARGS__);
#else
#define TRACE(...)
#endif

/* list of supported NAND devices according to the OMAP34xx TRM */
static const struct {
    uint8_t id;
    uint32_t pagesize;
    uint32_t capacity_Mb;
} omap3_boot_nand_devices[] = {
    {0xe6,  512,    64}, {0x33,  512,   128}, {0x73,  512,   128},
    {0x43,  512,   128}, {0x53,  512,   128}, {0x35,  512,   256},
    {0x75,  512,   256}, {0x45,  512,   256}, {0x55,  512,   256},
    {0x36,  512,   512}, {0x76,  512,   512}, {0x46,  512,   512},
    {0x56,  512,   512}, {0xa2, 2048,   512}, {0xf2, 2048,   512},
    {0xb2, 2048,   512}, {0xc2, 2048,   512}, {0x39,  512,  1024},
    {0x79,  512,  1024}, {0x49,  512,  1024}, {0x59,  512,  1024},
    {0x78,  512,  1024}, {0x72,  512,  1024}, {0x74,  512,  1024},
    {0xa1, 2048,  1024}, {0xf1, 2048,  1024}, {0xb1, 2048,  1024},
    {0xc1, 2048,  1024}, {0xaa, 2048,  2048}, {0xda, 2048,  2048},
    {0xba, 2048,  2048}, {0xca, 2048,  2048}, {0x71,  512,  2048},
    {0x51,  512,  2048}, {0x31,  512,  2048}, {0x41,  512,  2048},
    {0xac, 2048,  4096}, {0xdc, 2048,  4096}, {0xbc, 2048,  4096},
    {0xcc, 2048,  4096}, {0xa3, 2048,  8192}, {0xd3, 2048,  8192},
    {0xb3, 2048,  8192}, {0xc3, 2048,  8192}, {0xa5, 2048, 16384},
    {0xd5, 2048, 16384}, {0xb5, 2048, 16384}, {0xc5, 2048, 16384},
    {0xa7, 2048, 32768}, {0xb7, 2048, 32768}, {0xae, 2048, 65536},
    {0xbe, 2048, 65536},
    {0, 0, 0}
};

struct omap3_nand_boot_desc_s {
    uint32_t pagesize;
    uint32_t capacity_Mb;
    uint8_t bus16;
};

static const uint8_t omap3_boot_rom[] = { /* 0x40014000-0x4001bfff */
    /* 0x40014000: ROM Exception vectors */
    0x3e, 0x00, 0x00, 0xea, /* b 0x40014100 */
    0x18, 0xf0, 0x9f, 0xe5, /* ldr pc, [pc, #0x18] */
    0x18, 0xf0, 0x9f, 0xe5, /* ldr pc, [pc, #0x18] */
    0x18, 0xf0, 0x9f, 0xe5, /* ldr pc, [pc, #0x18] */
    0x18, 0xf0, 0x9f, 0xe5, /* ldr pc, [pc, #0x18] */
    0x18, 0xf0, 0x9f, 0xe5, /* ldr pc, [pc, #0x18] */
    0x18, 0xf0, 0x9f, 0xe5, /* ldr pc, [pc, #0x18] */
    0x18, 0xf0, 0x9f, 0xe5, /* ldr pc, [pc, #0x18] */
    /* 0x40014020: ROM CRC */
    0xff, 0xff, 0xff, 0xff, 
    /* 0x40014024: unused(?), we use it for some data */
    0xc8, 0xff, 0x20, 0x40, /* 0x40014024: undef sram vector address */
    0xcc, 0xff, 0x20, 0x40, /* 0x40014028: swi sram vector address */
    0xd0, 0xff, 0x20, 0x40, /* 0x4001402c: pabt sram vector address */
    0xd4, 0xff, 0x20, 0x40, /* 0x40014030: dabt sram vector address */
    0xd8, 0xff, 0x20, 0x40, /* 0x40014034: unused sram vector address */
    0xdc, 0xff, 0x20, 0x40, /* 0x40014038: irq sram vector address */
    0xe0, 0xff, 0x20, 0x40, /* 0x4001403c: fiq sram vector address */
    0xff, 0xff, 0xff, 0xff, /* 0x40014040: boot loader image start address */
    0xff, 0xff, 0xff, 0xff, /* 0x40014044: booting parameter structure 0-3 */
    0xff, 0xff, 0xff, 0xff, /* 0x40014048: booting parameter structure 4-7 */
    0xff, 0xff, 0xff, 0xff, /* 0x4001404c: booting parameter structure 8-11 */
    0x0e, 0xf0, 0xb0, 0xe1, /* 0x40014050: "movs pc, lr" */
    0xff, 0xff, 0xff, 0xff, /* 0x40014054 */
    0xff, 0xff, 0xff, 0xff, /* 0x40014058 */
    0xff, 0xff, 0xff, 0xff, /* 0x4001405c */
    0xfe, 0xff, 0xff, 0xea, /* 0x40014060: monitor vector 0 (unused) */
    0xfe, 0xff, 0xff, 0xea, /* 0x40014064: monitor vector 1 (unused) */
    0x15, 0x00, 0x00, 0xea, /* 0x40014068: monitor vector 2 (smc) */
    0xfe, 0xff, 0xff, 0xea, /* 0x4001406c: monitor vector 3 (pabt) */
    0xfe, 0xff, 0xff, 0xea, /* 0x40014070: monitor vector 4 (dabt) */
    0xfe, 0xff, 0xff, 0xea, /* 0x40014074: monitor vector 5 (unused) */
    0xfe, 0xff, 0xff, 0xea, /* 0x40014078: monitor vector 6 (irq) */
    0xfe, 0xff, 0xff, 0xea, /* 0x4001407c: monitor vector 7 (fiq) */
    /* 0x40014080: Dead loops */
    0xfe, 0xff, 0xff, 0xea, /* b 0x40014080 @ undefined exception */
    0xfe, 0xff, 0xff, 0xea, /* b 0x40014084 @ swi exception */
    0xfe, 0xff, 0xff, 0xea, /* b 0x40014088 @ prefetch abort exception */
    0xfe, 0xff, 0xff, 0xea, /* b 0x4001408c @ data abort exception */
    0xfe, 0xff, 0xff, 0xea, /* b 0x40014090 @ unused exception */
    0xfe, 0xff, 0xff, 0xea, /* b 0x40014094 @ irq exception */
    0xfe, 0xff, 0xff, 0xea, /* b 0x40014098 @ fiq exception */
    0xfe, 0xff, 0xff, 0xea, /* b 0x4001409c @ validation tests pass */
    0xfe, 0xff, 0xff, 0xea, /* b 0x400140a0 @ validation tests fail */
    0xfe, 0xff, 0xff, 0xea, /* b 0x400140a4 @ boot failed: no more devices */
    0xfe, 0xff, 0xff, 0xea, /* b 0x400140a8 @ image not executed or returned */
    0xfe, 0xff, 0xff, 0xea, /* b 0x400140ac @ reserved */
    0xfe, 0xff, 0xff, 0xea, /* b 0x400140b0 @ reserved */
    0xfe, 0xff, 0xff, 0xea, /* b 0x400140b4 @ reserved */
    0xfe, 0xff, 0xff, 0xea, /* b 0x400140b8 @ reserved */
    0xfe, 0xff, 0xff, 0xea, /* b 0x400140bc @ reserved */
    /* 0x400140c0: should perform a software reset & jump to r0 */
    0x00, 0xf0, 0xa0, 0xe1, /* mov pc, r0 */
    /* 0x400140c4: monitor mode smc vector handler */
                            /* @ r12=1 is used to invalidate l2, we skip it */
    0x02, 0x00, 0x5c, 0xe3, /* cmp r12, #2 @ write to l2 cache aux ctrl */
    0x50, 0x0f, 0x29, 0xee, /* mcreq p15, 1, r0, c9, c0, 2 */
    0x03, 0x00, 0x5c, 0xe3, /* cmp r12, #3 @ write to aux ctrl */
    0x30, 0x0f, 0x01, 0xee, /* mcreq p15, 0, r0, c1, c0, 1 @ aux ctrl*/
    0x0e, 0xf0, 0xb0, 0xe1, /* movs pc, r14 */
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    /* 0x40014100: code, ROM emulation uses this to launch the
     * boot loader after it has been read into memory */
    0xc8, 0x10, 0x1f, 0xe5, /* ldr r1, [#0x40014040] @ boot loader start */
    0xb0, 0x0c, 0x0f, 0xe3, /* movw r0, #0xfcb0 */
    0x20, 0x00, 0x44, 0xe3, /* movt r0, #0x4020    @ stack top at 0x4020fcb0 */
    0xdf, 0xf0, 0x21, 0xe3, /* msr cpsr_c, #0xdf   @ enter SYS mode */
    0x00, 0xd0, 0xa0, 0xe1, /* mov sp, r0 */
    0x80, 0x0c, 0x40, 0xe2, /* sub r0, r0, #32768  @ 32kB SYS/USR stack */
    0xd1, 0xf0, 0x21, 0xe3, /* msr cpsr_c, #0xd1   @ enter FIQ mode */
    0x00, 0xd0, 0xa0, 0xe1, /* mov sp, r0 */
    0x08, 0x0c, 0x40, 0xe2, /* sub r0, r0, #2048   @ 2kB FIQ stack */
    0xd2, 0xf0, 0x21, 0xe3, /* msr cpsr_c, #0xd2   @ enter IRQ mode */
    0x00, 0xd0, 0xa0, 0xe1, /* mov sp, r0 */
    0x08, 0x0c, 0x40, 0xe2, /* sub r0, r0, #2048   @ 2kB IRQ stack */
    0xd7, 0xf0, 0x21, 0xe3, /* msr cpsr_c, #0xd7   @ enter ABT mode */
    0x00, 0xd0, 0xa0, 0xe1, /* mov sp, r0 */
    0x08, 0x0c, 0x40, 0xe2, /* sub r0, r0, #2048   @ 2kB ABT stack */
    0xdb, 0xf0, 0x21, 0xe3, /* msr cpsr_c, #0xdb   @ enter UND mode */
    0x00, 0xd0, 0xa0, 0xe1, /* mov sp, r0 */
    0x08, 0x0c, 0x40, 0xe2, /* sub r0, r0, #2048   @ 2kB UND stack */
    0xd3, 0xf0, 0x21, 0xe3, /* msr cpsr_c, #0xd3   @ enter SVC mode */
    0x00, 0xd0, 0xa0, 0xe1, /* mov sp, r0          @ 23kB left for SVC stack */
    0xdf, 0xf0, 0x21, 0xe3, /* msr cpsr_c, #0xdf   @ enter SYS mode */
    0x40, 0x04, 0xa0, 0xe3, /* mov r0, #0x40000000 @ r0 -> vba */
    0x05, 0x09, 0x80, 0xe2, /* add r0, r0, #0x14000 */
    0x10, 0x0f, 0x0c, 0xfe, /* mcr2 p15, 0, r0, c12, c0, 0 */
    0x60, 0x00, 0x80, 0xe2, /* add r0, r0, #0x60  @ r0 -> monitor vba */
    0x30, 0x0f, 0x0c, 0xfe, /* mcr2 p15, 0, r0, c12, c0, 1 */
    0x1c, 0x00, 0x40, 0xe2, /* sub r0, r0, #1c    @ r0 -> booting parameter struct */
    0x01, 0xf0, 0xa0, 0xe1, /* mov pc, r1 */
};

/* SRAM exception vectors, to be placed at 0x4020ffc8 */
static const uint8_t omap3_sram_vectors[] = {
    0x14, 0xf0, 0x9f, 0xe5, /* ldr pc, [#0x4020ffe4] @ undefined */
    0x14, 0xf0, 0x9f, 0xe5, /* ldr pc, [#0x4020ffe8] @ swi */
    0x14, 0xf0, 0x9f, 0xe5, /* ldr pc, [#0x4020ffec] @ prefetch abort */
    0x14, 0xf0, 0x9f, 0xe5, /* ldr pc, [#0x4020fff0] @ data abort */
    0x14, 0xf0, 0x9f, 0xe5, /* ldr pc, [#0x4020fff4] @ unused */
    0x14, 0xf0, 0x9f, 0xe5, /* ldr pc, [#0x4020fff8] @ irq */
    0x14, 0xf0, 0x9f, 0xe5, /* ldr pc, [#0x4020fffc] @ fiq */
    0x80, 0x40, 0x01, 0x00, /* 0x14080 */
    0x50, 0x40, 0x01, 0x40, /* 0x40014050 (default is 0x14084) */
    0x88, 0x40, 0x01, 0x00, /* 0x14088 */
    0x8c, 0x40, 0x01, 0x00, /* 0x1408c */
    0x90, 0x40, 0x01, 0x00, /* 0x14090 */
    0x94, 0x40, 0x01, 0x00, /* 0x14094 */
    0x98, 0x40, 0x01, 0x00, /* 0x14098 */
};

static inline uint32_t omap3_get_le32(const void *p)
{
    const uint8_t *q = (const uint8_t *)p;
    uint32_t v;
    v = q[3]; v <<= 8;
    v |= q[2]; v <<= 8;
    v |= q[1]; v <<= 8;
    v |= q[0];
    return v;
}

static inline uint32_t omap3_get_le16(const void *p)
{
    const uint8_t *q = (const uint8_t *)p;
    uint32_t v;
    v = q[1]; v <<= 8;
    v |= q[0];
    return v;
}

static inline void omap3_boot_setlsb(hwaddr addr, uint16_t lsb)
{
    uint8_t x[4];
    
    cpu_physical_memory_read(addr, x, 4);
    x[0] = lsb & 0xff;
    x[1] = (lsb >> 8) & 0xff;
    cpu_physical_memory_write(addr, x, 4);
}

typedef enum {
    xip = 1,
    nand,
    onenand,
    doc,
    mmc2,
    mmc1,
    xipwait,
    uart = 0x10,
    hsusb,
}  omap3_boot_device_t;

struct omap3_boot_s {
    struct omap_mpu_state_s *mpu;
    omap3_boot_device_t devicetype;
    enum {
        undefined = 0,
        confighdr,
        chdone,
        imagehdr,
        copy,
        done
    } state;
    uint8_t chflags;
    hwaddr addr;
    uint32_t count;
};

static struct omap3_boot_s *omap3_boot_init(struct omap_mpu_state_s *mpu,
                                            omap3_boot_device_t dtype,
                                            const uint8_t *data,
                                            uint32_t data_len)
{
    struct omap3_boot_s *s = g_malloc0(sizeof(struct omap3_boot_s));
    s->mpu = mpu;
    s->devicetype = dtype;
    s->state = chdone;
    if (data_len >= 512) {
        if (!strncasecmp((char *)(data + 0x14), "chsettings", 10)
            || !strncasecmp((char *)(data + 0x14), "chram", 5)
            || !strncasecmp((char *)(data + 0x14), "chflash", 7)
            || !strncasecmp((char *)(data + 0x14), "chmmcsd", 7))
            s->state = confighdr;
    }
    return s;
}

static void omap3_boot_chsettings(struct omap3_boot_s *boot,
                                  const uint8_t *chtoc)
{
    uint32_t flags, x;
    
    if (omap3_get_le32(chtoc) != 0xc0c0c0c1) {
        TRACE("invalid section verification key");
        return;
    }
    if (!chtoc[4]) { /* section disabled? */
        return;
    }
    if (omap3_get_le16(chtoc + 5) != 0x0001) {
        TRACE("unsupported CH version (0x%04x)", omap3_get_le16(chtoc));
        return;
    }
    boot->chflags |= 0x01;
    flags = omap3_get_le32(chtoc + 8);
    chtoc += 12;
    if (flags & 1) {
        cpu_physical_memory_write(0x48307270, chtoc + 0x00, 4); /* PRM_CLKSRC_CTRL */
        cpu_physical_memory_write(0x48306d40, chtoc + 0x04, 4); /* PRM_CLKSEL */
        cpu_physical_memory_write(0x48005140, chtoc + 0x08, 4); /* CM_CLKSEL1_EMU */
        if (flags & (1 << 2)) { /* clock configuration */
            cpu_physical_memory_write(0x48004a40, chtoc + 0x0c, 4); /* CM_CLKSEL_CORE */
            cpu_physical_memory_write(0x48004c40, chtoc + 0x10, 4); /* CM_CLKSEL_WKUP */
        }
        if (flags & (1 << 5)) { /* DPLL3 CORE */
            if (flags & (1 << 8)) { /* enable DPLL3 bypass */
                cpu_physical_memory_read(0x48004d00, (uint8_t *)&x, 4);
                x &= ~7; x |= 5; /* set DPLL3 bypass */
                cpu_physical_memory_write(0x48004d00, (uint8_t *)&x, 4);
            }
            cpu_physical_memory_write(0x48004d00, chtoc + 0x14, 4); /* CM_CLKEN_PLL */
            cpu_physical_memory_write(0x48004d30, chtoc + 0x18, 4); /* CM_AUTOIDLE_PLL */
            cpu_physical_memory_write(0x48004d40, chtoc + 0x1c, 4); /* CM_CLKSEL1_PLL */
        }
        if (flags & (1 << 3)) { /* DPLL4 PER */
            if (flags & (1 << 6)) { /* enable DPLL4 bypass */
                cpu_physical_memory_read(0x48004d00, (uint8_t *)&x, 4);
                x &= ~0x70000; x |= 0x10000; /* set DPLL4 in stop mode */
                cpu_physical_memory_write(0x48004d00, (uint8_t *)&x, 4);
            }
            cpu_physical_memory_write(0x48004d00, chtoc + 0x20, 4); /* CM_CLKEN_PLL */
            cpu_physical_memory_write(0x48004d30, chtoc + 0x24, 4); /* CM_AUTOIDLE_PLL */
            cpu_physical_memory_write(0x48004d44, chtoc + 0x28, 4); /* CM_CLKSEL2_PLL */
            cpu_physical_memory_write(0x48004d48, chtoc + 0x2c, 4); /* CM_CLKSEL3_PLL */
        }
        if (flags & (1 << 3)) { /* DPLL1 MPU */
            if (flags & (1 << 7)) { /* enable DPLL1 bypass */
                cpu_physical_memory_read(0x48004904, (uint8_t *)&x, 4);
                x &= ~7; x |= 5; /* set DPLL1 bypass */
                cpu_physical_memory_write(0x48004904, (uint8_t *)&x, 4);
            }
            cpu_physical_memory_write(0x48004904, chtoc + 0x30, 4); /* CM_CLKEN_PLL_MPU */
            cpu_physical_memory_write(0x48004934, chtoc + 0x34, 4); /* CM_AUTOIDLE_PLL_MPU */
            cpu_physical_memory_write(0x48004940, chtoc + 0x38, 4); /* CM_CLKSEL1_PLL_MPU */
            cpu_physical_memory_write(0x48004944, chtoc + 0x3c, 4); /* CM_CLKSEL2_PLL_MPU */
            cpu_physical_memory_write(0x48004948, chtoc + 0x40, 4); /* CM_CLKSTCTRL_MPU */
        }
        switch ((flags >> 24) & 0xff) {
            case 0x01: x = 0; break; /* 12MHz */
            case 0x02: x = 1; break; /* 13MHz */
            case 0x03: x = 5; break; /* 16.8MHz */
            case 0x04: x = 2; break; /* 19.2MHz */
            case 0x05: x = 3; break; /* 26MHz */
            case 0x06: x = 4; break; /* 38.4MHz */
            default:
                TRACE("unsupported SYS.CLK setting (0x%02x)", (flags >> 24) & 0xff);
                x = 1;
                break;
        }
        if (x != omap3_get_le32(chtoc + 0x04)) {
            TRACE("mismatch in SYS.CLK id and PRM_CLKSEL value");
        }
    }
}

static void omap3_boot_chram(struct omap3_boot_s *boot,
                             const uint8_t *chtoc)
{
    if (omap3_get_le32(chtoc) != 0xc0c0c0c2) {
        TRACE("invalid section verification key");
        return;
    }
    if (!chtoc[4]) { /* section disabled? */
        return;
    }
    boot->chflags |= 0x02;
    omap3_boot_setlsb(0x6d000040, omap3_get_le16(chtoc + 0x0a)); /* SDRC_CS_CFG */
    omap3_boot_setlsb(0x6d000044, omap3_get_le16(chtoc + 0x0c)); /* SDRC_SHARING */
    cpu_physical_memory_write(0x6d000060, chtoc + 0x10, 4);      /* SDRC_DLLA_CTRL */
    
    cpu_physical_memory_write(0x6d000080, chtoc + 0x20, 4);      /* SDRC_MCFG_0 */
    omap3_boot_setlsb(0x6d000084, omap3_get_le16(chtoc + 0x24)); /* SDRC_MR_0 */
    omap3_boot_setlsb(0x6d000088, omap3_get_le16(chtoc + 0x26)); /* SDRC_EMR1_0? */
    omap3_boot_setlsb(0x6d00008c, omap3_get_le16(chtoc + 0x28)); /* SDRC_EMR2_0 */
    omap3_boot_setlsb(0x6d000090, omap3_get_le16(chtoc + 0x2a)); /* SDRC_EMR3_0? */
    cpu_physical_memory_write(0x6d00009c, chtoc + 0x2c, 4);      /* SDRC_ACTIM_CTRLA_0 */
    cpu_physical_memory_write(0x6d0000a0, chtoc + 0x30, 4);      /* SDRC_ACTIM_CTRLB_0 */
    cpu_physical_memory_write(0x6d0000a4, chtoc + 0x34, 4);      /* SDRC_RFR_CTRL_0 */
    
    cpu_physical_memory_write(0x6d0000b0, chtoc + 0x20, 4);      /* SDRC_MCFG_1 */
    omap3_boot_setlsb(0x6d0000b4, omap3_get_le16(chtoc + 0x24)); /* SDRC_MR_1 */
    omap3_boot_setlsb(0x6d0000b8, omap3_get_le16(chtoc + 0x26)); /* SDRC_EMR1_1? */
    omap3_boot_setlsb(0x6d0000bc, omap3_get_le16(chtoc + 0x28)); /* SDRC_EMR2_1 */
    omap3_boot_setlsb(0x6d0000c0, omap3_get_le16(chtoc + 0x2a)); /* SDRC_EMR3_1? */
    cpu_physical_memory_write(0x6d0000cc, chtoc + 0x2c, 4);      /* SDRC_ACTIM_CTRLA_1 */
    cpu_physical_memory_write(0x6d0000d0, chtoc + 0x30, 4);      /* SDRC_ACTIM_CTRLB_1 */
    cpu_physical_memory_write(0x6d0000d4, chtoc + 0x34, 4);      /* SDRC_RFR_CTRL_1 */
}

static void omap3_boot_chflash(struct omap3_boot_s *boot,
                               const uint8_t *chtoc)
{
    if (omap3_get_le32(chtoc) != 0xc0c0c0c3) {
        TRACE("invalid section verification key");
        return;
    }
    if (!chtoc[4]) { /* section disabled? */
        return;
    }
    boot->chflags |= 0x04;
    omap3_boot_setlsb(0x6e000010, omap3_get_le16(chtoc + 0x08)); /* GPMC_SYSCONFIG */
    omap3_boot_setlsb(0x6e00001c, omap3_get_le16(chtoc + 0x0a)); /* GPMC_IRQENABLE */
    omap3_boot_setlsb(0x6e000040, omap3_get_le16(chtoc + 0x0c)); /* GPMC_TIMEOUT_CONTROL */
    omap3_boot_setlsb(0x6e000050, omap3_get_le16(chtoc + 0x0e)); /* GPMC_CONFIG */
    cpu_physical_memory_write(0x6e000060, chtoc + 0x10, 4);      /* GPMC_CONFIG1_0 */
    cpu_physical_memory_write(0x6e000064, chtoc + 0x14, 4);      /* GPMC_CONFIG2_0 */
    cpu_physical_memory_write(0x6e000068, chtoc + 0x18, 4);      /* GPMC_CONFIG3_0 */
    cpu_physical_memory_write(0x6e00006c, chtoc + 0x1c, 4);      /* GPMC_CONFIG4_0 */
    cpu_physical_memory_write(0x6e000070, chtoc + 0x20, 4);      /* GPMC_CONFIG5_0 */
    cpu_physical_memory_write(0x6e000074, chtoc + 0x24, 4);      /* GPMC_CONFIG6_0 */
    cpu_physical_memory_write(0x6e000078, chtoc + 0x28, 4);      /* GPMC_CONFIG7_0 */
    cpu_physical_memory_write(0x6e0001e0, chtoc + 0x2c, 4);      /* GPMC_PREFETCH_CONFIG1 */
    omap3_boot_setlsb(0x6e0001e4, omap3_get_le16(chtoc + 0x30)); /* GPMC_PREFETCH_CONFIG2 */
    omap3_boot_setlsb(0x6e0001ec, omap3_get_le16(chtoc + 0x32)); /* GPMC_PREFETCH_CONTROL */
    /* TODO: ECC config registers. The TRM spec is not clear on these */
}

static void omap3_boot_chmmcsd(struct omap3_boot_s *boot,
                               const uint8_t *chtoc)
{
    if (omap3_get_le32(chtoc) != 0xc0c0c0c4) {
        TRACE("invalid section verification key");
        return;
    }
    if (!chtoc[4]) { /* section disabled? */
        return;
    }
    boot->chflags |= 0x08;
    /* TODO: MMCHS registers */
}

/* returns non-zero if more blocks are needed */
static uint32_t omap3_boot_block(const uint8_t *data,
                                 uint32_t data_len,
                                 struct omap3_boot_s *s)
{
    const uint8_t *p = 0;
    uint32_t i = 0;
    
    switch (s->state) {
        case confighdr:
            i = data_len;
            for (p = data; i >= 32 && omap3_get_le32(p) != 0xffffffff; p += 32, i -= 32) {
                if (!strcasecmp((char *)(p + 0x14), "chsettings"))
                    omap3_boot_chsettings(s, p + omap3_get_le32(p));
                else if (!strcasecmp((char *)(p + 0x14), "chram"))
                    omap3_boot_chram(s, p + omap3_get_le32(p));
                else if (!strcasecmp((char *)(p + 0x14), "chflash"))
                    omap3_boot_chflash(s, p + omap3_get_le32(p));
                else if (!strcasecmp((char *)(p + 0x14), "chmmcsd"))
                    omap3_boot_chmmcsd(s, p + omap3_get_le32(p));
                else {
                    TRACE("unknown CHTOC item \"%s\"", (char *)(p + 0x14));
                }
            }
            data += 512;
            data_len -= 512;
            s->state = chdone;
            /* fallthrough */
        case chdone:
            s->state = imagehdr;
            /* fallthrough */
        case imagehdr:
            if (!data_len)
                return 1;
            if (data_len < 8)
                break;
            s->count = omap3_get_le32(data);
            s->addr = omap3_get_le32(data + 4);
            if (!s->count || (s->count >> 24) || !s->addr || s->addr == 0xffffffff)
                break;
            /* patch image start address in boot ROM */
            cpu_physical_memory_write_rom(0x40014040, data + 4, 4);
            /* start copying image */
            data += 8;
            data_len -= 8;
            s->state = copy;
            /* fallthrough */
        case copy:
            i = (s->count >= data_len) ? data_len : s->count;
            cpu_physical_memory_write(s->addr, data, i);
            s->addr += i;
            s->count -= i;
            if (!s->count)
                s->state = done;
            return s->count;
        default:
            break;
    }
    return 0;
}

/* returns non-zero if boot has finished succesfully */
static int omap3_boot_finish(struct omap3_boot_s *s)
{
    uint8_t x[12] = {
        0, 0, 0, 0, /* last received booting message */
        (uint8_t)s->devicetype,
        0,
        1, /* POR */
        s->chflags,
        0, 0, 0, 0 /* device descriptor */
    };
    int result = (s->state == done);

    if (result) {
        /* fill in the booting parameter structure */
        cpu_physical_memory_write_rom(0x40014044, x, 12);
    }
    free(s);
    return result;
}

/* returns ptr to matching dir entry / zero entry or 0 if unsuccessful */
static const uint8_t *omap3_scan_fat_dir_sector(const uint8_t *s)
{
    int i;
    
    /* there are 0x10 items with 0x20 bytes per item */
    for (i = 0x10; i--; s += 0x20) {
        if (*s == 0xe5 || (s[0x0b] & 0x0f) == 0x0f) continue; /* erased/LFN */
        if (!*s || !strncasecmp((void *)s, "mlo        ", 8+3)) return s;
    }
    return 0;
}

struct omap3_fat_drv_s {
    BlockDriverState *bs;
    uint8_t ptype; /* 12, 16, 32 */
    uint64_t c0;   /* physical byte offset for data cluster 0 */
    uint64_t fat;  /* physical byte offset for used FAT sector 0 */
    uint32_t spc;  /* sectors per cluster */
};

/* returns cluster data in the buffer and next cluster chain number
 or 0 if unsuccessful */
static uint32_t omap3_read_fat_cluster(uint8_t *data,
                                       struct omap3_fat_drv_s *drv,
                                       uint32_t cl)
{
    uint8_t buf[ 4 ];
    uint32_t len = drv->spc * 0x200; /* number of bytes to read */
    
    switch (drv->ptype) { /* check for EOF */
        case 12: if (cl > 0xff0) return 0; break;
        case 16: if (cl > 0xfff0) return 0; break;
        case 32: if (cl > 0x0ffffff0) return 0; break;
        default: return 0;
    }
    
    if (bdrv_pread(drv->bs, 
                   drv->c0 + (cl - 2) * len,
                   data, len) != len)
        return 0;
    
    switch (drv->ptype) { /* determine next cluster # */
        case 12:
            hw_error("%s: FAT12 parsing not implemented!", __FUNCTION__);
            break;
        case 16:
            return (bdrv_pread(drv->bs, drv->fat + cl * 2, buf, 2) != 2)
            ? 0 : omap3_get_le16(buf);
        case 32:
            return (bdrv_pread(drv->bs, drv->fat + cl * 4, buf, 4) != 4)
            ? 0 : omap3_get_le32(buf) & 0x0fffffff;
        default:
            break;
    }
    return 0;
}

static int omap3_mmc_fat_boot(BlockDriverState *bs,
                              uint8_t *sector,
                              uint32_t pstart,
                              struct omap_mpu_state_s *mpu)
{
    struct omap3_fat_drv_s drv;
    struct omap3_boot_s *boot;
    uint32_t i, j, cluster0, fatsize, bootsize, rootsize;
    const uint8_t *p, *q;
    uint8_t *cluster;
    int result = 0;
    
    /* determine FAT type */
    
    drv.bs = bs;
    fatsize = omap3_get_le16(sector + 0x16);
    if (!fatsize) 
        fatsize = omap3_get_le32(sector + 0x24);
    bootsize = omap3_get_le16(sector + 0x0e);
    cluster0 = bootsize + fatsize * sector[0x10];
    rootsize = omap3_get_le16(sector + 0x11);
    if (rootsize & 0x0f)
        rootsize += 0x10;
    rootsize >>= 4;
    drv.spc = sector[0x0d];
    i = omap3_get_le16(sector + 0x13);
    if (!i)
        i = omap3_get_le32(sector + 0x20);
    i = (i - (cluster0 + rootsize)) / drv.spc;
    drv.ptype = (i < 4085) ? 12 : (i < 65525) ? 16 : 32;
    
    /* search for boot loader file */
    
    drv.fat = (bootsize + pstart) * 0x200;
    drv.c0 = (cluster0 + pstart) * 0x200;
    if (drv.ptype == 32) {
        i = omap3_get_le32(sector + 0x2c); /* first root cluster # */
        j = omap3_get_le16(sector + 0x28);
        if (j & 0x80)
            drv.fat += (j & 0x0f) * fatsize * 0x200;
        cluster = g_malloc0(drv.spc * 0x200);
        for (p = 0; !p && (i = omap3_read_fat_cluster(cluster, &drv, i)); ) {
            for (j = drv.spc, q=cluster; j-- && !p; q += 0x200)
                p = omap3_scan_fat_dir_sector(q);
            if (p) {
                memcpy(sector, p, 0x200); /* save the sector */
                /* allows referring to p after freeing cluster */
                p = sector;
            }
        }
        free(cluster);
    } else { /* FAT12/16 */
        /*
         * drv.c0 points to start of root directory
         */
        for (i = rootsize, j = 0, p = 0; i-- && !p; j++) {
            if (bdrv_pread(drv.bs, drv.c0 + j * 0x200, sector, 0x200) != 0x200)
                break;
            p = omap3_scan_fat_dir_sector(sector);
        }
        /*
         * Adjust start of clusters to point after the root directory.
         */
        drv.c0 += 0x200 * rootsize;
    }
    
    if (p && *p) { /* did we indeed find the file? */
        /*
         * FAT16 Spec says to ignore bytes at offset 0x14 (the FAT32
         * high-order cylinder number bytes); but the omap3 bootrom
         * seems always to use them.
         */
        i = omap3_get_le16(p + 0x14);
        i <<= 16;
        i |= omap3_get_le16(p + 0x1a);
        j = drv.spc * 0x200;
        uint8 *data = g_malloc0(j);
        if ((i = omap3_read_fat_cluster(data, &drv, i))) {
            boot = omap3_boot_init(mpu, mmc1, data, j);
            while (omap3_boot_block(data, j, boot))
                i = omap3_read_fat_cluster(data, &drv, i);
            result = omap3_boot_finish(boot);
        } else {
            TRACE("unable to read MLO file contents from SD card");
        }
        free(data);
    } else {
        TRACE("MLO file not found in the root directory");
    }
    return result;
}

static int omap3_mmc_raw_boot(BlockDriverState *bs,
                              uint8_t *sector,
                              struct omap_mpu_state_s *mpu)
{
    struct omap3_boot_s *boot;
    uint32_t i = 0;
    int result = 0;
    /* We try to load an image from sectors 0 and 256 */
    uint32_t boot_sectors[] = { 0, 256 };
    int idx;

    for (idx = 0; !result && idx < ARRAY_SIZE(boot_sectors); idx++) {
        i = boot_sectors[idx];
        if (bdrv_pread(bs, i * 0x200, sector, 0x200) != 0x200) {
            TRACE("error trying to read sector %u on boot device", i);
            continue;
        }

        boot = omap3_boot_init(mpu, mmc1, sector, 0x200);
        if (boot->state == confighdr) {
            /* CH must be present for raw boot */
            while (omap3_boot_block(sector, 0x200, boot)) {
                i++;
                if (bdrv_pread(bs, i * 0x200, sector, 0x200) != 0x200) {
                    TRACE("error trying to read sector %u on boot device", i);
                    break;
                }
            }
        }

        result = (boot->state == done);
        free(boot);
    }
    return result;
}

/* returns non-zero if successful, zero if unsuccessful */
static int omap3_mmc_boot(struct omap_mpu_state_s *s)
{
    DriveInfo *di = drive_get(IF_SD, 0, 0);
    uint8_t *sector, *p;
    uint32_t pstart, i;
    int result = 0;
    
    /* very simple implementation for GP device boot,
     supports only two modes:
     1. MBR partition table with an active FAT partition
     and boot loader file (MLO) in its root directory, or
     2. CH sector located on first sector, followed by boot loader image */
    if (di) {
        sector = g_malloc0(0x200);
        if (bdrv_pread(di->bdrv, 0, sector, 0x200) == 0x200) {
            for (i = 0, p = sector + 0x1be; i < 4; i++, p += 0x10) 
                if (p[0] == 0x80) break;
            if (sector[0x1fe] == 0x55 && sector[0x1ff] == 0xaa /* signature */
                && i < 4 /* active partition exists */
                && (p[4] == 1 || p[4] == 4 || p[4] == 6 || p[4] == 11
                    || p[4] == 12 || p[4] == 14 || p[4] == 15) /* FAT */
                && bdrv_pread(di->bdrv,
                              (pstart = omap3_get_le32(p + 8)) * 0x200,
                              sector, 0x200) == 0x200
                && sector[0x1fe] == 0x55 && sector[0x1ff] == 0xaa)
                result = omap3_mmc_fat_boot(di->bdrv, sector, pstart, s);
            else
                result = omap3_mmc_raw_boot(di->bdrv, sector, s);
        }
        free(sector);
    }
    return result;
}

static inline void omap3_nand_sendcmd(struct omap3_nand_boot_desc_s *nd,
                                      uint8_t cmd)
{
    uint8_t x[2] = {cmd, 0};
    cpu_physical_memory_write(0x6e00007c, x, nd->bus16 ? 2 : 1);
}

static inline void omap3_nand_sendaddr_byte(struct omap3_nand_boot_desc_s *nd,
                                            uint8_t a)
{
    uint8_t x[2] = { a, 0 };
    
    cpu_physical_memory_write(0x6e000080, x, nd->bus16 ? 2 : 1);
}

static inline uint8_t omap3_nand_readbyte(struct omap3_nand_boot_desc_s *nd)
{
    uint8_t x[2];
    
    cpu_physical_memory_read(0x6e000084, x, nd->bus16 ? 2 : 1);
    return x[0];
}

static inline void omap3_nand_readpage(struct omap3_nand_boot_desc_s *nd,
                                       uint32_t pageaddr,
                                       uint8_t *data)
{
    uint32_t i;
    
    omap3_nand_sendcmd(nd, 0x00); /* read page */
    omap3_nand_sendaddr_byte(nd, 0x00);
    if (nd->pagesize >= 2048) {
        omap3_nand_sendaddr_byte(nd, 0x00);
        omap3_nand_sendaddr_byte(nd, (uint8_t)(pageaddr & 0xff));
        omap3_nand_sendaddr_byte(nd, (uint8_t)((pageaddr >> 8) & 0xff));
        if (nd->capacity_Mb >= 2048)
            omap3_nand_sendaddr_byte(nd, (uint8_t)((pageaddr >> 16) & 0xff));
        omap3_nand_sendcmd(nd, 0x30); /* confirm read */
    } else {
        omap3_nand_sendaddr_byte(nd, (uint8_t)(pageaddr & 0xff));
        omap3_nand_sendaddr_byte(nd, (uint8_t)((pageaddr >> 8) & 0xff));
    }
    if (nd->bus16) {
        for (i = nd->pagesize / 2; i--; data += 2)
            cpu_physical_memory_read(0x6e000084, data, 2);
    } else {
        for (i = nd->pagesize; i--; data++)
            cpu_physical_memory_read(0x6e000084, data, 1);
    }
}

/* returns non-zero if successful, zero if unsuccessful */
static int omap3_nand_boot(struct omap_mpu_state_s *mpu, int bus16)
{
    struct omap3_nand_boot_desc_s *nd;
    struct omap3_boot_s *boot;
    uint8_t *data;
    uint32_t page = 0;
    int result = 0, i;
    uint8_t id[4];
    
    /* TODO: support bad block marks */
    nd = g_malloc0(sizeof(struct omap3_nand_boot_desc_s));
    nd->bus16 = bus16;
    omap3_nand_sendcmd(nd, 0xff); /* reset */
    omap3_nand_sendcmd(nd, 0x90); /* read id */
    omap3_nand_sendaddr_byte(nd, 0);
    id[0] = omap3_nand_readbyte(nd); /* manufacturer id */
    id[1] = omap3_nand_readbyte(nd); /* device id */
    id[2] = omap3_nand_readbyte(nd); /* don't care */
    id[3] = omap3_nand_readbyte(nd); /* attributes */
    for (i = 0; omap3_boot_nand_devices[i].id; i++) {
        if (omap3_boot_nand_devices[i].id == id[1]) {
            nd->capacity_Mb = omap3_boot_nand_devices[i].capacity_Mb;
            if (nd->capacity_Mb > 1024)
                nd->pagesize = 1024 * (1 << (id[3] & 3));
            else
                nd->pagesize = omap3_boot_nand_devices[i].pagesize;
            break;
        }
    }
    /* TODO: if device is not recognized at this state, we should
     * issue READ ID2 command to the device and get device parameters
     * from there */
    if (nd->pagesize) {
        data = g_malloc0(nd->pagesize);
        /* TODO: scan through 4 first blocks for image */
        omap3_nand_readpage(nd, 0, data);
        boot = omap3_boot_init(mpu, nand, data, nd->pagesize);
        while (omap3_boot_block(data, nd->pagesize, boot))
            omap3_nand_readpage(nd, ++page, data);
        result = omap3_boot_finish(boot);
        free(data);
    }
    free(nd);
    return result;
}

static inline void omap3_onenand_writereg(uint16_t reg, uint16_t value)
{
    cpu_to_le16s(&value);
    cpu_physical_memory_write(0x08000000 + (reg << 1), (void *)&value, 2);
}

static inline uint16_t omap3_onenand_readreg(uint16_t reg)
{
    uint16_t value;
    cpu_physical_memory_read(0x08000000 + (reg << 1), (void *)&value, 2);
    return le16_to_cpu(value);
}

static int omap3_onenand_readpage(uint16_t pagesize,
                                  uint16_t b,
                                  uint16_t p,
                                  uint8_t *d)
{
    omap3_onenand_writereg(0xf100, b);
    omap3_onenand_writereg(0xf107, (p & 0x3f) << 2);
    omap3_onenand_writereg(0xf200, 0x0800);
    omap3_onenand_writereg(0xf101, 0);
    omap3_onenand_writereg(0xf241, 0);
    omap3_onenand_writereg(0xf220, 0);
    if (!(omap3_onenand_readreg(0xf241) & 0x8000) ||
        (omap3_onenand_readreg(0xf240) & 0x0400))
        return 0;
    cpu_physical_memory_read(0x08000400, (void *)d, pagesize);
    return 1;
}

static int omap3_onenand_boot(struct omap_mpu_state_s *s)
{
    uint32_t x;
    uint16_t i, j, pagesize;
    uint8_t *page;
    struct omap3_boot_s *boot;
    int result = 0;

    /* reset device type at cs0: 16bit NOR, no wait monitoring */
    cpu_to_le32wu(&x, 0x79001000);
    cpu_physical_memory_write(0x6e000060, (void *)&x, 4); /* GPMC_CONFIG1_0 */
    /* map cs0 at 0x08000000 */
    cpu_to_le32wu(&x, 0x00000848);
    cpu_physical_memory_write(0x6e000078, (void *)&x, 4); /* GPMC_CONFIG7_0 */
    /* try to read onenand registers */
    x = omap3_onenand_readreg(0xf000);                    /* manufacturer id */
    if (!x || (x >> 8)) { /* we accept any non-zero 8bit id */
        return 0;
    }
    pagesize = omap3_onenand_readreg(0xf003);
    if (pagesize != 2048 && pagesize != 1024) {
        hw_error("%s: OneNAND page size %d not supported",
                __FUNCTION__, pagesize);
        return 0;
    }
    /* search for boot loader */
    page = g_malloc0(pagesize);
    for (i = 0; i < 4; i++) { /* search 4 blocks */
        if (omap3_onenand_readpage(pagesize, i, 0, page)) {
            boot = omap3_boot_init(s, onenand, page, pagesize);
            for (j = 1; omap3_boot_block(page, pagesize, boot); j++)
                if (!omap3_onenand_readpage(pagesize, i, j, page))
                    break;
            result = omap3_boot_finish(boot);
            if (result)
                break;
        }
    }
    free(page);
    return result;
}

void omap3_boot_rom_init(struct omap_mpu_state_s *s)
{
    const uint8_t rom_version[4] = { 0x00, 0x14, 0x00, 0x00 }; /* v. 14.00 */

    if (!s->bootrom_initialized) {
        s->bootrom_initialized = 1;
        memory_region_init_ram(&s->bootrom, "omap3_boot_rom",
                               OMAP3XXX_BOOTROM_SIZE);
        memory_region_set_readonly(&s->bootrom, true);
        memory_region_add_subregion(get_system_memory(),
                                    OMAP3_Q1_BASE + 0x14000,
                                    &s->bootrom);
        cpu_physical_memory_write_rom(OMAP3_Q1_BASE + 0x14000,
                                      omap3_boot_rom,
                                      sizeof(omap3_boot_rom));
        cpu_physical_memory_write_rom(OMAP3_Q1_BASE + 0x1bffc,
                                      rom_version,
                                      sizeof(rom_version));
        cpu_physical_memory_write(OMAP3_SRAM_BASE + 0xffc8,
                                  omap3_sram_vectors,
                                  sizeof(omap3_sram_vectors));
    }
}

void omap3_boot_rom_emu(struct omap_mpu_state_s *s)
{
    uint8_t x[4] = {0, 0, 0, 0};
    int result = 0;
    
    /* only emulate the boot rom if it was initialized earlier */
    if (!s->bootrom_initialized) {
        return;
    }
    
    /* here we are relying on all memories to be attached and gpmc_attach
     * to fill in DEVICETYPE field correctly for CS0 for us */
    cpu_physical_memory_read(0x6e000060, x, 4); /* GPMC_CONFIG1_0 */
    switch (((x[1] >> 2) & 3)) {
        case 0: /* NOR */
            result = omap3_onenand_boot(s);
            break;
        case 2: /* NAND */
            result = omap3_nand_boot(s, ((x[1] >> 4) & 3) == 1);
            break;
        default:
            break;
    }

    /* if no boot loader found yet, try the MMC/SD card... */
    if (!result)
        result = omap3_mmc_boot(s);

    /* move PC to the boot ROM reset vector */
    s->cpu->env.regs[15] = 0x40014000;

    if (!result) { /* no boot device found */
        /* move PC to the appropriate ROM dead loop address */
        s->cpu->env.regs[15] = 0x400140a4;
        /* ...on second thought, let's just call it a day and quit */
        hw_error("no boot device found");
    }
}
