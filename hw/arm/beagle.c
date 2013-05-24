/*
 * Beagle board emulation. http://beagleboard.org/
 *
 * Copyright (c) 2009 Nokia Corporation
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include "qemu-common.h"
#include "sysemu/sysemu.h"
#include "hw/arm/omap.h"
#include "hw/arm/arm.h"
#include "hw/boards.h"
#include "hw/i2c/i2c.h"
#include "net/net.h"
#include "hw/block/flash.h"
#include "hw/sysbus.h"
#include "sysemu/blockdev.h"
#include "exec/address-spaces.h"

#define BEAGLE_NAND_CS       0
#define BEAGLE_SMC_CS        1
#define BEAGLE_NAND_PAGESIZE 0x800
#define BEAGLE_SDRAM_SIZE    (256 * 1024 * 1024) /* 256MB */
#define BEAGLE_XM_SDRAM_SIZE (512 * 1024 * 1024) /* 512MB */
/* GPIO ID pins are used to identify which beagle variant we have */
#define BEAGLE_GPIO_ID1      171
#define BEAGLE_GPIO_ID2      172
#define BEAGLE_GPIO_ID3      173

/* Beagle board support */
struct beagle_s {
    struct omap_mpu_state_s *cpu;

    DeviceState *nand;
    void *twl4030;
    DeviceState *smc;
    DeviceState *ddc;
};

static void beagle_common_init(QEMUMachineInitArgs *args,
                               ram_addr_t ram_size, int cpu_model)
{
    MemoryRegion *sysmem = get_system_memory();
    struct beagle_s *s = (struct beagle_s *) g_malloc0(sizeof(*s));
    DriveInfo *dmtd = drive_get(IF_MTD, 0, 0);
    DriveInfo *dsd  = drive_get(IF_SD, 0, 0);

    if (!dmtd && !dsd) {
        hw_error("%s: SD or NAND image required", __FUNCTION__);
    }
#if MAX_SERIAL_PORTS < 1
#error MAX_SERIAL_PORTS must be at least 1!
#endif
    s->cpu = omap3_mpu_init(sysmem, cpu_model, ram_size,
                            NULL, NULL, serial_hds[0], NULL);

    s->nand = nand_init(dmtd ? dmtd->bdrv : NULL, NAND_MFR_MICRON, 0xba);
    nand_setpins(s->nand, 0, 0, 0, 1, 0); /* no write-protect */
    omap_gpmc_attach_nand(s->cpu->gpmc, BEAGLE_NAND_CS, s->nand);

    if (dsd) {
        omap3_mmc_attach(s->cpu->omap3_mmc[0], dsd->bdrv, 0, 0);
    }

    s->twl4030 = twl4030_init(omap_i2c_bus(s->cpu->i2c[0]),
                              qdev_get_gpio_in(s->cpu->ih[0],
                                               OMAP_INT_3XXX_SYS_NIRQ),
                              NULL, NULL);
    if (cpu_model == omap3430) {
        qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio, BEAGLE_GPIO_ID1),1);
        qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio, BEAGLE_GPIO_ID3),1);
    }

    /* Wire up an I2C slave which returns EDID monitor information;
     * newer Linux kernels won't turn on the display unless they
     * detect a monitor over DDC.
     */
    s->ddc = i2c_create_slave(omap_i2c_bus(s->cpu->i2c[2]), "i2c-ddc", 0x50);

    omap_lcd_panel_attach(s->cpu->dss);
}

static void beagle_xm_init(QEMUMachineInitArgs *args)
{
    beagle_common_init(args, BEAGLE_XM_SDRAM_SIZE, omap3630);
}
static void beagle_init(QEMUMachineInitArgs *args)
{
    beagle_common_init(args, BEAGLE_SDRAM_SIZE, omap3430);
}

QEMUMachine beagle_machine = {
    .name =        "beagle",
    .desc =        "Beagle board (OMAP3530)",
    .init =        beagle_init,
};

QEMUMachine beagle_xm_machine = {
    .name =        "beaglexm",
    .desc =        "Beagle board XM (OMAP3630)",
    .init =        beagle_xm_init,
};


static void beagle_machine_init(void)
{
    qemu_register_machine(&beagle_machine);
    qemu_register_machine(&beagle_xm_machine);
}

machine_init(beagle_machine_init);
