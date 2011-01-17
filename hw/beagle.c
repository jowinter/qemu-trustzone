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
#include "sysemu.h"
#include "omap.h"
#include "arm-misc.h"
#include "boards.h"
#include "i2c.h"
#include "net.h"
#include "devices.h"
#include "flash.h"
#include "sysbus.h"
#include "blockdev.h"

#define BEAGLE_NAND_CS       0
#define BEAGLE_SMC_CS        1
#define BEAGLE_NAND_PAGESIZE 0x800
#define BEAGLE_SDRAM_SIZE    (128 * 1024 * 1024) /* 128MB */

/* Beagle board support */
struct beagle_s {
    struct omap_mpu_state_s *cpu;
    
    DeviceState *nand;
    void *twl4030;
    DeviceState *smc;
};

static void beagle_init(ram_addr_t ram_size,
                        const char *boot_device,
                        const char *kernel_filename,
                        const char *kernel_cmdline,
                        const char *initrd_filename,
                        const char *cpu_model)
{
    struct beagle_s *s = (struct beagle_s *) qemu_mallocz(sizeof(*s));
    DriveInfo *dmtd = drive_get(IF_MTD, 0, 0);
    DriveInfo *dsd  = drive_get(IF_SD, 0, 0);
    
    if (!dmtd && !dsd) {
        hw_error("%s: SD or NAND image required", __FUNCTION__);
    }
#if MAX_SERIAL_PORTS < 1
#error MAX_SERIAL_PORTS must be at least 1!
#endif
	s->cpu = omap3_mpu_init(omap3430, ram_size,
                            NULL, NULL, serial_hds[0], NULL);

	s->nand = nand_init(NAND_MFR_MICRON, 0xba, dmtd ? dmtd->bdrv : NULL);
	nand_setpins(s->nand, 0, 0, 0, 1, 0); /* no write-protect */
    omap_gpmc_attach(s->cpu->gpmc, BEAGLE_NAND_CS, s->nand, 0, 2);
    if (dsd) {
        omap3_mmc_attach(s->cpu->omap3_mmc[0], dsd->bdrv, 0, 0);
    }

    s->twl4030 = twl4030_init(omap_i2c_bus(s->cpu->i2c, 0),
                              s->cpu->irq[0][OMAP_INT_3XXX_SYS_NIRQ],
                              NULL, NULL);
    int i;
    for (i = 0; i < nb_nics; i++) {
        if (!nd_table[i].model || !strcmp(nd_table[i].model, "smc91c111")) {
            break;
        }
    }
    if (i < nb_nics) {
        s->smc = qdev_create(NULL, "smc91c111");
        qdev_set_nic_properties(s->smc, &nd_table[i]);
        qdev_init_nofail(s->smc);
        sysbus_connect_irq(sysbus_from_qdev(s->smc), 0,
                           qdev_get_gpio_in(s->cpu->gpio, 54));
    } else {
        hw_error("%s: no NIC for smc91c111\n", __FUNCTION__);
    }
    omap_gpmc_attach(s->cpu->gpmc, BEAGLE_SMC_CS, s->smc, 0, 0);

    omap_lcd_panel_attach(s->cpu->dss);

    omap3_boot_rom_emu(s->cpu);
}

QEMUMachine beagle_machine = {
    .name =        "beagle",
    .desc =        "Beagle board (OMAP3530)",
    .init =        beagle_init,
};

static void beagle_machine_init(void)
{
    qemu_register_machine(&beagle_machine);
}

machine_init(beagle_machine_init);
