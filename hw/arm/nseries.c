/*
 * Nokia N-series internet tablets.
 *
 * Copyright (C) 2007 Nokia Corporation
 * Written by Andrzej Zaborowski <andrew@openedhand.com>
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu-common.h"
#include "sysemu/sysemu.h"
#include "hw/arm/omap.h"
#include "hw/arm/arm.h"
#include "hw/irq.h"
#include "ui/console.h"
#include "hw/boards.h"
#include "hw/i2c/i2c.h"
#include "hw/devices.h"
#include "hw/block/flash.h"
#include "hw/spi.h"
#include "hw/hw.h"
#include "hw/bt.h"
#include "hw/loader.h"
#include "net/net.h"
#include "sysemu/blockdev.h"
#include "hw/sysbus.h"
#include "exec/address-spaces.h"

//#define MIPID_DEBUG

#ifdef MIPID_DEBUG
#define TRACE_MIPID(fmt, ...) \
    fprintf(stderr, "%s@%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)
#else
#define TRACE_MIPID(...)
#endif

/* Nokia N8x0 support */
struct n800_s {
    struct omap_mpu_state_s *mpu;

    struct rfbi_chip_s blizzard;
    DeviceState *mipid;
    DeviceState *tsc;

    int keymap[0x80];
    DeviceState *kbd;

    DeviceState *usb;
    void *retu;
    void *tahvo;
    DeviceState *nand;
};

/* GPIO pins */
#define N8X0_TUSB_ENABLE_GPIO		0
#define N800_MMC2_WP_GPIO		8
#define N800_UNKNOWN_GPIO0		9	/* out */
#define N810_MMC2_VIOSD_GPIO		9
#define N810_HEADSET_AMP_GPIO		10
#define N800_CAM_TURN_GPIO		12
#define N810_GPS_RESET_GPIO		12
#define N800_BLIZZARD_POWERDOWN_GPIO	15
#define N800_MMC1_WP_GPIO		23
#define N810_MMC2_VSD_GPIO		23
#define N8X0_ONENAND_GPIO		26
#define N810_BLIZZARD_RESET_GPIO	30
#define N800_UNKNOWN_GPIO2		53	/* out */
#define N8X0_TUSB_INT_GPIO		58
#define N8X0_BT_WKUP_GPIO		61
#define N8X0_STI_GPIO			62
#define N8X0_CBUS_SEL_GPIO		64
#define N8X0_CBUS_DAT_GPIO		65
#define N8X0_CBUS_CLK_GPIO		66
#define N8X0_WLAN_IRQ_GPIO		87
#define N8X0_BT_RESET_GPIO		92
#define N8X0_TEA5761_CS_GPIO		93
#define N800_UNKNOWN_GPIO		94
#define N810_TSC_RESET_GPIO		94
#define N800_CAM_ACT_GPIO		95
#define N810_GPS_WAKEUP_GPIO		95
#define N8X0_MMC_CS_GPIO		96
#define N8X0_WLAN_PWR_GPIO		97
#define N8X0_BT_HOST_WKUP_GPIO		98
#define N810_SPEAKER_AMP_GPIO		101
#define N810_KB_LOCK_GPIO		102
#define N800_TSC_TS_GPIO		103
#define N810_TSC_TS_GPIO		106
#define N8X0_HEADPHONE_GPIO		107
#define N8X0_RETU_GPIO			108
#define N800_TSC_KP_IRQ_GPIO		109
#define N810_KEYBOARD_GPIO		109
#define N800_BAT_COVER_GPIO		110
#define N810_SLIDE_GPIO			110
#define N8X0_TAHVO_GPIO			111
#define N800_UNKNOWN_GPIO4		112	/* out */
#define N810_SLEEPX_LED_GPIO		112
#define N800_TSC_RESET_GPIO		118	/* ? */
#define N810_AIC33_RESET_GPIO		118
#define N800_TSC_UNKNOWN_GPIO		119	/* out */
#define N8X0_TMP105_GPIO		125

/* Config */
#define BT_UART				0
#define XLDR_LL_UART			1

/* Addresses on the I2C bus 0 */
#define N810_TLV320AIC33_ADDR		0x18	/* Audio CODEC */
#define N8X0_TCM825x_ADDR		0x29	/* Camera */
#define N810_LP5521_ADDR		0x32	/* LEDs */
#define N810_TSL2563_ADDR		0x3d	/* Light sensor */
#define N810_LM8323_ADDR		0x45	/* Keyboard */
/* Addresses on the I2C bus 1 */
#define N8X0_TMP105_ADDR		0x48	/* Temperature sensor */
#define N8X0_MENELAUS_ADDR		0x72	/* Power management */

/* Chipselects on GPMC NOR interface */
#define N8X0_ONENAND_CS			0
#define N8X0_USB_ASYNC_CS		1
#define N8X0_USB_SYNC_CS		4

#define N8X0_BD_ADDR			0x00, 0x1a, 0x89, 0x9e, 0x3e, 0x81

static void n800_mmc_cs_cb(void *opaque, int line, int level)
{
    /* TODO: this seems to actually be connected to the menelaus, to
     * which also both MMC slots connect.  */
    omap_mmc_enable((struct omap_mmc_s *) opaque, !level);
}

static void n8x0_gpio_setup(struct n800_s *s)
{
    qemu_irq *mmc_cs = qemu_allocate_irqs(n800_mmc_cs_cb, s->mpu->mmc, 1);
    qdev_connect_gpio_out(s->mpu->gpio, N8X0_MMC_CS_GPIO, mmc_cs[0]);

    qemu_irq_lower(qdev_get_gpio_in(s->mpu->gpio, N800_BAT_COVER_GPIO));
}

#define MAEMO_CAL_HEADER(...)				\
    'C',  'o',  'n',  'F',  0x02, 0x00, 0x04, 0x00,	\
    __VA_ARGS__,					\
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

static const uint8_t n8x0_cal_wlan_mac[] = {
    MAEMO_CAL_HEADER('w', 'l', 'a', 'n', '-', 'm', 'a', 'c')
    0x1c, 0x00, 0x00, 0x00, 0x47, 0xd6, 0x69, 0xb3,
    0x30, 0x08, 0xa0, 0x83, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x1a, 0x00, 0x00, 0x00,
    0x89, 0x00, 0x00, 0x00, 0x9e, 0x00, 0x00, 0x00,
    0x5d, 0x00, 0x00, 0x00, 0xc1, 0x00, 0x00, 0x00,
};

static const uint8_t n8x0_cal_bt_id[] = {
    MAEMO_CAL_HEADER('b', 't', '-', 'i', 'd', 0, 0, 0)
    0x0a, 0x00, 0x00, 0x00, 0xa3, 0x4b, 0xf6, 0x96,
    0xa8, 0xeb, 0xb2, 0x41, 0x00, 0x00, 0x00, 0x00,
    N8X0_BD_ADDR,
};

static void n8x0_nand_setup(struct n800_s *s)
{
    char *otp_region;
    DriveInfo *dinfo;

    s->nand = qdev_create(NULL, "onenand");
    qdev_prop_set_uint16(s->nand, "manufacturer_id", NAND_MFR_SAMSUNG);
    /* Either 0x40 or 0x48 are OK for the device ID */
    qdev_prop_set_uint16(s->nand, "device_id", 0x48);
    qdev_prop_set_uint16(s->nand, "version_id", 0);
    qdev_prop_set_int32(s->nand, "shift", 1);
    dinfo = drive_get(IF_MTD, 0, 0);
    if (dinfo && dinfo->bdrv) {
        qdev_prop_set_drive_nofail(s->nand, "drive", dinfo->bdrv);
    }
    qdev_init_nofail(s->nand);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->nand), 0,
                       qdev_get_gpio_in(s->mpu->gpio, N8X0_ONENAND_GPIO));
    omap_gpmc_attach(s->mpu->gpmc, N8X0_ONENAND_CS,
                     sysbus_mmio_get_region(SYS_BUS_DEVICE(s->nand), 0));
    otp_region = onenand_raw_otp(s->nand);

    memcpy(otp_region + 0x000, n8x0_cal_wlan_mac, sizeof(n8x0_cal_wlan_mac));
    memcpy(otp_region + 0x800, n8x0_cal_bt_id, sizeof(n8x0_cal_bt_id));
    /* XXX: in theory should also update the OOB for both pages */
}

static qemu_irq n8x0_system_powerdown;

static void n8x0_powerdown_req(Notifier *n, void *opaque)
{
    qemu_irq_raise(n8x0_system_powerdown);
}

static Notifier n8x0_system_powerdown_notifier = {
    .notify = n8x0_powerdown_req
};

static void n8x0_i2c_setup(struct n800_s *s)
{
    DeviceState *dev;
    qemu_irq tmp_irq = qdev_get_gpio_in(s->mpu->gpio, N8X0_TMP105_GPIO);
    i2c_bus *i2c = omap_i2c_bus(s->mpu->i2c[0]);

    /* Attach a menelaus PM chip */
    dev = i2c_create_slave(i2c, "twl92230", N8X0_MENELAUS_ADDR);
    qdev_connect_gpio_out(dev, 3,
                          qdev_get_gpio_in(s->mpu->ih[0],
                                           OMAP_INT_24XX_SYS_NIRQ));

    n8x0_system_powerdown = qdev_get_gpio_in(dev, 3);
    qemu_register_powerdown_notifier(&n8x0_system_powerdown_notifier);

    /* Attach a TMP105 PM chip (A0 wired to ground) */
    dev = i2c_create_slave(i2c, "tmp105", N8X0_TMP105_ADDR);
    qdev_connect_gpio_out(dev, 0, tmp_irq);
}

/* Touchscreen and keypad controller */
static MouseTransformInfo n800_pointercal = {
    .x = 800,
    .y = 480,
    .a = { 14560, -68, -3455208, -39, -9621, 35152972, 65536 },
};

static MouseTransformInfo n810_pointercal = {
    .x = 800,
    .y = 480,
    .a = { 15041, 148, -4731056, 171, -10238, 35933380, 65536 },
};

#define RETU_KEYCODE	61	/* F3 */

static void n800_key_event(void *opaque, int keycode)
{
    struct n800_s *s = (struct n800_s *) opaque;
    int code = s->keymap[keycode & 0x7f];

    if (code == -1) {
        if ((keycode & 0x7f) == RETU_KEYCODE)
            retu_key_event(s->retu, !(keycode & 0x80));
        return;
    }

    tsc2301_key_event(s->tsc, code, !(keycode & 0x80));
}

static const int n800_keys[16] = {
    -1,
    72,	/* Up */
    63,	/* Home (F5) */
    -1,
    75,	/* Left */
    28,	/* Enter */
    77,	/* Right */
    -1,
     1,	/* Cycle (ESC) */
    80,	/* Down */
    62,	/* Menu (F4) */
    -1,
    66,	/* Zoom- (F8) */
    64,	/* FullScreen (F6) */
    65,	/* Zoom+ (F7) */
    -1,
};

static void n800_tsc_kbd_setup(struct n800_s *s)
{
    int i;

    /* XXX: are the three pins inverted inside the chip between the
     * tsc and the cpu (N4111)?  */
    s->tsc = spi_create_device(omap_mcspi_bus(s->mpu->mcspi, 0), "tsc2301", 0);
    /* penirq NC */
    qdev_connect_gpio_out(s->tsc, 1, qdev_get_gpio_in(s->mpu->gpio,
                                                      N800_TSC_KP_IRQ_GPIO));
    qdev_connect_gpio_out(s->tsc, 2, qdev_get_gpio_in(s->mpu->gpio,
                                                      N800_TSC_TS_GPIO));

    for (i = 0; i < 0x80; i ++)
        s->keymap[i] = -1;
    for (i = 0; i < 0x10; i ++)
        if (n800_keys[i] >= 0)
            s->keymap[n800_keys[i]] = i;

    qemu_add_kbd_event_handler(n800_key_event, s);

    tsc2301_set_transform(s->tsc, &n800_pointercal);
}

static void n810_tsc_setup(struct n800_s *s)
{
    s->tsc = spi_create_device(omap_mcspi_bus(s->mpu->mcspi, 0), "tsc2005", 0);
    qdev_connect_gpio_out(s->tsc, 0, qdev_get_gpio_in(s->mpu->gpio,
                                                      N810_TSC_TS_GPIO));
    tsc2005_set_transform(s->tsc, &n810_pointercal, 400, 4000);
}

/* N810 Keyboard controller */
static void n810_key_event(void *opaque, int keycode)
{
    struct n800_s *s = (struct n800_s *) opaque;
    int code = s->keymap[keycode & 0x7f];

    if (code == -1) {
        if ((keycode & 0x7f) == RETU_KEYCODE)
            retu_key_event(s->retu, !(keycode & 0x80));
        return;
    }

    lm832x_key_event(s->kbd, code, !(keycode & 0x80));
}

#define M	0

static int n810_keys[0x80] = {
    [0x01] = 16,	/* Q */
    [0x02] = 37,	/* K */
    [0x03] = 24,	/* O */
    [0x04] = 25,	/* P */
    [0x05] = 14,	/* Backspace */
    [0x06] = 30,	/* A */
    [0x07] = 31,	/* S */
    [0x08] = 32,	/* D */
    [0x09] = 33,	/* F */
    [0x0a] = 34,	/* G */
    [0x0b] = 35,	/* H */
    [0x0c] = 36,	/* J */

    [0x11] = 17,	/* W */
    [0x12] = 62,	/* Menu (F4) */
    [0x13] = 38,	/* L */
    [0x14] = 40,	/* ' (Apostrophe) */
    [0x16] = 44,	/* Z */
    [0x17] = 45,	/* X */
    [0x18] = 46,	/* C */
    [0x19] = 47,	/* V */
    [0x1a] = 48,	/* B */
    [0x1b] = 49,	/* N */
    [0x1c] = 42,	/* Shift (Left shift) */
    [0x1f] = 65,	/* Zoom+ (F7) */

    [0x21] = 18,	/* E */
    [0x22] = 39,	/* ; (Semicolon) */
    [0x23] = 12,	/* - (Minus) */
    [0x24] = 13,	/* = (Equal) */
    [0x2b] = 56,	/* Fn (Left Alt) */
    [0x2c] = 50,	/* M */
    [0x2f] = 66,	/* Zoom- (F8) */

    [0x31] = 19,	/* R */
    [0x32] = 29 | M,	/* Right Ctrl */
    [0x34] = 57,	/* Space */
    [0x35] = 51,	/* , (Comma) */
    [0x37] = 72 | M,	/* Up */
    [0x3c] = 82 | M,	/* Compose (Insert) */
    [0x3f] = 64,	/* FullScreen (F6) */

    [0x41] = 20,	/* T */
    [0x44] = 52,	/* . (Dot) */
    [0x46] = 77 | M,	/* Right */
    [0x4f] = 63,	/* Home (F5) */
    [0x51] = 21,	/* Y */
    [0x53] = 80 | M,	/* Down */
    [0x55] = 28,	/* Enter */
    [0x5f] =  1,	/* Cycle (ESC) */

    [0x61] = 22,	/* U */
    [0x64] = 75 | M,	/* Left */

    [0x71] = 23,	/* I */
#if 0
    [0x75] = 28 | M,	/* KP Enter (KP Enter) */
#else
    [0x75] = 15,	/* KP Enter (Tab) */
#endif
};

#undef M

static void n810_kbd_setup(struct n800_s *s)
{
    qemu_irq kbd_irq = qdev_get_gpio_in(s->mpu->gpio, N810_KEYBOARD_GPIO);
    int i;

    for (i = 0; i < 0x80; i ++)
        s->keymap[i] = -1;
    for (i = 0; i < 0x80; i ++)
        if (n810_keys[i] > 0)
            s->keymap[n810_keys[i]] = i;

    qemu_add_kbd_event_handler(n810_key_event, s);

    /* Attach the LM8322 keyboard to the I2C bus,
     * should happen in n8x0_i2c_setup and s->kbd be initialised here.  */
    s->kbd = i2c_create_slave(omap_i2c_bus(s->mpu->i2c[0]),
                           "lm8323", N810_LM8323_ADDR);
    qdev_connect_gpio_out(s->kbd, 0, kbd_irq);
}

/* LCD MIPI DBI-C controller (URAL) */
struct mipid_s {
    SPIDevice spi;
    int resp[4];
    int param[4];
    int p;
    int pm;
    int cmd;

    int sleep;
    int booster;
    int te;
    int selfcheck;
    int partial;
    int normal;
    int vscr;
    int invert;
    int onoff;
    int gamma;
    uint32_t id;
    
    uint8_t n900;
    int cabc;
    int brightness;
    int ctrl;
};

static void mipid_reset(DeviceState *qdev)
{
    struct mipid_s *s = FROM_SPI_DEVICE(struct mipid_s,
                                        SPI_DEVICE_FROM_QDEV(qdev));

    s->pm = 0;
    s->cmd = 0;

    s->sleep = 1;
    s->booster = 0;
    s->selfcheck =
            (1 << 7) |	/* Register loading OK.  */
            (1 << 5) |	/* The chip is attached.  */
            (1 << 4);	/* Display glass still in one piece.  */
    s->te = 0;
    s->partial = 0;
    s->normal = 1;
    s->vscr = 0;
    s->invert = 0;
    s->onoff = 1;
    s->gamma = 0;
}

static uint32_t mipid_txrx(SPIDevice *spidev, uint32_t cmd, int len)
{
    struct mipid_s *s = FROM_SPI_DEVICE(struct mipid_s, spidev);
    uint8_t ret;

    if (s->n900 && len == 10) {
        cmd >>= 1;
        len--;
    }
    
    if (len > 9)
        hw_error("%s: FIXME: bad SPI word width %i\n", __FUNCTION__, len);

    if (s->p >= ARRAY_SIZE(s->resp))
        ret = 0;
    else
        ret = s->resp[s->p ++];
    if (s->pm --> 0)
        s->param[s->pm] = cmd;
    else
        s->cmd = cmd;

    switch (s->cmd) {
    case 0x00:	/* NOP */
        TRACE_MIPID("NOP");
        break;

    case 0x01:	/* SWRESET */
        TRACE_MIPID("SWRESET");
        mipid_reset(&s->spi.qdev);
        break;

    case 0x02:	/* BSTROFF */
        TRACE_MIPID("BSTROFF");
        s->booster = 0;
        break;
    case 0x03:	/* BSTRON */
        TRACE_MIPID("BSTRON");
        s->booster = 1;
        break;

    case 0x04:	/* RDDID */
        s->p = 0;
        s->resp[0] = (s->id >> 16) & 0xff;
        s->resp[1] = (s->id >>  8) & 0xff;
        s->resp[2] = (s->id >>  0) & 0xff;
        TRACE_MIPID("RDDID 0x%02x 0x%02x 0x%02x",
                    s->resp[0], s->resp[1], s->resp[2]);
        break;

    case 0x06:	/* RD_RED */
    case 0x07:	/* RD_GREEN */
        /* XXX the bootloader sometimes issues RD_BLUE meaning RDDID so
         * for the bootloader one needs to change this.  */
    case 0x08:	/* RD_BLUE */
        TRACE_MIPID("RD_RED/GREEN_BLUE 0x01");
        s->p = 0;
        /* TODO: return first pixel components */
        s->resp[0] = 0x01;
        break;

    case 0x09:	/* RDDST */
        s->p = 0;
        s->resp[0] = s->booster << 7;
        s->resp[1] = (5 << 4) | (s->partial << 2) |
                (s->sleep << 1) | s->normal;
        s->resp[2] = (s->vscr << 7) | (s->invert << 5) |
                (s->onoff << 2) | (s->te << 1) | (s->gamma >> 2);
        s->resp[3] = s->gamma << 6;
        TRACE_MIPID("RDDST 0x%02x 0x%02x 0x%02x 0x%02x",
                    s->resp[0], s->resp[1], s->resp[2], s->resp[3]);
        break;

    case 0x0a:	/* RDDPM */
        s->p = 0;
        s->resp[0] = (s->onoff << 2) | (s->normal << 3) | (s->sleep << 4) |
                (s->partial << 5) | (s->sleep << 6) | (s->booster << 7);
        TRACE_MIPID("RDDPM 0x%02x", s->resp[0]);
        break;
    case 0x0b:	/* RDDMADCTR */
        s->p = 0;
        s->resp[0] = 0;
        TRACE_MIPID("RDDMACTR 0x%02x", s->resp[0]);
        break;
    case 0x0c:	/* RDDCOLMOD */
        s->p = 0;
        s->resp[0] = 5;	/* 65K colours */
        TRACE_MIPID("RDDCOLMOD 0x%02x", s->resp[0]);
        break;
    case 0x0d:	/* RDDIM */
        s->p = 0;
        s->resp[0] = (s->invert << 5) | (s->vscr << 7) | s->gamma;
        TRACE_MIPID("RDDIM 0x%02x", s->resp[0]);
        break;
    case 0x0e:	/* RDDSM */
        s->p = 0;
        s->resp[0] = s->te << 7;
        TRACE_MIPID("RDDSM 0x%02x", s->resp[0]);
        break;
    case 0x0f:	/* RDDSDR */
        s->p = 0;
        s->resp[0] = s->selfcheck;
        TRACE_MIPID("RDDSDR 0x%02x", s->resp[0]);
        break;

    case 0x10:	/* SLPIN */
        TRACE_MIPID("SLPIN");
        s->sleep = 1;
        break;
    case 0x11:	/* SLPOUT */
        TRACE_MIPID("SLPOUT");
        s->sleep = 0;
        s->selfcheck ^= 1 << 6;	/* POFF self-diagnosis Ok */
        break;

    case 0x12:	/* PTLON */
        TRACE_MIPID("PTLON");
        s->partial = 1;
        s->normal = 0;
        s->vscr = 0;
        break;
    case 0x13:	/* NORON */
        TRACE_MIPID("NORON");
        s->partial = 0;
        s->normal = 1;
        s->vscr = 0;
        break;

    case 0x20:	/* INVOFF */
        TRACE_MIPID("INVOFF");
        s->invert = 0;
        break;
    case 0x21:	/* INVON */
        TRACE_MIPID("INVON");
        s->invert = 1;
        break;

    case 0x22:	/* APOFF */
    case 0x23:	/* APON */
        TRACE_MIPID("APON/OFF");
        goto bad_cmd;

    case 0x25:	/* WRCNTR */
        TRACE_MIPID("WRCNTR");
        if (s->pm < 0)
            s->pm = 1;
        goto bad_cmd;

    case 0x26:	/* GAMSET */
        if (!s->pm) {
            s->gamma = ffs(s->param[0] & 0xf) - 1;
            TRACE_MIPID("GAMSET 0x%02x", s->gamma);
        } else if (s->pm < 0) {
            s->pm = 1;
        }
        break;

    case 0x28:	/* DISPOFF */
        TRACE_MIPID("DISPOFF");
        s->onoff = 0;
        break;
    case 0x29:	/* DISPON */
        TRACE_MIPID("DISPON");
        s->onoff = 1;
        break;

    case 0x2a:	/* CASET */
    case 0x2b:	/* RASET */
    case 0x2c:	/* RAMWR */
    case 0x2d:	/* RGBSET */
    case 0x2e:	/* RAMRD */
    case 0x30:	/* PTLAR */
    case 0x33:	/* SCRLAR */
        goto bad_cmd;

    case 0x34:	/* TEOFF */
        TRACE_MIPID("TEOFF");
        s->te = 0;
        break;
    case 0x35:	/* TEON */
        if (!s->pm) {
            s->te = 1;
            TRACE_MIPID("TEON 0x%02x", s->param[0] & 0xff);
        } else if (s->pm < 0) {
            s->pm = 1;
        }
        break;

    case 0x36:	/* MADCTR */
        TRACE_MIPID("MADCTR");
        goto bad_cmd;

    case 0x37:	/* VSCSAD */
        TRACE_MIPID("VSCSAD");
        s->partial = 0;
        s->normal = 0;
        s->vscr = 1;
        break;

    case 0x38:	/* IDMOFF */
    case 0x39:	/* IDMON */
        TRACE_MIPID("IDMON/OFF");
        goto bad_cmd;
    case 0x3a:	/* COLMOD */
        if (!s->pm) {
            TRACE_MIPID("COLMOD 0x%02x", s->param[0] & 0xff);
        } else if (s->pm < 0) {
            s->pm = 1;
        }
        break;
    
    case 0x51: /* WRITE_BRIGHTNESS */
        if (s->n900) {
            if (!s->pm) {
                s->brightness = s->param[0] & 0xff;
                TRACE_MIPID("WRITE_BRIGHTNESS 0x%02x", s->brightness);
            } else if (s->pm < 0) {
                s->pm = 1;
            }
        } else {
            goto bad_cmd;
        }
        break;
    case 0x52: /* READ_BRIGHTNESS */
        if (s->n900) {
            s->p = 0;
            s->resp[0] = s->brightness;
            TRACE_MIPID("READ_BRIGHTNESS 0x%02x", s->resp[0]);
        } else {
            goto bad_cmd;
        }
        break;
    case 0x53: /* WRITE_CTRL */
        if (s->n900) {
            if (!s->pm) {
                s->ctrl = s->param[0] & 0xff;
                TRACE_MIPID("WRITE_CTRL 0x%02x", s->ctrl);
            } else if (s->pm < 0) {
                s->pm = 1;
            }
        } else {
            goto bad_cmd;
        }
        break;
    case 0x54: /* READ_CTRL */
        if (s->n900) {
            s->p = 0;
            s->resp[0] = s->ctrl;
            TRACE_MIPID("READ_CTRL 0x%02x", s->resp[0]);
        } else {
            goto bad_cmd;
        }
        break;
    case 0x55: /* WRITE_CABC */
        if (s->n900) {
            if (!s->pm) {
                s->cabc = s->param[0] & 0xff;
                TRACE_MIPID("WRITE_CABC 0x%02x", s->cabc);
            } else if (s->pm < 0) {
                s->pm = 1;
            }
        } else {
            goto bad_cmd;
        }
        break;
    case 0x56: /* READ_CABC */
        if (s->n900) {
            s->p = 0;
            s->resp[0] = s->cabc;
            TRACE_MIPID("READ_CABC 0x%02x", s->resp[0]);
        } else {
            goto bad_cmd;
        }
        break;
            
    case 0xb0:	/* CLKINT / DISCTL */
    case 0xb1:	/* CLKEXT */
        if (!s->pm) {
            TRACE_MIPID("CLKINT/EXT");
        } else if (s->pm < 0) {
            s->pm = 2;
        }
        break;

    case 0xb4:	/* FRMSEL */
        TRACE_MIPID("FRMSEL");
        break;

    case 0xb5:	/* FRM8SEL */
    case 0xb6:	/* TMPRNG / INIESC */
    case 0xb7:	/* TMPHIS / NOP2 */
    case 0xb8:	/* TMPREAD / MADCTL */
    case 0xba:	/* DISTCTR */
    case 0xbb:	/* EPVOL */
        goto bad_cmd;

    case 0xbd:	/* Unknown */
        s->p = 0;
        s->resp[0] = 0;
        s->resp[1] = 1;
        TRACE_MIPID("??? 0x%02x 0x%02x", s->resp[0], s->resp[1]);
        break;

    case 0xc2:	/* IFMOD */
        if (!s->pm) {
            TRACE_MIPID("IFMOD");
        } else if (s->pm < 0) {
            s->pm = (s->n900) ? 3 : 2;
        }
        break;

    case 0xc6:	/* PWRCTL */
    case 0xc7:	/* PPWRCTL */
    case 0xd0:	/* EPWROUT */
    case 0xd1:	/* EPWRIN */
    case 0xd4:	/* RDEV */
    case 0xd5:	/* RDRR */
        goto bad_cmd;

    case 0xda:	/* RDID1 */
        s->p = 0;
        s->resp[0] = (s->id >> 16) & 0xff;
        TRACE_MIPID("RDID1 0x%02x", s->resp[0]);
        break;
    case 0xdb:	/* RDID2 */
        s->p = 0;
        s->resp[0] = (s->id >>  8) & 0xff;
        TRACE_MIPID("RDID2 0x%02x", s->resp[0]);
        break;
    case 0xdc:	/* RDID3 */
        s->p = 0;
        s->resp[0] = (s->id >>  0) & 0xff;
        TRACE_MIPID("RDID3 0x%02x", s->resp[0]);
        break;

    default:
    bad_cmd:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: unknown command %02x\n", __func__, s->cmd);
        break;
    }

    return ret;
}

static int mipid_init(SPIDevice *spidev)
{
    return 0;
}

static Property mipid_properties[] = {
    DEFINE_PROP_UINT32("id", struct mipid_s, id, 0),
    DEFINE_PROP_UINT8("n900", struct mipid_s, n900, 0),
    DEFINE_PROP_END_OF_LIST()
};

static void mipid_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SPIDeviceClass *k = SPI_DEVICE_CLASS(klass);

    k->init = mipid_init;
    k->txrx = mipid_txrx;
    dc->reset = mipid_reset;
    dc->props = mipid_properties;
}

static TypeInfo mipid_info = {
    .name = "lcd_mipid",
    .parent = TYPE_SPI_DEVICE,
    .instance_size = sizeof(struct mipid_s),
    .class_init = mipid_class_init,
};

static void n8x0_spi_setup(struct n800_s *s)
{
    s->mipid = spi_create_device_noinit(omap_mcspi_bus(s->mpu->mcspi, 0),
                                        "lcd_mipid", 1);
    qdev_prop_set_uint32(s->mipid, "id", 0x838f03);
    qdev_init_nofail(s->mipid);
}

/* This task is normally performed by the bootloader.  If we're loading
 * a kernel directly, we need to enable the Blizzard ourselves.  */
static void n800_dss_init(struct rfbi_chip_s *chip)
{
    uint8_t *fb_blank;

    chip->write(chip->opaque, 0, 0x2a);		/* LCD Width register */
    chip->write(chip->opaque, 1, 0x64);
    chip->write(chip->opaque, 0, 0x2c);		/* LCD HNDP register */
    chip->write(chip->opaque, 1, 0x1e);
    chip->write(chip->opaque, 0, 0x2e);		/* LCD Height 0 register */
    chip->write(chip->opaque, 1, 0xe0);
    chip->write(chip->opaque, 0, 0x30);		/* LCD Height 1 register */
    chip->write(chip->opaque, 1, 0x01);
    chip->write(chip->opaque, 0, 0x32);		/* LCD VNDP register */
    chip->write(chip->opaque, 1, 0x06);
    chip->write(chip->opaque, 0, 0x68);		/* Display Mode register */
    chip->write(chip->opaque, 1, 1);		/* Enable bit */

    chip->write(chip->opaque, 0, 0x6c);	
    chip->write(chip->opaque, 1, 0x00);		/* Input X Start Position */
    chip->write(chip->opaque, 1, 0x00);		/* Input X Start Position */
    chip->write(chip->opaque, 1, 0x00);		/* Input Y Start Position */
    chip->write(chip->opaque, 1, 0x00);		/* Input Y Start Position */
    chip->write(chip->opaque, 1, 0x1f);		/* Input X End Position */
    chip->write(chip->opaque, 1, 0x03);		/* Input X End Position */
    chip->write(chip->opaque, 1, 0xdf);		/* Input Y End Position */
    chip->write(chip->opaque, 1, 0x01);		/* Input Y End Position */
    chip->write(chip->opaque, 1, 0x00);		/* Output X Start Position */
    chip->write(chip->opaque, 1, 0x00);		/* Output X Start Position */
    chip->write(chip->opaque, 1, 0x00);		/* Output Y Start Position */
    chip->write(chip->opaque, 1, 0x00);		/* Output Y Start Position */
    chip->write(chip->opaque, 1, 0x1f);		/* Output X End Position */
    chip->write(chip->opaque, 1, 0x03);		/* Output X End Position */
    chip->write(chip->opaque, 1, 0xdf);		/* Output Y End Position */
    chip->write(chip->opaque, 1, 0x01);		/* Output Y End Position */
    chip->write(chip->opaque, 1, 0x01);		/* Input Data Format */
    chip->write(chip->opaque, 1, 0x01);		/* Data Source Select */

    fb_blank = memset(g_malloc(800 * 480 * 2), 0xff, 800 * 480 * 2);
    /* Display Memory Data Port */
    chip->block(chip->opaque, 1, fb_blank, 800 * 480 * 2, 800);
    g_free(fb_blank);
}

static void n8x0_dss_setup(struct n800_s *s)
{
    s->blizzard.opaque = s1d13745_init(NULL);
    s->blizzard.block = s1d13745_write_block;
    s->blizzard.write = s1d13745_write;
    s->blizzard.read = s1d13745_read;

    omap_rfbi_attach(s->mpu->dss, 0, &s->blizzard);
}

static void n8x0_cbus_setup(struct n800_s *s)
{
    qemu_irq dat_out = qdev_get_gpio_in(s->mpu->gpio, N8X0_CBUS_DAT_GPIO);
    qemu_irq retu_irq = qdev_get_gpio_in(s->mpu->gpio, N8X0_RETU_GPIO);
    qemu_irq tahvo_irq = qdev_get_gpio_in(s->mpu->gpio, N8X0_TAHVO_GPIO);

    CBus *cbus = cbus_init(dat_out);

    qdev_connect_gpio_out(s->mpu->gpio, N8X0_CBUS_CLK_GPIO, cbus->clk);
    qdev_connect_gpio_out(s->mpu->gpio, N8X0_CBUS_DAT_GPIO, cbus->dat);
    qdev_connect_gpio_out(s->mpu->gpio, N8X0_CBUS_SEL_GPIO, cbus->sel);

    cbus_attach(cbus, s->retu = retu_init(retu_irq, 1));
    cbus_attach(cbus, s->tahvo = tahvo_init(tahvo_irq, 1));
}

static void n8x0_uart_setup(struct n800_s *s)
{
    CharDriverState *radio = uart_hci_init(
                    qdev_get_gpio_in(s->mpu->gpio, N8X0_BT_HOST_WKUP_GPIO));

    qdev_connect_gpio_out(s->mpu->gpio, N8X0_BT_RESET_GPIO,
                    csrhci_pins_get(radio)[csrhci_pin_reset]);
    qdev_connect_gpio_out(s->mpu->gpio, N8X0_BT_WKUP_GPIO,
                    csrhci_pins_get(radio)[csrhci_pin_wakeup]);

    omap_uart_attach(s->mpu->uart[BT_UART], radio, "bt-uart");
}

static void n8x0_usb_setup(struct n800_s *s)
{
    SysBusDevice *dev;
    s->usb = qdev_create(NULL, "tusb6010");
    dev = SYS_BUS_DEVICE(s->usb);
    qdev_init_nofail(s->usb);
    sysbus_connect_irq(dev, 0,
                       qdev_get_gpio_in(s->mpu->gpio, N8X0_TUSB_INT_GPIO));
    /* Using the NOR interface */
    omap_gpmc_attach(s->mpu->gpmc, N8X0_USB_ASYNC_CS,
                     sysbus_mmio_get_region(dev, 0));
    omap_gpmc_attach(s->mpu->gpmc, N8X0_USB_SYNC_CS,
                     sysbus_mmio_get_region(dev, 1));
    qdev_connect_gpio_out(s->mpu->gpio, N8X0_TUSB_ENABLE_GPIO,
                          qdev_get_gpio_in(s->usb, 0)); /* tusb_pwr */
}

/* Setup done before the main bootloader starts by some early setup code
 * - used when we want to run the main bootloader in emulation.  This
 * isn't documented.  */
static uint32_t n800_pinout[104] = {
    0x080f00d8, 0x00d40808, 0x03080808, 0x080800d0,
    0x00dc0808, 0x0b0f0f00, 0x080800b4, 0x00c00808,
    0x08080808, 0x180800c4, 0x00b80000, 0x08080808,
    0x080800bc, 0x00cc0808, 0x08081818, 0x18180128,
    0x01241800, 0x18181818, 0x000000f0, 0x01300000,
    0x00001b0b, 0x1b0f0138, 0x00e0181b, 0x1b031b0b,
    0x180f0078, 0x00740018, 0x0f0f0f1a, 0x00000080,
    0x007c0000, 0x00000000, 0x00000088, 0x00840000,
    0x00000000, 0x00000094, 0x00980300, 0x0f180003,
    0x0000008c, 0x00900f0f, 0x0f0f1b00, 0x0f00009c,
    0x01140000, 0x1b1b0f18, 0x0818013c, 0x01400008,
    0x00001818, 0x000b0110, 0x010c1800, 0x0b030b0f,
    0x181800f4, 0x00f81818, 0x00000018, 0x000000fc,
    0x00401808, 0x00000000, 0x0f1b0030, 0x003c0008,
    0x00000000, 0x00000038, 0x00340000, 0x00000000,
    0x1a080070, 0x00641a1a, 0x08080808, 0x08080060,
    0x005c0808, 0x08080808, 0x08080058, 0x00540808,
    0x08080808, 0x0808006c, 0x00680808, 0x08080808,
    0x000000a8, 0x00b00000, 0x08080808, 0x000000a0,
    0x00a40000, 0x00000000, 0x08ff0050, 0x004c0808,
    0xffffffff, 0xffff0048, 0x0044ffff, 0xffffffff,
    0x000000ac, 0x01040800, 0x08080b0f, 0x18180100,
    0x01081818, 0x0b0b1808, 0x1a0300e4, 0x012c0b1a,
    0x02020018, 0x0b000134, 0x011c0800, 0x0b1b1b00,
    0x0f0000c8, 0x00ec181b, 0x000f0f02, 0x00180118,
    0x01200000, 0x0f0b1b1b, 0x0f0200e8, 0x0000020b,
};

static void n800_setup_nolo_tags(void *sram_base)
{
    int i;
    uint32_t *p = sram_base + 0x8000;
    uint32_t *v = sram_base + 0xa000;

    memset(p, 0, 0x3000);

    strcpy((void *) (p + 0), "QEMU N800");

    strcpy((void *) (p + 8), "F5");

    stl_raw(p + 10, 0x04f70000);
    strcpy((void *) (p + 9), "RX-34");

    /* RAM size in MB? */
    stl_raw(p + 12, 0x80);

    /* Pointer to the list of tags */
    stl_raw(p + 13, OMAP2_SRAM_BASE + 0x9000);

    /* The NOLO tags start here */
    p = sram_base + 0x9000;
#define ADD_TAG(tag, len)				\
    stw_raw((uint16_t *) p + 0, tag);			\
    stw_raw((uint16_t *) p + 1, len); p ++;		\
    stl_raw(p ++, OMAP2_SRAM_BASE | (((void *) v - sram_base) & 0xffff));

    /* OMAP STI console? Pin out settings? */
    ADD_TAG(0x6e01, 414);
    for (i = 0; i < ARRAY_SIZE(n800_pinout); i ++)
        stl_raw(v ++, n800_pinout[i]);

    /* Kernel memsize? */
    ADD_TAG(0x6e05, 1);
    stl_raw(v ++, 2);

    /* NOLO serial console */
    ADD_TAG(0x6e02, 4);
    stl_raw(v ++, XLDR_LL_UART);	/* UART number (1 - 3) */

#if 0
    /* CBUS settings (Retu/AVilma) */
    ADD_TAG(0x6e03, 6);
    stw_raw((uint16_t *) v + 0, 65);	/* CBUS GPIO0 */
    stw_raw((uint16_t *) v + 1, 66);	/* CBUS GPIO1 */
    stw_raw((uint16_t *) v + 2, 64);	/* CBUS GPIO2 */
    v += 2;
#endif

    /* Nokia ASIC BB5 (Retu/Tahvo) */
    ADD_TAG(0x6e0a, 4);
    stw_raw((uint16_t *) v + 0, 111);	/* "Retu" interrupt GPIO */
    stw_raw((uint16_t *) v + 1, 108);	/* "Tahvo" interrupt GPIO */
    v ++;

    /* LCD console? */
    ADD_TAG(0x6e04, 4);
    stw_raw((uint16_t *) v + 0, 30);	/* ??? */
    stw_raw((uint16_t *) v + 1, 24);	/* ??? */
    v ++;

#if 0
    /* LCD settings */
    ADD_TAG(0x6e06, 2);
    stw_raw((uint16_t *) (v ++), 15);	/* ??? */
#endif

    /* I^2C (Menelaus) */
    ADD_TAG(0x6e07, 4);
    stl_raw(v ++, 0x00720000);		/* ??? */

    /* Unknown */
    ADD_TAG(0x6e0b, 6);
    stw_raw((uint16_t *) v + 0, 94);	/* ??? */
    stw_raw((uint16_t *) v + 1, 23);	/* ??? */
    stw_raw((uint16_t *) v + 2, 0);	/* ??? */
    v += 2;

    /* OMAP gpio switch info */
    ADD_TAG(0x6e0c, 80);
    strcpy((void *) v, "bat_cover");	v += 3;
    stw_raw((uint16_t *) v + 0, 110);	/* GPIO num ??? */
    stw_raw((uint16_t *) v + 1, 1);	/* GPIO num ??? */
    v += 2;
    strcpy((void *) v, "cam_act");	v += 3;
    stw_raw((uint16_t *) v + 0, 95);	/* GPIO num ??? */
    stw_raw((uint16_t *) v + 1, 32);	/* GPIO num ??? */
    v += 2;
    strcpy((void *) v, "cam_turn");	v += 3;
    stw_raw((uint16_t *) v + 0, 12);	/* GPIO num ??? */
    stw_raw((uint16_t *) v + 1, 33);	/* GPIO num ??? */
    v += 2;
    strcpy((void *) v, "headphone");	v += 3;
    stw_raw((uint16_t *) v + 0, 107);	/* GPIO num ??? */
    stw_raw((uint16_t *) v + 1, 17);	/* GPIO num ??? */
    v += 2;

    /* Bluetooth */
    ADD_TAG(0x6e0e, 12);
    stl_raw(v ++, 0x5c623d01);		/* ??? */
    stl_raw(v ++, 0x00000201);		/* ??? */
    stl_raw(v ++, 0x00000000);		/* ??? */

    /* CX3110x WLAN settings */
    ADD_TAG(0x6e0f, 8);
    stl_raw(v ++, 0x00610025);		/* ??? */
    stl_raw(v ++, 0xffff0057);		/* ??? */

    /* MMC host settings */
    ADD_TAG(0x6e10, 12);
    stl_raw(v ++, 0xffff000f);		/* ??? */
    stl_raw(v ++, 0xffffffff);		/* ??? */
    stl_raw(v ++, 0x00000060);		/* ??? */

    /* OneNAND chip select */
    ADD_TAG(0x6e11, 10);
    stl_raw(v ++, 0x00000401);		/* ??? */
    stl_raw(v ++, 0x0002003a);		/* ??? */
    stl_raw(v ++, 0x00000002);		/* ??? */

    /* TEA5761 sensor settings */
    ADD_TAG(0x6e12, 2);
    stl_raw(v ++, 93);			/* GPIO num ??? */

#if 0
    /* Unknown tag */
    ADD_TAG(6e09, 0);

    /* Kernel UART / console */
    ADD_TAG(6e12, 0);
#endif

    /* End of the list */
    stl_raw(p ++, 0x00000000);
    stl_raw(p ++, 0x00000000);
}

/* This task is normally performed by the bootloader.  If we're loading
 * a kernel directly, we need to set up GPMC mappings ourselves.  */
static void n800_gpmc_init(struct n800_s *s)
{
    uint32_t config7 =
            (0xf << 8) |	/* MASKADDRESS */
            (1 << 6) |		/* CSVALID */
            (4 << 0);		/* BASEADDRESS */

    cpu_physical_memory_write(0x6800a078,		/* GPMC_CONFIG7_0 */
                              &config7, sizeof(config7));
}

/* Setup sequence done by the bootloader */
static void n8x0_boot_init(void *opaque)
{
    struct n800_s *s = (struct n800_s *) opaque;
    uint32_t buf;

    /* PRCM setup */
#define omap_writel(addr, val)	\
    buf = (val);			\
    cpu_physical_memory_write(addr, &buf, sizeof(buf))

    omap_writel(0x48008060, 0x41);		/* PRCM_CLKSRC_CTRL */
    omap_writel(0x48008070, 1);			/* PRCM_CLKOUT_CTRL */
    omap_writel(0x48008078, 0);			/* PRCM_CLKEMUL_CTRL */
    omap_writel(0x48008090, 0);			/* PRCM_VOLTSETUP */
    omap_writel(0x48008094, 0);			/* PRCM_CLKSSETUP */
    omap_writel(0x48008098, 0);			/* PRCM_POLCTRL */
    omap_writel(0x48008140, 2);			/* CM_CLKSEL_MPU */
    omap_writel(0x48008148, 0);			/* CM_CLKSTCTRL_MPU */
    omap_writel(0x48008158, 1);			/* RM_RSTST_MPU */
    omap_writel(0x480081c8, 0x15);		/* PM_WKDEP_MPU */
    omap_writel(0x480081d4, 0x1d4);		/* PM_EVGENCTRL_MPU */
    omap_writel(0x480081d8, 0);			/* PM_EVEGENONTIM_MPU */
    omap_writel(0x480081dc, 0);			/* PM_EVEGENOFFTIM_MPU */
    omap_writel(0x480081e0, 0xc);		/* PM_PWSTCTRL_MPU */
    omap_writel(0x48008200, 0x047e7ff7);	/* CM_FCLKEN1_CORE */
    omap_writel(0x48008204, 0x00000004);	/* CM_FCLKEN2_CORE */
    omap_writel(0x48008210, 0x047e7ff1);	/* CM_ICLKEN1_CORE */
    omap_writel(0x48008214, 0x00000004);	/* CM_ICLKEN2_CORE */
    omap_writel(0x4800821c, 0x00000000);	/* CM_ICLKEN4_CORE */
    omap_writel(0x48008230, 0);			/* CM_AUTOIDLE1_CORE */
    omap_writel(0x48008234, 0);			/* CM_AUTOIDLE2_CORE */
    omap_writel(0x48008238, 7);			/* CM_AUTOIDLE3_CORE */
    omap_writel(0x4800823c, 0);			/* CM_AUTOIDLE4_CORE */
    omap_writel(0x48008240, 0x04360626);	/* CM_CLKSEL1_CORE */
    omap_writel(0x48008244, 0x00000014);	/* CM_CLKSEL2_CORE */
    omap_writel(0x48008248, 0);			/* CM_CLKSTCTRL_CORE */
    omap_writel(0x48008300, 0x00000000);	/* CM_FCLKEN_GFX */
    omap_writel(0x48008310, 0x00000000);	/* CM_ICLKEN_GFX */
    omap_writel(0x48008340, 0x00000001);	/* CM_CLKSEL_GFX */
    omap_writel(0x48008400, 0x00000004);	/* CM_FCLKEN_WKUP */
    omap_writel(0x48008410, 0x00000004);	/* CM_ICLKEN_WKUP */
    omap_writel(0x48008440, 0x00000000);	/* CM_CLKSEL_WKUP */
    omap_writel(0x48008500, 0x000000cf);	/* CM_CLKEN_PLL */
    omap_writel(0x48008530, 0x0000000c);	/* CM_AUTOIDLE_PLL */
    omap_writel(0x48008540,			/* CM_CLKSEL1_PLL */
                    (0x78 << 12) | (6 << 8));
    omap_writel(0x48008544, 2);			/* CM_CLKSEL2_PLL */

    /* GPMC setup */
    n800_gpmc_init(s);

    /* Video setup */
    n800_dss_init(&s->blizzard);

    /* CPU setup */
    s->mpu->cpu->env.GE = 0x5;

    /* If the machine has a slided keyboard, open it */
    if (s->kbd)
        qemu_irq_raise(qdev_get_gpio_in(s->mpu->gpio, N810_SLIDE_GPIO));
}

#define OMAP_TAG_NOKIA_BT	0x4e01
#define OMAP_TAG_WLAN_CX3110X	0x4e02
#define OMAP_TAG_CBUS		0x4e03
#define OMAP_TAG_EM_ASIC_BB5	0x4e04

static struct omap_gpiosw_info_s {
    const char *name;
    int line;
    int type;
} n800_gpiosw_info[] = {
    {
        "bat_cover", N800_BAT_COVER_GPIO,
        OMAP_GPIOSW_TYPE_COVER | OMAP_GPIOSW_INVERTED,
    }, {
        "cam_act", N800_CAM_ACT_GPIO,
        OMAP_GPIOSW_TYPE_ACTIVITY,
    }, {
        "cam_turn", N800_CAM_TURN_GPIO,
        OMAP_GPIOSW_TYPE_ACTIVITY | OMAP_GPIOSW_INVERTED,
    }, {
        "headphone", N8X0_HEADPHONE_GPIO,
        OMAP_GPIOSW_TYPE_CONNECTION | OMAP_GPIOSW_INVERTED,
    },
    { NULL }
}, n810_gpiosw_info[] = {
    {
        "gps_reset", N810_GPS_RESET_GPIO,
        OMAP_GPIOSW_TYPE_ACTIVITY | OMAP_GPIOSW_OUTPUT,
    }, {
        "gps_wakeup", N810_GPS_WAKEUP_GPIO,
        OMAP_GPIOSW_TYPE_ACTIVITY | OMAP_GPIOSW_OUTPUT,
    }, {
        "headphone", N8X0_HEADPHONE_GPIO,
        OMAP_GPIOSW_TYPE_CONNECTION | OMAP_GPIOSW_INVERTED,
    }, {
        "kb_lock", N810_KB_LOCK_GPIO,
        OMAP_GPIOSW_TYPE_COVER | OMAP_GPIOSW_INVERTED,
    }, {
        "sleepx_led", N810_SLEEPX_LED_GPIO,
        OMAP_GPIOSW_TYPE_ACTIVITY | OMAP_GPIOSW_INVERTED | OMAP_GPIOSW_OUTPUT,
    }, {
        "slide", N810_SLIDE_GPIO,
        OMAP_GPIOSW_TYPE_COVER | OMAP_GPIOSW_INVERTED,
    },
    { NULL }
};

static struct omap_partition_info_s {
    uint32_t offset;
    uint32_t size;
    int mask;
    const char *name;
} n800_part_info[] = {
    { 0x00000000, 0x00020000, 0x3, "bootloader" },
    { 0x00020000, 0x00060000, 0x0, "config" },
    { 0x00080000, 0x00200000, 0x0, "kernel" },
    { 0x00280000, 0x00200000, 0x3, "initfs" },
    { 0x00480000, 0x0fb80000, 0x3, "rootfs" },

    { 0, 0, 0, NULL }
}, n810_part_info[] = {
    { 0x00000000, 0x00020000, 0x3, "bootloader" },
    { 0x00020000, 0x00060000, 0x0, "config" },
    { 0x00080000, 0x00220000, 0x0, "kernel" },
    { 0x002a0000, 0x00400000, 0x0, "initfs" },
    { 0x006a0000, 0x0f960000, 0x0, "rootfs" },

    { 0, 0, 0, NULL }
};

static bdaddr_t n8x0_bd_addr = {{ N8X0_BD_ADDR }};

static int n8x0_atag_setup(void *p, int model)
{
    uint8_t *b;
    uint16_t *w;
    uint32_t *l;
    struct omap_gpiosw_info_s *gpiosw;
    struct omap_partition_info_s *partition;
    const char *tag;

    w = p;

    stw_raw(w ++, OMAP_TAG_UART);		/* u16 tag */
    stw_raw(w ++, 4);				/* u16 len */
    stw_raw(w ++, (1 << 2) | (1 << 1) | (1 << 0)); /* uint enabled_uarts */
    w ++;

#if 0
    stw_raw(w ++, OMAP_TAG_SERIAL_CONSOLE);	/* u16 tag */
    stw_raw(w ++, 4);				/* u16 len */
    stw_raw(w ++, XLDR_LL_UART + 1);		/* u8 console_uart */
    stw_raw(w ++, 115200);			/* u32 console_speed */
#endif

    stw_raw(w ++, OMAP_TAG_LCD);		/* u16 tag */
    stw_raw(w ++, 36);				/* u16 len */
    strcpy((void *) w, "QEMU LCD panel");	/* char panel_name[16] */
    w += 8;
    strcpy((void *) w, "blizzard");		/* char ctrl_name[16] */
    w += 8;
    stw_raw(w ++, N810_BLIZZARD_RESET_GPIO);	/* TODO: n800 s16 nreset_gpio */
    stw_raw(w ++, 24);				/* u8 data_lines */

    stw_raw(w ++, OMAP_TAG_CBUS);		/* u16 tag */
    stw_raw(w ++, 8);				/* u16 len */
    stw_raw(w ++, N8X0_CBUS_CLK_GPIO);		/* s16 clk_gpio */
    stw_raw(w ++, N8X0_CBUS_DAT_GPIO);		/* s16 dat_gpio */
    stw_raw(w ++, N8X0_CBUS_SEL_GPIO);		/* s16 sel_gpio */
    w ++;

    stw_raw(w ++, OMAP_TAG_EM_ASIC_BB5);	/* u16 tag */
    stw_raw(w ++, 4);				/* u16 len */
    stw_raw(w ++, N8X0_RETU_GPIO);		/* s16 retu_irq_gpio */
    stw_raw(w ++, N8X0_TAHVO_GPIO);		/* s16 tahvo_irq_gpio */

    gpiosw = (model == 810) ? n810_gpiosw_info : n800_gpiosw_info;
    for (; gpiosw->name; gpiosw ++) {
        stw_raw(w ++, OMAP_TAG_GPIO_SWITCH);	/* u16 tag */
        stw_raw(w ++, 20);			/* u16 len */
        strcpy((void *) w, gpiosw->name);	/* char name[12] */
        w += 6;
        stw_raw(w ++, gpiosw->line);		/* u16 gpio */
        stw_raw(w ++, gpiosw->type);
        stw_raw(w ++, 0);
        stw_raw(w ++, 0);
    }

    stw_raw(w ++, OMAP_TAG_NOKIA_BT);		/* u16 tag */
    stw_raw(w ++, 12);				/* u16 len */
    b = (void *) w;
    stb_raw(b ++, 0x01);			/* u8 chip_type	(CSR) */
    stb_raw(b ++, N8X0_BT_WKUP_GPIO);		/* u8 bt_wakeup_gpio */
    stb_raw(b ++, N8X0_BT_HOST_WKUP_GPIO);	/* u8 host_wakeup_gpio */
    stb_raw(b ++, N8X0_BT_RESET_GPIO);		/* u8 reset_gpio */
    stb_raw(b ++, BT_UART + 1);			/* u8 bt_uart */
    memcpy(b, &n8x0_bd_addr, 6);		/* u8 bd_addr[6] */
    b += 6;
    stb_raw(b ++, 0x02);			/* u8 bt_sysclk (38.4) */
    w = (void *) b;

    stw_raw(w ++, OMAP_TAG_WLAN_CX3110X);	/* u16 tag */
    stw_raw(w ++, 8);				/* u16 len */
    stw_raw(w ++, 0x25);			/* u8 chip_type */
    stw_raw(w ++, N8X0_WLAN_PWR_GPIO);		/* s16 power_gpio */
    stw_raw(w ++, N8X0_WLAN_IRQ_GPIO);		/* s16 irq_gpio */
    stw_raw(w ++, -1);				/* s16 spi_cs_gpio */

    stw_raw(w ++, OMAP_TAG_MMC);		/* u16 tag */
    stw_raw(w ++, 16);				/* u16 len */
    if (model == 810) {
        stw_raw(w ++, 0x23f);			/* unsigned flags */
        stw_raw(w ++, -1);			/* s16 power_pin */
        stw_raw(w ++, -1);			/* s16 switch_pin */
        stw_raw(w ++, -1);			/* s16 wp_pin */
        stw_raw(w ++, 0x240);			/* unsigned flags */
        stw_raw(w ++, 0xc000);			/* s16 power_pin */
        stw_raw(w ++, 0x0248);			/* s16 switch_pin */
        stw_raw(w ++, 0xc000);			/* s16 wp_pin */
    } else {
        stw_raw(w ++, 0xf);			/* unsigned flags */
        stw_raw(w ++, -1);			/* s16 power_pin */
        stw_raw(w ++, -1);			/* s16 switch_pin */
        stw_raw(w ++, -1);			/* s16 wp_pin */
        stw_raw(w ++, 0);			/* unsigned flags */
        stw_raw(w ++, 0);			/* s16 power_pin */
        stw_raw(w ++, 0);			/* s16 switch_pin */
        stw_raw(w ++, 0);			/* s16 wp_pin */
    }

    stw_raw(w ++, OMAP_TAG_TEA5761);		/* u16 tag */
    stw_raw(w ++, 4);				/* u16 len */
    stw_raw(w ++, N8X0_TEA5761_CS_GPIO);	/* u16 enable_gpio */
    w ++;

    partition = (model == 810) ? n810_part_info : n800_part_info;
    for (; partition->name; partition ++) {
        stw_raw(w ++, OMAP_TAG_PARTITION);	/* u16 tag */
        stw_raw(w ++, 28);			/* u16 len */
        strcpy((void *) w, partition->name);	/* char name[16] */
        l = (void *) (w + 8);
        stl_raw(l ++, partition->size);		/* unsigned int size */
        stl_raw(l ++, partition->offset);	/* unsigned int offset */
        stl_raw(l ++, partition->mask);		/* unsigned int mask_flags */
        w = (void *) l;
    }

    stw_raw(w ++, OMAP_TAG_BOOT_REASON);	/* u16 tag */
    stw_raw(w ++, 12);				/* u16 len */
#if 0
    strcpy((void *) w, "por");			/* char reason_str[12] */
    strcpy((void *) w, "charger");		/* char reason_str[12] */
    strcpy((void *) w, "32wd_to");		/* char reason_str[12] */
    strcpy((void *) w, "sw_rst");		/* char reason_str[12] */
    strcpy((void *) w, "mbus");			/* char reason_str[12] */
    strcpy((void *) w, "unknown");		/* char reason_str[12] */
    strcpy((void *) w, "swdg_to");		/* char reason_str[12] */
    strcpy((void *) w, "sec_vio");		/* char reason_str[12] */
    strcpy((void *) w, "pwr_key");		/* char reason_str[12] */
    strcpy((void *) w, "rtc_alarm");		/* char reason_str[12] */
#else
    strcpy((void *) w, "pwr_key");		/* char reason_str[12] */
#endif
    w += 6;

    tag = (model == 810) ? "RX-44" : "RX-34";
    stw_raw(w ++, OMAP_TAG_VERSION_STR);	/* u16 tag */
    stw_raw(w ++, 24);				/* u16 len */
    strcpy((void *) w, "product");		/* char component[12] */
    w += 6;
    strcpy((void *) w, tag);			/* char version[12] */
    w += 6;

    stw_raw(w ++, OMAP_TAG_VERSION_STR);	/* u16 tag */
    stw_raw(w ++, 24);				/* u16 len */
    strcpy((void *) w, "hw-build");		/* char component[12] */
    w += 6;
    strcpy((void *) w, "QEMU");		/* char version[12] */
    w += 6;

    tag = (model == 810) ? "1.1.10-qemu" : "1.1.6-qemu";
    stw_raw(w ++, OMAP_TAG_VERSION_STR);	/* u16 tag */
    stw_raw(w ++, 24);				/* u16 len */
    strcpy((void *) w, "nolo");			/* char component[12] */
    w += 6;
    strcpy((void *) w, tag);			/* char version[12] */
    w += 6;

    return (void *) w - p;
}

static int n800_atag_setup(const struct arm_boot_info *info, void *p)
{
    return n8x0_atag_setup(p, 800);
}

static int n810_atag_setup(const struct arm_boot_info *info, void *p)
{
    return n8x0_atag_setup(p, 810);
}

static void n8x0_init(QEMUMachineInitArgs *args,
                      struct arm_boot_info *binfo, int model)
{
    MemoryRegion *sysmem = get_system_memory();
    struct n800_s *s = (struct n800_s *) g_malloc0(sizeof(*s));
    int sdram_size = binfo->ram_size;

    s->mpu = omap2420_mpu_init(sysmem, sdram_size, args->cpu_model);

    /* Setup peripherals
     *
     * Believed external peripherals layout in the N810:
     * (spi bus 1)
     *   tsc2005
     *   lcd_mipid
     * (spi bus 2)
     *   Conexant cx3110x (WLAN)
     *   optional: pc2400m (WiMAX)
     * (i2c bus 0)
     *   TLV320AIC33 (audio codec)
     *   TCM825x (camera by Toshiba)
     *   lp5521 (clever LEDs)
     *   tsl2563 (light sensor, hwmon, model 7, rev. 0)
     *   lm8323 (keypad, manf 00, rev 04)
     * (i2c bus 1)
     *   tmp105 (temperature sensor, hwmon)
     *   menelaus (pm)
     * (somewhere on i2c - maybe N800-only)
     *   tea5761 (FM tuner)
     * (serial 0)
     *   GPS
     * (some serial port)
     *   csr41814 (Bluetooth)
     */
    n8x0_gpio_setup(s);
    n8x0_nand_setup(s);
    n8x0_i2c_setup(s);
    if (model == 800)
        n800_tsc_kbd_setup(s);
    else if (model == 810) {
        n810_tsc_setup(s);
        n810_kbd_setup(s);
    }
    cursor_hide = 0; // who wants to use touchscreen without a pointer?
    n8x0_spi_setup(s);
    n8x0_dss_setup(s);
    n8x0_cbus_setup(s);
    n8x0_uart_setup(s);
    if (usb_enabled(false)) {
        n8x0_usb_setup(s);
    }

    if (args->kernel_filename) {
        /* Or at the linux loader.  */
        binfo->kernel_filename = args->kernel_filename;
        binfo->kernel_cmdline = args->kernel_cmdline;
        binfo->initrd_filename = args->initrd_filename;
        arm_load_kernel(s->mpu->cpu, binfo);

        qemu_register_reset(n8x0_boot_init, s);
    }

    if (option_rom[0].name &&
        (args->boot_device[0] == 'n' || !args->kernel_filename)) {
        uint8_t nolo_tags[0x10000];
        /* No, wait, better start at the ROM.  */
        s->mpu->cpu->env.regs[15] = OMAP2_Q2_BASE + 0x400000;

        /* This is intended for loading the `secondary.bin' program from
         * Nokia images (the NOLO bootloader).  The entry point seems
         * to be at OMAP2_Q2_BASE + 0x400000.
         *
         * The `2nd.bin' files contain some kind of earlier boot code and
         * for them the entry point needs to be set to OMAP2_SRAM_BASE.
         *
         * The code above is for loading the `zImage' file from Nokia
         * images.  */
        load_image_targphys(option_rom[0].name,
                            OMAP2_Q2_BASE + 0x400000,
                            sdram_size - 0x400000);

        n800_setup_nolo_tags(nolo_tags);
        cpu_physical_memory_write(OMAP2_SRAM_BASE, nolo_tags, 0x10000);
    }
}

static struct arm_boot_info n800_binfo = {
    .loader_start = OMAP2_Q2_BASE,
    /* Actually two chips of 0x4000000 bytes each */
    .ram_size = 0x08000000,
    .board_id = 0x4f7,
    .atag_board = n800_atag_setup,
};

static struct arm_boot_info n810_binfo = {
    .loader_start = OMAP2_Q2_BASE,
    /* Actually two chips of 0x4000000 bytes each */
    .ram_size = 0x08000000,
    /* 0x60c and 0x6bf (WiMAX Edition) have been assigned but are not
     * used by some older versions of the bootloader and 5555 is used
     * instead (including versions that shipped with many devices).  */
    .board_id = 0x60c,
    .atag_board = n810_atag_setup,
};

static void n800_init(QEMUMachineInitArgs *args)
{
    return n8x0_init(args, &n800_binfo, 800);
}

static void n810_init(QEMUMachineInitArgs *args)
{
    return n8x0_init(args, &n810_binfo, 810);
}

static QEMUMachine n800_machine = {
    .name = "n800",
    .desc = "Nokia N800 tablet aka. RX-34 (OMAP2420)",
    .init = n800_init,
    DEFAULT_MACHINE_OPTIONS,
};

static QEMUMachine n810_machine = {
    .name = "n810",
    .desc = "Nokia N810 tablet aka. RX-44 (OMAP2420)",
    .init = n810_init,
    DEFAULT_MACHINE_OPTIONS,
};

#define N900_SDRAM_SIZE (256 * 1024 * 1024)
#define N900_ONENAND_CS 0
#define N900_ONENAND_BUFSIZE (0xc000 << 1)
#define N900_SMC_CS 1

#define N900_ONENAND_GPIO       65
#define N900_CAMFOCUS_GPIO      68
#define N900_CAMLAUNCH_GPIO     69
#define N900_SLIDE_GPIO         71
#define N900_PROXIMITY_GPIO     89
#define N900_HEADPHONE_EN_GPIO  98
#define N900_TSC2005_IRQ_GPIO   100
#define N900_TSC2005_RESET_GPIO 104
#define N900_CAMCOVER_GPIO      110
#define N900_KBLOCK_GPIO        113
#define N900_HEADPHONE_GPIO     177
#define N900_LIS302DL_INT2_GPIO 180
#define N900_LIS302DL_INT1_GPIO 181

//#define DEBUG_BQ2415X
//#define DEBUG_TPA6130
//#define DEBUG_LIS302DL

#define N900_TRACE(fmt, ...) \
    fprintf(stderr, "%s@%d: " fmt "\n", __FUNCTION__, __LINE__, ##__VA_ARGS__)

#ifdef DEBUG_BQ2415X
#define TRACE_BQ2415X(fmt, ...) N900_TRACE(fmt, ##__VA_ARGS__)
#else
#define TRACE_BQ2415X(...)
#endif
#ifdef DEBUG_TPA6130
#define TRACE_TPA6130(fmt, ...) N900_TRACE(fmt, ##__VA_ARGS__)
#else
#define TRACE_TPA6130(...)
#endif
#ifdef DEBUG_LIS302DL
#define TRACE_LIS302DL(fmt, ...) N900_TRACE(fmt, ##__VA_ARGS__)
#else
#define TRACE_LIS302DL(...)
#endif

static uint64_t ssi_read(void *opaque, hwaddr addr, unsigned size)
{
    switch (addr) {
        case 0x00: /* REVISION */
            return 0x10;
        case 0x14: /* SYSSTATUS */
            return 1; /* RESETDONE */
        default:
            break;
    }
    //printf("%s: addr= " OMAP_FMT_plx "\n", __FUNCTION__, addr);
    return 0;
}

static void ssi_write(void *opaque, hwaddr addr, uint64_t value,
                      unsigned size)
{
    //printf("%s: addr=" OMAP_FMT_plx ", value=0x%08x\n", __FUNCTION__, addr, value);
}

static const MemoryRegionOps ssi_ops = {
    .read = ssi_read,
    .write = ssi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

typedef struct LIS302DLState_s {
    I2CSlave i2c;
    int firstbyte;
    uint8_t reg;

    qemu_irq irq[2];
    int8_t axis_max, axis_step;
    int noise, dr_test_ack;

    uint8_t ctrl1, ctrl2, ctrl3;
    uint8_t status;
    struct {
        uint8_t cfg, src, ths, dur;
    } ff_wu[2];
    struct {
        uint8_t cfg, src, thsy_x, thsz;
        uint8_t timelimit, latency, window;
    } click;
    
    int32_t x, y, z;
} LIS302DLState;

static void lis302dl_interrupt_update(LIS302DLState *s)
{
#ifdef DEBUG_LIS302DL
    static const char *rules[8] = {
        "GND", "FF_WU_1", "FF_WU_2", "FF_WU_1|2", "DR",
        "???", "???", "CLICK"
    };
#endif
    int active = (s->ctrl3 & 0x80) ? 0 : 1;
    int cond, latch;
    int i;
    for (i = 0; i < 2; i++) {
        switch ((s->ctrl3 >> (i * 3)) & 0x07) {
            case 0:
                cond = 0;
                break;
            case 1:
                cond = s->ff_wu[0].src & 0x40;
                latch = s->ff_wu[0].cfg & 0x40;
                break;
            case 2:
                cond = s->ff_wu[1].src & 0x40;
                latch = s->ff_wu[1].cfg & 0x40;
                break;
            case 3:
                cond = ((s->ff_wu[0].src | s->ff_wu[1].src) & 0x40);
                latch = ((s->ff_wu[0].cfg | s->ff_wu[1].cfg) & 0x40);
                break;
            case 4:
                cond = (((s->ff_wu[0].src | s->ff_wu[1].src) & 0x3f) &
                        (((s->ctrl1 & 0x01) ? 0x03 : 0x00) |
                         ((s->ctrl1 & 0x02) ? 0x0c : 0x00) |
                         ((s->ctrl1 & 0x04) ? 0x30 : 0x00)));
                latch = 0;
                break;
            case 7:
                cond = s->click.src & 0x40;
                latch = s->click.cfg & 0x40;
                break;
            default:
                TRACE_LIS302DL("unsupported irq config (%d)",
                               (s->ctrl3 >> (i * 3)) & 0x07);
                cond = 0;
                latch = 0;
                break;
        }
        TRACE_LIS302DL("%s: %s irq%d", rules[(s->ctrl3 >> (i * 3)) & 0x07],
                       cond ? (latch ? "activate" : "pulse") : "deactivate",
                       i);
        qemu_set_irq(s->irq[i], cond ? active : !active);
        if (cond && !latch) {
            qemu_set_irq(s->irq[i], !active);
        }
    }
}

static void lis302dl_trigger(LIS302DLState *s, int axis, int value)
{
    if (value > s->axis_max) value = s->axis_max;
    if (value < -s->axis_max) value = -s->axis_max;
    switch (axis) {
        case 0: s->x = value; break;
        case 1: s->y = value; break;
        case 2: s->z = value; break;
        default: break;
    }
    if (s->status & (0x01 << axis)) {
        s->status |= 0x10 << axis;
    } else {
        s->status |= 0x01 << axis;
    }
    if ((s->status & 0x07) == 0x07) {
        s->status |= 0x08;
    }
    if ((s->status & 0x70) == 0x70) {
        s->status |= 0x80;
    }
    uint8_t bit = 0x02 << (axis << 1); /* over threshold */
    s->ff_wu[0].src |= bit;
    s->ff_wu[1].src |= bit;
    
    int i = 0;
    for (; i < 2; i++) {
        if (s->ff_wu[i].src & 0x3f) {
            if (s->ff_wu[i].cfg & 0x80) {
                if ((s->ff_wu[i].cfg & 0x3f) == (s->ff_wu[i].src & 0x3f)) {
                    s->ff_wu[i].src |= 0x40;
                }
            } else {
                if (s->ff_wu[i].src & s->ff_wu[i].cfg & 0x3f) {
                    s->ff_wu[i].src |= 0x40;
                }
            }
        }
        TRACE_LIS302DL("FF_WU_%d: CFG=0x%02x, SRC=0x%02x",
                       i, s->ff_wu[i].cfg, s->ff_wu[i].src);
    }
    
    lis302dl_interrupt_update(s);
}

static void lis302dl_step(void *opaque, int axis, int high, int activate)
{
    TRACE_LIS302DL("axis=%d, high=%d, activate=%d", axis, high, activate);
    LIS302DLState *s = opaque;
    if (activate) {
        int v = 0;
        switch (axis) {
            case 0: v = s->x + (high ? s->axis_step : -s->axis_step); break;
            case 1: v = s->y + (high ? s->axis_step : -s->axis_step); break;
            case 2: v = s->z + (high ? s->axis_step : -s->axis_step); break;
            default: break;
        }
        if (v > s->axis_max) v = -(s->axis_max - s->axis_step);
        if (v < -s->axis_max) v = s->axis_max - s->axis_step;
        lis302dl_trigger(s, axis, v);
    }
}

static void lis302dl_reset(DeviceState *ds)
{
    LIS302DLState *s = FROM_I2C_SLAVE(LIS302DLState, I2C_SLAVE(ds));
    
    s->firstbyte = 0;
    s->reg = 0;

    s->noise = 4;
    s->dr_test_ack = 0;

    s->ctrl1 = 0x03;
    s->ctrl2 = 0x00;
    s->ctrl3 = 0x00;
    s->status = 0x00;

    memset(s->ff_wu, 0x00, sizeof(s->ff_wu));
    memset(&s->click, 0x00, sizeof(s->click));
    
    s->x = 0;
    s->y = -s->axis_max;
    s->z = 0;

    lis302dl_interrupt_update(s);
}

static void lis302dl_event(I2CSlave *i2c, enum i2c_event event)
{
    LIS302DLState *s = FROM_I2C_SLAVE(LIS302DLState, i2c);
    if (event == I2C_START_SEND)
        s->firstbyte = 1;
}

static uint8_t lis302dl_readcoord(LIS302DLState *s, int coord)
{
    int v;

    switch (coord) {
        case 0:
            v = s->x;
            break;
        case 1:
            v = s->y;
            break;
        case 2:
            v = s->z;
            break;
        default:
            hw_error("%s: unknown axis %d", __FUNCTION__, coord);
            break;
    }
    s->status &= ~(0x88 | (0x11 << coord));
    if (s->ctrl1 & 0x10) {
        switch (coord) {
            case 0:
                v -= s->noise;
                break;
            case 1:
            case 2:
                v += s->noise;
                break;
            default:
                break;
        }
        if (++s->noise == 32) {
            s->noise = 4;
        }
        int dr1 = ((s->ctrl3 & 0x07) == 4);
        int dr2 = (((s->ctrl3 >> 3) & 0x07) == 4);
        if (!s->dr_test_ack++) {
            if (dr1) {
                qemu_irq_pulse(s->irq[0]);
            }
            if (dr2) {
                qemu_irq_pulse(s->irq[1]);
            }
        } else if (s->dr_test_ack == 1 + (dr1 + dr2) * 3) {
            s->dr_test_ack = 0;
        }
    }
    return (uint8_t)v;
}

static int lis302dl_rx(I2CSlave *i2c)
{
    LIS302DLState *s = FROM_I2C_SLAVE(LIS302DLState, i2c);
    int value = -1;
    int n = 0;
    switch (s->reg & 0x7f) {
        case 0x00 ... 0x0e:
        case 0x10 ... 0x1f:
        case 0x23 ... 0x26:
        case 0x28:
        case 0x2a:
        case 0x2c:
        case 0x2e ... 0x2f:
        case 0x3a:
            value = 0;
            TRACE_LIS302DL("reg 0x%02x = 0x%02x (unused/reserved reg)",
                           s->reg & 0x7f, value);
            break;
        case 0x0f:
            value = 0x3b;
            TRACE_LIS302DL("WHOAMI = 0x%02x", value);
            break;
        case 0x20:
            value = s->ctrl1;
            TRACE_LIS302DL("CTRL1 = 0x%02x", value);
            break;
        case 0x21:
            value = s->ctrl2;
            TRACE_LIS302DL("CTRL2 = 0x%02x", value);
            break;
        case 0x22:
            value = s->ctrl3;
            TRACE_LIS302DL("CTRL3 = 0x%02x", value);
            break;
        case 0x27:
            value = s->status;
            TRACE_LIS302DL("STATUS = 0x%02x", value);
            break;
        case 0x29:
            value = lis302dl_readcoord(s, 0);
            TRACE_LIS302DL("X = 0x%02x", value);
            break;
        case 0x2b:
            value = lis302dl_readcoord(s, 1);
            TRACE_LIS302DL("Y = 0x%02x", value);
            break;
        case 0x2d:
            value = lis302dl_readcoord(s, 2);
            TRACE_LIS302DL("Z = 0x%02x", value);
            break;
        case 0x34: n++;
        case 0x30:
            value = s->ff_wu[n].cfg;
            TRACE_LIS302DL("FF_WU%d.CFG = 0x%02x", n + 1, value);
            break;
        case 0x35: n++;
        case 0x31:
            value = s->ff_wu[n].src;
            TRACE_LIS302DL("FF_WU%d.SRC = 0x%02x", n + 1, value);
            s->ff_wu[n].src = 0; //&= ~0x40;
            lis302dl_interrupt_update(s);
            break;
        case 0x36: n++;
        case 0x32:
            value = s->ff_wu[n].ths;
            TRACE_LIS302DL("FF_WU%d.THS = 0x%02x", n + 1, value);
            break;
        case 0x37: n++;
        case 0x33:
            value = s->ff_wu[n].dur;
            TRACE_LIS302DL("FF_WU%d.DUR = 0x%02x", n + 1, value);
            break;
        case 0x38:
            value = s->click.cfg;
            TRACE_LIS302DL("CLICK_CFG = 0x%02x", value);
            break;
        case 0x39:
            value = s->click.src;
            TRACE_LIS302DL("CLICK_SRC = 0x%02x", value);
            s->click.src &= ~0x40;
            lis302dl_interrupt_update(s);
            break;
        case 0x3b:
            value = s->click.thsy_x;
            TRACE_LIS302DL("CLICK_THSY_X = 0x%02x", value);
            break;
        case 0x3c:
            value = s->click.thsz;
            TRACE_LIS302DL("CLICK_THSZ = 0x%02x", value);
            break;
        case 0x3d:
            value = s->click.timelimit;
            TRACE_LIS302DL("CLICK_TIMELIMIT = 0x%02x", value);
            break;
        case 0x3e:
            value = s->click.latency;
            TRACE_LIS302DL("CLICK_LATENCY = 0x%02x", value);
            break;
        case 0x3f:
            value = s->click.window;
            TRACE_LIS302DL("CLICK_WINDOW = 0x%02x", value);
            break;
        default:
            hw_error("%s: unknown register 0x%02x", __FUNCTION__,
                     s->reg & 0x7f);
            value = 0;
            break;
    }
    if (s->reg & 0x80) { /* auto-increment? */
        s->reg = (s->reg + 1) | 0x80;
    }
    return value;
}

static int lis302dl_tx(I2CSlave *i2c, uint8_t data)
{
    LIS302DLState *s = FROM_I2C_SLAVE(LIS302DLState, i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        int n = 0;
        switch (s->reg & 0x7f) {
            case 0x20:
                TRACE_LIS302DL("CTRL1 = 0x%02x", data);
                s->ctrl1 = data;
                break;
            case 0x21:
                TRACE_LIS302DL("CTRL2 = 0x%02x", data);
                s->ctrl2 = data;
                break;
            case 0x22:
                TRACE_LIS302DL("CTRL3 = 0x%02x", data);
                s->ctrl3 = data;
                lis302dl_interrupt_update(s);
                break;
            case 0x34: n++;
            case 0x30:
                TRACE_LIS302DL("FF_WU%d.CFG = 0x%02x", n + 1, data);
                s->ff_wu[n].cfg = data;
                break;
            case 0x36: n++;
            case 0x32:
                TRACE_LIS302DL("FF_WU%d.THS = 0x%02x", n + 1, data);
                s->ff_wu[n].ths = data;
                break;
            case 0x37: n++;
            case 0x33:
                TRACE_LIS302DL("FF_WU%d.DUR = 0x%02x", n + 1, data);
                s->ff_wu[n].dur = data;
                break;
            case 0x38:
                TRACE_LIS302DL("CLICK_CFG = 0x%02x", data);
                s->click.cfg = data;
                break;
            case 0x39:
                TRACE_LIS302DL("CLICK_SRC = 0x%02x", data);
                s->click.src = data;
                break;
            case 0x3b:
                TRACE_LIS302DL("CLICK_THSY_X = 0x%02x", data);
                s->click.thsy_x = data;
                break;
            case 0x3c:
                TRACE_LIS302DL("CLICK_THSZ = 0x%02x", data);
                s->click.thsz = data;
                break;
            case 0x3d:
                TRACE_LIS302DL("CLICK_TIMELIMIT = 0x%02x", data);
                s->click.timelimit = data;
                break;
            case 0x3e:
                TRACE_LIS302DL("CLICK_LATENCY = 0x%02x", data);
                s->click.latency = data;
                break;
            case 0x3f:
                TRACE_LIS302DL("CLICK_WINDOW = 0x%02x", data);
                s->click.window = data;
                break;
            default:
                hw_error("%s: unknown register 0x%02x (value 0x%02x)",
                         __FUNCTION__, s->reg & 0x7f, data);
                break;
        }
        if (s->reg & 0x80) { /* auto-increment? */
            s->reg = (s->reg + 1) | 0x80;
        }
    }
    return 1;
}

static int lis302dl_init(I2CSlave *i2c)
{
    LIS302DLState *s = FROM_I2C_SLAVE(LIS302DLState, i2c);
    s->axis_max = 58;
    s->axis_step = s->axis_max;// / 2;
    qdev_init_gpio_out(&i2c->qdev, s->irq, 2);
    return 0;
}

/* TODO: ideally x, y, z should be runtime modifiable properties,
 * which can be set by calling
 *     lis302dl_trigger(s, axis, value);
 * where axis is 0,1,2 for x,y,z
 */
static Property lis302dl_properties[] = {
    DEFINE_PROP_INT32("x", LIS302DLState, x, 0),
    DEFINE_PROP_INT32("y", LIS302DLState, y, 0),
    DEFINE_PROP_INT32("z", LIS302DLState, z, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void lis302dl_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    k->init = lis302dl_init;
    k->event = lis302dl_event;
    k->recv = lis302dl_rx;
    k->send = lis302dl_tx;
    dc->props = lis302dl_properties;
    dc->reset = lis302dl_reset;
}

static TypeInfo lis302dl_info = {
    .name = "lis302dl",
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(LIS302DLState),
    .class_init = lis302dl_class_init,
};

typedef struct BQ2415XState_s {
    I2CSlave i2c;
    int firstbyte;
    uint8 reg;
    
    uint8_t id;
    uint8_t st_ctrl;
    uint8_t ctrl;
    uint8_t bat_v;
    uint8_t tcc;
} BQ2415XState;

static void bq2415x_reset(DeviceState *ds)
{
    BQ2415XState *s = FROM_I2C_SLAVE(BQ2415XState, I2C_SLAVE(ds));
    
    s->firstbyte = 0;
    s->reg = 0;

    s->st_ctrl = 0x50 | 0x80; // 40
    s->ctrl = 0x30;
    s->bat_v = 0x0a;
    s->tcc = 0xa1; // 89
}

static void bq2415x_event(I2CSlave *i2c, enum i2c_event event)
{
    BQ2415XState *s = FROM_I2C_SLAVE(BQ2415XState, i2c);
    if (event == I2C_START_SEND)
        s->firstbyte = 1;
}

static int bq2415x_rx(I2CSlave *i2c)
{
    BQ2415XState *s = FROM_I2C_SLAVE(BQ2415XState, i2c);
    int value = -1;
    switch (s->reg) {
        case 0x00:
            value = s->st_ctrl;
            TRACE_BQ2415X("st_ctrl = 0x%02x", value);
            break;
        case 0x01:
            value = s->ctrl;
            TRACE_BQ2415X("ctrl = 0x%02x", value);
            break;
        case 0x02:
            value = s->bat_v;
            TRACE_BQ2415X("bat_v = 0x%02x", value);
            break;
        case 0x03:
        case 0x3b:
            value = s->id;
            TRACE_BQ2415X("id = 0x%02x", value);
            break;
        case 0x04:
            value = s->tcc;
            TRACE_BQ2415X("tcc = 0x%02x", value);
            break;
        default:
            TRACE_BQ2415X("unknown register 0x%02x", s->reg);
            value = 0;
            break;
    }
    s->reg++;
    return value;
}

static int bq2415x_tx(I2CSlave *i2c, uint8_t data)
{
    BQ2415XState *s = FROM_I2C_SLAVE(BQ2415XState, i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        switch (s->reg) {
            case 0x00:
                TRACE_BQ2415X("st_ctrl = 0x%02x", data);
                s->st_ctrl = (s->st_ctrl & 0x3f) | (data & 0x40) | 0x80;
                break;
            case 0x01:
                TRACE_BQ2415X("ctrl = 0x%02x", data);
                s->ctrl = data;
                break;
            case 0x02:
                TRACE_BQ2415X("bat_v = 0x%02x", data);
                s->bat_v = data;
                break;
            case 0x04:
                TRACE_BQ2415X("tcc = 0x%02x", data);
                s->tcc = data | 0x80;
                break;
            default:
                TRACE_BQ2415X("unknown register 0x%02x (value 0x%02x)",
                              s->reg, data);
                break;
        }
        s->reg++;
    }
    return 1;
}

static int bq2415x_init(I2CSlave *i2c)
{
    return 0;
}

static Property bq2415x_properties[] = {
    DEFINE_PROP_UINT8("id", BQ2415XState, id, 0x49),
    DEFINE_PROP_END_OF_LIST(),
};

static void bq2415x_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    k->init = bq2415x_init;
    k->event = bq2415x_event;
    k->recv = bq2415x_rx;
    k->send = bq2415x_tx;
    dc->props = bq2415x_properties;
    dc->reset = bq2415x_reset;
}

static TypeInfo bq2415x_info = {
    .name = "bq2415x",
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(BQ2415XState),
    .class_init = bq2415x_class_init,
};

typedef struct tpa6130_s {
    I2CSlave i2c;
    int firstbyte;
    int reg;
    uint8_t data[3];
} TPA6130State;

static void tpa6130_reset(DeviceState *ds)
{
    TPA6130State *s = FROM_I2C_SLAVE(TPA6130State, I2C_SLAVE(ds));
    s->firstbyte = 0;
    s->reg = 0;
    memset(s->data, 0, sizeof(s->data));
}

static void tpa6130_event(I2CSlave *i2c, enum i2c_event event)
{
    TPA6130State *s = FROM_I2C_SLAVE(TPA6130State, i2c);
    if (event == I2C_START_SEND)
        s->firstbyte = 1;
}

static int tpa6130_rx(I2CSlave *i2c)
{
    TPA6130State *s = FROM_I2C_SLAVE(TPA6130State, i2c);
    int value = 0;
    switch (s->reg) {
        case 1 ... 3:
            value = s->data[s->reg - 1];
            TRACE_TPA6130("reg %d = 0x%02x", s->reg, value);
            break;
        case 4: /* VERSION */
            value = 0x01;
            TRACE_TPA6130("version = 0x%02x", value);
            break;
        default:
            TRACE_TPA6130("unknown register 0x%02x", s->reg);
            break;
    }
    s->reg++;
    return value;
}

static int tpa6130_tx(I2CSlave *i2c, uint8_t data)
{
    TPA6130State *s = FROM_I2C_SLAVE(TPA6130State, i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        switch (s->reg) {
            case 1 ... 3:
                TRACE_TPA6130("reg %d = 0x%02x", s->reg, data);
                s->data[s->reg - 1] = data;
                break;
            default:
                TRACE_TPA6130("unknown register 0x%02x", s->reg);
                break;
        }
        s->reg++;
    }
    return 1;
}

static void tpa6130_irq(void *opaque, int n, int level)
{
    if (n) {
        hw_error("%s: unknown interrupt source %d\n", __FUNCTION__, n);
    } else {
        /* headphone enable */
        TRACE_TPA6130("enable = %d", level);
    }
}

static int tpa6130_init(I2CSlave *i2c)
{
    qdev_init_gpio_in(&i2c->qdev, tpa6130_irq, 1);
    return 0;
}

static void tpa6130_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    k->init = tpa6130_init;
    k->event = tpa6130_event;
    k->recv = tpa6130_rx;
    k->send = tpa6130_tx;
    dc->reset = tpa6130_reset;
}

static TypeInfo tpa6130_info = {
    .name = "tpa6130",
    .parent = TYPE_I2C_SLAVE,
    .instance_size = sizeof(TPA6130State),
    .class_init = tpa6130_class_init,
};

struct n900_s {
    struct omap_mpu_state_s *cpu;
    void *twl4030;
    DeviceState *nand;
    DeviceState *mipid;
    DeviceState *tsc2005;
    DeviceState *bq2415x;
    DeviceState *tpa6130;
    DeviceState *lis302dl;
    DeviceState *smc;
#ifdef CONFIG_GLES2
    void *gles2;
#endif
    int extended_key;
    int slide_open;
    int camera_cover_open;
    int headphone_connected;
};

/* this takes care of the keys which are not located on the
 * n900 keypad (note that volume up/down keys are handled by
 * the keypad eventhough the keys are not located on the keypad)
 * as well as triggering some other hardware button/switch-like
 * events that are mapped to the host keyboard:
 *
 * escape ... power
 * f1 ....... keypad slider open/close
 * f2 ....... keypad lock
 * f3 ....... camera lens cover open/close
 * f4 ....... camera focus
 * f5 ....... camera take picture
 * f6 ....... stereo headphone connect/disconnect
 * kp1 ...... decrease accelerometer x axis value
 * kp2 ...... increase accelerometer x axis value
 * kp4 ...... decrease accelerometer y axis value
 * kp5 ...... increase accelerometer y axis value
 * kp7 ...... decrease accelerometer z axis value
 * kp8 ...... increase accelerometer z axis value
 */
static void n900_key_handler(void *opaque, int keycode)
{
    struct n900_s *s = opaque;
    if (!s->extended_key && keycode == 0xe0) {
        s->extended_key = 0x80;
    } else {
        int release = keycode & 0x80;
        keycode = (keycode & 0x7f) | s->extended_key;
        s->extended_key = 0;
        switch (keycode) {
            case 0x01: /* escape */
                twl4030_set_powerbutton_state(s->twl4030, !release);
                break;
            case 0x3b: /* f1 */
                if (release) {
                    s->slide_open = !s->slide_open;
                    qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio,
                                                  N900_SLIDE_GPIO),
                                 !s->slide_open);
                }
                break;
            case 0x3c: /* f2 */
                qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio, N900_KBLOCK_GPIO),
                             !!release);
                break;
            case 0x3d: /* f3 */
                if (release) {
                    s->camera_cover_open = !s->camera_cover_open;
                    qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio,
                                                  N900_CAMCOVER_GPIO),
                                 s->camera_cover_open);
                }
                break;
            case 0x3e: /* f4 */
                qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio,
                                              N900_CAMFOCUS_GPIO),
                             !!release);
                break;
            case 0x3f: /* f5 */
                qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio,
                                              N900_CAMLAUNCH_GPIO),
                             !!release);
                break;
            case 0x40: /* f6 */
                if (release) {
                    s->headphone_connected = !s->headphone_connected;
                    qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio,
                                                  N900_HEADPHONE_GPIO),
                                 !s->headphone_connected);
                }
                break;
            case 0x4f ... 0x50: /* kp1,2 */
                lis302dl_step(s->lis302dl, 0, keycode - 0x4f, !release);
                break;
            case 0x4b ... 0x4c: /* kp4,5 */
                lis302dl_step(s->lis302dl, 1, keycode - 0x4b, !release);
                break;
            case 0x47 ... 0x48: /* kp7,8 */
                lis302dl_step(s->lis302dl, 2, keycode - 0x47, !release);
                break;
            default:
                break;
        }
    }
}

static void n900_reset(void *opaque)
{
    struct n900_s *s = opaque;
    s->slide_open = 1;
    s->camera_cover_open = 0;
    s->headphone_connected = 0;
    qemu_irq_raise(qdev_get_gpio_in(s->cpu->gpio, N900_KBLOCK_GPIO));
    qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio, N900_HEADPHONE_GPIO), 
                 !s->headphone_connected);
    qemu_irq_raise(qdev_get_gpio_in(s->cpu->gpio, N900_CAMLAUNCH_GPIO));
    qemu_irq_raise(qdev_get_gpio_in(s->cpu->gpio, N900_CAMFOCUS_GPIO));
    qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio, N900_CAMCOVER_GPIO),
                 s->camera_cover_open);
    qemu_set_irq(qdev_get_gpio_in(s->cpu->gpio, N900_SLIDE_GPIO),
                 !s->slide_open);
    omap3_boot_rom_emu(s->cpu);
}


static uint16_t n900_twl4030_madc_callback(twl4030_adc_type type, int ch)
{
    return 0x3ff;
}

static const TWL4030KeyMap n900_twl4030_keymap[] = {
    {0x10, 0, 0}, /* Q */
    {0x11, 0, 1}, /* W */
    {0x12, 0, 2}, /* E */
    {0x13, 0, 3}, /* R */
    {0x14, 0, 4}, /* T */
    {0x15, 0, 5}, /* Y */
    {0x16, 0, 6}, /* U */
    {0x17, 0, 7}, /* I */
    {0x18, 1, 0}, /* O */
    {0x20, 1, 1}, /* D */
    {0x34, 1, 2}, /* . */
    {0x2f, 1, 3}, /* V */
    {0xd0, 1, 4}, /* DOWN */
    {0x41, 1, 7}, /* F7 -- volume/zoom down */
    {0x19, 2, 0}, /* P */
    {0x21, 2, 1}, /* F */
    {0xc8, 2, 2}, /* UP */
    {0x30, 2, 3}, /* B */
    {0xcd, 2, 4}, /* RIGHT */
    {0x42, 2, 7}, /* F8 -- volume/zoom up */
    {0x33, 3, 0}, /* , */
    {0x22, 3, 1}, /* G */
    {0x1c, 3, 2}, /* ENTER */
    {0x31, 3, 3}, /* N */
    {0x0e, 4, 0}, /* BACKSPACE */
    {0x23, 4, 1}, /* H */
    {0x32, 4, 3}, /* M */
    {0x1d, 4, 4}, /* LEFTCTRL */
    {0x9d, 4, 4}, /* RIGHTCTRL */
    {0x24, 5, 1}, /* J */
    {0x2c, 5, 2}, /* Z */
    {0x39, 5, 3}, /* SPACE */
    {0x38, 5, 4}, /* LEFTALT -- "fn" */
    {0xb8, 5, 4}, /* RIGHTALT -- "fn" */
    {0x1e, 6, 0}, /* A */
    {0x25, 6, 1}, /* K */
    {0x2d, 6, 2}, /* X */
    {0x39, 6, 3}, /* SPACE */
    {0x2a, 6, 4}, /* LEFTSHIFT */
    {0x36, 6, 4}, /* RIGHTSHIFT */
    {0x1f, 7, 0}, /* S */
    {0x26, 7, 1}, /* L */
    {0x2e, 7, 2}, /* C */
    {0xcb, 7, 3}, /* LEFT */
    //    {0x10, 0xff, 2}, /* F9 */
    //    {0x10, 0xff, 4}, /* F10 */
    //    {0x10, 0xff, 5}, /* F11 */
    {-1, -1, -1}
};

static MouseTransformInfo n900_pointercal = {
    .x = 800,
    .y = 480,
    .a = {14114,  18, -2825064,  34,  -8765, 32972906, 65536},
};

static void n900_init(QEMUMachineInitArgs *args)
{
    MemoryRegion *sysmem = get_system_memory();
    MemoryRegion *ssi_iomem = g_new(MemoryRegion, 1);
    struct n900_s *s = g_malloc0(sizeof(*s));
    DriveInfo *dmtd = drive_get(IF_MTD, 0, 0);
    DriveInfo *dsd  = drive_get(IF_SD, 0, 0);

    if (!dmtd && !dsd) {
        hw_error("%s: SD or NAND image required", __FUNCTION__);
    }
#if MAX_SERIAL_PORTS < 3
#error MAX_SERIAL_PORTS must be at least 3!
#endif
    s->cpu = omap3_mpu_init(sysmem, omap3430, N900_SDRAM_SIZE,
                            serial_hds[1], serial_hds[2],
                            serial_hds[0], NULL);
    omap_lcd_panel_attach(s->cpu->dss);

    s->tsc2005 = spi_create_device(omap_mcspi_bus(s->cpu->mcspi, 0),
                                   "tsc2005", 0);
    qdev_connect_gpio_out(s->tsc2005, 0,
                          qdev_get_gpio_in(s->cpu->gpio,
                                           N900_TSC2005_IRQ_GPIO));
    tsc2005_set_transform(s->tsc2005, &n900_pointercal, 600, 1500);
    cursor_hide = 0; // who wants to use touchscreen without a pointer?

    s->mipid = spi_create_device_noinit(omap_mcspi_bus(s->cpu->mcspi, 0),
                                        "lcd_mipid", 2);
    qdev_prop_set_uint32(s->mipid, "id", 0x101234);
    qdev_prop_set_uint8(s->mipid, "n900", 1);
    qdev_init_nofail(s->mipid);


    s->nand = qdev_create(NULL, "onenand");
    qdev_prop_set_uint16(s->nand, "manufacturer_id", NAND_MFR_SAMSUNG);
    qdev_prop_set_uint16(s->nand, "device_id", 0x40);
    qdev_prop_set_uint16(s->nand, "version_id", 0x121);
    qdev_prop_set_int32(s->nand, "shift", 1);
    if (dmtd && dmtd->bdrv) {
        qdev_prop_set_drive_nofail(s->nand, "drive", dmtd->bdrv);
    }
    qdev_init_nofail(s->nand);
    sysbus_connect_irq(SYS_BUS_DEVICE(s->nand), 0,
                       qdev_get_gpio_in(s->cpu->gpio, N900_ONENAND_GPIO));
    omap_gpmc_attach(s->cpu->gpmc, 0,
                     sysbus_mmio_get_region(SYS_BUS_DEVICE(s->nand), 0));

    if (dsd) {
        omap3_mmc_attach(s->cpu->omap3_mmc[1], dsd->bdrv, 0, 1);
    }
    if ((dsd = drive_get(IF_SD, 0, 1)) != NULL) {
        omap3_mmc_attach(s->cpu->omap3_mmc[0], dsd->bdrv, 0, 0);
        //qemu_irq_raise(omap2_gpio_in_get(s->cpu->gpif, N900_SDCOVER_GPIO));
    }

    memory_region_init_io(ssi_iomem, &ssi_ops, 0, "n900_ssi", 0x3c00);
    memory_region_add_subregion(sysmem, 0x48058000, ssi_iomem);

    s->twl4030 = twl4030_init(omap_i2c_bus(s->cpu->i2c[0]),
                              qdev_get_gpio_in(s->cpu->ih[0],
                                               OMAP_INT_3XXX_SYS_NIRQ),
                              NULL, n900_twl4030_keymap);
    twl4030_madc_attach(s->twl4030, n900_twl4030_madc_callback);
    i2c_bus *i2c2 = omap_i2c_bus(s->cpu->i2c[1]);
    s->bq2415x = i2c_create_slave(i2c2, "bq2415x", 0x6b);
    s->tpa6130 = i2c_create_slave(i2c2, "tpa6130", 0x60);
    qdev_connect_gpio_out(s->cpu->gpio, N900_HEADPHONE_EN_GPIO,
                          qdev_get_gpio_in(s->tpa6130, 0));
    i2c_bus *i2c3 = omap_i2c_bus(s->cpu->i2c[2]);
    s->lis302dl = i2c_create_slave(i2c3, "lis302dl", 0x1d);
    qdev_connect_gpio_out(s->lis302dl, 0,
                          qdev_get_gpio_in(s->cpu->gpio,
                                           N900_LIS302DL_INT1_GPIO));
    qdev_connect_gpio_out(s->lis302dl, 1,
                          qdev_get_gpio_in(s->cpu->gpio,
                                           N900_LIS302DL_INT2_GPIO));

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
        sysbus_connect_irq(SYS_BUS_DEVICE(s->smc), 0,
                           qdev_get_gpio_in(s->cpu->gpio, 54));
        omap_gpmc_attach(s->cpu->gpmc, 1,
                         sysbus_mmio_get_region(SYS_BUS_DEVICE(s->smc), 0));
    } else {
        hw_error("%s: no NIC for smc91c111\n", __FUNCTION__);
    }

    qemu_add_kbd_event_handler(n900_key_handler, s);

    qemu_register_reset(n900_reset, s);
}

static QEMUMachine n900_machine = {
    .name = "n900",
    .desc = "Nokia N900 (OMAP3)",
    .init = n900_init,
};

static void nseries_register_types(void)
{
    type_register_static(&bq2415x_info);
    type_register_static(&tpa6130_info);
    type_register_static(&lis302dl_info);
    type_register_static(&mipid_info);
}

static void nseries_machine_init(void)
{
    qemu_register_machine(&n800_machine);
    qemu_register_machine(&n810_machine);
    qemu_register_machine(&n900_machine);
}

type_init(nseries_register_types);
machine_init(nseries_machine_init);
