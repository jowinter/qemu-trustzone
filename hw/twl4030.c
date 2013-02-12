/*
 * TI TWL4030 emulation
 *
 * Copyright (C) 2008 yajin<yajin@vm-kernel.org>
 * Copyright (C) 2009-2010 Nokia Corporation
 *
 * Register implementation based on TPS65950 ES1.0 specification.
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

#include <sys/time.h>
#include "hw.h"
#include "qemu/timer.h"
#include "i2c.h"
#include "sysemu/sysemu.h"
#include "ui/console.h"
#include "exec/cpu-all.h"

//#define DEBUG_GENERAL
//#define DEBUG_RTC

#define DEBUG_TRACE(fmt, ...) fprintf(stderr, "%s@%d: " fmt "\n", \
                                      __FUNCTION__, __LINE__, ##__VA_ARGS__)

#ifdef DEBUG_GENERAL
#define TRACE(...) DEBUG_TRACE(__VA_ARGS__)
#else
#define TRACE(...)
#endif

#ifdef DEBUG_RTC
#define TRACE_RTC(...) DEBUG_TRACE(__VA_ARGS__)
#else
#define TRACE_RTC(...)
#endif

typedef struct TWL4030State TWL4030State;
typedef struct TWL4030NodeState TWL4030NodeState;

typedef uint8_t (*twl4030_read_func)(TWL4030NodeState *s,
                                     uint8_t addr);
typedef void (*twl4030_write_func)(TWL4030NodeState *s,
                                   uint8_t addr, uint8_t value);

struct TWL4030NodeState {
    I2CSlave i2c;
    int firstbyte;
    uint8_t reg;

    twl4030_read_func read_func;
    twl4030_write_func write_func;
    TWL4030State *twl4030;

    uint8 reg_data[256];
};

struct TWL4030State {
    qemu_irq irq1;
    qemu_irq irq2;
    QEMUTimer *alarm_timer;
    QEMUTimer *periodic_timer;
    const TWL4030KeyMap *keymap;
    int extended_key;
    uint8_t twl5031;
    uint8_t twl5031_aciid;
    twl4030_madc_callback madc_cb;

    int key_cfg;
    int key_tst;

    TWL4030NodeState *i2c[4];

    uint8_t seq_mem[64][4]; /* power-management sequencing memory */
};

static const uint8_t addr_48_reset_values[256] = {
    0x51, 0x04, 0x02, 0xc0, 0x41, 0x41, 0x41, 0x10, /* 0x00...0x07 */
    0x10, 0x10, 0x06, 0x06, 0x06, 0x1f, 0x1f, 0x1f, /* 0x08...0x0f */
    0x1f, 0x1f, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x10...0x17 */
    0x00, 0x00, 0x00, 0x00, 0x52, 0x00, 0x00, 0x00, /* 0x18...0x1f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x0a, 0x03, /* 0x20...0x27 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x28...0x2f */
    0x00, 0x00, 0x00, 0x04, 0x04, 0x04, 0x00, 0x00, /* 0x30...0x37 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x38...0x3f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x40...0x47 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x48...0x4f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x50...0x57 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x58...0x5f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x60...0x67 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x68...0x6f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x70...0x77 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x78...0x7f */
    0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x00, 0x00, /* 0x80...0x87 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x88...0x8f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x90...0x97 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x98...0x9f */
    0x00, 0x10, 0x08, 0x08, 0x00, 0x00, 0x00, 0x00, /* 0xa0...0xa7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xa8...0xaf */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xb0...0xb7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xb8...0xb8 */
    0xa0, 0xa0, 0x64, 0x7f, 0x6c, 0x75, 0x64, 0x20, /* 0xc0...0xc7 */
    0x01, 0x17, 0x01, 0x02, 0x00, 0x36, 0x44, 0x07, /* 0xc8...0xcf */
    0x3b, 0x17, 0x6b, 0x04, 0x00, 0x00, 0x00, 0x00, /* 0xd0...0xd7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xd8...0xdf */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xe0...0xe7 */
    0x00, 0x00, 0x10, 0x10, 0x00, 0x00, 0x00, 0x00, /* 0xe8...0xef */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xf0...0xf7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00  /* 0xf8...0xff */
};

static const uint8_t addr_49_reset_values[256] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x00...0x07 */
    0x00, 0x00, 0x0f, 0x0f, 0x0f, 0x0f, 0x00, 0x00, /* 0x08...0x0f */
    0x3f, 0x3f, 0x3f, 0x3f, 0x25, 0x00, 0x00, 0x00, /* 0x10...0x17 */
    0x00, 0x32, 0x32, 0x32, 0x32, 0x00, 0x00, 0x55, /* 0x18...0x1f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x20...0x27 */
    0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, /* 0x28...0x2f */
    0x13, 0x00, 0x00, 0x00, 0x00, 0x79, 0x11, 0x00, /* 0x30...0x37 */
    0x00, 0x00, 0x06, 0x00, 0x44, 0x69, 0x00, 0x00, /* 0x38...0x3f */
    0x00, 0x00, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, /* 0x40...0x47 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x48...0x4f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x50...0x57 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x58...0x5f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x60...0x67 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x68...0x6f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x70...0x77 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x78...0x7f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x80...0x87 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x88...0x8f */
    0x00, 0x90, 0x00, 0x00, 0x55, 0x00, 0x00, 0x00, /* 0x90...0x97 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x98...0x9f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xa0...0xa7 */
    0x00, 0x00, 0x04, 0x00, 0x55, 0x01, 0x55, 0x05, /* 0xa8...0xaf */
    0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x03, 0x00, /* 0xb0...0xb7 */
    0x00, 0x00, 0xff, 0xff, 0x03, 0x00, 0x00, 0x00, /* 0xb8...0xbf */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, /* 0xc0...0xc7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xc8...0xcf */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xd0...0xd7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xd8...0xdf */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xe0...0xe7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xe8...0xef */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xf0...0xf7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xf8...0xff */
};

static const uint8_t addr_4a_reset_values[256] = {
    0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x00...0x07 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x08...0x0f */
    0xc0, 0x8c, 0xde, 0xde, 0x00, 0x00, 0x00, 0x00, /* 0x10...0x17 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x18...0x1f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x20...0x27 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x28...0x2f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x30...0x37 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x38...0x3f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x40...0x47 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x48...0x4f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x50...0x57 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x58...0x5f */
    0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, 0x55, 0x07, /* 0x60...0x67 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x68...0x6f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x70...0x77 */
    0xff, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x78...0x7f */
    0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, /* 0x80...0x87 */
    0x00, 0x68, 0x9b, 0x86, 0x48, 0x2a, 0x07, 0x28, /* 0x88...0x8f */
    0x09, 0x69, 0x90, 0x00, 0x2a, 0x00, 0x02, 0x00, /* 0x90...0x97 */
    0x10, 0xcd, 0x02, 0x68, 0x03, 0x00, 0x00, 0x00, /* 0x98...0x9f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xa0...0xa7 */
    0x00, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, /* 0xa8...0xaf */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xb0...0xb7 */
    0x00, 0x00, 0x00, 0xff, 0x0f, 0x00, 0x00, 0xff, /* 0xb8...0xbf */
    0x0f, 0x00, 0x00, 0xbf, 0x00, 0x00, 0x01, 0x00, /* 0xc0...0xc7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xc8...0xcf */
    0x00, 0x00, 0x03, 0x00, 0x00, 0xe0, 0x00, 0x00, /* 0xd0...0xd7 */
    0xff, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xd8...0xdf */
    0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x0f, 0x00, /* 0xe0...0xe7 */
    0x55, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xe8...0xef */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xf0...0xf7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* 0xf8...0xff */
};

static const uint8_t addr_4b_reset_values[256] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x00...0x07 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x08...0x0f */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x10...0x17 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, /* 0x18...0x1f */
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, /* 0x20...0x27 */
    0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x01, 0x00, /* 0x28...0x2f */
    0x00, 0x00, 0x00, 0xff, 0xff, 0x01, 0xbf, 0xbf, /* 0x30...0x37 */
    0xbf, 0xab, 0x00, 0x08, 0x3f, 0x15, 0x40, 0x0e, /* 0x38...0x3f */
    0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x40...0x47 */
    0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, /* 0x48...0x4f */
    0x00, 0x02, 0x00, 0x04, 0x0d, 0x00, 0x00, 0x00, /* 0x50...0x57 */
    0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x58...0x5f */
    0x00, 0x00, 0x2f, 0x18, 0x0f, 0x08, 0x0f, 0x08, /* 0x60...0x67 */
    0x12, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0x68...0x6f */
    0x00, 0x00, 0x00, 0x00, 0x08, 0x02, 0x80, 0x03, /* 0x70...0x77 */
    0x08, 0x09, 0x00, 0x00, 0x08, 0x03, 0x80, 0x03, /* 0x78...0x7f */
    0x08, 0x02, 0x00, 0x00, 0x08, 0x00, 0x80, 0x03, /* 0x80...0x87 */
    0x08, 0x08, 0x20, 0x00, 0x00, 0x02, 0x80, 0x04, /* 0x88...0x8f */
    0x08, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, /* 0x90...0x97 */
    0x08, 0x02, 0xe0, 0x01, 0x08, 0x00, 0xe0, 0x00, /* 0x98...0x9f */
    0x08, 0x01, 0xe0, 0x01, 0x08, 0x04, 0xe0, 0x03, /* 0xa0...0xa7 */
    0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xa8...0xaf */
    0x20, 0x04, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xb0...0xb7 */
    0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x40, 0x05, /* 0xb8...0xbf */
    0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, /* 0xc0...0xc7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x08, /* 0xc8...0xcf */
    0x00, 0x08, 0xe0, 0x00, 0x08, 0x00, 0x00, 0x00, /* 0xd0...0xd7 */
    0x14, 0x08, 0xe0, 0x02, 0x08, 0xe0, 0x00, 0x08, /* 0xd8...0xdf */
    0xe0, 0x05, 0x08, 0xe0, 0x06, 0x08, 0xe0, 0x00, /* 0xe0...0xe7 */
    0x08, 0xe0, 0x00, 0x08, 0xe0, 0x06, 0x06, 0xe0, /* 0xe8...0xef */
    0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* 0xf0...0xf7 */
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  /* 0xf8...0xff */
};

static void twl4030_interrupt_update(TWL4030State *s)
{
    uint8_t x = 0;
    /* TODO: USB, BCI and GPIO interrupts */
    if (s->irq1) {
        /* KEYPAD */
        if (((s->i2c[2]->reg_data[0xda] & 0x10) && /* SIR_EN */
             s->i2c[2]->reg_data[0xe7]) ||         /* KEYP_SIR */
            (s->i2c[2]->reg_data[0xe3] &           /* KEYP_ISR1 */
             ~s->i2c[2]->reg_data[0xe4]))          /* KEYP_IMR1 */
            x |= 0x02;                             /* PIH_ISR1 */
        /* MADC */
        if (s->i2c[2]->reg_data[0x65] ||           /* MADC_SIR */
            (s->i2c[2]->reg_data[0x61] &           /* MADC_ISR1 */
             ~s->i2c[2]->reg_data[0x62]))          /* MADC_IMR1 */
            x |= 0x08;                             /* PIH_ISR3 */
        /* PM */
        if ((s->i2c[3]->reg_data[0x2e] &           /* PWR_ISR1 */
             ~s->i2c[3]->reg_data[0x2f]))          /* PWR_IMR1 */
            x |= 0x20;                             /* PIH_ISR5 */

        s->i2c[1]->reg_data[0x81] = x;             /* PIH_ISR_P1 */
        qemu_set_irq(s->irq1, x);
    }
    if (s->irq2) {
        /* KEYPAD */
        if (((s->i2c[2]->reg_data[0xda] & 0x10) && /* SIR_EN */
             s->i2c[2]->reg_data[0xe7]) ||         /* KEYP_SIR */
            (s->i2c[2]->reg_data[0xe5] &           /* KEYP_ISR2 */
             ~s->i2c[2]->reg_data[0xe6]))          /* KEYP_IMR2 */
            x |= 0x02;                             /* PIH_ISR1 */
        /* MADC */
        if (s->i2c[2]->reg_data[0x65] ||           /* MADC_SIR */
            (s->i2c[2]->reg_data[0x63] &           /* MADC_ISR2 */
             ~s->i2c[2]->reg_data[0x64]))          /* MADC_IMR2 */
            x |= 0x08;                             /* PIH_ISR3 */
        /* PM */
        if ((s->i2c[3]->reg_data[0x30] &           /* PWR_ISR2 */
             ~s->i2c[3]->reg_data[0x31]))          /* PWR_IMR2 */
            x |= 0x20;                             /* PIH_ISR5 */

        s->i2c[1]->reg_data[0x82] = x;             /* PIH_ISR_P2 */
        qemu_set_irq(s->irq2, x);
    }
}

static uint8_t twl4030_48_read(TWL4030NodeState *s, uint8_t addr)
{
    TRACE("addr=0x%02x", addr);
    switch (addr) {
        case 0x00: /* VENDOR_ID_LO */
        case 0x01: /* VENDOR_ID_HI */
        case 0x02: /* PRODUCT_ID_LO */
        case 0x03: /* PRODUCT_ID_HI */
            return s->reg_data[addr];
        case 0x04: /* FUNC_CTRL */
        case 0x05: /* FUNC_CRTL_SET */
        case 0x06: /* FUNC_CRTL_CLR */
            return s->reg_data[0x04];
        case 0x07: /* IFC_CTRL */
        case 0x08: /* IFC_CRTL_SET */
        case 0x09: /* IFC_CRTL_CLR */
            return s->reg_data[0x07];
        case 0x13: /* USB_INT_STS */
        case 0x16: /* SCRATCH_REG */
            return s->reg_data[addr];
        case 0xac: /* POWER_CTRL */
        case 0xad: /* POWER_SET */
        case 0xae: /* POWER_CLR */
            return s->reg_data[0xac];
        case 0xbb: /* CARKIT_AND_CTRL */
        case 0xbc: /* CARKIT_ANA_SET */
        case 0xbd: /* CARKIT_ANA_CLR */
            return s->reg_data[0xbb];
        case 0xfd: /* PHY_PWR_CTRL */
        case 0xfe: /* PHY_CLK_CTRL */
            return s->reg_data[addr];
        case 0xff: /* PHY_CLK_CTRL_STS */
            if (s->reg_data[0xfd] & 1) /* PHY_PWR_CTRL */
                return 0;
            if (s->reg_data[0xfe] & 1) /* REQ_PHY_DPLL_CLK */
                return 1;
            return (s->reg_data[0x04] >> 6) & 1; /* SUSPENDM */
        default:
            hw_error("%s: unknown register 0x%02x", __FUNCTION__, addr);
            break;
    }
    return 0;
}

static void twl4030_48_write(TWL4030NodeState *s, uint8_t addr, uint8_t value)
{
    TRACE("addr=0x%02x, value=0x%02x", addr, value);
    switch (addr) {
        case 0x04: /* FUNC_CTRL */
            s->reg_data[0x04] = value & 0x7f;
            break;
        case 0x05: /* FUNC_CRTL_SET */
            s->reg_data[0x04] = (s->reg_data[0x04] | value) & 0x7f;
            break;
        case 0x06: /* FUNC_CTRL_CLEAR */
            s->reg_data[0x04] = (s->reg_data[0x04] & ~value) & 0x7f;
            break;
        case 0x07: /* IFC_CTRL */
            s->reg_data[0x07] = value & 0x9e;
            break;
        case 0x08: /* IFC_CRTL_SET */
            s->reg_data[0x07] = (s->reg_data[0x07] | value) & 0x9e;
            break;
        case 0x09: /* IFC_CRTL_CLEAR */
            s->reg_data[0x07] = (s->reg_data[0x07] & ~value) & 0x9e;
            break;
        case 0x16: /* SCRATCH_REG */
            s->reg_data[0x16] = value;
            break;
        case 0xa1: /* CARKIT_SM_CTRL */
            s->reg_data[0xa1] = value & 0x3f;
            break;
        case 0xa2: /* CARKIT_SM_CTRL_SET */
            s->reg_data[0xa1] = (s->reg_data[0xa1] | value) & 0x3f;
            break;
        case 0xa3: /* CARKIT_SM_CTRL_CLR */
            s->reg_data[0xa1] = (s->reg_data[0xa1] & ~value) & 0x3f;
            break;
        case 0xac: /* POWER_CTRL */
            s->reg_data[0xac] = value & 0x20;
            break;
        case 0xad: /* POWER_SET */
            s->reg_data[0xac] = (s->reg_data[0xac] | value) & 0x20;
            break;
        case 0xae: /* POWER_CLEAR */
            s->reg_data[0xac] = (s->reg_data[0xac] & ~value) & 0x20;
            break;
        case 0xbb: /* CARKIT_ANA_CTRL */
            s->reg_data[0xbb] = value;
            break;
        case 0xbc: /* CARKIT_ANA_CTRL_SET */
            s->reg_data[0xbb] |= value;
            break;
        case 0xbd: /* CARKIT_ANA_CTRL_CLR */
            s->reg_data[0xbb] &= ~value;
            break;
        case 0xfd: /* PHY_PWR_CTRL */
            s->reg_data[addr] = value & 0x1;
            break;
        case 0xfe: /* PHY_CLK_CTRL */
            s->reg_data[addr] = value & 0x7;
            break;
        default:
            hw_error("%s: unknown register 0x%02x", __FUNCTION__, addr);
			break;
    }
}

static uint8_t twl4030_49_read(TWL4030NodeState *s, uint8_t addr)
{
    TRACE("addr=0x%02x", addr);
    switch (addr) {
        /* AUDIO_VOICE region */
        case 0x01 ... 0x49:
            return s->reg_data[addr];
        /* Test region */
        case 0x4c ... 0x60:
            return s->reg_data[addr];
        /* PIH region */
        case 0x81: /* PIH_ISR_P1 */
        case 0x82: /* PIH_ISR_P2 */
        case 0x83: /* PIH_SIR */
            return s->reg_data[addr];
        /* INTBR region */
        case 0x85 ... 0x90:
            if (s->reg_data[0x97] != 0x49) {
                return 0;
            }
            /* fallthrough */
        case 0x91 ... 0x97:
            return s->reg_data[addr];
        /* GPIO region */
        case 0x98 ... 0xc5:
            return s->reg_data[addr];
        default:
            hw_error("%s: unknown register 0x%02x", __FUNCTION__, addr);
			break;
    }
    return 0;
}

static void twl4030_49_write(TWL4030NodeState *s, uint8_t addr, uint8_t value)
{
    TRACE("addr=0x%02x, value=0x%02x", addr, value);
    switch (addr) {
        /* AUDIO_VOICE region */
        case 0x01 ... 0x49:
            s->reg_data[addr] = value;
            break;
        /* Test region */
        case 0x4c ... 0x59:
            s->reg_data[addr] = value;
            break;
        case 0x5a ... 0x60:
            /* read-only, ignore */
            break;
        /* PIH region */
        case 0x81: /* PIH_ISR_P1 */
        case 0x82: /* PIH_ISR_P2 */
        case 0x83: /* PIH_SIR */
            s->reg_data[addr] = value;
            twl4030_interrupt_update(s->twl4030);
            break;
        /* INTBR region */
        case 0x85 ... 0x90:
            /* read-only, ignore */
            break;
        case 0x91 ... 0x97:
            s->reg_data[addr] = value;
            break;
        /* GPIO region */
        case 0x98 ... 0x9a:
            /* read-only, ignore */
            break;
        case 0x9b ... 0xae:
            s->reg_data[addr] = value;
            break;
        case 0xaf: /* GPIOPUPDCTR5 */
            s->reg_data[addr] = value & 0x0f;
            break;
        case 0xb0 ... 0xb5:
            s->reg_data[addr] = value;
            break;
	    case 0xb6: /* GPIO_IMR3A */
            s->reg_data[addr] = value & 0x03;
            break;
        case 0xb7 ... 0xc4:
            s->reg_data[addr] = value;
            break;
	    case 0xc5: /* GPIO_SIH_CTRL */
            s->reg_data[addr] = value & 0x07;
            break;
        default:
            hw_error("%s: unknown register 0x%02x", __FUNCTION__, addr);
            break;
    }
}

static uint8_t twl4030_4a_read(TWL4030NodeState *s, uint8_t addr)
{
    static const uint8_t twl5031_aciid_data[] = {
        0x55, 0xaa, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    TRACE("addr=0x%02x", addr);
    switch (addr) {
        /* MADC region */
        case 0x00 ... 0x13:
        case 0x62:
        case 0x64 ... 0x67:
            return s->reg_data[addr];
        case 0x17 ... 0x36: /* RT conversion result */
            if (s->twl4030->madc_cb) {
                uint16_t x = s->twl4030->madc_cb(TWL4030_ADC_RT,
                                                 (addr - 0x17) >> 1);
                return (addr & 1) ? (uint8_t)((x & 3) << 6)
                                  : (uint8_t)((x >> 2) & 0xff);
            }
            return s->reg_data[addr];
        case 0x37 ... 0x56: /* GP conversion result */
            if (s->twl4030->madc_cb) {
                uint16_t x = s->twl4030->madc_cb(TWL4030_ADC_GP,
                                                 (addr - 0x37) >> 1);
                return (addr & 1) ? (uint8_t)((x & 3) << 6)
                                  : (uint8_t)((x >> 2) & 0xff);
            }
            return s->reg_data[addr];
        case 0x57 ... 0x60: /* BCI conversion result */
            if (s->twl4030->madc_cb) {
                uint16_t x = s->twl4030->madc_cb(TWL4030_ADC_BCI,
                                                 (addr - 0x57) >> 1);
                return (addr & 1) ? (uint8_t)((x & 3) << 6)
                                  : (uint8_t)((x >> 2) & 0xff);
            }
            return s->reg_data[addr];
        case 0x61: /* MADC_ISR1 */
        case 0x63: /* MADC_ISR2 */
            {
                uint8_t data = s->reg_data[addr];
                if (s->reg_data[0x67] & 0x04) { /* COR */
                    s->reg_data[addr] = 0x00;
                    twl4030_interrupt_update(s->twl4030);
                }
                return data;
            }
        /* MAIN_CHARGE(TWL4030) / ACCESSORY(TWL5031) region */
        case 0x74 ... 0xa9:
            if (s->twl4030->twl5031) {
                switch (addr) {
                    case 0x74: /* ACIID */
                        if (s->twl4030->twl5031_aciid >=
                            sizeof(twl5031_aciid_data)) {
                            s->twl4030->twl5031_aciid = 0;
                        }
                        return twl5031_aciid_data[s->twl4030->twl5031_aciid++];
                    case 0x79: /* ACIIMR_LSB */
                    case 0x7a: /* ACIIMR_MSB */
                    case 0x7b: /* ACIIDR_LSB */
                    case 0x7c: /* ACIIDR_MSB */
                    case 0x80: /* AV_CTRL */
                    case 0x82: /* BCIA_CTRL */
                    case 0x83: /* ACCISR1 */
                        return s->reg_data[addr];
                    default:
                        hw_error("%s: unknown twl5031 register 0x%02x",
                                 __FUNCTION__, addr);
                        break;
                }
            }
            return s->reg_data[addr];
        /* PRECHARGE region */
        case 0xaa ... 0xb8:
            return s->reg_data[addr];
        /* Interrupt region */
        case 0xb9 ... 0xc6:
            return s->reg_data[addr];
        /* KEYPAD region */
        case 0xd2 ... 0xe2:
        case 0xe4:
        case 0xe6 ... 0xe9:
            return s->reg_data[addr];
        case 0xe3: /* KEYP_ISR1 */
        case 0xe5: /* KEYP_ISR2 */
            {
                uint8_t data = s->reg_data[addr];
                if (s->reg_data[0xe9] & 0x04) { /* COR */
                    s->reg_data[addr] = 0x00;
                    twl4030_interrupt_update(s->twl4030);
                }
                return data;
            }
        /* LED region */
        case 0xee: /* LEDEN */
            return s->reg_data[addr];
        /* PWMA region */
        case 0xef: /* PWMAON */
        case 0xf0: /* PWMAOFF */
            return s->reg_data[addr];
        /* PWMB region */
        case 0xf1: /* PWMBON */
        case 0xf2: /* PWMBOFF */
            return s->reg_data[addr];
        /* PWM0 region */
        case 0xf8: /* PWM0ON */
        case 0xf9: /* PWM0OFF */
            return s->reg_data[addr];
        /* PWM1 region */
        case 0xfb: /* PWM1ON */
        case 0xfc: /* PWM1OFF */
            return s->reg_data[addr];
        default:
	        hw_error("%s: unknown register 0x%02x", __FUNCTION__, addr);
            break;
    }
    return 0;
}

static void twl4030_4a_write(TWL4030NodeState *s, uint8_t addr, uint8_t value)
{
    TRACE("addr=0x%02x, value=0x%02x", addr, value);
    switch (addr) {
        /* MADC region */

        case 0x00: /* CTRL1 */
        case 0x01: /* CTRL2 */
            s->reg_data[addr] = value;
            break;
        case 0x06: /* SW1SELECT_LSB */
        case 0x07: /* SW1SELECT_MSB */
        case 0x08: /* SW1AVERAGE_LSB */
        case 0x09: /* SW1AVERAGE_MSB */
        case 0x0a: /* SW2SELECT_LSB */
        case 0x0b: /* SW2SELECT_MSB */
        case 0x0c: /* SW2AVERAGE_LSB */
        case 0x0d: /* SW2AVERAGE_MSB */
            s->reg_data[addr] = value;
            break;
        case 0x12: /* CTRL_SW1 */
        case 0x13: /* CTRL_SW2 */
            /* always set all conversions ready, not busy */
            s->reg_data[addr] = 0x3e;
            if (value & 0x20) { /* SW1/SW2 */
                s->reg_data[0x61] |= 2 << (addr - 0x12); /* SW1_ISR/SW2_ISR */
                s->reg_data[0x63] |= 2 << (addr - 0x12); /* SW1_ISR/SW2_ISR */
                twl4030_interrupt_update(s->twl4030);
            }
            break;
        case 0x17 ... 0x60: /* conversion results */
            /* read-only, ignore */
            break;
        case 0x61: /* MADC_ISR1 */
        case 0x63: /* MADC_ISR2 */
            if (!(s->reg_data[0x67] & 0x04)) { /* COR */
                s->reg_data[addr] &= ~(value & 0x0f);
                twl4030_interrupt_update(s->twl4030);
            }
            break;
        case 0x62: /* MADC_IMR1 */
        case 0x64: /* MADC_IMR2 */
        case 0x65: /* MADC_SIR */
            s->reg_data[addr] = value & 0x0f;
            twl4030_interrupt_update(s->twl4030);
            break;
        case 0x66: /* MADC_EDR */
            s->reg_data[addr] = value;
            break;
        case 0x67: /* MADC_SIH_CTRL */
            s->reg_data[addr] = value & 0x07;
            break;

        /* MAIN_CHARGE(TWL4030) / ACCESSORY(TWL5031) region */

        case 0x74: /* BCIMDEN(TWL4030) / ACIID(TWL5031) */
            if (s->twl4030->twl5031) {
                s->twl4030->twl5031_aciid = 0;
            } else {
                /* read-only */
            }
            break;
        case 0x75: /* BCIMDKEY(TWL4030) / ACICOMR_LSB(TWL5031) */
            s->reg_data[addr] = value;
            if (!s->twl4030->twl5031) {
                switch (value) {
                    case 0x25: s->reg_data[0x74] = 0x12; break;
                    case 0x26: s->reg_data[0x74] = 0x11; break;
                    case 0x27: s->reg_data[0x74] = 0x0a; break;
                    case 0x28: s->reg_data[0x74] = 0x06; break;
                    case 0x29: s->reg_data[0x74] = 0x05; break;
                    default: s->reg_data[0x74] = 0; break;
                }
            }
            break;
        case 0x76 ... 0x84:
            if (s->twl4030->twl5031) {
                switch (addr) {
                    case 0x79: /* ACIIMR_LSB */
                        s->reg_data[addr] = value;
                        twl4030_interrupt_update(s->twl4030);
                        break;
                    case 0x7a: /* ACIIMR_MSB */
                        s->reg_data[addr] = value & 0x01;
                        twl4030_interrupt_update(s->twl4030);
                        break;
                    case 0x7b: /* ACIIDR_LSB */
                        s->reg_data[addr] &= ~value;
                        twl4030_interrupt_update(s->twl4030);
                        break;
                    case 0x7c: /* ACIIDR_MSB */
                        s->reg_data[addr] &= ~(value & 0x01);
                        twl4030_interrupt_update(s->twl4030);
                        break;
                    case 0x7f: /* ECI_DBI_CTRL */
                        s->reg_data[addr] = value;
                        twl4030_interrupt_update(s->twl4030);
                        break;
                    case 0x80: /* ACI_AV_CTRL */
                        s->reg_data[addr] = (s->reg_data[addr] & 0x18) |
                                            (value & ~0x18);
                        twl4030_interrupt_update(s->twl4030);
                        break;
                    case 0x82: /* BCIA_CTRL */
                        s->reg_data[addr] = value & 0x07;
                        break;
                    case 0x83: /* ACCISR1 */
                        s->reg_data[addr] &= ~(value & 0x03);
                        twl4030_interrupt_update(s->twl4030);
                        break;
                    case 0x84: /* ACCIMR1 */
                        s->reg_data[addr] = value & 0x03;
                        twl4030_interrupt_update(s->twl4030);
                        break;
                    default:
                        hw_error("%s: unknown twl5031 register 0x%02x",
                                 __FUNCTION__, addr);
                        break;
                }
            } else {
                /* read-only registers */
            }
            break;
        case 0x97: /* BCICTL1 */
            if (!s->twl4030->twl5031) {
                s->reg_data[addr] = value;
            } else {
                hw_error("%s: unknown twl5031 register 0x%02x",
                         __FUNCTION__, addr);
            }
            break;

        /* PRECHARGE region */

        case 0xaa ... 0xb8: /* FIXME: unknown registers */
            s->reg_data[addr] = value;
            break;

        /* Interrupt region */

        case 0xb9: /* BCIISR1A */
            s->reg_data[addr] &= ~value;
            break;
        case 0xba: /* BCIISR2A */
            s->reg_data[addr] &= ~(value & 0x0f);
            break;
        case 0xbb: /* BCIIMR1A */
            s->reg_data[addr] = value;
            break;
        case 0xbc: /* BCIIMR2A */
            s->reg_data[addr] = value & 0x0f;
            break;
        case 0xc6: /* BCISIHCTRL */
            s->reg_data[addr] = value & 0x07;
            break;

        /* KEYPAD region */

        case 0xd2: /* KEYP_CTRL_REG */
            s->reg_data[addr] = value & 0x7f;
            break;
        case 0xd3: /* KEYP_DEB_REG */
            s->reg_data[addr] = value & 0x3f;
            break;
        case 0xd5: /* LK_PTV_REG */
            s->reg_data[addr] = value & 0xef;
            break;
        case 0xda: /* KEYP_SMS */
            s->reg_data[addr] = (s->reg_data[addr] & ~0x30) | (value & 0x30);
            twl4030_interrupt_update(s->twl4030);
            break;
        case 0xe3: /* KEYP_ISR1 */
        case 0xe5: /* KEYP_ISR2 */
            if (!(s->reg_data[0xe9] & 0x04)) { /* COR */
                s->reg_data[addr] &= ~value;
                twl4030_interrupt_update(s->twl4030);
            }
            break;
        case 0xe4: /* KEYP_IMR1 */
        case 0xe6: /* KEYP_IMR2 */
        case 0xe7: /* KEYP_SIR */
            s->reg_data[addr] = value & 0x0f;
            twl4030_interrupt_update(s->twl4030);
            break;
        case 0xe9: /* KEYP_SIH_CTRL */
            s->reg_data[addr] = value & 0x07;
            break;
        case 0xd4: /* LONG_KEY_REG1 */
        case 0xd6: /* TIME_OUT_REG1 */
        case 0xd7: /* TIME_OUT_REG2 */
        case 0xd8: /* KBC_REG */
        case 0xe8: /* KEYP_EDR */
            s->reg_data[addr] = value;
            break;
        case 0xd9: /* KBR_REG */
        case 0xdb ... 0xe2: /* FULL_CODE_xx_yy */
            /* read-only, ignore */
            break;

        /* LED region */

        case 0xee: /* LEDEN */
            s->reg_data[addr] = value;
            TRACE("LEDA power=%s/enable=%s, LEDB power=%s/enable=%s",
                    value & 0x10 ? "on" : "off", value & 0x01 ? "yes" : "no",
                    value & 0x20 ? "on" : "off", value & 0x02 ? "yes" : "no");
            break;

        /* PWMA/B/0/1 regions */

        case 0xef: /* PWMAON */
        case 0xf1: /* PWMBON */
        case 0xf8: /* PWM0ON */
        case 0xfb: /* PWM1ON */
            s->reg_data[addr] = value;
            break;
        case 0xf0: /* PWMAOFF */
        case 0xf2: /* PWMBOFF */
        case 0xf9: /* PWM0OFF */
        case 0xfc: /* PWM1OFF */
            s->reg_data[addr] = value & 0x7f;
            break;

        default:
	        hw_error("%s: unknown register 0x%02x", __FUNCTION__, addr);
            break;
    }
}

static inline struct tm *twl4030_gettime(void)
{
    time_t epoch_time = time(NULL);
    return gmtime(&epoch_time);//localtime(&epoch_time);
}

static uint8_t twl4030_4b_read(TWL4030NodeState *s, uint8_t addr)
{
    uint8_t x;
	TRACE("addr=0x%02x value=0x%02x", addr, s->reg_data[addr]);
    switch (addr) {
        /* SECURED_REG region */
        case 0x00 ... 0x13:
            return s->reg_data[addr];
        /* BACKUP_REG region */
        case 0x14 ... 0x1b:
            return s->reg_data[addr];
        /* RTC region */
        case 0x1c: /* SECONDS_REG */
            x = s->reg_data[addr];
            if (x == 0xff) {
                struct tm *t = twl4030_gettime();
                x = ((t->tm_sec / 10) << 4) | (t->tm_sec % 10);
            } else {
                s->reg_data[addr] = 0xff;
            }
            TRACE_RTC("SECONDS_REG returns 0x%02x", x);
            return x;
        case 0x1d: /* MINUTES_REG */
            x = s->reg_data[addr];
            if (x == 0xff) {
                struct tm *t = twl4030_gettime();
                x = ((t->tm_min / 10) << 4) | (t->tm_min % 10);
            } else {
                s->reg_data[addr] = 0xff;
            }
            TRACE_RTC("MINUTES_REG returns 0x%02x", x);
            return x;
        case 0x1e: /* HOURS_REG */
            x = s->reg_data[addr];
            if (x == 0xff) {
                struct tm *t = twl4030_gettime();
                if (s->reg_data[0x29] & 0x08) { /* MODE_12_24 */
                    int h12 = t->tm_hour;
                    if (h12 > 11) {
                        h12 -= 12;
                        x = ((h12 / 10) << 4) | (h12 % 10) | 0x80; /* PM_NAM */
                    } else {
                        x = ((h12 / 10) << 4) | (h12 % 10);
                    }
                } else {
                    x = ((t->tm_hour / 10) << 4) | (t->tm_hour % 10);
                }
            } else {
                s->reg_data[addr] = 0xff;
            }
            TRACE_RTC("HOURS_REG returns 0x%02x", x);
            return x;
        case 0x1f: /* DAYS_REG */
            x = s->reg_data[addr];
            if (x == 0xff) {
                struct tm *t = twl4030_gettime();
                x = ((t->tm_mday / 10) << 4) | (t->tm_mday % 10);
            } else {
                s->reg_data[addr] = 0xff;
            }
            TRACE_RTC("DAYS_REG returns 0x%02x", x);
            return x;
        case 0x20: /* MONTHS_REG */
            x = s->reg_data[addr];
            if (x == 0xff) {
                struct tm *t = twl4030_gettime();
                x = (((t->tm_mon + 1) / 10) << 4) | ((t->tm_mon + 1) % 10);
            } else {
                s->reg_data[addr] = 0xff;
            }
            TRACE_RTC("MONTHS_REG returns 0x%02x", x);
            return x;
        case 0x21: /* YEARS_REG */
            x = s->reg_data[addr];
            if (x == 0xff) {
                struct tm *t = twl4030_gettime();
                x = (((t->tm_year % 100) / 10) << 4) | (t->tm_year % 10);
            } else {
                s->reg_data[addr] = 0xff;
            }
            TRACE_RTC("YEARS_REG returns 0x%02x", x);
            return x;
        case 0x22: /* WEEKS_REG */
            x = s->reg_data[addr];
            if (x == 0xff) {
                struct tm *t = twl4030_gettime();
                x = t->tm_wday;
            } else {
                s->reg_data[addr] = 0xff;
            }
            TRACE_RTC("WEEKS_REG returns 0x%02x", x);
            return x;
        case 0x23: /* ALARM_SECONDS_REG */
            x = s->reg_data[addr];
            TRACE_RTC("ALARM_SECONDS_REG returns 0x%02x", x);
            return x;
        case 0x24: /* ALARM_MINUTES_REG */
            x = s->reg_data[addr];
            TRACE_RTC("ALARM_MINUTES_REG returns 0x%02x", x);
            return x;
        case 0x25: /* ALARM_HOURS_REG */
            x = s->reg_data[addr];
            TRACE_RTC("ALARM_HOURS_REG returns 0x%02x", x);
            return x;
        case 0x26: /* ALARM_DAYS_REG */
            x = s->reg_data[addr];
            TRACE_RTC("ALARM_DAYS_REG returns 0x%02x", x);
            return x;
        case 0x27: /* ALARM_MONTHS_REG */
            x = s->reg_data[addr];
            TRACE_RTC("ALARM_MONTHS_REG returns 0x%02x", x);
            return x;
        case 0x28: /* ALARM_YEARS_REG */
            x = s->reg_data[addr];
            TRACE_RTC("ALARM_YEARS_REG returns 0x%02x", x);
            return x;
        case 0x29: /* RTC_CTRL_REG */
            x = s->reg_data[addr];
            TRACE_RTC("RTC_CTRL_REG returns 0x%02x", x);
            return x;
        case 0x2a: /* RTC_STATUS_REG */
            x = s->reg_data[addr];
            TRACE_RTC("RTC_STATUS_REG returns 0x%02x", x);
            return x;
        case 0x2b: /* RTC_INTERRUPTS_REG */
            x = s->reg_data[addr];
            TRACE_RTC("RTC_INTERRUPTS_REG returns 0x%02x", x);
            return x;
        case 0x2c: /* RTC_COMP_LSB_REG */
            x = s->reg_data[addr];
            TRACE_RTC("RTC_COMP_LSB_REG returns 0x%02x", x);
            return x;
        case 0x2d: /* RTC_COMP_MSB_REG */
            x = s->reg_data[addr];
            TRACE_RTC("RTC_CTRL_REG returns 0x%02x", x);
            return x;
        /* INT region */
        case 0x2f:
        case 0x31 ... 0x35:
            return s->reg_data[addr];
        case 0x2e: /* PWR_ISR1 */
        case 0x30: /* PWR_ISR2 */
            {
                uint8_t data = s->reg_data[addr];
                if (s->reg_data[0x35] & 0x04) { /* COR */
                    s->reg_data[addr] = 0x00;
                    twl4030_interrupt_update(s->twl4030);
                }
                return data;
            }
        /* PM_MASTER region */
        case 0x36 ... 0x44:
            return s->reg_data[addr];
        case 0x45: /* STS_HW_CONDITIONS */
            /* FIXME: force USB always connected, no VBUS (host usb) */
            return (s->reg_data[addr] & ~0x80) | 0x04;
        case 0x46 ... 0x5a:
            return s->reg_data[addr];
        /* PM_RECEIVER region */
        case 0x5b ... 0xf1:
            return s->reg_data[addr];
        default:
	        hw_error("%s: unknown register 0x%02x", __FUNCTION__, addr);
            break;
    }
    return 0;
}

static void twl4030_setup_alarm(TWL4030NodeState *s)
{
    if (s->reg_data[0x2b] & 0x08) { /* IT_ALARM */
        struct tm a = {
            .tm_sec = ((s->reg_data[0x23] >> 4) & 7) * 10
                      + (s->reg_data[0x23] & 0x0f),
            .tm_min = ((s->reg_data[0x24] >> 4) & 7) * 10
                      + (s->reg_data[0x24] & 0x0f),
            .tm_hour = ((s->reg_data[0x29] & 0x08)
                        ? (s->reg_data[0x25] >> 7) * 12 : 0)
                       + ((s->reg_data[0x25] >> 4) & 3) * 10
                       + (s->reg_data[0x25] & 0x0f),
            .tm_mday = ((s->reg_data[0x26] >> 4) & 3) * 10
                       + (s->reg_data[0x26] & 0x0f),
            .tm_mon = ((s->reg_data[0x27] >> 4) & 1) * 10
                      + (s->reg_data[0x27] & 0x0f)
                      - 1,
            .tm_year = (s->reg_data[0x28] >> 4) * 10
                       + (s->reg_data[0x28] & 0x0f)
                       + 100,
            .tm_isdst = -1,
        };
        TRACE_RTC("enable alarm on %02d/%02d/%04d at %02d:%02d:%02d (UTC)",
                  a.tm_mday, a.tm_mon + 1, a.tm_year + 1900,
                  a.tm_hour, a.tm_min, a.tm_sec);
        time_t at = mktime(&a); /* alarm time interpreted in local time */
        if (at < 0) {
            TRACE_RTC("unable to parse alarm calendar time");
        } else {
            /* fix alarm time to utc */
            struct timezone tz;
            struct timeval tv;
            if (!gettimeofday(&tv, &tz)) {
                at -= tz.tz_minuteswest * 60;
            }
            int64_t delta = (int64_t)difftime(at, time(NULL));
            if (delta <= 0) {
                TRACE_RTC("alarm is in the past");
            } else {
                TRACE_RTC("new alarm interrupt in %" PRId64 " seconds", delta);
                qemu_mod_timer(s->twl4030->alarm_timer,
                               qemu_get_clock_ns(vm_clock)
                               + get_ticks_per_sec() * delta);
            }
        }
    } else {
        qemu_del_timer(s->twl4030->alarm_timer);
    }
}

static void twl4030_setup_periodic(TWL4030NodeState *s)
{
    if (s->reg_data[0x2b] & 0x04) { /* IT_TIMER */
        uint32_t t = 0;
        switch (s->reg_data[0x2b] & 3) {
            case 0: t = 1; break;
            case 1: t = 60; break;
            case 2: t = 60 * 60; break;
            case 3: t = 24 * 60 * 60; break;
        }
        TRACE_RTC("new periodic interrupt in %u seconds", t);
        qemu_mod_timer(s->twl4030->periodic_timer,
                       qemu_get_clock_ns(vm_clock) + get_ticks_per_sec() * t);
    } else {
        qemu_del_timer(s->twl4030->periodic_timer);
    }
}

static void twl4030_alarm(void *opaque)
{
    TWL4030State *s = opaque;
    s->i2c[3]->reg_data[0x2a] |= 0x40;      /* RTC_STATUS_REG |= ALARM */
    if (s->i2c[3]->reg_data[0x33] & 0xc0) { /* RTC_IT_RISING|RTC_IT_FALLING */
        TRACE_RTC("triggering RTC alarm interrupt");
        s->i2c[3]->reg_data[0x2e] |= 0x08;  /* PWR_ISR1 |= RTC_IT */
        s->i2c[3]->reg_data[0x30] |= 0x08;  /* PWR_ISR2 |= RTC_IT */
        twl4030_interrupt_update(s);
    }
    qemu_del_timer(s->alarm_timer);
}

static void twl4030_periodic(void *opaque)
{
    TWL4030State *s = opaque;
    uint8_t b = 0x04 << (s->i2c[3]->reg_data[0x2b] & 3);
    s->i2c[3]->reg_data[0x2a] |= b;         /* TODO: when are these cleared? */
    if (s->i2c[3]->reg_data[0x33] & 0xc0) { /* RTC_IT_RISING|RTC_IT_FALLING */
        TRACE_RTC("triggering RTC periodic interrupt");
        s->i2c[3]->reg_data[0x2e] |= 0x08;  /* PWR_ISR1 |= RTC_IT */
        s->i2c[3]->reg_data[0x30] |= 0x08;  /* PWR_ISR2 |= RTC_IT */
        twl4030_interrupt_update(s);
    }
    twl4030_setup_periodic(s->i2c[3]);
}

static void twl4030_4b_write(TWL4030NodeState *s, uint8_t addr, uint8_t value)
{
    uint8_t seq_addr, seq_sub;

	TRACE("addr=0x%02x, value=0x%02x", addr, value);
    switch (addr) {
        case 0x1c: /* SECONDS_REG */
            TRACE_RTC("SECONDS_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0x7f;
            break;
        case 0x1d: /* MINUTES_REG */
            TRACE_RTC("MINUTES_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0x7f;
            break;
        case 0x1e: /* HOURS_REG */
            TRACE_RTC("HOURS_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0xbf;
            break;
        case 0x1f: /* DAYS_REG */
            TRACE_RTC("DAYS_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0x3f;
            break;
        case 0x20: /* MONTHS_REG */
            TRACE_RTC("MONTHS_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0x1f;
            break;
        case 0x21: /* YEARS_REG */
            TRACE_RTC("YEARS_REG = 0x%02x", value);
            s->reg_data[addr] = value;
            break;
        case 0x22: /* WEEKS_REG */
            TRACE_RTC("WEEKS_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0x07;
            break;
        case 0x23: /* ALARM_SECONDS_REG */
            TRACE_RTC("ALARM_SECONDS_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0x7f;
            twl4030_setup_alarm(s);
            break;
        case 0x24: /* ALARM_MINUTES_REG */
            TRACE_RTC("ALARM_MINUTES_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0x7f;
            twl4030_setup_alarm(s);
            break;
        case 0x25: /* ALARM_HOURS_REG */
            TRACE_RTC("ALARM_HOURS_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0xbf;
            twl4030_setup_alarm(s);
            break;
        case 0x26: /* ALARM_DAYS_REG */
            TRACE_RTC("ALARM_DAYS_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0x3f;
            twl4030_setup_alarm(s);
            break;
        case 0x27: /* ALARM_MONTHS_REG */
            TRACE_RTC("ALARM_MONTHS_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0x1f;
            twl4030_setup_alarm(s);
            break;
        case 0x28: /* ALARM_YEARS_REG */
            TRACE_RTC("ALARM_YEARS_REG = 0x%02x", value);
            s->reg_data[addr] = value;
            twl4030_setup_alarm(s);
            break;
        case 0x29: /* RTC_CTRL_REG */
            TRACE_RTC("RTC_CTRL_REG = 0x%02x", value);
            s->reg_data[addr] = value & 0x3f;
            s->reg_data[0x2a] = (s->reg_data[0x2a] & ~0x02) |
                                ((value & 0x01) << 1);
            if (value & 0x40) { /* GET_TIME */
                struct tm *t = twl4030_gettime();
                s->reg_data[0x1c] = ((t->tm_sec / 10) << 4) | (t->tm_sec % 10);
                s->reg_data[0x1d] = ((t->tm_min / 10) << 4) | (t->tm_min % 10);
                if (value & 0x08) { /* MODE_12_24 */
                    int h12 = t->tm_hour;
                    /* TODO: should we report hours 0-11 or 1-12? */
                    if (h12 > 11) {
                        h12 -= 12;
                        s->reg_data[0x1e] = ((h12 / 10) << 4) | (h12 % 10) |
                                            0x80; /* PM_NAM */
                    } else {
                        s->reg_data[0x1e] = ((h12 / 10) << 4) | (h12 % 10);
                    }
                } else {
                    s->reg_data[0x1e] = ((t->tm_hour / 10) << 4) |
                                        (t->tm_hour % 10);
                }
                s->reg_data[0x1f] = ((t->tm_mday / 10) << 4) |
                                    (t->tm_mday % 10);
                s->reg_data[0x20] = (((t->tm_mon + 1) / 10) << 4) |
                                    ((t->tm_mon + 1) % 10);
                s->reg_data[0x21] = (((t->tm_year % 100) / 10) << 4) |
                                    (t->tm_year % 10);
                s->reg_data[0x22] = t->tm_wday;
            }
            /* TODO: support bits 1, 2, 4 and 5 */
            break;
        case 0x2a: /* RTC_STATUS_REG */
            TRACE_RTC("RTC_STATUS_REG = 0x%02x", value);
            s->reg_data[addr] &= ~(value & 0xc0);
            break;
        case 0x2b: /* RTC_INTERRUPTS_REG */
            TRACE_RTC("RTC_INTERRUPTS_REG = 0x%02x", value);
            {
                uint8_t change = s->reg_data[addr] ^ value;
                s->reg_data[addr] = value & 0x0f;
                if (change & 0x08) { /* IT_ALARM */
                    twl4030_setup_alarm(s);
                }
                if (change & 0x04) { /* IT_TIMER */
                    twl4030_setup_periodic(s);
                }
            }
            break;
        case 0x2c: /* RTC_COMP_LSB_REG */
        case 0x2d: /* RTC_COMP_MSB_REG */
            TRACE_RTC("RTC_COMP_%s_REG = 0x%02x",
                      (addr == 0x2c) ? "LSB" : "MSB", value);
            s->reg_data[addr] = value;
            break;
        case 0x2e: /* PWR_ISR1 */
        case 0x30: /* PWR_ISR2 */
            if (!(s->reg_data[0x35] & 0x04)) { /* COR */
                s->reg_data[addr] &= ~value;
                twl4030_interrupt_update(s->twl4030);
            }
            break;
        case 0x2f: /* PWR_IMR1 */
        case 0x31: /* PWR_IMR2 */
            s->reg_data[addr] = value;
            twl4030_interrupt_update(s->twl4030);
            break;
        case 0x33: /* PWR_EDR1 */
        case 0x34: /* PWR_EDR2 */
            s->reg_data[addr] = value;
            break;
        case 0x35: /* PWR_SIH_CTRL */
            s->reg_data[addr] = value & 0x07;
            break;
        case 0x36: /* CFG_P1_TRANSITION */
        case 0x37: /* CFG_P2_TRANSITION */
        case 0x38: /* CFG_P3_TRANSITION */
            if (s->twl4030->key_cfg)
                s->reg_data[addr] = (s->reg_data[addr] & 0x40) | (value & 0xbf);
            break;
        case 0x39: /* CFG_P123_TRANSITION */
            if (s->twl4030->key_cfg)
                s->reg_data[addr] = value;
            break;
        case 0x3a: /* STS_BOOT */
            s->reg_data[addr] = value;
            break;
        case 0x3b: /* CFG_BOOT */
            if (s->twl4030->key_cfg)
                s->reg_data[addr] = (s->reg_data[addr] & 0x70) | (value & 0x8f);
            break;
        case 0x3c: /* SHUNDAN */
            s->reg_data[addr] = value & 0x3f;
            break;
        case 0x3d: /* BOOT_BCI */
            s->reg_data[addr] = (s->reg_data[addr] & 0x20) | (value & 0x17);
            break;
        case 0x3e: /* CFG_PWRANA1 */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value & 0x7f;
            break;
        case 0x3f: /* CFG_PWRANA2 */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value;
            break;
        case 0x44: /* PROTECT_KEY */
            s->twl4030->key_cfg = 0;
            s->twl4030->key_tst = 0;
            switch (value) {
                case 0x0C:
                    if (s->reg_data[addr] == 0xC0)
                        s->twl4030->key_cfg = 1;
                    break;
                case 0xE0:
                    if (s->reg_data[addr] == 0x0E)
                        s->twl4030->key_tst = 1;
                    break;
                case 0xEC:
                    if (s->reg_data[addr] == 0xCE) {
                        s->twl4030->key_cfg = 1;
                        s->twl4030->key_tst = 1;
                    }
                    break;
                default:
                    break;
            }
            s->reg_data[addr] = value;
            break;
        case 0x46: /* P1_SW_EVENTS */
        case 0x47: /* P2_SW_EVENTS */
        case 0x48: /* P3_SW_EVENTS */
            s->reg_data[addr] = value & 0x78;
            if (value & 0x01) { /* DEVOFF */
                TRACE("device power off sequence requested");
                qemu_system_shutdown_request();
            }
            break;
        case 0x4a: /* PB_CFG */
            s->reg_data[addr] = value & 0xf;
            break;
        case 0x4b: /* PB_MSB */
        case 0x4c: /* PB_LSB */
            s->reg_data[addr] = value;
            break;
        case 0x52: /* SEQ_ADD_W2P */
        case 0x53: /* SEQ_ADD_P2A */
        case 0x54: /* SEQ_ADD_A2W */
        case 0x55: /* SEQ_ADD_A2S */
        case 0x56: /* SEQ_ADD_S2A12 */
        case 0x57: /* SEQ_ADD_S2A3 */
        case 0x58: /* SEQ_ADD_WARM */
            if (s->twl4030->key_cfg)
                s->reg_data[addr] = value & 0x3f;
            break;
        case 0x59: /* MEMORY_ADDRESS */
            if (s->twl4030->key_cfg)
                s->reg_data[addr] = value;
            break;
        case 0x5a: /* MEMORY_DATA */
            if (s->twl4030->key_cfg) {
                s->reg_data[addr] = value;
                seq_addr = s->reg_data[0x59];
                seq_sub = seq_addr & 3;
                seq_addr >>= 2;
                if ((seq_addr >= 0x2b && seq_addr <= 0x3e) ||
                    (seq_addr <= 0x0e && seq_sub == 3))
                    s->twl4030->seq_mem[seq_addr][seq_sub] = value;
            }
            /* TODO: check if autoincrement is write-protected as well */
            s->reg_data[0x59]++;
            break;
        case 0x5e: /* WATCHDOG_CFG */
        case 0x60: /* VIBRATOR_CFG */
        case 0x6d: /* BB_CFG */
        case 0x73: /* VAUX1_TYPE */
        case 0x77: /* VAUX2_TYPE */
        case 0x7b: /* VAUX3_TYPE */
        case 0x7f: /* VAUX4_TYPE */
        case 0x83: /* VMMC1_TYPE */
        case 0x87: /* VMMC2_TYPE */
        case 0x8b: /* VPLL1_TYPE */
        case 0x8f: /* VPLL2_TYPE */
        case 0x93: /* VSIM_TYPE */
        case 0x97: /* VDAC_TYPE */
        case 0x9b: /* VINTANA1_TYPE */
        case 0x9f: /* VINTANA2_TYPE */
        case 0xa3: /* VINTDIG_TYPE */
        case 0xa7: /* VIO_TYPE */
        case 0xaa: /* VIO_MISC_CFG */
        case 0xb1: /* VDD1_TYPE */
        case 0xb4: /* VDD1_MISC_CFG */
        case 0xbd: /* VDD1_STEP */
        case 0xbf: /* VDD2_TYPE */
        case 0xc2: /* VDD2_MISC_CFG */
        case 0xcb: /* VDD2_STEP */
        case 0xcd: /* VUSB1V5_TYPE */
        case 0xd0: /* VUSB1V8_TYPE */
        case 0xd3: /* VUSB3V1_TYPE */
        case 0xdb: /* REGEN_TYPE */
        case 0xde: /* NRESPWRON_TYPE */
        case 0xe1: /* CLKEN_TYPE */
        case 0xe4: /* SYSEN_TYPE */
        case 0xe7: /* HFCLKOUT_TYPE */
        case 0xea: /* 2KCLKOUT_TYPE */
        case 0xed: /* TRITON_RESET_TYPE */
        case 0xf0: /* MAINREF_TYPE */
            s->reg_data[addr] = value & 0x1f;
            break;
        case 0x5f: /* IT_CHECK_CFG */
        case 0xb9: /* VDD1_VSEL */
        case 0xbb: /* VDD1_VFLOOR */
        case 0xbc: /* VDD1_VROOF */
        case 0xc7: /* VDD2_VSEL */
        case 0xc9: /* VDD2_VFLOOR */
        case 0xca: /* VDD2_VROOF */
            s->reg_data[addr] = value & 0x7f;
            break;
        case 0x61: /* DC/DC_GLOBAL_CFG */
        case 0x68: /* MISC_CFG */
            s->reg_data[addr] = value;
            break;
        case 0x62: /* VDD1_TRIM1 */
        case 0x64: /* VDD2_TRIM1 */
        case 0x66: /* VIO_TRIM1 */
        case 0xac: /* VIO_TEST2 */
        case 0xb6: /* VDD1_TEST2 */
        case 0xc4: /* VDD2_TEST2 */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value & 0x7f;
            break;
        case 0x63: /* VDD1_TRIM2 */
        case 0x65: /* VDD2_TRIM2 */
        case 0x67: /* VIO_TRIM2 */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value & 0x3f;
            break;
        case 0x72: /* VAUX1_DEV_GRP */
        case 0x76: /* VAUX2_DEV_GRP */
        case 0x7a: /* VAUX3_DEV_GRP */
        case 0x7e: /* VAUX4_DEV_GRP */
        case 0x82: /* VMMC1_DEV_GRP */
        case 0x86: /* VMMC2_DEV_GRP */
        case 0x8a: /* VPLL1_DEV_GRP */
        case 0x8e: /* VPLL2_DEV_GRP */
        case 0x92: /* VSIM_DEV_GRP */
        case 0x96: /* VDAC_DEV_GRP */
        case 0x9a: /* VINTANA1_DEV_GRP */
        case 0x9e: /* VINTANA2_DEV_GRP */
        case 0xa2: /* VINTDIG_DEV_GRP */
        case 0xa6: /* VIO_DEV_GRP */
        case 0xb0: /* VDD1_DEV_GRP */
        case 0xbe: /* VDD2_DEV_GRP */
        case 0xcc: /* VUSB1V5_DEV_GRP */
        case 0xcf: /* VUSB1V8_DEV_GRP */
        case 0xd2: /* VUSB3V1_DEV_GRP */
        case 0xda: /* REGEN_DEV_GRP */
        case 0xdd: /* NRESPWRON_DEV_GRP */
        case 0xe0: /* CLKEN_DEV_GRP */
        case 0xe3: /* SYSEN_DEV_GRP */
        case 0xe6: /* HFCLKOUT_DEV_GRP */
        case 0xe9: /* 2KCLKOUT_DEV_GRP */
        case 0xec: /* TRITON_RESET_DEV_GRP */
        case 0xef: /* MAINREF_DEV_GRP */
            s->reg_data[addr] = (s->reg_data[addr] & 0x0f) | (value & 0xf0);
            break;
        case 0x75: /* VAUX1_DEDICATED */
        case 0x7d: /* VAUX3_DEDICATED */
        case 0x8d: /* VPLL1_DEDICATED */
        case 0x95: /* VSIM_DEDICATED */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value & 0x77;
            else
                s->reg_data[addr] = (s->reg_data[addr] & 0x70) | (value & 0x07);
            break;
        case 0x79: /* VAUX2_DEDICATED */
        case 0x81: /* VAUX4_DEDICATED */
        case 0x91: /* VPLL2_DEDICATED */
        case 0xa5: /* VINTDIG_DEDICATED */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value & 0x7f;
            else
                s->reg_data[addr] = (s->reg_data[addr] & 0x70) | (value & 0x0f);
            break;
        case 0x85: /* VMMC1_DEDICATED */
        case 0x99: /* VDAC_DEDICATED */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value & 0x73;
            else
                s->reg_data[addr] = (s->reg_data[addr] & 0x70) | (value & 0x03);
            break;
        case 0x89: /* VMMC2_DEDICATED */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value & 0x7f;
            else
                s->reg_data[addr] = (s->reg_data[addr] & 0x70) | (value & 0x0f);
            break;
        case 0x9d: /* VINTANA1_DEDICATED */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value & 0x70;
            break;
        case 0xa1: /* VINTANA2_DEDICATED */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value & 0x71;
            else
                s->reg_data[addr] = (s->reg_data[addr] & 0x70) | (value & 0x01);
            break;
        case 0x74: /* VAUX1_REMAP */
        case 0x78: /* VAUX2_REMAP */
        case 0x7c: /* VAUX3_REMAP */
        case 0x80: /* VAUX4_REMAP */
        case 0x84: /* VMMC1_REMAP */
        case 0x88: /* VMMC2_REMAP */
        case 0x8c: /* VPLL1_REMAP */
        case 0x90: /* VPLL2_REMAP */
        case 0x94: /* VSIM_REMAP */
        case 0x98: /* VDAC_REMAP */
        case 0x9c: /* VINTANA1_REMAP */
        case 0xa0: /* VINTANA2_REMAP */
        case 0xa4: /* VINTDIG_REMAP */
        case 0xa8: /* VIO_REMAP */
        case 0xb2: /* VDD1_REMAP */
        case 0xc0: /* VDD2_REMAP */
        case 0xce: /* VUSB1V5_REMAP */
        case 0xd1: /* VUSB1V8_REMAP */
        case 0xd4: /* VUSB3V1_REMAP */
        case 0xdc: /* REGEN_REMAP */
        case 0xdf: /* NRESPWRON_REMAP */
        case 0xe2: /* CLKEN_REMAP */
        case 0xe5: /* SYSEN_REMAP */
        case 0xe8: /* HFCLKOUT_REMAP */
        case 0xeb: /* 2KCLKOUT_REMAP */
        case 0xee: /* TRITON_RESET_REMAP */
        case 0xf1: /* MAINREF_REMAP */
            s->reg_data[addr] = value;
            break;
        case 0xa9: /* VIO_CFG */
        case 0xb3: /* VDD1_CFG */
        case 0xc1: /* VDD2_CFG */
            s->reg_data[addr] = value & 0x03;
            break;
        case 0xab: /* VIO_TEST1 */
        case 0xb5: /* VDD1_TEST1 */
        case 0xc3: /* VDD2_TEST1 */
            if (s->twl4030->key_tst)
                s->reg_data[addr] = value;
            break;
        case 0xad: /* VIO_OSC */
        case 0xb7: /* VDD1_OSC */
        case 0xc5: /* VDD2_OSC */
            s->reg_data[addr] = value & 0x5f;
            break;
        case 0xae: /* VIO_RESERVED */
        case 0xb8: /* VDD1_RESERVED */
        case 0xc6: /* VDD2_RESERVED */
            break;
        case 0xaf: /* VIO_VSEL */
            s->reg_data[addr] = value & 0x01;
            break;
        case 0xba: /* VDD1_VMODE_CFG */
        case 0xc8: /* VDD2_VMODE_CFG */
            s->reg_data[addr] = (s->reg_data[addr] & 0x38) | (value & 0x07);
            break;
        case 0xd8: /* VUSB_DEDICATED1 */
            s->reg_data[addr] = value & 0x1f;
            break;
        case 0xd9: /* VUSB_DEDICATED2 */
            s->reg_data[addr] = value & 0x08;
            break;

        default:
	        hw_error("%s: unknown register 0x%02x value 0x%02x",
                    __FUNCTION__, addr, value);
            break;
    }
}

static void twl4030_key_setstate(TWL4030NodeState *s,
                                 int col, int row, int state)
{
    TRACE("col=%d, row=%d, state=%d", col, row, state);
    if (col >= 0 && col < 8 && row >= 0 && row < 8) {
        s->reg_data[0xd8] = col; /* KBC_REG */
        s->reg_data[0xd9] = row; /* KBR_REG */
        int gen_int = 0;
        if (state) {
            s->reg_data[0xdb + row] |= 1 << col; /* FULL_CODE_xx_yy */
            gen_int = s->reg_data[0xe8] & 0x02;  /* ITKPRISING */
        } else {
            s->reg_data[0xdb + row] &= ~(1 << col); /* FULL_CODE_xx_yy */
            gen_int = s->reg_data[0xe8] & 0x01;     /* ITKPFALLING */
        }
        if (gen_int) {
            s->reg_data[0xe3] |= 0x01; /* ITKPISR1 */
            s->reg_data[0xe5] |= 0x01; /* ITKPISR2 */
            twl4030_interrupt_update(s->twl4030);
        }
    }
}

static void twl4030_key_handler(void *opaque, int keycode)
{
    TWL4030NodeState *s = (TWL4030NodeState *)opaque;
    if (!s->twl4030->extended_key && keycode == 0xe0) {
        s->twl4030->extended_key = 0x80;
    } else {
        const TWL4030KeyMap *k = s->twl4030->keymap;
        int fullcode = (keycode & 0x7f) | (s->twl4030->extended_key);
        for (; k && k->code >= 0; k++) {
            if (k->code == fullcode) {
                twl4030_key_setstate(s, k->column, k->row, !(keycode & 0x80));
            }
        }
        s->twl4030->extended_key = 0;
    }
}

static void twl4030_node_reset(TWL4030NodeState *s,
                               const uint8_t *reset_values)
{
    s->firstbyte = 0;
    s->reg = 0x00;
    memcpy(s->reg_data, reset_values, 256);
}

static void twl4030_node_init(TWL4030NodeState *s,
                              twl4030_read_func read,
                              twl4030_write_func write,
                              const uint8_t *reset_values)
{
    s->read_func = read;
    s->write_func = write;
    twl4030_node_reset(s, reset_values);
}

static int twl4030_48_init(I2CSlave *i2c)
{
    twl4030_node_init(FROM_I2C_SLAVE(TWL4030NodeState, i2c),
                      twl4030_48_read, twl4030_48_write,
                      addr_48_reset_values);
    return 0;
}

static int twl4030_49_init(I2CSlave *i2c)
{
    twl4030_node_init(FROM_I2C_SLAVE(TWL4030NodeState, i2c),
                      twl4030_49_read, twl4030_49_write,
                      addr_49_reset_values);
    return 0;
}

static int twl4030_4a_init(I2CSlave *i2c)
{
    TWL4030NodeState *s = FROM_I2C_SLAVE(TWL4030NodeState, i2c);
    twl4030_node_init(s,
                      twl4030_4a_read, twl4030_4a_write,
                      addr_4a_reset_values);
    qemu_add_kbd_event_handler(twl4030_key_handler, s);
    return 0;
}

static int twl4030_4b_init(I2CSlave *i2c)
{
    twl4030_node_init(FROM_I2C_SLAVE(TWL4030NodeState, i2c),
                      twl4030_4b_read, twl4030_4b_write,
                      addr_4b_reset_values);
    return 0;
}

static void twl4030_event(I2CSlave *i2c, enum i2c_event event)
{
    if (event == I2C_START_SEND) {
        TWL4030NodeState *s = FROM_I2C_SLAVE(TWL4030NodeState, i2c);
        s->firstbyte = 1;
    }
}

static int twl4030_rx(I2CSlave *i2c)
{
    TWL4030NodeState *s = FROM_I2C_SLAVE(TWL4030NodeState, i2c);
    return s->read_func(s, s->reg++);
}

static int twl4030_tx(I2CSlave *i2c, uint8_t data)
{
    TWL4030NodeState *s = FROM_I2C_SLAVE(TWL4030NodeState, i2c);
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else {
        s->write_func(s, s->reg++, data);
	}
    return 1;
}

static void twl4030_reset(void *opaque)
{
    TWL4030State *s = opaque;
    
    qemu_del_timer(s->alarm_timer);
    qemu_del_timer(s->periodic_timer);

    twl4030_node_reset(s->i2c[0], addr_48_reset_values);
    twl4030_node_reset(s->i2c[1], addr_49_reset_values);
    twl4030_node_reset(s->i2c[2], addr_4a_reset_values);
    twl4030_node_reset(s->i2c[3], addr_4b_reset_values);

    s->extended_key = 0;
    s->key_cfg = 0;
    s->key_tst = 0;
    
    memset(s->seq_mem, 0, sizeof(s->seq_mem));

    /* TODO: indicate correct reset reason */
}

static void twl4030_48_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    k->init = twl4030_48_init;
    k->event = twl4030_event;
    k->recv = twl4030_rx;
    k->send = twl4030_tx;
}

static void twl4030_49_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    k->init = twl4030_49_init;
    k->event = twl4030_event;
    k->recv = twl4030_rx;
    k->send = twl4030_tx;
}

static void twl4030_4a_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    k->init = twl4030_4a_init;
    k->event = twl4030_event;
    k->recv = twl4030_rx;
    k->send = twl4030_tx;
}
static void twl4030_4b_class_init(ObjectClass *klass, void *data)
{
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    k->init = twl4030_4b_init;
    k->event = twl4030_event;
    k->recv = twl4030_rx;
    k->send = twl4030_tx;
}

static TypeInfo twl4030_info[] = {
    {
        .name = "twl4030_48",
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(TWL4030NodeState),
        .class_init = twl4030_48_class_init,
    },
    {
        .name = "twl4030_49",
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(TWL4030NodeState),
        .class_init = twl4030_49_class_init,
    },
    {
        .name = "twl4030_4a",
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(TWL4030NodeState),
        .class_init = twl4030_4a_class_init,
    },
    {
        .name = "twl4030_4b",
        .parent = TYPE_I2C_SLAVE,
        .instance_size = sizeof(TWL4030NodeState),
        .class_init = twl4030_4b_class_init,
    },
};

void *twl4030_init(i2c_bus *bus, qemu_irq irq1, qemu_irq irq2,
                   const TWL4030KeyMap *keymap)
{
    TWL4030State *s = (TWL4030State *)g_malloc0(sizeof(*s));

    s->irq1 = irq1;
    s->irq2 = irq2;
    s->key_cfg = 0;
    s->key_tst = 0;
    s->keymap = keymap;

    s->alarm_timer = qemu_new_timer_ns(vm_clock, twl4030_alarm, s);
    s->periodic_timer = qemu_new_timer_ns(vm_clock, twl4030_periodic, s);

    int i;
    for (i = 0; i < ARRAY_SIZE(twl4030_info); i++) {
        DeviceState *ds = i2c_create_slave(bus, twl4030_info[i].name,
                                           0x48 + i);
        s->i2c[i] = FROM_I2C_SLAVE(TWL4030NodeState, I2C_SLAVE(ds));
        s->i2c[i]->twl4030 = s;
    }

    qemu_register_reset(twl4030_reset, s);
    return s;
}

void *twl5031_init(i2c_bus *bus, qemu_irq irq1, qemu_irq irq2,
                   const TWL4030KeyMap *keymap)
{
    TWL4030State *s = twl4030_init(bus, irq1, irq2, keymap);
    s->twl5031 = 1;
    return s;
}

void twl4030_set_powerbutton_state(void *opaque, int pressed)
{
    TWL4030State *s = opaque;
    if (pressed) {
        if (!(s->i2c[3]->reg_data[0x45] & 0x01) && /* STS_PWON */
            (s->i2c[3]->reg_data[0x33] & 0x02)) {  /* PWRON_RISING */
            s->i2c[3]->reg_data[0x2e] |= 0x01;     /* PWRON */
            s->i2c[3]->reg_data[0x30] |= 0x01;     /* PWRON */
            twl4030_interrupt_update(s);
        }
        s->i2c[3]->reg_data[0x45] |= 0x01;         /* STS_PWON */
    } else {
        if ((s->i2c[3]->reg_data[0x45] & 0x01) &&  /* STS_PWON */
            (s->i2c[3]->reg_data[0x33] & 0x01)) {  /* PWRON_FALLING */
            s->i2c[3]->reg_data[0x2e] |= 0x01;     /* PWRON */
            s->i2c[3]->reg_data[0x30] |= 0x01;     /* PWRON */
            twl4030_interrupt_update(s);
        }
        s->i2c[3]->reg_data[0x45] &= ~0x01;        /* STS_PWON */
    }
}

void twl4030_madc_attach(void *opaque, twl4030_madc_callback cb)
{
    TWL4030State *s = opaque;
    if (s->madc_cb) {
        fprintf(stderr,
                "%s: warning - overriding previously registered callback\n",
                __FUNCTION__);
    }
    s->madc_cb = cb;
}

static void twl4030_register_types(void)
{
    TypeInfo *p = twl4030_info;
    int i;
    for (i = 0; i < ARRAY_SIZE(twl4030_info); p++, i++) {
        type_register_static(p);
    }
}

type_init(twl4030_register_types);
