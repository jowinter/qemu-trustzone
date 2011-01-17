/*
 * OMAP2/3 Display Subsystem internal interfaces.
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
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
#include "hw.h"
#include "qemu-common.h"
#include "qemu-timer.h"

struct omap_dss_s;

struct omap_dss_dispc_s {
    QEMUTimer *lcdframer;
    
    uint8_t rev;
    uint32_t idlemode;
    uint32_t irqst;
    uint32_t irqen;
    uint32_t control;
    uint32_t config;
    uint32_t capable;
    uint32_t timing[4];
    uint32_t line;
    uint32_t bg[2];
    uint32_t trans[2];
    uint32_t size_dig;
    uint32_t size_lcd;
    uint32_t global_alpha;
    uint32_t cpr_coef_r;
    uint32_t cpr_coef_g;
    uint32_t cpr_coef_b;
    
    struct omap_dss_plane_s {
        int enable;
        int bpp;
        int posx;
        int posy;
        int nx;
        int ny;
        
        int rotation_flag;
        int gfx_format;
        int gfx_channel;
        
        target_phys_addr_t addr[3]; /* BA0, BA1, TABLE_BA */
        
        uint32_t attr;
        uint32_t tresh;
        int rowinc;
        int colinc;
        int wininc;
        
        uint32_t preload;
        
        /* following used for planes 1 and 2 only (VID1 and VID2) */
        uint32_t fir;
        uint32_t fir_coef_h[8];
        uint32_t fir_coef_hv[8];
        uint32_t fir_coef_v[8];
        uint32_t conv_coef[5];
        uint32_t picture_size;
        uint32_t accu[2];
    } plane[3]; /* GFX, VID1, VID2 */
    
    uint16_t palette[256];
};

void omap_dss_lcd_framedone(void *opaque); /* should point to omap_dss_s */
