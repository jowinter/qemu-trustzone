/*
 * QEMU OMAP DSS display plane rendering emulation templates
 *
 * Copyright (c) 2008 yajin  <yajin@vm-kernel.org>
 * Copyright (c) 2008-2009 Nokia Corporation
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


#if DEPTH == 8
#define PIXEL_TYPE		uint8_t
#define COPY_PIXEL1(to, from)	*to ++ = from
#elif DEPTH == 15 || DEPTH == 16
#define PIXEL_TYPE		uint16_t
#define COPY_PIXEL1(to, from)	*to ++ = from
#elif DEPTH == 24
#define PIXEL_TYPE		uint8_t
#define COPY_PIXEL1(to, from)	\
    *to ++ = from; *to ++ = (from) >> 8; *to ++ = (from) >> 16
#elif DEPTH == 32
#define PIXEL_TYPE		uint32_t
#define COPY_PIXEL1(to, from)	*to ++ = from
#else
#error unknown bit depth
#endif

#ifdef WORDS_BIGENDIAN
#define SWAP_WORDS	1
#endif

static void glue(omap_dss_draw_line1_, DEPTH)(void *opaque,
                                              uint8_t *dest_,
                                              const uint8_t *src,
                                              int width,
                                              int pixelsize)
{
    PIXEL_TYPE *dest = (PIXEL_TYPE *)dest_;
    const uint32_t *palette = opaque;
    unsigned int r, g, b;
    const uint8_t *end = src + (width >> 3);
    while (src < end) {
        uint8_t data = ldub_raw(src++);
        int i = 8;
        for (; i--; data <<= 1) {
            uint32_t color = palette[data >> 7];
            b = color & 0xff;
            g = (color >> 8) & 0xff;
            r = (color >> 16) & 0xff;
            COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
        }
    }
}

static void glue(omap_dss_draw_line2_, DEPTH)(void *opaque,
                                              uint8_t *dest_,
                                              const uint8_t *src,
                                              int width,
                                              int pixelsize)
{
    PIXEL_TYPE *dest = (PIXEL_TYPE *)dest_;
    const uint32_t *palette = opaque;
    unsigned int r, g, b;
    const uint8_t *end = src + (width >> 2);
    while (src < end) {
        uint8_t data = ldub_raw(src++);
        uint32_t color = palette[data >> 6];
        b = color & 0xff;
        g = (color >> 8) & 0xff;
        r = (color >> 16) & 0xff;
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
        color = palette[(data >> 4) & 3];
        b = color & 0xff;
        g = (color >> 8) & 0xff;
        r = (color >> 16) & 0xff;
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
        color = palette[(data >> 2) & 3];
        b = color & 0xff;
        g = (color >> 8) & 0xff;
        r = (color >> 16) & 0xff;
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
        color = palette[data & 3];
        b = color & 0xff;
        g = (color >> 8) & 0xff;
        r = (color >> 16) & 0xff;
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
    }
}

static void glue(omap_dss_draw_line4_, DEPTH)(void *opaque,
                                              uint8_t *dest_,
                                              const uint8_t *src,
                                              int width,
                                              int pixelsize)
{
    PIXEL_TYPE *dest = (PIXEL_TYPE *)dest_;
    const uint32_t *palette = opaque;
    unsigned int r, g, b;
    const uint8_t *end = src + (width >> 1);
    while (src < end) {
        uint8_t data = ldub_raw(src++);
        uint32_t color = palette[data >> 4];
        b = color & 0xff;
        g = (color >> 8) & 0xff;
        r = (color >> 16) & 0xff;
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
        color = palette[data & 0x0f];
        b = color & 0xff;
        g = (color >> 8) & 0xff;
        r = (color >> 16) & 0xff;
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
    }
}

static void glue(omap_dss_draw_line8_, DEPTH)(void *opaque,
                                              uint8_t *dest_,
                                              const uint8_t *src,
                                              int width,
                                              int pixelsize)
{
    PIXEL_TYPE *dest = (PIXEL_TYPE *)dest_;
    const uint32_t *palette = opaque;
    unsigned int r, g, b;
    const uint8_t *end = src + width;
    while (src < end) {
        uint32_t color = palette[ldub_raw(src++)];
        b = color & 0xff;
        g = (color >> 8) & 0xff;
        r = (color >> 16) & 0xff;
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
    }
}

static void glue(omap_dss_draw_line12_, DEPTH)(void *opaque,
                                               uint8_t *dest_,
                                               const uint8_t *src_,
                                               int width,
                                               int pixelsize)
{
    const uint16_t *src = (const uint16_t *)src_;
    PIXEL_TYPE *dest = (PIXEL_TYPE *)dest_;
    uint16_t data;
    unsigned int r, g, b;
    const uint16_t *end = src + width;
    while (src < end) {
        data = lduw_raw(src++);
        b = (data & 0x0f) << 4;
        g = (data & 0xf0);
        r = (data & 0xf00) >> 4;
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
    }
}

static void glue(omap_dss_draw_line16_, DEPTH)(void *opaque,
                                               uint8_t *dest_,
                                               const uint8_t *src_,
                                               int width,
                                               int pixelsize)
{
#if !defined(SWAP_WORDS) && DEPTH == 16
    memcpy(dest_, src_, width << 1);
#else
    const uint16_t *src = (const uint16_t *)src_;
    PIXEL_TYPE *dest = (PIXEL_TYPE *)dest_;
    uint16_t data;
    unsigned int r, g, b;
    const uint16_t *end = src + width;
    while (src < end) {
        data = lduw_raw(src++);
        b = (data & 0x1f) << 3;
        g = (data & 0x7e0) >> 3;
        r = data >> 8;
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
    }
#endif
}

static void glue(omap_dss_draw_line24a_, DEPTH)(void *opaque,
                                                uint8_t *dest_,
                                                const uint8_t *src,
                                                int width,
                                                int pixelsize)
{
#if !defined(SWAP_WORDS) && DEPTH == 32
    memcpy(dest_, src, width << 2);
#else
    PIXEL_TYPE *dest = (PIXEL_TYPE *)dest_;
    unsigned int r, g, b;
    const uint8_t *end = src + (width << 2);
    while (src < end) {
        b = ldub_raw(src++);
        g = ldub_raw(src++);
        r = ldub_raw(src++);
        src++;
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
    }
#endif
}

static void glue(omap_dss_draw_line24b_, DEPTH)(void *opaque,
                                                uint8_t *dest_,
                                                const uint8_t *src,
                                                int width,
                                                int pixelsize)
{
#if DEPTH == 24
    memcpy(dest_, src, width * 3);
#else
    PIXEL_TYPE *dest = (PIXEL_TYPE *)dest_;
    unsigned int r, g, b;
    const uint8_t *end = src + width * 3;
    while (src < end) {
        b = ldub_raw(src++);
        g = ldub_raw(src++);
        r = ldub_raw(src++);
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
    }
#endif
}

static void glue(omap_dss_draw_line24c_, DEPTH)(void *opaque,
                                                uint8_t *dest_,
                                                const uint8_t *src,
                                                int width,
                                                int pixelsize)
{
    PIXEL_TYPE *dest = (PIXEL_TYPE *)dest_;
    unsigned int r, g, b;
    const uint8_t *end = src + (width << 2);
    while (src < end) {
        src++;
        b = ldub_raw(src++);
        g = ldub_raw(src++);
        r = ldub_raw(src++);
        COPY_PIXEL1(dest, glue(rgb_to_pixel, DEPTH)(r, g, b));
    }
}

/* No rotation */
static const drawfn glue(omap_dss_drawfn_, DEPTH)[0x10] = {
    (drawfn)glue(omap_dss_draw_line1_, DEPTH),
    (drawfn)glue(omap_dss_draw_line2_, DEPTH),
    (drawfn)glue(omap_dss_draw_line4_, DEPTH),
    (drawfn)glue(omap_dss_draw_line8_, DEPTH),
    (drawfn)glue(omap_dss_draw_line12_, DEPTH),
    NULL, /* ARGB16 */
    (drawfn)glue(omap_dss_draw_line16_, DEPTH),
    NULL,
    (drawfn)glue(omap_dss_draw_line24a_, DEPTH),
    (drawfn)glue(omap_dss_draw_line24b_, DEPTH),
    NULL, /* YUV2 4:2:2 */
    NULL, /* UYVY 4:2:2 */
    (drawfn)glue(omap_dss_draw_line24a_, DEPTH), /* FIXME: handle alpha */
    (drawfn)glue(omap_dss_draw_line24c_, DEPTH), /* FIXME: handle alpha */
    (drawfn)glue(omap_dss_draw_line24c_, DEPTH),
    NULL,
};

/* 90deg, 180deg and 270deg rotation */
//static omap3_lcd_panel_fn_t glue(omap3_lcd_panel_draw_fn_r_, DEPTH)[0x10] = {
//    /* TODO */
//    [0 ... 0xf] = NULL,
//};

#undef DEPTH
#undef SKIP_PIXEL
#undef COPY_PIXEL
#undef COPY_PIXEL1
#undef PIXEL_TYPE

#undef SWAP_WORDS
