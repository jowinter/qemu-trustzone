/*
 * MIPI DSI interface.
 *
 * Copyright (C) 2008-2010 Nokia Corporation
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
#ifndef HW_DSI_H__
#define HW_DSI_H__
#include "qdev.h"
#include "hw/display/framebuffer.h"

#define DSI_MAKERETURNBYTE(b) ((((b) & 0xff) << 8) | 0x21)
#define DSI_MAKERETURNWORD(w) ((((w) & 0xffff) << 8) | 0x22)
#define DSI_MAKERETURNERROR(e) ((((e) & 0xffff) << 8) | 0x02)

typedef struct DSIDevice DSIDevice;
typedef struct DSICommonDevice DSICommonDevice;
typedef struct DSIHost DSIHost;

#define TYPE_DSI_DEVICE "dsi-device"
#define DSI_DEVICE(obj) \
    OBJECT_CHECK(DSIDevice, (obj), TYPE_DSI_DEVICE)
#define DSI_DEVICE_CLASS(klass) \
    OBJECT_CLASS_CHECK(DSIDeviceClass, (klass), TYPE_DSI_DEVICE)
#define DSI_DEVICE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(DSIDeviceClass, (obj), TYPE_DSI_DEVICE)

#define TYPE_DSI_COMMON_DEVICE "dsi-common-device"
#define DSI_COMMON_DEVICE(obj) \
    OBJECT_CHECK(DSICommonDevice, (obj), TYPE_DSI_COMMON_DEVICE)
#define DSI_COMMON_DEVICE_CLASS(klass) \
    OBJECT_CLASS_CHECK(DSICommonDeviceClass, (klass), TYPE_DSI_COMMON_DEVICE)
#define DSI_COMMON_DEVICE_GET_CLASS(obj) \
    OBJECT_GET_CLASS(DSICommonDeviceClass, (obj), TYPE_DSI_COMMON_DEVICE)



/* device callbacks */
typedef int (*dsi_device_initfn)(DSIDevice *dev);
typedef void (*dsi_write_cb)(DSIDevice *dev, uint32_t data, int len);
typedef uint32_t (*dsi_read_cb)(DSIDevice *dev, uint32_t data, int len);
typedef int (*dsi_blt_cb)(DSIDevice *dev, void *data, int width, int height,
                          int col_pitch, int row_pitch, int format);
typedef void (*dsi_bltdone_cb)(DSIDevice *dev);

/* common device callbacks */
typedef void (*dsi_common_write_cb)(DSICommonDevice *dev, uint32_t data,
                                    int len);
typedef uint32_t (*dsi_common_read_cb)(DSICommonDevice *dev, uint32_t data,
                                       int len);
typedef void (*dsi_common_device_resetfn)(DSICommonDevice *dev);
typedef void (*dsi_powermode_changed_cb)(DSICommonDevice *dev);
typedef void (*dsi_temode_changed_cb)(DSICommonDevice *dev);

/* host callbacks */
typedef void (*dsi_te_trigger_cb)(DeviceState *dev, int vc);
typedef drawfn (*dsi_get_drawfn_cb)(const DeviceState *dev, int format,
                                    int bpp);

typedef struct {
    DeviceClass parent_class;
    dsi_device_initfn init;
    dsi_write_cb write;
    dsi_read_cb read;
    dsi_blt_cb blt;
    dsi_bltdone_cb bltdone;
} DSIDeviceClass;

typedef struct {
    DSIDeviceClass parent_class;
    dsi_common_write_cb write;
    dsi_common_read_cb read;
    dsi_common_device_resetfn reset;
    dsi_powermode_changed_cb powermode_changed;
    dsi_temode_changed_cb temode_changed;
} DSICommonDeviceClass;

struct DSIDevice {
    DeviceState qdev;

    /* internal fields used by DSI code */
    DSIHost *host;
    uint8_t vchannel;
    uint16_t max_return_size;
};

struct DSICommonDevice {
    DSIDevice dsi;
    enum { bs_cmd, bs_data } bs;
    uint8_t cmd;
    uint8_t powermode;
    uint8_t addrmode;
    uint8_t bpp_dpi;
    uint8_t bpp_dbi;
    uint8_t dr;
    uint32_t sc;
    uint32_t ec;
    uint32_t cc;
    uint32_t sp;
    uint32_t ep;
    uint32_t cp;
    enum {
        te_off = -1,
        te_vsync = 0,
        te_hvsync = 1
    } te_mode;
};

/* host functions */
DSIHost *dsi_init_host(DeviceState *parent, const char *name,
                       dsi_te_trigger_cb te_trigger_cb,
                       dsi_get_drawfn_cb get_drawfn_cb);
uint32_t dsi_short_write(DSIHost *host, uint32_t data);
void dsi_long_write(DSIHost *host, uint32_t header, uint32_t payload,
                    uint32_t counter);
int dsi_blt(DSIHost *host, int vc, void *data, int width, int height,
            int col_pitch, int row_pitch, int format);
void dsi_bltdone(DSIHost *host, int vc);

/* device -> host functions */
void dsi_te_trigger(const DSIDevice *dev);
drawfn dsi_get_drawfn(const DSIDevice *dev, int format, int bpp);

#define DSI_DEVICE_FROM_QDEV(dev) DO_UPCAST(DSIDevice, qdev, dev)
#define DSI_COMMON_DEVICE_FROM_QDEV(dev) DO_UPCAST(DSICommonDevice, dsi, \
                                                   DSI_DEVICE_FROM_QDEV(dev))
#define FROM_DSI_DEVICE(type, dev) DO_UPCAST(type, dsi, dev)

DeviceState *dsi_create_device(DSIHost *host, const char *name, int vc);
DeviceState *dsi_create_device_noinit(DSIHost *host, const char *name, int vc);
DeviceState *dsi_create_common_device(DSIHost *host, const char *name, int vc);
DeviceState *dsi_create_common_device_noinit(DSIHost *host, const char *name,
                                             int vc);

#endif
