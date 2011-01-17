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
#include "dsi.h"

#define DSI_ERROR_NODEVICE(channel) \
    fprintf(stderr, "%s: no device attached on virtual channel %d\n", \
            __FUNCTION__, channel)
#define DSI_EXTRACTPARAM(var, data, nb) \
    { \
        int i; \
        for (i = nb; i--; data >>= 8) \
        var = (var << 8) | (data & 0xff); \
    }

struct dsi_host_s {
    BusState qbus;
    DSIDevice *device[4];
    dsi_te_trigger_cb te_trigger;
    dsi_get_drawfn_cb get_drawfn;
};

static struct BusInfo dsi_bus_info = {
    .name = "DSI",
    .size = sizeof(DSIHost),
    .props = (Property[]) {
        DEFINE_PROP_UINT8("virtual_channel", DSIDevice, vchannel, 0),
        DEFINE_PROP_END_OF_LIST()
    }
};

DSIHost *dsi_init_host(DeviceState *parent, const char *name,
                       dsi_te_trigger_cb te_trigger_cb,
                       dsi_get_drawfn_cb get_drawfn_cb)
{
    DSIHost *host = FROM_QBUS(DSIHost,
                              qbus_create(&dsi_bus_info, parent, name));
    host->te_trigger = te_trigger_cb;
    host->get_drawfn = get_drawfn_cb;
    return host;
}

uint32_t dsi_short_write(DSIHost *host, uint32_t data)
{
    DSIDevice *dev = host->device[(data >> 6) & 3];
    if (dev) {
        uint16_t payload = data >> 8;
        switch (data & 0x3f) { /* id */
        case 0x05: /* short_write_0 */
            dev->info->write(dev, payload & 0xff, 1);
            break;
        case 0x06: /* read */
            return dev->info->read(dev, payload & 0xff, 1);
        case 0x15: /* short_write_1 */
            dev->info->write(dev, payload, 2);
            break;
        case 0x37: /* set maximum return packet size */
            dev->max_return_size = data;
            break;
        default:
            hw_error("%s: unknown/unimplemented DSI id (0x%02x)",
                     __FUNCTION__, data & 0x3f);
            break;
        }
    } else {
        DSI_ERROR_NODEVICE((data >> 6) & 3);
    }
    return 0;
}

void dsi_long_write(DSIHost *host, uint32_t header, uint32_t payload,
                    uint32_t counter)
{
    DSIDevice *dev = host->device[(header >> 6) & 3];
    if (dev) {
        switch (header & 0x3f) { /* id */
        case 0x09: /* null packet */
            /* ignore */
            break;
        case 0x39: /* long write */
            dev->info->write(dev, payload, counter > 4 ? 4 : counter);
            break;
        default:
            hw_error("%s: unknown/unimplemented DSI id (0x%02x)",
                     __FUNCTION__, header & 0x3f);
            break;
        }
    } else {
        DSI_ERROR_NODEVICE((header >> 6) & 3);
    }
}

int dsi_blt(DSIHost *host, int vc, void *data, int width, int height,
            int col_pitch, int row_pitch, int format)
{
    if (vc >= 0 && vc < 4) {
        DSIDevice *dev = host->device[vc];
        if (dev) {
            return dev->info->blt(dev, data, width, height, col_pitch,
                                  row_pitch, format);
        } else {
            DSI_ERROR_NODEVICE(vc);
        }
    } else {
        hw_error("%s: invalid virtual channel id (%d)\n", __FUNCTION__, vc);
    }
    return 0;
}

void dsi_bltdone(DSIHost *host, int vc)
{
    if (vc >=0 && vc < 4) {
        DSIDevice *dev = host->device[vc];
        if (dev) {
            dev->info->bltdone(dev);
        } else {
            DSI_ERROR_NODEVICE(vc);
        }
    } else {
        hw_error("%s: invalid virtual channel id (%d)\n", __FUNCTION__, vc);
    }
}

void dsi_te_trigger(const DSIDevice *dev)
{
    if (dev && dev->host && dev->host->te_trigger) {
        dev->host->te_trigger(dev->host->qbus.parent, dev->vchannel);
    }
}

drawfn dsi_get_drawfn(const DSIDevice *dev, int format, int bpp)
{
    if (dev && dev->host && dev->host->get_drawfn) {
        return dev->host->get_drawfn(dev->host->qbus.parent, format, bpp);
    }
    return NULL;
}

static void dsi_common_reset(DeviceState *dev)
{
    DSICommonDevice *s = DSI_COMMON_DEVICE_FROM_QDEV(dev);
    DSICommonDeviceInfo *i = DO_UPCAST(DSICommonDeviceInfo, dsi, s->dsi.info);
    s->bs = bs_cmd;
    s->cmd = 0;
    s->powermode = 0x08;
    s->addrmode = 0;
    s->bpp_dbi = 0;
    s->bpp_dpi = 0;
    s->dr = 0;
    s->sc = 0;
    s->ec = 0;
    s->cc = 0;
    s->sp = 0;
    s->ep = 0;
    s->cp = 0;
    s->te_mode = te_off;
    if (i->reset) {
        i->reset(s);
    }
}

static void dsi_common_write(DSIDevice *dev, uint32_t data, int len)
{
    uint8_t x;
    DSICommonDevice *s = FROM_DSI_DEVICE(DSICommonDevice, dev);
    DSICommonDeviceInfo *i = DO_UPCAST(DSICommonDeviceInfo, dsi, s->dsi.info);
    if  (s->bs == bs_cmd) {
        s->cmd = data & 0xff;
        data >>= 8;
        len--;
    }
    switch (s->cmd) {
    case 0x10: /* enter sleep */
        x = s->powermode;
        s->powermode &= ~0x10;
        if ((x ^ s->powermode) && i->powermode_changed) {
            i->powermode_changed(s);
        }
        break;
    case 0x11: /* exit sleep */
        x = s->powermode;
        s->powermode |= 0x10;
        s->dr ^= 0xe0;
        if ((x ^ s->powermode) && i->powermode_changed) {
            i->powermode_changed(s);
        }
        break;
    case 0x28: /* display off */
        x = s->powermode;
        s->powermode &= ~0x04;
        if ((x ^ s->powermode) && i->powermode_changed) {
            i->powermode_changed(s);
        }
        break;
    case 0x29: /* display on */
        x = s->powermode;
        s->powermode |= 0x04;
        if ((x ^ s->powermode) && i->powermode_changed) {
            i->powermode_changed(s);
        }
        break;
    case 0x2a: /* set column address */
        if (s->bs == bs_cmd) {
            s->bs = bs_data;
            s->sc = 0;
            s->ec = 0;
            DSI_EXTRACTPARAM(s->sc, data, 2);
            DSI_EXTRACTPARAM(s->ec, data, 1);
            s->cc = s->sc;
        } else {
            s->bs = bs_cmd;
            DSI_EXTRACTPARAM(s->ec, data, 1);
            s->cc = s->sc;
        }
        break;
    case 0x2b: /* set page address */
        if (s->bs == bs_cmd) {
            s->bs = bs_data;
            s->sp = 0;
            s->ep = 0;
            DSI_EXTRACTPARAM(s->sp, data, 2);
            DSI_EXTRACTPARAM(s->ep, data, 1);
            s->cp = s->sp;
        } else {
            s->bs = bs_cmd;
            DSI_EXTRACTPARAM(s->ep, data, 1);
            s->cp = s->sp;
        }
        break;
    case 0x34: /* disable tear effect control */
        x = s->te_mode;
        s->te_mode = te_off;
        if ((x ^ s->te_mode) && i->temode_changed) {
            i->temode_changed(s);
        }
        break;
    case 0x35: /* enable tear effect control */
        x = s->te_mode;
        s->te_mode = (data & 0x01) ? te_hvsync : te_vsync;
        if ((x ^ s->te_mode) && i->temode_changed) {
            i->temode_changed(s);
        }
        break;
    case 0x36: /* set address mode */
        s->addrmode = data & 0xff;
        break;
    case 0x3a: /* set pixel format */
        switch ((data >> 4) & 7) {
        case 0:
            s->bpp_dpi = 0;
            break;
        case 2: /* 8bpp */
            s->bpp_dpi = 1;
            break;
        case 5: /* 16bpp */
            s->bpp_dpi = 2;
            break;
        case 7: /* 24bpp */
            s->bpp_dpi = 4; /* faster to process than 3 */
            break;
        default:
            hw_error("%s: unsupported dpi pixel format %d",
                     __FUNCTION__, (data >> 4) & 7);
            break;
        }
        switch ((data & 7)) {
        case 0:
            s->bpp_dbi = 0;
            break;
        case 2: /* 8bpp */
            s->bpp_dbi = 1;
            break;
        case 5: /* 16bpp */
            s->bpp_dbi = 2;
            break;
        case 7: /* 24bpp */
            s->bpp_dbi = 4; /* faster to process than 3 */
            break;
        default:
            hw_error("%s: unsupported dbi pixel format %d",
                     __FUNCTION__, data & 7);
            break;
        }
        break;
    default:
        if (i->write) {
            if (s->bs == bs_cmd) {
                data = (data << 8) | s->cmd;
                len++;
            }
            i->write(s, data, len);
        } else {
            hw_error("%s: unknown command 0x%02x\n", __FUNCTION__, s->cmd);
        }
        break;
    }
}

static uint32_t dsi_common_read(DSIDevice *dev, uint32_t data, int len)
{
    DSICommonDevice *s = FROM_DSI_DEVICE(DSICommonDevice, dev);
    DSICommonDeviceInfo *i = DO_UPCAST(DSICommonDeviceInfo, dsi, dev->info);
    if (s->bs != bs_cmd) {
        hw_error("%s: previous WRITE command not completed", __FUNCTION__);
    }
    s->cmd = data & 0xff;
    switch (s->cmd) {
    case 0x0a: /* get power mode */
        return DSI_MAKERETURNBYTE(s->powermode);
    case 0x0b: /* get address mode */
        return DSI_MAKERETURNBYTE(s->addrmode);
    case 0x0f: /* get diagnostic result */
        return DSI_MAKERETURNBYTE(s->dr);
    default:
        if (i->read) {
            return i->read(s, data, len);
        } else {
            hw_error("%s: unknown command 0x%02x\n", __FUNCTION__, s->cmd);
        }
        break;
    }
    return 0;
}

static int dsi_device_init(DeviceState *dev, DeviceInfo *base)
{
    DSIDeviceInfo *info = container_of(base, DSIDeviceInfo, qdev);
    DSIDevice *dsi_dev = DSI_DEVICE_FROM_QDEV(dev);
    dsi_dev->info = info;
    return info->init(dsi_dev);
}

void dsi_register_device(DSIDeviceInfo *info)
{
    assert(info->qdev.size >= sizeof(DSIDevice));
    info->qdev.init = dsi_device_init;
    info->qdev.bus_info = &dsi_bus_info;
    qdev_register(&info->qdev);
}

void dsi_register_common_device(DSICommonDeviceInfo *info)
{
    dsi_register_device(&info->dsi);
    info->dsi.write = dsi_common_write;
    info->dsi.read = dsi_common_read;
    info->dsi.qdev.reset = dsi_common_reset;
}

DeviceState *dsi_create_device_noinit(DSIHost *host, const char *name, int vc)
{
    if (host->device[vc]) {
        hw_error("%s: virtual channel %d already has a device attached\n",
                 __FUNCTION__, vc);
    }
    DeviceState *dev = qdev_create(&host->qbus, name);
    qdev_prop_set_uint8(dev, "virtual_channel", vc);
    DSIDevice *dsi_dev = DSI_DEVICE_FROM_QDEV(dev);
    host->device[vc] = dsi_dev;
    dsi_dev->host = host;
    return dev;
}

DeviceState *dsi_create_device(DSIHost *host, const char *name, int vc)
{
    DeviceState *dev = dsi_create_device_noinit(host, name, vc);
    qdev_init_nofail(dev);
    return dev;
}

DeviceState *dsi_create_common_device_noinit(DSIHost *host, const char *name,
                                             int vc)
{
    return dsi_create_device_noinit(host, name, vc);
}

DeviceState *dsi_create_common_device(DSIHost *host, const char *name, int vc)
{
    DeviceState *dev = dsi_create_common_device_noinit(host, name, vc);
    qdev_init_nofail(dev);
    return dev;
}
