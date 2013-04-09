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
#include "hw/dsi.h"

#define DSI_ERROR_NODEVICE(channel) \
    fprintf(stderr, "%s: no device attached on virtual channel %d\n", \
            __FUNCTION__, channel)
#define DSI_EXTRACTPARAM(var, data, nb) \
    { \
        int i; \
        for (i = nb; i--; data >>= 8) \
        var = (var << 8) | (data & 0xff); \
    }

struct DSIHost {
    BusState qbus;
    DSIDevice *device[4];
    dsi_te_trigger_cb te_trigger;
    dsi_get_drawfn_cb get_drawfn;
};

static Property dsi_props[] = {
    DEFINE_PROP_UINT8("virtual_channel", DSIDevice, vchannel, 0),
    DEFINE_PROP_END_OF_LIST()
};

#define TYPE_DSI_BUS "dsi-bus"
#define DSI_BUS(obj) OBJECT_CHECK(DSIHost, (obj), TYPE_DSI_BUS)

static const TypeInfo dsi_bus_info = {
    .name = TYPE_DSI_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(DSIHost),
};

DSIHost *dsi_init_host(DeviceState *parent, const char *name,
                       dsi_te_trigger_cb te_trigger_cb,
                       dsi_get_drawfn_cb get_drawfn_cb)
{
    DSIHost *host = FROM_QBUS(DSIHost, qbus_create(TYPE_DSI_BUS, parent, name));
    host->te_trigger = te_trigger_cb;
    host->get_drawfn = get_drawfn_cb;
    return host;
}

uint32_t dsi_short_write(DSIHost *host, uint32_t data)
{
    DSIDevice *dev = host->device[(data >> 6) & 3];
    if (dev) {
        DSIDeviceClass *dc = DSI_DEVICE_GET_CLASS(dev);
        uint16_t payload = data >> 8;
        switch (data & 0x3f) { /* id */
        case 0x05: /* short_write_0 */
            dc->write(dev, payload & 0xff, 1);
            break;
        case 0x06: /* read */
            return dc->read(dev, payload & 0xff, 1);
        case 0x15: /* short_write_1 */
            dc->write(dev, payload, 2);
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
        DSIDeviceClass *dc = DSI_DEVICE_GET_CLASS(dev);
        switch (header & 0x3f) { /* id */
        case 0x09: /* null packet */
            /* ignore */
            break;
        case 0x39: /* long write */
            dc->write(dev, payload, counter > 4 ? 4 : counter);
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
            DSIDeviceClass *dc = DSI_DEVICE_GET_CLASS(dev);
            return dc->blt(dev, data, width, height,
                           col_pitch, row_pitch, format);
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
            DSIDeviceClass *dc = DSI_DEVICE_GET_CLASS(dev);
            dc->bltdone(dev);
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
    DSICommonDeviceClass *dcc = DSI_COMMON_DEVICE_GET_CLASS(dev);
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
    if (dcc->reset) {
        dcc->reset(s);
    }
}

static void dsi_common_write(DSIDevice *dev, uint32_t data, int len)
{
    uint8_t x;
    DSICommonDevice *s = FROM_DSI_DEVICE(DSICommonDevice, dev);
    DSICommonDeviceClass *dcc = DSI_COMMON_DEVICE_GET_CLASS(s);
    if  (s->bs == bs_cmd) {
        s->cmd = data & 0xff;
        data >>= 8;
        len--;
    }
    switch (s->cmd) {
    case 0x10: /* enter sleep */
        x = s->powermode;
        s->powermode &= ~0x10;
        if ((x ^ s->powermode) && dcc->powermode_changed) {
            dcc->powermode_changed(s);
        }
        break;
    case 0x11: /* exit sleep */
        x = s->powermode;
        s->powermode |= 0x10;
        s->dr ^= 0xe0;
        if ((x ^ s->powermode) && dcc->powermode_changed) {
            dcc->powermode_changed(s);
        }
        break;
    case 0x28: /* display off */
        x = s->powermode;
        s->powermode &= ~0x04;
        if ((x ^ s->powermode) && dcc->powermode_changed) {
            dcc->powermode_changed(s);
        }
        break;
    case 0x29: /* display on */
        x = s->powermode;
        s->powermode |= 0x04;
        if ((x ^ s->powermode) && dcc->powermode_changed) {
            dcc->powermode_changed(s);
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
        if ((x ^ s->te_mode) && dcc->temode_changed) {
            dcc->temode_changed(s);
        }
        break;
    case 0x35: /* enable tear effect control */
        x = s->te_mode;
        s->te_mode = (data & 0x01) ? te_hvsync : te_vsync;
        if ((x ^ s->te_mode) && dcc->temode_changed) {
            dcc->temode_changed(s);
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
        if (dcc->write) {
            if (s->bs == bs_cmd) {
                data = (data << 8) | s->cmd;
                len++;
            }
            dcc->write(s, data, len);
        } else {
            hw_error("%s: unknown command 0x%02x\n", __FUNCTION__, s->cmd);
        }
        break;
    }
}

static uint32_t dsi_common_read(DSIDevice *dev, uint32_t data, int len)
{
    DSICommonDevice *s = FROM_DSI_DEVICE(DSICommonDevice, dev);
    DSICommonDeviceClass *dcc = DSI_COMMON_DEVICE_GET_CLASS(s);
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
        if (dcc->read) {
            return dcc->read(s, data, len);
        } else {
            hw_error("%s: unknown command 0x%02x\n", __FUNCTION__, s->cmd);
        }
        break;
    }
    return 0;
}

static int dsi_device_init(DeviceState *dev)
{
    DSIDevice *dsi_dev = DSI_DEVICE_FROM_QDEV(dev);
    DSIDeviceClass *dc = DSI_DEVICE_GET_CLASS(dsi_dev);
    return dc->init(dsi_dev);
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

static void dsi_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);
    k->init = dsi_device_init;
    k->bus_type = TYPE_DSI_BUS;
    k->props = dsi_props;
}

static void dsi_common_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);
    DSIDeviceClass *dc = DSI_DEVICE_CLASS(klass);
    dc->write = dsi_common_write;
    dc->read = dsi_common_read;
    k->reset = dsi_common_reset;
}

static TypeInfo dsi_device_type_info = {
    .name = TYPE_DSI_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(DSIDevice),
    .abstract = true,
    .class_size = sizeof(DSIDeviceClass),
    .class_init = dsi_device_class_init,
};

static TypeInfo dsi_common_device_type_info = {
    .name = TYPE_DSI_COMMON_DEVICE,
    .parent = TYPE_DSI_DEVICE,
    .instance_size = sizeof(DSICommonDevice),
    .abstract = true,
    .class_size = sizeof(DSICommonDeviceClass),
    .class_init = dsi_common_device_class_init,
};

static void dsi_register_types(void)
{
    type_register_static(&dsi_bus_info);
    type_register_static(&dsi_device_type_info);
    type_register_static(&dsi_common_device_type_info);
}

type_init(dsi_register_types)
