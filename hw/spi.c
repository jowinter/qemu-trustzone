/*
 * SPI interface.
 *
 * Copyright (C) 2007-2010 Nokia Corporation
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
#include "hw/spi.h"

struct SPIBus {
    BusState qbus;
    int channels;
    SPIDevice **device;
};

#define TYPE_SPI_BUS "spi-bus"
#define SPI_BUS(obj) OBJECT_CHECK(SPIBus, (obj), TYPE_SPI_BUS)

static Property spi_props[] = {
    DEFINE_PROP_UINT8("channel", SPIDevice, channel, 0),
    DEFINE_PROP_END_OF_LIST()
};

static const TypeInfo spi_bus_info = {
    .name = TYPE_SPI_BUS,
    .parent = TYPE_BUS,
    .instance_size = sizeof(SPIBus),
};

SPIBus *spi_init_bus(DeviceState *parent, const char *name, int num_channels)
{
    SPIBus *bus = FROM_QBUS(SPIBus, qbus_create(TYPE_SPI_BUS, parent, name));
    bus->channels = num_channels;
    bus->device = g_new0(SPIDevice*, bus->channels);
    return bus;
}

uint32_t spi_txrx(SPIBus *bus, int channel, uint32_t data, int len)
{
    SPIDevice *dev;
    SPIDeviceClass *sc;

    if (channel < bus->channels) {
        if ((dev = bus->device[channel])) {
            sc = SPI_DEVICE_GET_CLASS(dev);
            if (sc->txrx) {
                return sc->txrx(dev, data, len);
            }
        }
    } else {
        hw_error("%s: invalid channel %d\n", __FUNCTION__, channel);
    }
    return 0;
}

DeviceState *spi_create_device_noinit(SPIBus *bus, const char *name, int ch)
{
    if (ch >= bus->channels) {
        hw_error("%s: invalid channel %d\n", __FUNCTION__, ch);
    }
    if (bus->device[ch]) {
        hw_error("%s: channel %d already has a device attached\n",
                 __FUNCTION__, ch);
    }
    DeviceState *qdev = qdev_create(&bus->qbus, name);
    qdev_prop_set_uint8(qdev, "channel", ch);
    SPIDevice *dev = SPI_DEVICE_FROM_QDEV(qdev);
    bus->device[ch] = dev;
    return qdev;
}

DeviceState *spi_create_device(SPIBus *bus, const char *name, int ch)
{
    DeviceState *dev = spi_create_device_noinit(bus, name, ch);
    qdev_init_nofail(dev);
    return dev;
}

static int spi_device_qdev_init(DeviceState *dev)
{
    SPIDevice *s = SPI_DEVICE_FROM_QDEV(dev);
    SPIDeviceClass *sc = SPI_DEVICE_GET_CLASS(s);
    return sc->init(s);
}

static void spi_device_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *k = DEVICE_CLASS(klass);
    k->init = spi_device_qdev_init;
    k->bus_type = TYPE_SPI_BUS;
    k->props = spi_props;
}

static TypeInfo spi_device_type_info = {
    .name = TYPE_SPI_DEVICE,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(SPIDevice),
    .abstract = true,
    .class_size = sizeof(SPIDeviceClass),
    .class_init = spi_device_class_init,
};

static void spi_device_register_types(void)
{
    type_register_static(&spi_bus_info);
    type_register_static(&spi_device_type_info);
}

type_init(spi_device_register_types)
