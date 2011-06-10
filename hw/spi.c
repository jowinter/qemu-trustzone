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
#include "spi.h"

struct spi_bus_s {
    BusState qbus;
    int channels;
    SPIDevice **device;
};

static struct BusInfo spi_bus_info = {
    .name = "SPI",
    .size = sizeof(SPIBus),
    .props = (Property[]) {
        DEFINE_PROP_UINT8("channel", SPIDevice, channel, 0),
        DEFINE_PROP_END_OF_LIST()
    }
};

SPIBus *spi_init_bus(DeviceState *parent, const char *name, int num_channels)
{
    SPIBus *bus = FROM_QBUS(SPIBus, qbus_create(&spi_bus_info, parent, name));
    bus->channels = num_channels;
    bus->device = qemu_mallocz(bus->channels * sizeof(SPIDevice *));
    return bus;
}

uint32_t spi_txrx(SPIBus *bus, int channel, uint32_t data, int len)
{
    SPIDevice *dev;
    
    if (channel < bus->channels) {
        if ((dev = bus->device[channel])) {
            if (dev->info->txrx) {
                return dev->info->txrx(dev, data, len);
            }
        }
    } else {
        hw_error("%s: invalid channel %d\n", __FUNCTION__, channel);
    }
    return 0;
}

static int spi_device_init(DeviceState *qdev, DeviceInfo *base)
{
    SPIDeviceInfo *info = container_of(base, SPIDeviceInfo, qdev);
    SPIDevice *dev = SPI_DEVICE_FROM_QDEV(qdev);
    dev->info = info;
    return info->init(dev);
}

void spi_register_device(SPIDeviceInfo *info)
{
    assert(info->qdev.size >= sizeof(SPIDevice));
    info->qdev.init = spi_device_init;
    info->qdev.bus_info = &spi_bus_info;
    qdev_register(&info->qdev);
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
