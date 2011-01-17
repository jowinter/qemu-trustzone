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
#ifndef HW_SPI_H__
#define HW_SPI_H__
#include "hw.h"
#include "qdev.h"

typedef struct spi_device_s SPIDevice;
typedef struct spi_bus_s SPIBus;

typedef int (*spi_device_initfn)(SPIDevice *dev);
typedef uint32_t (*spi_txrx_cb)(SPIDevice *dev, uint32_t, int);

typedef struct {
    DeviceInfo qdev;
    spi_device_initfn init;
    spi_txrx_cb txrx;
} SPIDeviceInfo;

struct spi_device_s {
    DeviceState qdev;
    SPIDeviceInfo *info;
    
    /* internal fields used by SPI code */
    uint8_t channel;
};

SPIBus *spi_init_bus(DeviceState *parent, const char *name, int num_channels);
uint32_t spi_txrx(SPIBus *bus, int channel, uint32_t data, int len);

#define SPI_DEVICE_FROM_QDEV(dev) DO_UPCAST(SPIDevice, qdev, dev)
#define FROM_SPI_DEVICE(type, dev) DO_UPCAST(type, spi, dev)

void spi_register_device(SPIDeviceInfo *info);
DeviceState *spi_create_device(SPIBus *bus, const char *name, int ch);
DeviceState *spi_create_device_noinit(SPIBus *bus, const char *name, int ch);

#endif
