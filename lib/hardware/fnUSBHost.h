/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#ifndef FNUSBHOST_H
#define FNUSBHOST_H

#include "sdkconfig.h" // for CONFIG_IDF_TARGET_ESP32S3

#if CONFIG_IDF_TARGET_ESP32S3
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define VCP_BAUDRATE        115200   // Baudrate for USB Virtual Serial Port
#define VCP_STOP_BITS       (0)      // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
#define VCP_PARITY          (0)      // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
#define VCP_DATA_BITS       (8)

#define EXAMPLE_USB_HOST_PRIORITY   (20)
#define EXAMPLE_USB_DEVICE_VID      (0x303A)
#define EXAMPLE_USB_DEVICE_PID      (0x4001) // 0x303A:0x4001 (TinyUSB CDC device)
#define EXAMPLE_USB_DEVICE_DUAL_PID (0x4002) // 0x303A:0x4002 (TinyUSB Dual CDC device)
#define EXAMPLE_TX_STRING           ("CDC test string!")
#define EXAMPLE_TX_TIMEOUT_MS       (1000)

#define MNT_PATH         "/usb"     // Path in the Virtual File System where USB flash drive is to be mounted

/**
 * @brief FujiNet USB Host Class
 */
class USBHost
{
    private:
        
    protected:

    public:
        void setup_host();
};

extern USBHost fnUSBHost;

#endif /* CONFIG_IDF_TARGET_ESP32S3 */
#endif /* FNUSBHOST_H */