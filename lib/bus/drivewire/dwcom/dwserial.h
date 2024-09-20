#ifndef DWSERIAL_H
#define DWSERIAL_H

#include <stdio.h>

#include "fnUART.h"
#include "sdkconfig.h" // for CONFIG_IDF_TARGET_ESP32S3
#include "dwport.h"

#if CONFIG_IDF_TARGET_ESP32S3
#include "USBHostSerial.h"
#endif

/*
 * Implementation of DriveWire Port using UART serial port
 * wrapper around existing UARTManager
 */

class SerialDwPort : public DwPort
{
private:
#ifdef ESP_PLATFORM
#if CONFIG_IDF_TARGET_ESP32S3
    USBHostSerial _uart = USBHostSerial();
#else
    UARTManager _uart = UARTManager(FN_UART_BUS);
#endif
#else
    UARTManager _uart;
#endif
public:
    SerialDwPort() {}
    virtual void begin(int baud) override;
    virtual void end() override { _uart.end(); }
    virtual bool poll(int ms) override 
    {
#ifdef ESP_PLATFORM
        return false;
#else
        return _uart.poll(ms); 
#endif
    }

    virtual int available() override { return _uart.available(); }
    virtual void flush() override { _uart.flush(); }
    virtual void flush_input() override { _uart.flush_input(); }

    // read bytes into buffer
    virtual size_t read(uint8_t *buffer, size_t size) override;
    // write buffer
    virtual ssize_t write(const uint8_t *buffer, size_t size) override;

    // specific to SerialDwPort/UART
    void set_baudrate(uint32_t baud) override { _uart.set_baudrate(baud); }
    uint32_t get_baudrate() override { return _uart.get_baudrate(); }
#ifndef ESP_PLATFORM
    void set_port(const char *device) { _uart.set_port(device); }
    const char* get_port() { return _uart.get_port(); }
#endif
};

#endif // DWSERIAL_H
