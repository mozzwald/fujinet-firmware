/*
 * SPDX-FileCopyrightText: 2015-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */
#include "fnUSBHost.h"

#if CONFIG_IDF_TARGET_ESP32S3

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "freertos/task.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"
#include "usb/vcp_ch34x.hpp"
#include "usb/vcp_cp210x.hpp"
#include "usb/vcp_ftdi.hpp"
#include "usb/vcp.hpp"
#include "usb/msc_host.h"
#include "usb/msc_host_vfs.h"

#include "utils.h"
#include "../../include/debug.h"

using namespace esp_usb;

static const char *TAG = "fnUSBHost"; // switch to fuji debug!
static SemaphoreHandle_t device_disconnected_sem;

USBHost fnUSBHost; // Global USB Host object

/**
 * @brief Application Queue and its messages ID (for MSC)
 */
static QueueHandle_t app_queue;
typedef struct {
    enum {
        APP_QUIT,                // Signals request to exit the application
        APP_DEVICE_CONNECTED,    // USB device connect event
        APP_DEVICE_DISCONNECTED, // USB device disconnect event
    } id;
    union {
        uint8_t new_dev_address; // Address of new USB device for APP_DEVICE_CONNECTED event if
    } data;
} app_message_t;

/**
 * @brief CDC Data received callback
 *
 * @param[in] data     Pointer to received data
 * @param[in] data_len Length of received data in bytes
 * @param[in] arg      Argument we passed to the device open function
 * @return
 *   true:  We have processed the received data
 *   false: We expect more data
 */
static bool handle_rx(const uint8_t *data, size_t data_len, void *arg)
{
    #ifdef USB_HOST_DEBUG
    Debug_printf("USB CDC Data recv:\n");
    util_dump_bytes(data, data_len);
    #endif
    // Put the data in a buffer
    return true;
}

/**
 * @brief MSC driver callback
 *
 * Signal device connection/disconnection to the main task
 *
 * @param[in] event MSC event
 * @param[in] arg   MSC event data
 */
static void msc_event_cb(const msc_host_event_t *event, void *arg)
{
    app_message_t message;
    if (event->event == msc_host_event_t::MSC_DEVICE_CONNECTED) {
        ESP_LOGI(TAG, "MSC device connected");
        message.id = app_message_t::APP_DEVICE_CONNECTED;
        message.data.new_dev_address = event->device.address;
    } else if (event->event == msc_host_event_t::MSC_DEVICE_DISCONNECTED) {
        ESP_LOGI(TAG, "MSC device disconnected");
        message.id = app_message_t::APP_DEVICE_CONNECTED;
        message.data.new_dev_address = event->device.address;
    }
    xQueueSend(app_queue, &message, portMAX_DELAY);
}

/**
 * @brief USB Device event callback
 *
 * Apart from handling device disconnection it doesn't do anything useful
 *
 * @param[in] event    Device event type and data
 * @param[in] user_ctx Argument we passed to the device open function
 */
static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx)
{
    switch (event->type) {
    case CDC_ACM_HOST_ERROR:
        Debug_printf("USB CDC error, err_no = %i\n", event->data.error);
        break;
    case CDC_ACM_HOST_DEVICE_DISCONNECTED:
        Debug_printf("USB CDC suddenly disconnected!\n");
        ESP_ERROR_CHECK(cdc_acm_host_close(event->data.cdc_hdl));
        xSemaphoreGive(device_disconnected_sem);
        break;
    case CDC_ACM_HOST_SERIAL_STATE:
        Debug_printf("USB CDC Serial State: 0x%04X\n", event->data.serial_state.val);
        break;
    case CDC_ACM_HOST_NETWORK_CONNECTION:
    default:
        Debug_printf("USB CDC unsupported event %i\n", event->type);
        break;
    }
}

/**
 * @brief USB Host library handling task
 *
 * @param arg Unused
 */
static void usb_lib_task(void *arg)
{
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            Debug_printf("USB Host: All devices freed!\n");
            // Continue handling USB events to allow device reconnection
        }
    }
}

/**
 * @brief USB CDC Driver Task
 *
 * @param arg Unused
 */
void usb_vcp_task(void *arg)
{
    while (true) {
        const cdc_acm_host_device_config_t dev_config = {
            .connection_timeout_ms = 5000, // 5 seconds, enough time to plug the device in or experiment with timeout
            .out_buffer_size = 512,
            .in_buffer_size = 512,
            .event_cb = handle_event,
            .data_cb = handle_rx,
            .user_arg = NULL,
        };

        // You don't need to know the device's VID and PID. Just plug in any device and the VCP service will load correct (already registered) driver for the device
        Debug_printf("USB VCP: opening any device\n");
        auto vcp = std::unique_ptr<CdcAcmDevice>(VCP::open(&dev_config));

        if (vcp == nullptr) {
            Debug_printf("USB VCP: failed to open device\n");
            continue;
        }
        vTaskDelay(10);

        Debug_printf("USB VCP: Setting up line coding\n");
        cdc_acm_line_coding_t line_coding = {
            .dwDTERate = VCP_BAUDRATE,
            .bCharFormat = VCP_STOP_BITS,
            .bParityType = VCP_PARITY,
            .bDataBits = VCP_DATA_BITS,
        };
        ESP_ERROR_CHECK(vcp->line_coding_set(&line_coding));

        /*
        Now the USB-to-UART converter is configured and receiving data.
        You can use standard CDC-ACM API to interact with it. E.g.

        ESP_ERROR_CHECK(vcp->set_control_line_state(false, true));
        ESP_ERROR_CHECK(vcp->tx_blocking((uint8_t *)"Test string", 12));
        */

        // Send some dummy data
        ESP_LOGI(TAG, "Sending data through CdcAcmDevice");
        uint8_t data[] = "test_string";
        ESP_ERROR_CHECK(vcp->tx_blocking(data, sizeof(data)));
        ESP_ERROR_CHECK(vcp->set_control_line_state(true, true));

        // We are done. Wait for device disconnection and start over
        ESP_LOGI(TAG, "Done. You can reconnect the VCP device to run again.");
        xSemaphoreTake(device_disconnected_sem, portMAX_DELAY);
    }
}

/**
 * @brief Setup USB Host
 *
 * Here we open a USB CDC device
 */
void USBHost::setup_host(void)
{
    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    // Install USB Host driver. Should only be called once in entire application
    usb_host_config_t host_config = {};
    host_config.skip_phy_setup = false;
    host_config.intr_flags = ESP_INTR_FLAG_LEVEL1;
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    Debug_printf("USB Host Installed\n");
 
    // Create a task that will handle USB library events
    BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
    assert(task_created == pdTRUE);

    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));
    Debug_printf("USB Host CDC-ACM Installed\n");

    // Register VCP drivers to VCP service
    VCP::register_driver<FT23x>();
    VCP::register_driver<CP210x>();
    VCP::register_driver<CH34x>();
    Debug_printf("USB Host VCP Drivers Registered\n");

    // Create a task that will handle the VCP
    BaseType_t task_created = xTaskCreate(usb_vcp_task, "usb_vcp", 4096, NULL, 10, NULL);
    assert(task_created == pdTRUE);
    Debug_printf("USB Host VCP Task Started\n");
}
#endif /* CONFIG_IDF_TARGET_ESP32S3 */