#ifdef BUILD_ATARI

#include "udpstream.h"

#include "../../include/debug.h"
#include "../../include/pinmap.h"

#include "fnSystem.h"
#include "utils.h"

static uint64_t udpstream_time_us()
{
#ifdef ESP_PLATFORM
    return (uint64_t)esp_timer_get_time();
#else
    return (uint64_t)fnSystem.millis() * 1000ULL;
#endif
}

void sioUDPStream::pace_to_atari(uint32_t min_gap_us)
{
    // Deadline-driven pacing to keep UDP->SIO output moving even when other paths wait.
    uint8_t send_count = 0;
    while (rx_count > 0 && send_count < 16)
    {
        uint64_t now_us = udpstream_time_us();
        if ((now_us - last_tx_us) < min_gap_us)
            break;
        uint8_t out = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % UDPSTREAM_RX_RING_SIZE;
        rx_count--;
        SYSTEM_BUS.write(&out, 1);
        last_tx_us += min_gap_us;
        send_count++;
    }

    if (udpstreamKeepaliveEnabled && stream_started && rx_count == 0)
    {
        uint64_t now_us = udpstream_time_us();
        if ((now_us - last_tx_us) >= UDPSTREAM_KEEPALIVE_TIMEOUT)
        {
            uint8_t keepalive = 0x00;
            SYSTEM_BUS.write(&keepalive, 1);
            last_tx_us = now_us;
        }
    }
}

void sioUDPStream::sio_enable_udpstream()
{
    if (udpstream_port == MIDI_PORT)
    {
#ifdef ESP_PLATFORM
        // Setup PWM timer for CLOCK IN
        ledc_timer_config_t ledc_timer = {};
        ledc_timer.clk_cfg = LEDC_AUTO_CLK;
        ledc_timer.speed_mode = LEDC_ESP32XX_HIGH_SPEED;
        ledc_timer.duty_resolution = LEDC_TIMER_RESOLUTION;
        ledc_timer.timer_num = LEDC_TIMER_1;
        ledc_timer.freq_hz = MIDI_BAUDRATE;

        // Setup PWM channel for CLOCK IN
        ledc_channel_config_t ledc_channel_sio_ckin = {};
        ledc_channel_sio_ckin.gpio_num = PIN_CKI;
        ledc_channel_sio_ckin.speed_mode = LEDC_ESP32XX_HIGH_SPEED;
        ledc_channel_sio_ckin.channel = LEDC_CHANNEL_1;
        ledc_channel_sio_ckin.intr_type = LEDC_INTR_DISABLE;
        ledc_channel_sio_ckin.timer_sel = LEDC_TIMER_1;
        ledc_channel_sio_ckin.duty = 1;
        ledc_channel_sio_ckin.hpoint = 0;

        // Enable PWM on CLOCK IN
        ledc_channel_config(&ledc_channel_sio_ckin);
        ledc_timer_config(&ledc_timer);
        ledc_set_duty(LEDC_ESP32XX_HIGH_SPEED, LEDC_CHANNEL_1, 1);
        ledc_update_duty(LEDC_ESP32XX_HIGH_SPEED, LEDC_CHANNEL_1);
#endif

        // Change baud rate
        SYSTEM_BUS.setBaudrate(MIDI_BAUDRATE);
    }

    // Open the UDP connection
    udpStream.begin(udpstream_port);

    udpstreamActive = true;
    last_rx_us = udpstream_time_us();
    last_tx_us = last_rx_us;
    stream_started = false;
    rx_head = 0;
    rx_tail = 0;
    rx_count = 0;
    rx_drop_count = 0;
    Debug_println("UDPSTREAM mode ENABLED");
    if (udpstreamIsServer)
    {
        // Register with the server
        Debug_println("UDPSTREAM registering with server");
        const char* str = "REGISTER";
        memcpy(buf_stream, str, strlen(str));
        udpStream.beginPacket(udpstream_host_ip, udpstream_port); // remote IP and port
        udpStream.write(buf_stream, strlen(str));
        udpStream.endPacket();
        // number the outgoing packet for the server to handle sequencing
        packet_seq = 0;
        buf_stream_index = 0;
        packet_seq += 1;
        *(uint16_t *)buf_stream = packet_seq;
        buf_stream_index += 2;
    }
}

void sioUDPStream::sio_disable_udpstream()
{
    udpStream.stop();
    if (udpstream_port == MIDI_PORT)
    {
#ifdef ESP_PLATFORM
        ledc_stop(LEDC_ESP32XX_HIGH_SPEED, LEDC_CHANNEL_1, 0);
#endif
        SYSTEM_BUS.setBaudrate(SIO_STANDARD_BAUDRATE);
    }
#ifdef ESP_PLATFORM
    // Reset CKI pin back to output open drain high
    fnSystem.set_pin_mode(PIN_CKI, gpio_mode_t::GPIO_MODE_OUTPUT_OD);
    fnSystem.digital_write(PIN_CKI, DIGI_HIGH);
#endif
    udpstreamActive = false;
    udpstreamIsServer = false;
    Debug_println("UDPSTREAM mode DISABLED");
}

void sioUDPStream::sio_handle_udpstream()
{
    const uint32_t min_gap_us = (udpstream_port == MIDI_PORT) ? UDPSTREAM_MIN_GAP_US_MIDI : UDPSTREAM_MIN_GAP_US_SIO;
    const uint64_t now_us = udpstream_time_us();

    // if there’s data available, read a packet
    int packetSize = udpStream.parsePacket();
    if (packetSize > 0)
    {
        udpStream.read(buf_net, UDPSTREAM_BUFFER_SIZE);
        // Look for game start in incoming stream (debug only).
        for (int i = 0; i < packetSize; i++)
        {
            if (buf_net[i] == 0x87 && !stream_started)
            {
                stream_started = true;
#ifdef DEBUG_UDPSTREAM
                Debug_println("UDPSTREAM: 0x87 seen (UDP-IN)");
#endif
                break;
            }
        }

        // Buffer incoming UDP bytes for paced UART output.
        for (int i = 0; i < packetSize; i++)
        {
            if (rx_count < UDPSTREAM_RX_RING_SIZE)
            {
                rx_ring[rx_head] = buf_net[i];
                rx_head = (rx_head + 1) % UDPSTREAM_RX_RING_SIZE;
                rx_count++;
            }
            else
            {
                // Drop oldest byte to keep the most recent stream data.
                rx_tail = (rx_tail + 1) % UDPSTREAM_RX_RING_SIZE;
                rx_ring[rx_head] = buf_net[i];
                rx_head = (rx_head + 1) % UDPSTREAM_RX_RING_SIZE;
                rx_drop_count++;
            }
        }
        last_rx_us = now_us;
#ifdef DEBUG_UDPSTREAM
        Debug_printf("UDP-IN [%llu ms]: ", (unsigned long long)(udpstream_time_us() / 1000ULL));
        util_dump_bytes(buf_net, packetSize);
#endif
    }

    pace_to_atari(min_gap_us);

    // Keepalive disabled for now to avoid desync during setup.

    // Read the data until there's a pause in the incoming stream
    if (SYSTEM_BUS.available() > 0)
    {
        while (true)
        {
            // Break out of UDPStream mode if COMMAND is asserted
#ifdef ESP_PLATFORM
            if (fnSystem.digital_read(PIN_CMD) == DIGI_LOW)
#else
            if (SYSTEM_BUS.commandAsserted())
#endif
            {
                Debug_println("CMD Asserted in LOOP, stopping UDPStream");
                sio_disable_udpstream();
                return;
            }
            if (SYSTEM_BUS.available() > 0)
            {
                // Collect bytes read in our buffer
                int in_byte = SYSTEM_BUS.read(); // TODO apc: check for error first
                if (in_byte == 0x87 && !stream_started)
                {
                    stream_started = true;
#ifdef DEBUG_UDPSTREAM
                    Debug_println("UDPSTREAM: 0x87 seen (SIO-OUT)");
#endif
                }
                buf_stream[buf_stream_index] = (unsigned char)in_byte;
                if (buf_stream_index < UDPSTREAM_BUFFER_SIZE - 1)
                    buf_stream_index++;
            }
            else
            {
                // Short, bounded waits prevent starving the pacing loop.
                const uint32_t wait_step_us = 250;
                const uint32_t wait_budget_us = 10000;
                uint32_t waited_us = 0;
                while (waited_us < wait_budget_us && SYSTEM_BUS.available() <= 0)
                {
                    fnSystem.delay_microseconds(wait_step_us);
                    waited_us += wait_step_us;
                    pace_to_atari(min_gap_us);
                }
                if (SYSTEM_BUS.available() <= 0)
                    break;
            }
        }

        // Send what we've collected
        udpStream.beginPacket(udpstream_host_ip, udpstream_port); // remote IP and port
        udpStream.write(buf_stream, buf_stream_index);
        udpStream.endPacket();

#ifdef DEBUG_UDPSTREAM
        Debug_printf("UDP-OUT [%llu ms]: ", (unsigned long long)(udpstream_time_us() / 1000ULL));
        util_dump_bytes(buf_stream, buf_stream_index);
#endif
        buf_stream_index = 0;
        if (udpstreamIsServer)
        {
            // number the outgoing packet for the server to handle sequencing
            packet_seq += 1;
            *(uint16_t *)buf_stream = packet_seq;
            buf_stream_index += 2;
        }
    }

    // Pace again after serial->UDP handling to avoid starving during outbound bursts.
    pace_to_atari(min_gap_us);
}

void sioUDPStream::sio_status()
{
    // Nothing to do here
    return;
}

void sioUDPStream::sio_process(uint32_t commanddata, uint8_t checksum)
{
    // Nothing to do here
    return;
}

#endif /* BUILD_ATARI */
