#ifdef BUILD_ATARI

#include "netstream.h"

#include "../../include/debug.h"
#include "../../include/pinmap.h"

#include "cassette.h"
#include "fnSystem.h"
#include "utils.h"

// TODO: merge/fix this at global level
#ifdef ESP_PLATFORM
#include "fnUART.h"
#include "esp_heap_caps.h"
#define FN_BUS_LINK fnUartBUS
#else
#define FN_BUS_LINK fnSioCom
#endif

#ifndef ESP_PLATFORM
#include <arpa/inet.h>
#endif

static uint64_t netstream_time_us()
{
#ifdef ESP_PLATFORM
    return (uint64_t)esp_timer_get_time();
#else
    return (uint64_t)fnSystem.millis() * 1000ULL;
#endif
}

static inline int16_t netstream_seq_diff(uint16_t seq, uint16_t expected)
{
    return (int16_t)(seq - expected);
}

bool sioNetStream::netstream_alloc_buffers()
{
#ifdef ESP_PLATFORM
#if CONFIG_SPIRAM
    auto alloc = [](size_t n) -> void *
    {
        return heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    };
#else
    auto alloc = [](size_t n) -> void *
    {
        return heap_caps_malloc(n, MALLOC_CAP_DEFAULT);
    };
#endif
    auto free_buf = [](void *p)
    {
        if (p != nullptr)
            heap_caps_free(p);
    };
#else
    auto alloc = [](size_t n) -> void *
    {
        return malloc(n);
    };
    auto free_buf = [](void *p)
    {
        if (p != nullptr)
            free(p);
    };
#endif

    buf_net = (uint8_t *)alloc(NETSTREAM_BUFFER_SIZE);
    buf_stream = (uint8_t *)alloc(NETSTREAM_BUFFER_SIZE);
    rx_ring = (uint8_t *)alloc(NETSTREAM_RX_RING_SIZE);
    if (buf_net == nullptr || buf_stream == nullptr || rx_ring == nullptr)
    {
        free_buf(buf_net);
        free_buf(buf_stream);
        free_buf(rx_ring);
        buf_net = nullptr;
        buf_stream = nullptr;
        rx_ring = nullptr;
        return false;
    }

    for (int i = 0; i < NETSTREAM_SEQ_CACHE_SLOTS; i++)
    {
        netstream_seq_cache[i].data = (uint8_t *)alloc(NETSTREAM_BUFFER_SIZE);
        if (netstream_seq_cache[i].data == nullptr)
        {
            for (int j = 0; j <= i; j++)
            {
                free_buf(netstream_seq_cache[j].data);
                netstream_seq_cache[j].data = nullptr;
            }
            free_buf(buf_net);
            free_buf(buf_stream);
            free_buf(rx_ring);
            buf_net = nullptr;
            buf_stream = nullptr;
            rx_ring = nullptr;
            return false;
        }
    }

    return true;
}

void sioNetStream::netstream_free_buffers()
{
#ifdef ESP_PLATFORM
    auto free_buf = [](void *p)
    {
        if (p != nullptr)
            heap_caps_free(p);
    };
#else
    auto free_buf = [](void *p)
    {
        if (p != nullptr)
            free(p);
    };
#endif

    for (int i = 0; i < NETSTREAM_SEQ_CACHE_SLOTS; i++)
    {
        free_buf(netstream_seq_cache[i].data);
        netstream_seq_cache[i].data = nullptr;
    }
    free_buf(buf_net);
    free_buf(buf_stream);
    free_buf(rx_ring);
    buf_net = nullptr;
    buf_stream = nullptr;
    rx_ring = nullptr;
}

struct NetstreamAudf3Baud
{
    uint8_t audf3;
    int baud;
};

static const NetstreamAudf3Baud kNetstreamAudf3Ntsc[] = {
    {159, 300},   {204, 600},   {162, 750},   {226, 1201},  {109, 2405},
    {179, 4811},  {86, 9622},   {39, 19454},  {21, 31960},  {16, 38908},
    {15, 40677},  {14, 42614},  {13, 44744},  {12, 47099},  {11, 49716},
    {10, 52640},  {9, 55930},   {8, 59659},   {7, 63621},   {6, 68453},
    {5, 74281},   {4, 81444},   {3, 90493},   {2, 102273},  {1, 118250},
    {0, 127841},
};

static const NetstreamAudf3Baud kNetstreamAudf3Pal[] = {
    {132, 300},   {190, 600},   {151, 750},   {219, 1201},  {106, 2403},
    {177, 4819},  {85, 9638},   {39, 19276},  {21, 31668},  {16, 38553},
    {15, 40305},  {14, 42224},  {13, 44336},  {12, 46669},  {11, 49262},
    {10, 52159},  {9, 55420},   {8, 59114},   {7, 63042},   {6, 67829},
    {5, 73604},   {4, 80702},   {3, 89669},   {2, 101341},  {1, 117171},
    {0, 126673},
};

static int netstream_baud_from_audf3(uint8_t audf3, bool is_pal)
{
    const NetstreamAudf3Baud *table = is_pal ? kNetstreamAudf3Pal : kNetstreamAudf3Ntsc;
    size_t count = is_pal ? (sizeof(kNetstreamAudf3Pal) / sizeof(kNetstreamAudf3Pal[0]))
                          : (sizeof(kNetstreamAudf3Ntsc) / sizeof(kNetstreamAudf3Ntsc[0]));
    for (size_t i = 0; i < count; i++)
    {
        if (table[i].audf3 == audf3)
            return table[i].baud;
    }
    return 0;
}

#ifdef ESP_PLATFORM
static void netstream_enable_clock_pwm(int baud)
{
    if (baud <= 0)
        return;

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer.speed_mode = LEDC_ESP32XX_HIGH_SPEED;
    ledc_timer.duty_resolution = LEDC_TIMER_RESOLUTION;
    ledc_timer.timer_num = LEDC_TIMER_1;
    ledc_timer.freq_hz = baud;

    ledc_channel_config_t ledc_channel_sio_ckin = {};
    ledc_channel_sio_ckin.gpio_num = PIN_CKI;
    ledc_channel_sio_ckin.speed_mode = LEDC_ESP32XX_HIGH_SPEED;
    ledc_channel_sio_ckin.channel = LEDC_CHANNEL_1;
    ledc_channel_sio_ckin.intr_type = LEDC_INTR_DISABLE;
    ledc_channel_sio_ckin.timer_sel = LEDC_TIMER_1;
    ledc_channel_sio_ckin.duty = 1;
    ledc_channel_sio_ckin.hpoint = 0;

    ledc_channel_config(&ledc_channel_sio_ckin);
    ledc_timer_config(&ledc_timer);
    ledc_set_duty(LEDC_ESP32XX_HIGH_SPEED, LEDC_CHANNEL_1, 1);
    ledc_update_duty(LEDC_ESP32XX_HIGH_SPEED, LEDC_CHANNEL_1);
}
#endif

bool sioNetStream::ensure_netstream_ready()
{
    if (netstreamMode == NetStreamMode::UDP)
        return true;
    if (netStreamTcp.connected())
        return true;
    if (netstream_host_ip == IPADDR_NONE || netstream_port <= 0)
        return false;
    if (!netStreamTcp.connect(netstream_host_ip, (uint16_t)netstream_port))
    {
#ifdef DEBUG_NETSTREAM
        Debug_println("NETSTREAM: TCP connect failed");
#endif
        return false;
    }
    netStreamTcp.setNoDelay(true);
    if (netstreamRegisterEnabled)
    {
        const char* str = "REGISTER";
        netStreamTcp.write((const uint8_t *)str, strlen(str));
    }
    buf_stream_index = 0;
    return true;
}

void sioNetStream::pace_to_atari(uint32_t min_gap_us)
{
    // Pacing to keep NET->SIO output moving even when other paths wait.
    uint8_t send_count = 0;
    while (rx_count > 0 && send_count < 16)
    {
        uint64_t now_us = netstream_time_us();
        if ((now_us - last_tx_us) < min_gap_us)
            break;
        uint8_t out = rx_ring[rx_tail];
        rx_tail = (rx_tail + 1) % NETSTREAM_RX_RING_SIZE;
        rx_count--;
        FN_BUS_LINK.write(&out, 1);
        last_tx_us += min_gap_us;
        send_count++;
    }
}

void sioNetStream::sio_enable_netstream()
{
    int baud = 0;

    // Disable cassette so it doesn't interfere with SIO Motor Control toggle
    if (SIO.getCassette() != nullptr)
    {
        cassette_was_active = SIO.getCassette()->is_active();
        if (cassette_was_active)
            SIO.getCassette()->sio_disable_cassette();
    }
    else
    {
        cassette_was_active = false;
    }

    if (netstream_has_audf3)
        baud = netstream_baud_from_audf3(netstream_audf3, netstream_video_pal);
    if (baud <= 0 && netstream_port == MIDI_PORT)
        baud = MIDI_BAUDRATE;
    if (baud <= 0)
        baud = SIO_STANDARD_BAUDRATE;

    netstream_baud = baud;
    // Don't set baud until MOTOR asserted
    //FN_BUS_LINK.set_baudrate(netstream_baud);

#ifdef DEBUG_NETSTREAM
    Debug_printf("NETSTREAM baud: %d (AUDF3=%u %s)\n",
                 netstream_baud,
                 netstream_audf3,
                 netstream_video_pal ? "PAL" : "NTSC");
#endif

    if (!netstream_alloc_buffers())
    {
#ifdef DEBUG_NETSTREAM
        Debug_println("NETSTREAM: PSRAM buffer allocation failed");
#endif
        if (cassette_was_active && SIO.getCassette() != nullptr && SIO.getCassette()->is_mounted())
            SIO.getCassette()->sio_enable_cassette();
        cassette_was_active = false;
        return;
    }

#ifdef ESP_PLATFORM
    if (netstream_tx_clock_external || netstream_rx_clock_external)
        netstream_enable_clock_pwm(netstream_baud);
#endif

    if (netstreamMode == NetStreamMode::UDP)
    {
        // Open the UDP connection. Use ephemeral port when targeting localhost.
        uint32_t host_ip = ntohl((uint32_t)netstream_host_ip);
        bool is_loopback = (host_ip & 0xFF000000u) == 0x7F000000u;
        uint16_t local_port = is_loopback ? 0 : (uint16_t)netstream_port;
        netStreamUdp.begin(local_port);
        if (netstreamRegisterEnabled)
        {
            const char* str = "REGISTER";
            netStreamUdp.beginPacket(netstream_host_ip, netstream_port); // remote IP and port
            netStreamUdp.write((const uint8_t *)str, strlen(str));
            netStreamUdp.endPacket();
        }
    }
    else
    {
        // Open the TCP connection
        ensure_netstream_ready();
    }

    netstreamActive = true;
    last_rx_us = netstream_time_us();
    last_tx_us = last_rx_us;
    rx_head = 0;
    rx_tail = 0;
    rx_count = 0;
    rx_drop_count = 0;
    netstream_seq_gap_start_us = 0;
    netstream_seq_expected = 0;
    netstream_seq_tx = 0;
    netstream_uart_rx_total = 0;
    netstream_udp_tx_attempts = 0;
    netstream_udp_tx_fails = 0;
    for (int i = 0; i < NETSTREAM_SEQ_CACHE_SLOTS; i++)
        netstream_seq_cache[i].valid = false;
    Debug_println("NETSTREAM mode ENABLED");
}

void sioNetStream::sio_disable_netstream()
{
    netStreamTcp.stop();
    netStreamUdp.stop();
    if (cassette_was_active && SIO.getCassette() != nullptr && SIO.getCassette()->is_mounted())
        SIO.getCassette()->sio_enable_cassette();
    cassette_was_active = false;
#ifdef ESP_PLATFORM
        ledc_stop(LEDC_ESP32XX_HIGH_SPEED, LEDC_CHANNEL_1, 0);
#endif
        FN_BUS_LINK.set_baudrate(SIO_STANDARD_BAUDRATE);
#ifdef ESP_PLATFORM
    // Reset CKI pin back to output open drain high
    fnSystem.set_pin_mode(PIN_CKI, gpio_mode_t::GPIO_MODE_OUTPUT_OD);
    fnSystem.digital_write(PIN_CKI, DIGI_HIGH);
#endif
    netstreamActive = false;
    netstream_free_buffers();
    Debug_println("NETSTREAM mode DISABLED");
}

void sioNetStream::sio_handle_netstream()
{
    if (buf_net == nullptr || buf_stream == nullptr || rx_ring == nullptr)
        return;
    const uint32_t min_gap_us = (netstream_port == MIDI_PORT) ? NETSTREAM_MIN_GAP_US_MIDI : NETSTREAM_MIN_GAP_US_SIO;
    uint64_t batch_start_us = 0;
    bool batch_active = false;
    uint32_t batch_uart_rx = 0;

    auto push_bytes = [&](const uint8_t *data, int len)
    {
        for (int i = 0; i < len; i++)
        {
            if (rx_count < NETSTREAM_RX_RING_SIZE)
            {
                rx_ring[rx_head] = data[i];
                rx_head = (rx_head + 1) % NETSTREAM_RX_RING_SIZE;
                rx_count++;
            }
            else
            {
                // Drop oldest byte to keep the most recent stream data.
                rx_tail = (rx_tail + 1) % NETSTREAM_RX_RING_SIZE;
                rx_ring[rx_head] = data[i];
                rx_head = (rx_head + 1) % NETSTREAM_RX_RING_SIZE;
                rx_drop_count++;
            }
        }
    };

    auto has_seq_cache = [&]() -> bool
    {
        for (int i = 0; i < NETSTREAM_SEQ_CACHE_SLOTS; i++)
        {
            if (netstream_seq_cache[i].valid)
                return true;
        }
        return false;
    };

    auto flush_cached_in_order = [&]()
    {
        bool progressed = true;
        while (progressed)
        {
            progressed = false;
            for (int i = 0; i < NETSTREAM_SEQ_CACHE_SLOTS; i++)
            {
                if (!netstream_seq_cache[i].valid)
                    continue;
                if (netstream_seq_diff(netstream_seq_cache[i].seq, netstream_seq_expected) == 0)
                {
                    push_bytes(netstream_seq_cache[i].data, netstream_seq_cache[i].len);
                    netstream_seq_cache[i].valid = false;
                    netstream_seq_expected++;
                    progressed = true;
                    break;
                }
            }
        }
    };

    auto cache_seq_packet = [&](uint16_t seq, const uint8_t *data, uint16_t len, uint64_t now_us)
    {
        for (int i = 0; i < NETSTREAM_SEQ_CACHE_SLOTS; i++)
        {
            if (netstream_seq_cache[i].valid && netstream_seq_cache[i].seq == seq)
            {
#ifdef DEBUG_NETSTREAM
                Debug_printf("NETSTREAM dup seq %u cached, drop\n", seq);
#endif
                return;
            }
        }

        int free_idx = -1;
        int far_idx = -1;
        int16_t far_diff = 0;
        for (int i = 0; i < NETSTREAM_SEQ_CACHE_SLOTS; i++)
        {
            if (!netstream_seq_cache[i].valid)
            {
                if (free_idx < 0)
                    free_idx = i;
                continue;
            }
            int16_t diff = netstream_seq_diff(netstream_seq_cache[i].seq, netstream_seq_expected);
            if (diff > 0 && (far_idx < 0 || diff > far_diff))
            {
                far_idx = i;
                far_diff = diff;
            }
        }

        int16_t new_diff = netstream_seq_diff(seq, netstream_seq_expected);
        int target = free_idx;
        if (target < 0)
        {
            if (far_idx >= 0 && new_diff < far_diff)
                target = far_idx;
            else
            {
#ifdef DEBUG_NETSTREAM
                Debug_printf("NETSTREAM seq %u too far ahead, drop\n", seq);
#endif
                return;
            }
        }

        netstream_seq_cache[target].valid = true;
        netstream_seq_cache[target].seq = seq;
        netstream_seq_cache[target].len = len;
        memcpy(netstream_seq_cache[target].data, data, len);
        if (netstream_seq_gap_start_us == 0)
            netstream_seq_gap_start_us = now_us;
    };

    auto maybe_timeout_advance = [&](uint64_t now_us)
    {
        if (netstream_seq_gap_start_us == 0)
            return;
        if ((now_us - netstream_seq_gap_start_us) < NETSTREAM_SEQ_TIMEOUT_US)
            return;

        int best_idx = -1;
        int16_t best_diff = 0;
        for (int i = 0; i < NETSTREAM_SEQ_CACHE_SLOTS; i++)
        {
            if (!netstream_seq_cache[i].valid)
                continue;
            int16_t diff = netstream_seq_diff(netstream_seq_cache[i].seq, netstream_seq_expected);
            if (diff > 0 && (best_idx < 0 || diff < best_diff))
            {
                best_idx = i;
                best_diff = diff;
            }
        }

        if (best_idx >= 0)
        {
            uint16_t seq = netstream_seq_cache[best_idx].seq;
            uint16_t len = netstream_seq_cache[best_idx].len;
            push_bytes(netstream_seq_cache[best_idx].data, len);
            netstream_seq_cache[best_idx].valid = false;
            netstream_seq_expected = (uint16_t)(seq + 1);
            flush_cached_in_order();
        }

        netstream_seq_gap_start_us = has_seq_cache() ? now_us : 0;
    };

    auto flush_udp_out_batch = [&]()
    {
        if (buf_stream_index == 0)
            return;

        if (netstreamMode == NetStreamMode::UDP)
        {
            if (netstream_host_ip == IPADDR_NONE || netstream_port <= 0)
            {
                buf_stream_index = 0;
                batch_active = false;
                batch_start_us = 0;
                batch_uart_rx = 0;
                return;
            }
            int begin_rc = netStreamUdp.beginPacket(netstream_host_ip, netstream_port); // remote IP and port
            if (netstream_seq_enabled)
            {
                uint8_t hdr[2];
                hdr[0] = (uint8_t)((netstream_seq_tx >> 8) & 0xFF);
                hdr[1] = (uint8_t)(netstream_seq_tx & 0xFF);
                netStreamUdp.write(hdr, sizeof(hdr));
                netStreamUdp.write(buf_stream, buf_stream_index);
            }
            else
            {
                netStreamUdp.write(buf_stream, buf_stream_index);
            }
            int end_rc = -1;
            if (begin_rc)
                end_rc = netStreamUdp.endPacket();
            netstream_udp_tx_attempts++;
            if (!begin_rc || end_rc <= 0)
            {
                netstream_udp_tx_fails++;
#ifdef DEBUG_NETSTREAM
                Debug_printf("NETSTREAM UDP TX failed (begin=%d end=%d) attempts=%lu fails=%lu\n",
                             begin_rc,
                             end_rc,
                             (unsigned long)netstream_udp_tx_attempts,
                             (unsigned long)netstream_udp_tx_fails);
#endif
                fnSystem.delay_microseconds(1000);
            }
            else if (netstream_seq_enabled)
            {
                netstream_seq_tx++;
            }
        }
        else
        {
            if (!ensure_netstream_ready())
            {
                buf_stream_index = 0;
                batch_active = false;
                batch_start_us = 0;
                return;
            }
            netStreamTcp.write(buf_stream, buf_stream_index);
        }

#ifdef DEBUG_NETSTREAM
        Debug_printf("STREAM-OUT [%llu ms]: ", (unsigned long long)(netstream_time_us() / 1000ULL));
        if (netstream_seq_enabled && netstreamMode == NetStreamMode::UDP)
            Debug_printf("SEQ=%u ", netstream_seq_tx);
        Debug_printf("LEN=%u ", (unsigned)buf_stream_index);
        util_dump_bytes(buf_stream, buf_stream_index);
#endif

        buf_stream_index = 0;
        batch_active = false;
        batch_start_us = 0;
#ifdef DEBUG_NETSTREAM
        Debug_printf("NETSTREAM UART-IN total=%lu batch=%lu\n",
                     (unsigned long)netstream_uart_rx_total,
                     (unsigned long)batch_uart_rx);
#endif
        batch_uart_rx = 0;
    };

    if (netstreamMode == NetStreamMode::UDP)
    {
        // if there’s data available, read a packet
        int packetSize = netStreamUdp.parsePacket();
        if (packetSize > 0)
        {
            netStreamUdp.read(buf_net, NETSTREAM_BUFFER_SIZE);

            if (netstream_seq_enabled)
            {
                if (packetSize >= 2)
                {
                    uint16_t seq = ((uint16_t)buf_net[0] << 8) | (uint16_t)buf_net[1];
                    const uint8_t *payload = buf_net + 2;
                    uint16_t payload_len = (uint16_t)(packetSize - 2);
                    int16_t diff = netstream_seq_diff(seq, netstream_seq_expected);
                    uint64_t now_us = netstream_time_us();

                    if (diff == 0)
                    {
                        push_bytes(payload, payload_len);
                        netstream_seq_expected++;
                        flush_cached_in_order();
                        netstream_seq_gap_start_us = has_seq_cache() ? now_us : 0;
                    }
                    else if (diff > 0)
                    {
                        cache_seq_packet(seq, payload, payload_len, now_us);
                    }
                    else
                    {
                        // Duplicate or late packet, drop.
#ifdef DEBUG_NETSTREAM
                        Debug_printf("NETSTREAM dup/late seq %u (expected %u), drop\n",
                                     seq,
                                     netstream_seq_expected);
#endif
                    }
                    maybe_timeout_advance(now_us);
                }
            }
            else
            {
                // Buffer incoming UDP bytes for paced UART output.
                push_bytes(buf_net, packetSize);
            }
            last_rx_us = netstream_time_us();
#ifdef DEBUG_NETSTREAM
            Debug_printf("STREAM-IN [%llu ms]: ", (unsigned long long)(netstream_time_us() / 1000ULL));
            if (netstream_seq_enabled && packetSize >= 2)
            {
                uint16_t dbg_seq = ((uint16_t)buf_net[0] << 8) | (uint16_t)buf_net[1];
                Debug_printf("SEQ=%u ", dbg_seq);
                util_dump_bytes(buf_net + 2, packetSize - 2);
            }
            else
                util_dump_bytes(buf_net, packetSize);
#endif
        }
        if (netstream_seq_enabled && netstream_seq_gap_start_us != 0)
            maybe_timeout_advance(netstream_time_us());
    }
    else if (ensure_netstream_ready())
    {
        // if there’s data available, read from the TCP stream
        size_t available = netStreamTcp.available();
        while (available > 0)
        {
            size_t to_read = (available > NETSTREAM_BUFFER_SIZE) ? NETSTREAM_BUFFER_SIZE : available;
            int packetSize = netStreamTcp.read(buf_net, to_read);
            if (packetSize <= 0)
                break;

            // Buffer incoming TCP bytes for paced UART output.
            push_bytes(buf_net, packetSize);
            last_rx_us = netstream_time_us();
#ifdef DEBUG_NETSTREAM
            Debug_printf("STREAM-IN [%llu ms]: ", (unsigned long long)(netstream_time_us() / 1000ULL));
            util_dump_bytes(buf_net, packetSize);
#endif
            available = netStreamTcp.available();
        }
    }

    pace_to_atari(min_gap_us);

    // Read the data until there's a pause in the incoming stream
    if (FN_BUS_LINK.available() > 0)
    {
        while (true)
        {
            // Break out of NetStream mode if COMMAND is asserted
#ifdef ESP_PLATFORM
            if (fnSystem.digital_read(PIN_CMD) == DIGI_LOW)
#else
            if (FN_BUS_LINK.command_asserted())
#endif
            {
                Debug_println("CMD Asserted, stopping NetStream");
                sio_disable_netstream();
                return;
            }
            if (FN_BUS_LINK.available() > 0)
            {
                // Collect bytes read in our buffer
                int in_byte = FN_BUS_LINK.read(); // TODO apc: check for error first
                if (!batch_active)
                {
                    batch_start_us = netstream_time_us();
                    batch_active = true;
                    batch_uart_rx = 0;
                }
                buf_stream[buf_stream_index] = (unsigned char)in_byte;
                if (buf_stream_index < NETSTREAM_BUFFER_SIZE - 1)
                    buf_stream_index++;
                netstream_uart_rx_total++;
                batch_uart_rx++;
                if (buf_stream_index >= NETSTREAM_FLUSH_THRESHOLD)
                {
                    // Flush when nearly full to avoid overwrite/drops.
                    flush_udp_out_batch();
                    continue;
                }
                if (batch_active && (netstream_time_us() - batch_start_us) >= NETSTREAM_MAX_BATCH_AGE_US)
                {
                    // Flush old batches even if the stream never pauses.
                    flush_udp_out_batch();
                    continue;
                }
            }
            else
            {
                // Short, bounded waits prevent starving the pacing loop.
                const uint32_t wait_step_us = 250;
                const uint32_t wait_budget_us = 10000;
                uint32_t waited_us = 0;
                bool flushed = false;
                while (waited_us < wait_budget_us && FN_BUS_LINK.available() <= 0)
                {
                    fnSystem.delay_microseconds(wait_step_us);
                    waited_us += wait_step_us;
                    pace_to_atari(min_gap_us);
                    if (buf_stream_index >= NETSTREAM_FLUSH_THRESHOLD ||
                        (batch_active && (netstream_time_us() - batch_start_us) >= NETSTREAM_MAX_BATCH_AGE_US))
                    {
                        flush_udp_out_batch();
                        flushed = true;
                        break;
                    }
                }
                if (flushed && FN_BUS_LINK.available() > 0)
                    continue;
                if (FN_BUS_LINK.available() <= 0)
                    break;
            }
        }

        // Send what we've collected after a pause.
        flush_udp_out_batch();
    }

    // Pace again after serial->UDP handling to avoid starving during outbound bursts.
    pace_to_atari(min_gap_us);
}

void sioNetStream::sio_status()
{
    // Nothing to do here
    return;
}

void sioNetStream::sio_process(uint32_t commanddata, uint8_t checksum)
{
    // Nothing to do here
    return;
}

#endif /* BUILD_ATARI */
