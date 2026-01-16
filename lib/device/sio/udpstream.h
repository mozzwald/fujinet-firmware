#ifndef UDPSTREAM_H
#define UDPSTREAM_H

#ifdef ESP_PLATFORM
#include <driver/ledc.h>
#include "sdkconfig.h"
#endif

#include "bus.h"

#include "fnUDP.h"

#define LEDC_TIMER_RESOLUTION  LEDC_TIMER_1_BIT

#ifdef ESP_PLATFORM
#ifdef CONFIG_IDF_TARGET_ESP32
   #define LEDC_ESP32XX_HIGH_SPEED LEDC_HIGH_SPEED_MODE
#else
   #define LEDC_ESP32XX_HIGH_SPEED LEDC_LOW_SPEED_MODE
#endif
#endif

#define UDPSTREAM_BUFFER_SIZE 2048
#define UDPSTREAM_RX_RING_SIZE 2048
#define UDPSTREAM_PACKET_TIMEOUT 5000
#define UDPSTREAM_KEEPALIVE_TIMEOUT 250000                      // MIDI Keep Alive is 300ms
#define UDPSTREAM_MIN_GAP_US_MIDI 320                           // ~1 byte at 31.25kbps
#define UDPSTREAM_MIN_GAP_US_SIO 520                            // ~1 byte at 19.2kbps
#define UDPSTREAM_MAX_BATCH_AGE_US 3000                         // Max SIO->UDP batch age before forced flush.
#define UDPSTREAM_FLUSH_THRESHOLD (UDPSTREAM_BUFFER_SIZE - 16)  // Flush when nearly full.
#define MIDI_PORT 5004
#define MIDI_BAUDRATE 31250

class sioUDPStream : public virtualDevice
{
private:
    fnUDP udpStream;

    uint8_t buf_net[UDPSTREAM_BUFFER_SIZE];
    uint8_t buf_stream[UDPSTREAM_BUFFER_SIZE];

    unsigned int buf_stream_index=0;

    uint8_t rx_ring[UDPSTREAM_RX_RING_SIZE];
    uint16_t rx_head = 0;
    uint16_t rx_tail = 0;
    uint16_t rx_count = 0;
    uint32_t rx_drop_count = 0;
    bool stream_started = false;

    uint16_t packet_seq = 0;
    uint64_t last_rx_us = 0;
    uint64_t last_tx_us = 0;
    void pace_to_atari(uint32_t min_gap_us);
    void sio_status() override;
    void sio_process(uint32_t commanddata, uint8_t checksum) override;

public:
    bool udpstreamActive = false; // If we are in udpstream mode or not
    bool udpstreamIsServer = false; // If we are connecting to a server
    bool udpstreamKeepaliveEnabled = false; // Enable 0x00 keepalive injection after 0x87
    in_addr_t udpstream_host_ip = IPADDR_NONE;
    int udpstream_port;

    void sio_enable_udpstream();  // setup udpstream
    void sio_disable_udpstream(); // stop udpstream
    void sio_handle_udpstream();  // Handle incoming & outgoing data for udpstream
};

#endif
