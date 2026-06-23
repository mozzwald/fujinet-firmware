#ifndef AUDIODEVICE_H
#define AUDIODEVICE_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "bus.h"
#include "global_types.h"

#include "../audio/AudioService.h"
#include "../audio/AudioSinkNull.h"
#include "../audio/AudioTypes.h"

#ifdef ESP_PLATFORM
#include "../audio/AudioSinkEsp32Dac.h"
#else
#include "../audio/AudioSinkMiniaudio.h"
#endif

class audioDevice
{
protected:
    virtual void transaction_begin(transState_t expectMoreData) = 0;
    virtual void transaction_complete() = 0;
    virtual void transaction_error() = 0;
    virtual success_is_true transaction_get(void *data, size_t len) = 0;
    virtual void transaction_put(const void *data, size_t len, bool err = false) = 0;

public:
    audioDevice();
    virtual ~audioDevice() = default;

    virtual void setup();

    void audiocmd_status();
    void audiocmd_capabilities(uint8_t requested_protocol);
    void audiocmd_set_source(uint16_t length);
    void audiocmd_play(uint8_t flags);
    void audiocmd_pause();
    void audiocmd_resume();
    void audiocmd_stop();
    void audiocmd_set_volume(uint8_t aux_volume, bool read_payload);
    void audiocmd_get_info();
    void audiocmd_get_metadata(uint8_t field_id, uint16_t requested_length);
    void audiocmd_seek();

protected:
    AudioSinkNull _null_sink;
#ifdef ESP_PLATFORM
    AudioSinkEsp32Dac _esp32_dac_sink;
#else
    AudioSinkMiniaudio _miniaudio_sink;
#endif
    AudioService _service;
    std::string _source;
    AudioError _last_error = AudioError::NONE;

private:
    void put_basic_status();
    void put_extended_status();
    void set_error(AudioError error);
    void clear_error();
    bool submit_test_tone();
    bool submit_sd_wav();
    bool is_generated_test_source() const;
    bool is_sd_source() const;
};

#endif // AUDIODEVICE_H
