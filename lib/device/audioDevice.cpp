#include "audioDevice.h"

#include <algorithm>
#include <cstring>

#include "fujiCommandID.h"
#include "../audio/AudioTestTone.h"

namespace
{
constexpr uint8_t AUDIO_PROTOCOL_VERSION = 1;
constexpr uint16_t AUDIO_MAX_SOURCE_LENGTH = 1024;
constexpr uint16_t AUDIO_MAX_METADATA_CHUNK = 64;
constexpr size_t AUDIO_TEST_TONE_FRAMES = 22050;

enum AudioCapabilityFlags : uint8_t
{
    AUDIO_CAP_OUTPUT_AVAILABLE = 0x01,
    AUDIO_CAP_METADATA = 0x04,
    AUDIO_CAP_GENERATED_PCM = 0x08,
};

enum AudioSourceMask : uint16_t
{
    AUDIO_SOURCE_HTTP = 0x0001,
    AUDIO_SOURCE_HTTPS = 0x0002,
    AUDIO_SOURCE_SD = 0x0004,
    AUDIO_SOURCE_GENERATED = 0x0008,
};

enum AudioCodecMask : uint16_t
{
    AUDIO_CODEC_PCM = 0x0001,
    AUDIO_CODEC_WAV = 0x0002,
    AUDIO_CODEC_MP3 = 0x0004,
};

enum AudioOutputMask : uint16_t
{
    AUDIO_OUTPUT_NULL = 0x0001,
    AUDIO_OUTPUT_ESP32_DAC = 0x0002,
    AUDIO_OUTPUT_PC_MINIAUDIO = 0x0008,
};

static void put_u16le(uint8_t *dest, uint16_t value)
{
    dest[0] = static_cast<uint8_t>(value & 0xff);
    dest[1] = static_cast<uint8_t>((value >> 8) & 0xff);
}

static void put_u32le(uint8_t *dest, uint32_t value)
{
    dest[0] = static_cast<uint8_t>(value & 0xff);
    dest[1] = static_cast<uint8_t>((value >> 8) & 0xff);
    dest[2] = static_cast<uint8_t>((value >> 16) & 0xff);
    dest[3] = static_cast<uint8_t>((value >> 24) & 0xff);
}

static uint8_t audio_flags_from_status(const AudioStatusSnapshot &status)
{
    uint8_t flags = 0;
    if (status.metadata_generation != 0)
        flags |= 0x04;
    if (status.state == AudioState::BUFFERING)
        flags |= 0x10;
    return flags;
}
}

audioDevice::audioDevice()
{
#ifndef ESP_PLATFORM
    _service.set_sink(&_miniaudio_sink);
#else
    _service.set_sink(&_esp32_dac_sink);
#endif
}

void audioDevice::setup()
{
    _service.start();
}

void audioDevice::audiocmd_status()
{
    transaction_begin(TRANS_STATE::NO_GET);
    put_basic_status();
}

void audioDevice::audiocmd_capabilities(uint8_t requested_protocol)
{
    (void)requested_protocol;

    uint8_t response[16] = {};
    uint16_t output_mask = AUDIO_OUTPUT_NULL;
#ifndef ESP_PLATFORM
    output_mask |= AUDIO_OUTPUT_PC_MINIAUDIO;
#else
    if (_esp32_dac_sink.supported())
        output_mask = AUDIO_OUTPUT_ESP32_DAC;
#endif

    response[0] = AUDIO_PROTOCOL_VERSION;
    response[1] = AUDIO_PROTOCOL_VERSION;
    response[2] = AUDIO_PROTOCOL_VERSION;
    response[3] = AUDIO_CAP_METADATA | AUDIO_CAP_GENERATED_PCM;
    if (output_mask != AUDIO_OUTPUT_NULL)
        response[3] |= AUDIO_CAP_OUTPUT_AVAILABLE;
    put_u16le(&response[4], AUDIO_MAX_SOURCE_LENGTH);
    put_u16le(&response[6], AUDIO_MAX_METADATA_CHUNK);
    put_u16le(&response[8], AUDIO_SOURCE_HTTP | AUDIO_SOURCE_HTTPS | AUDIO_SOURCE_SD | AUDIO_SOURCE_GENERATED);
    put_u16le(&response[10], AUDIO_CODEC_PCM | AUDIO_CODEC_WAV | AUDIO_CODEC_MP3);
    put_u16le(&response[12], output_mask);

    transaction_begin(TRANS_STATE::NO_GET);
    transaction_put(response, sizeof(response));
}

void audioDevice::audiocmd_set_source(uint16_t length)
{
    if (length == 0 || length > AUDIO_MAX_SOURCE_LENGTH)
    {
        transaction_begin(TRANS_STATE::NO_GET);
        set_error(length > AUDIO_MAX_SOURCE_LENGTH ? AudioError::URI_TOO_LONG : AudioError::INVALID_ARGUMENT);
        transaction_error();
        return;
    }

    std::vector<char> source(length + 1, 0);
    transaction_begin(TRANS_STATE::WILL_GET);
    if (!transaction_get(source.data(), length))
    {
        set_error(AudioError::INVALID_ARGUMENT);
        transaction_error();
        return;
    }

    _source.assign(source.data(), length);
    clear_error();
    transaction_complete();
}

void audioDevice::audiocmd_play(uint8_t flags)
{
    (void)flags;
    transaction_begin(TRANS_STATE::NO_GET);

    if (_source.empty())
    {
        set_error(AudioError::INVALID_ARGUMENT);
        transaction_error();
        return;
    }

    if (!is_generated_test_source())
    {
        set_error(AudioError::UNSUPPORTED_URI_SCHEME);
        transaction_error();
        return;
    }

    if (!submit_test_tone())
    {
        set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
        transaction_error();
        return;
    }

    clear_error();
    transaction_complete();
}

void audioDevice::audiocmd_pause()
{
    transaction_begin(TRANS_STATE::NO_GET);
    _service.pause();
    transaction_complete();
}

void audioDevice::audiocmd_resume()
{
    transaction_begin(TRANS_STATE::NO_GET);
    _service.resume();
    transaction_complete();
}

void audioDevice::audiocmd_stop()
{
    transaction_begin(TRANS_STATE::NO_GET);
    _service.stop();
    transaction_complete();
}

void audioDevice::audiocmd_set_volume(uint8_t aux_volume, bool read_payload)
{
    uint8_t volume = aux_volume;

    if (read_payload)
    {
        transaction_begin(TRANS_STATE::WILL_GET);
        if (!transaction_get(&volume, sizeof(volume)))
        {
            set_error(AudioError::INVALID_ARGUMENT);
            transaction_error();
            return;
        }
    }
    else
    {
        transaction_begin(TRANS_STATE::NO_GET);
    }

    _service.set_volume(volume);
    clear_error();
    transaction_complete();
}

void audioDevice::audiocmd_get_info()
{
    transaction_begin(TRANS_STATE::NO_GET);
    put_extended_status();
}

void audioDevice::audiocmd_get_metadata(uint8_t field_id, uint16_t requested_length)
{
    const char *text = "";
    if (field_id == 4)
        text = "generated test tone";
    else if (field_id == 5)
        text = _source.c_str();

    uint8_t response[72] = {};
    const uint16_t total = static_cast<uint16_t>(std::min<size_t>(std::strlen(text), AUDIO_MAX_METADATA_CHUNK));
    const uint16_t returned = static_cast<uint16_t>(std::min<uint16_t>(total, std::min<uint16_t>(requested_length, AUDIO_MAX_METADATA_CHUNK)));

    response[0] = field_id;
    put_u16le(&response[1], 0);
    put_u16le(&response[3], total);
    put_u16le(&response[5], returned);
    std::memcpy(&response[7], text, returned);

    transaction_begin(TRANS_STATE::NO_GET);
    transaction_put(response, sizeof(response));
}

void audioDevice::audiocmd_seek(uint32_t position_ms)
{
    (void)position_ms;
    transaction_begin(TRANS_STATE::NO_GET);
    set_error(AudioError::SEEK_UNSUPPORTED);
    transaction_error();
}

void audioDevice::put_basic_status()
{
    AudioStatusSnapshot status = _service.status();
    uint8_t response[4] = {
        AUDIO_PROTOCOL_VERSION,
        static_cast<uint8_t>(status.state),
        static_cast<uint8_t>(_last_error == AudioError::NONE ? status.error : _last_error),
        audio_flags_from_status(status),
    };
    transaction_put(response, sizeof(response));
}

void audioDevice::put_extended_status()
{
    AudioStatusSnapshot status = _service.status();
    uint8_t response[32] = {};

    response[0] = AUDIO_PROTOCOL_VERSION;
    response[1] = static_cast<uint8_t>(status.state);
    response[2] = static_cast<uint8_t>(_last_error == AudioError::NONE ? status.error : _last_error);
    response[3] = audio_flags_from_status(status);
    response[4] = static_cast<uint8_t>(status.codec);
    response[5] = status.format.channels;
    response[6] = 1;
    response[7] = status.volume;
    put_u32le(&response[8], status.format.sample_rate);
    put_u32le(&response[12], 0);
    put_u32le(&response[16], status.buffered_ms);
    put_u32le(&response[20], status.position_ms);
    put_u32le(&response[24], status.duration_ms);
    put_u16le(&response[28], status.underrun_count);
    put_u16le(&response[30], status.metadata_generation);

    transaction_put(response, sizeof(response));
}

void audioDevice::set_error(AudioError error)
{
    _last_error = error;
}

void audioDevice::clear_error()
{
    _last_error = AudioError::NONE;
}

bool audioDevice::submit_test_tone()
{
    AudioFormat format;
    format.sample_rate = 22050;
    format.channels = 1;
    format.bits_per_sample = 16;

    AudioTestTone tone(format);
    std::vector<int16_t> pcm(AUDIO_TEST_TONE_FRAMES);
    tone.generate(pcm.data(), pcm.size());
    return _service.submit_pcm(AudioSourceKind::TEST_TONE, format, pcm.data(), pcm.size());
}

bool audioDevice::is_generated_test_source() const
{
    return _source == "gen:test-tone" || _source == "test:tone";
}
