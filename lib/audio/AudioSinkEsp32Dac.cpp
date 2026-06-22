#include "AudioSinkEsp32Dac.h"

#include <algorithm>

#ifdef ESP_PLATFORM
#include "pinmap.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#ifndef CONFIG_IDF_TARGET_ESP32S3
#include <driver/dac_continuous.h>
#include <hal/gpio_types.h>
#endif
#endif

namespace
{
constexpr size_t DAC_CHUNK_BYTES = 1024;

static uint8_t pcm16_to_dac8(int16_t sample, uint8_t volume)
{
    int32_t scaled = static_cast<int32_t>(sample) * std::min<uint8_t>(volume, 100) / 100;
    scaled = std::max<int32_t>(-32768, std::min<int32_t>(32767, scaled));
    return static_cast<uint8_t>((scaled + 32768) >> 8);
}
}

AudioSinkEsp32Dac::~AudioSinkEsp32Dac()
{
    close();
}

bool AudioSinkEsp32Dac::supported() const
{
#if defined(ESP_PLATFORM) && !defined(CONFIG_IDF_TARGET_ESP32S3) && SOC_DAC_SUPPORTED
    return PIN_DAC1 == GPIO_NUM_25 || PIN_DAC1 == GPIO_NUM_26;
#else
    return false;
#endif
}

bool AudioSinkEsp32Dac::open(const AudioFormat &format)
{
    if (!supported() || format.channels != 1 || format.bits_per_sample != 16 || format.sample_rate == 0)
        return false;

    close();

#if defined(ESP_PLATFORM) && !defined(CONFIG_IDF_TARGET_ESP32S3) && SOC_DAC_SUPPORTED
    dac_continuous_config_t config = {};
    config.chan_mask = PIN_DAC1 == GPIO_NUM_26 ? DAC_CHANNEL_MASK_CH1 : DAC_CHANNEL_MASK_CH0;
    config.desc_num = 8;
    config.buf_size = 1024;
    config.freq_hz = format.sample_rate;
    config.offset = 0;
    config.clk_src = DAC_DIGI_CLK_SRC_DEFAULT;
    config.chan_mode = DAC_CHANNEL_MODE_SIMUL;

    dac_continuous_handle_t handle = nullptr;
    if (dac_continuous_new_channels(&config, &handle) != ESP_OK)
        return false;

    if (dac_continuous_enable(handle) != ESP_OK)
    {
        dac_continuous_del_channels(handle);
        return false;
    }

    _handle = handle;
    _format = format;
    _open = true;
    _paused = false;
    _dac_buffer.assign(DAC_CHUNK_BYTES, 128);
    return true;
#else
    return false;
#endif
}

size_t AudioSinkEsp32Dac::write(const int16_t *pcm_frames, size_t frame_count)
{
    if (!_open || _paused || pcm_frames == nullptr || frame_count == 0)
        return 0;

#if defined(ESP_PLATFORM) && !defined(CONFIG_IDF_TARGET_ESP32S3) && SOC_DAC_SUPPORTED
    dac_continuous_handle_t handle = static_cast<dac_continuous_handle_t>(_handle);
    if (handle == nullptr)
        return 0;

    size_t frames_written = 0;
    while (_open && !_paused && frames_written < frame_count)
    {
        const size_t chunk_frames = std::min(frame_count - frames_written, _dac_buffer.size());
        for (size_t i = 0; i < chunk_frames; ++i)
            _dac_buffer[i] = pcm16_to_dac8(pcm_frames[frames_written + i], _volume);

        size_t bytes_loaded = 0;
        esp_err_t err = dac_continuous_write(handle, _dac_buffer.data(), chunk_frames, &bytes_loaded, -1);
        if (err != ESP_OK)
            break;

        frames_written += bytes_loaded;
    }

    return frames_written;
#else
    return 0;
#endif
}

void AudioSinkEsp32Dac::pause()
{
    _paused = true;
}

void AudioSinkEsp32Dac::resume()
{
    _paused = false;
}

void AudioSinkEsp32Dac::drain()
{
#if defined(ESP_PLATFORM) && !defined(CONFIG_IDF_TARGET_ESP32S3) && SOC_DAC_SUPPORTED
    if (!_open || _handle == nullptr)
        return;

    std::fill(_dac_buffer.begin(), _dac_buffer.end(), 128);
    size_t bytes_loaded = 0;
    dac_continuous_write(static_cast<dac_continuous_handle_t>(_handle),
                         _dac_buffer.data(),
                         std::min<size_t>(_dac_buffer.size(), 256),
                         &bytes_loaded,
                         100);
    vTaskDelay(pdMS_TO_TICKS(20));
#endif
}

void AudioSinkEsp32Dac::close()
{
#if defined(ESP_PLATFORM) && !defined(CONFIG_IDF_TARGET_ESP32S3) && SOC_DAC_SUPPORTED
    dac_continuous_handle_t handle = static_cast<dac_continuous_handle_t>(_handle);
    if (handle != nullptr)
    {
        dac_continuous_disable(handle);
        dac_continuous_del_channels(handle);
    }
#endif

    _handle = nullptr;
    _dac_buffer.clear();
    _open = false;
    _paused = false;
}

void AudioSinkEsp32Dac::set_volume(uint8_t percent)
{
    _volume = std::min<uint8_t>(percent, 100);
}
