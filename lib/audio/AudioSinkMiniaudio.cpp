#ifndef ESP_PLATFORM

#include "AudioSinkMiniaudio.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>

#include "miniaudio.h"

struct AudioSinkMiniaudio::Impl
{
    ma_device device;
    std::mutex mutex;
    bool initialized = false;
    bool paused = false;
};

static void audio_sink_miniaudio_callback(ma_device *device, void *output, const void *input, ma_uint32 frame_count)
{
    (void)input;

    AudioSinkMiniaudio *sink = static_cast<AudioSinkMiniaudio *>(device->pUserData);
    if (sink == nullptr || output == nullptr)
        return;

    sink->fill_output(output, frame_count, device->playback.channels);
}

AudioSinkMiniaudio::AudioSinkMiniaudio()
{
    _impl = new Impl();
}

AudioSinkMiniaudio::~AudioSinkMiniaudio()
{
    close();
    delete _impl;
    _impl = nullptr;
}

bool AudioSinkMiniaudio::open(const AudioFormat &format)
{
    if (_impl == nullptr || format.channels == 0 || format.bits_per_sample != 16)
        return false;

    close();

    _format = format;
    const size_t bytes_per_second = static_cast<size_t>(format.sample_rate) * format.channels * sizeof(int16_t);
    if (!_queue.reset(bytes_per_second))
        return false;

    ma_device_config config = ma_device_config_init(ma_device_type_playback);
    config.playback.format = ma_format_s16;
    config.playback.channels = format.channels;
    config.sampleRate = format.sample_rate;
    config.dataCallback = audio_sink_miniaudio_callback;
    config.pUserData = this;

    if (ma_device_init(nullptr, &config, &_impl->device) != MA_SUCCESS)
        return false;

    _impl->initialized = true;
    _impl->paused = false;
    _open = true;
    set_volume(_volume);

    if (ma_device_start(&_impl->device) != MA_SUCCESS)
    {
        close();
        return false;
    }

    return true;
}

size_t AudioSinkMiniaudio::write(const int16_t *pcm_frames, size_t frame_count)
{
    if (!_open || _impl == nullptr || pcm_frames == nullptr || frame_count == 0)
        return 0;

    const size_t bytes_requested = frame_count * _format.channels * sizeof(int16_t);
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(pcm_frames);
    size_t bytes_written = 0;

    while (_open && bytes_written < bytes_requested)
    {
        size_t chunk_written = 0;
        {
            std::lock_guard<std::mutex> lock(_impl->mutex);
            chunk_written = _queue.write(bytes + bytes_written, bytes_requested - bytes_written);
        }

        bytes_written += chunk_written;
        if (chunk_written == 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    return bytes_written / (_format.channels * sizeof(int16_t));
}

void AudioSinkMiniaudio::fill_output(void *output, uint32_t frame_count, uint32_t channels)
{
    if (output == nullptr)
        return;

    const size_t bytes_requested = frame_count * channels * sizeof(int16_t);
    std::memset(output, 0, bytes_requested);

    if (_impl == nullptr || _impl->paused)
        return;

    if (!_impl->mutex.try_lock())
        return;

    _queue.read(static_cast<uint8_t *>(output), bytes_requested);
    _impl->mutex.unlock();
}

void AudioSinkMiniaudio::pause()
{
    if (_impl != nullptr)
        _impl->paused = true;
}

void AudioSinkMiniaudio::resume()
{
    if (_impl != nullptr)
        _impl->paused = false;
}

void AudioSinkMiniaudio::drain()
{
    while (_open)
    {
        {
            std::lock_guard<std::mutex> lock(_impl->mutex);
            if (_queue.empty())
                return;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void AudioSinkMiniaudio::close()
{
    if (_impl == nullptr)
        return;

    if (_impl->initialized)
    {
        ma_device_uninit(&_impl->device);
        _impl->initialized = false;
    }

    _queue.clear();
    _open = false;
}

void AudioSinkMiniaudio::set_volume(uint8_t percent)
{
    _volume = std::min<uint8_t>(percent, 100);
    if (_impl != nullptr && _impl->initialized)
        ma_device_set_master_volume(&_impl->device, static_cast<float>(_volume) / 100.0f);
}

bool AudioSinkMiniaudio::supported() const
{
    return true;
}

#endif // ESP_PLATFORM
