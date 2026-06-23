#include "AudioService.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>

#include "AudioDebug.h"
#include "AudioDecoderWav.h"
#include "AudioSourceSD.h"
#include "AudioTestTone.h"

AudioService::AudioService()
    : _commands(1)
{
    _status.volume = 100;

#ifndef ESP_PLATFORM
    _worker = std::thread(&AudioService::worker_loop, this);
#endif
}

AudioService::~AudioService()
{
    stop();

#ifndef ESP_PLATFORM
    {
        std::lock_guard<std::mutex> lock(_lock);
        _stop_worker = true;
    }
    _wake.notify_all();
    if (_worker.joinable())
        _worker.join();
#else
    _stop_worker = true;
#endif
}

#ifdef ESP_PLATFORM
void AudioService::ensure_sync()
{
    if (_queue_mutex == nullptr)
        _queue_mutex = xSemaphoreCreateMutex();
    if (_status_mutex == nullptr)
        _status_mutex = xSemaphoreCreateMutex();
}

void AudioService::lock_queue()
{
    ensure_sync();
    xSemaphoreTake(_queue_mutex, portMAX_DELAY);
}

void AudioService::unlock_queue()
{
    xSemaphoreGive(_queue_mutex);
}

void AudioService::lock_status()
{
    ensure_sync();
    xSemaphoreTake(_status_mutex, portMAX_DELAY);
}

void AudioService::unlock_status()
{
    xSemaphoreGive(_status_mutex);
}
#endif

void AudioService::start()
{
#ifdef ESP_PLATFORM
    ensure_sync();
    if (_worker == nullptr)
    {
        _stop_worker = false;
        if (xTaskCreatePinnedToCore(worker_task, "audioSvc", 8192, this, 5, &_worker, 0) != pdPASS)
            set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
    }
#endif
}

void AudioService::set_sink(AudioSink *sink)
{
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#endif
    _sink = sink;
}

AudioStatusSnapshot AudioService::status() const
{
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#else
    const_cast<AudioService *>(this)->lock_status();
#endif
    AudioStatusSnapshot snapshot = _status;
#ifdef ESP_PLATFORM
    const_cast<AudioService *>(this)->unlock_status();
#endif
    return snapshot;
}

AudioCounters AudioService::counters() const
{
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#else
    const_cast<AudioService *>(this)->lock_status();
#endif
    AudioCounters snapshot = _counters;
#ifdef ESP_PLATFORM
    const_cast<AudioService *>(this)->unlock_status();
#endif
    return snapshot;
}

bool AudioService::play_source(const std::string &source)
{
    if (source.empty() || source.size() > AudioCommand::MAX_SOURCE_LENGTH)
    {
        set_error(AudioError::INVALID_ARGUMENT);
        return false;
    }

    AudioCommand command;
    command.kind = AudioCommandKind::PLAY_SOURCE;
    command.source_length = static_cast<uint16_t>(source.size());
    std::memcpy(command.source.data(), source.data(), source.size());
    command.source[source.size()] = '\0';
    command.generation = ++_generation;
    _pause_requested = false;
    _seek_pending = false;
    return enqueue(command);
}

bool AudioService::play_test_tone()
{
    AudioCommand command;
    command.kind = AudioCommandKind::PLAY_TEST_TONE;
    command.generation = ++_generation;
    _pause_requested = false;
    _seek_pending = false;
    return enqueue(command);
}

bool AudioService::process_pending()
{
    AudioCommand command;

#ifndef ESP_PLATFORM
    {
        std::lock_guard<std::mutex> lock(_lock);
        if (!_commands.pop(&command))
            return false;
    }
#else
    lock_queue();
    if (!_commands.pop(&command))
    {
        unlock_queue();
        return false;
    }
    unlock_queue();
#endif

    process_command(command);
    return true;
}

void AudioService::stop()
{
    ++_generation;
    _pause_requested = false;
    _seek_pending = false;
    {
#ifndef ESP_PLATFORM
        std::lock_guard<std::mutex> lock(_status_lock);
#else
        lock_status();
#endif
        _counters.cancellations++;
        if (_status.state == AudioState::OPENING ||
            _status.state == AudioState::BUFFERING ||
            _status.state == AudioState::PLAYING ||
            _status.state == AudioState::PAUSED)
        {
            _status.state = AudioState::STOPPING;
        }
        else
        {
            _status.state = AudioState::IDLE;
            _status.source_kind = AudioSourceKind::NONE;
            _status.error = AudioError::NONE;
        }
#ifdef ESP_PLATFORM
        unlock_status();
#endif
    }
#ifndef ESP_PLATFORM
    _wake.notify_all();
#endif
}

void AudioService::pause()
{
    _pause_requested = true;
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#else
    lock_status();
#endif
    if (_status.state == AudioState::PLAYING || _status.state == AudioState::BUFFERING)
        _status.state = AudioState::PAUSED;
#ifdef ESP_PLATFORM
    unlock_status();
#endif
}

void AudioService::resume()
{
    _pause_requested = false;
#ifndef ESP_PLATFORM
    {
        std::lock_guard<std::mutex> lock(_status_lock);
        if (_status.state == AudioState::PAUSED)
            _status.state = AudioState::PLAYING;
    }
    _wake.notify_all();
#else
    lock_status();
    if (_status.state == AudioState::PAUSED)
        _status.state = AudioState::PLAYING;
    unlock_status();
#endif
}

void AudioService::set_volume(uint8_t percent)
{
    percent = std::min<uint8_t>(percent, 100);
    _requested_volume = percent;
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#else
    lock_status();
#endif
    _status.volume = percent;
#ifdef ESP_PLATFORM
    unlock_status();
#endif
}

bool AudioService::seek(uint32_t position_ms)
{
    const AudioStatusSnapshot snapshot = status();
    if (snapshot.source_kind != AudioSourceKind::URI_STREAM ||
        snapshot.codec != AudioCodec::WAV ||
        (snapshot.state != AudioState::BUFFERING &&
         snapshot.state != AudioState::PLAYING &&
         snapshot.state != AudioState::PAUSED))
    {
        return false;
    }

    _seek_position_ms = position_ms;
    _seek_generation = _generation.load();
    _seek_pending = true;
#ifndef ESP_PLATFORM
    _wake.notify_all();
#endif
    return true;
}

bool AudioService::enqueue(const AudioCommand &command)
{
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_lock);
#else
    lock_queue();
#endif

    // Only play requests are queued. A newer request supersedes an older
    // pending request whose generation has already been invalidated.
    if (_commands.full())
        _commands.clear();

    if (!_commands.push(command))
    {
#ifdef ESP_PLATFORM
        unlock_queue();
#endif
        {
#ifndef ESP_PLATFORM
            std::lock_guard<std::mutex> status_lock(_status_lock);
#else
            lock_status();
#endif
            _counters.commands_rejected++;
#ifdef ESP_PLATFORM
            unlock_status();
#endif
        }
        set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
        return false;
    }
#ifdef ESP_PLATFORM
    unlock_queue();
#endif

    {
#ifndef ESP_PLATFORM
        std::lock_guard<std::mutex> status_lock(_status_lock);
#else
        lock_status();
#endif
        _counters.commands_queued++;
        _status.state = AudioState::OPENING;
        _status.error = AudioError::NONE;
        _status.position_ms = 0;
        _status.duration_ms = 0;
        _status.buffered_ms = 0;
#ifdef ESP_PLATFORM
        unlock_status();
#endif
    }

#ifndef ESP_PLATFORM
    _wake.notify_one();
#endif
    return true;
}

void AudioService::process_command(const AudioCommand &command)
{
    if (cancelled(command.generation))
        return;

    switch (command.kind)
    {
    case AudioCommandKind::PLAY_SOURCE:
        process_source(command);
        break;
    case AudioCommandKind::PLAY_TEST_TONE:
        process_test_tone(command);
        break;
    case AudioCommandKind::NONE:
        break;
    }
}

void AudioService::process_source(const AudioCommand &command)
{
    if (_sink == nullptr || !_sink->supported())
    {
        set_error(AudioError::UNSUPPORTED_HARDWARE);
        return;
    }

    const std::string source_uri(command.source.data(), command.source_length);
    if (source_uri.rfind("sd:", 0) != 0)
    {
        set_error(AudioError::UNSUPPORTED_URI_SCHEME);
        return;
    }

    AudioSourceSD source;
    if (!source.open(source_uri))
    {
        if (!cancelled(command.generation))
            set_error(AudioError::SOURCE_NOT_FOUND);
        return;
    }
    if (cancelled(command.generation))
    {
        source.cancel();
        source.close();
        finish_cancelled(command.generation);
        return;
    }

    AudioDecoderWav decoder;
    AudioError error = AudioError::NONE;
    if (!decoder.open(source, &error))
    {
        source.close();
        if (!cancelled(command.generation))
            set_error(error);
        else
            finish_cancelled(command.generation);
        return;
    }

    const AudioFormat format = decoder.format();
    set_stream_status(AudioSourceKind::URI_STREAM, AudioCodec::WAV, format, decoder.duration_ms());
    if (cancelled(command.generation))
    {
        source.cancel();
        source.close();
        finish_cancelled(command.generation);
        return;
    }

    if (!_sink->open(format))
    {
        source.close();
        set_error(AudioError::OUTPUT_INITIALIZATION_FAILURE);
        return;
    }
    _sink->set_volume(_requested_volume.load());
    set_state(AudioState::PLAYING);

    std::array<int16_t, 512> pcm = {};
    while (decoder.frames_remaining() != 0 ||
           (_seek_pending.load() && _seek_generation.load() == command.generation))
    {
        if (!process_seek(source, decoder, format, command.generation))
            break;
        if (!wait_if_paused(command.generation))
            break;
        if (_seek_pending.load() && _seek_generation.load() == command.generation)
            continue;
        if (decoder.frames_remaining() == 0)
            break;

        size_t frames_produced = 0;
        uint32_t bytes_consumed = 0;
        if (!decoder.decode(source, pcm.data(), pcm.size(), &frames_produced, &bytes_consumed, &error))
        {
            if (!cancelled(command.generation))
                set_error(error);
            break;
        }
        if (frames_produced == 0)
        {
            if (!cancelled(command.generation))
                set_error(AudioError::DECODE_FAILURE);
            break;
        }

        _sink->set_volume(_requested_volume.load());
        if (!write_frames(pcm.data(), frames_produced, command.generation))
            break;
        update_progress(bytes_consumed, frames_produced);
    }

    source.cancel();
    source.close();
    if (cancelled(command.generation))
    {
        _sink->close();
        finish_cancelled(command.generation);
        return;
    }

    if (status().state == AudioState::ERROR)
    {
        _sink->close();
        return;
    }

    _sink->drain();
    _sink->close();
    if (!cancelled(command.generation))
        set_state(AudioState::FINISHED);
}

void AudioService::process_test_tone(const AudioCommand &command)
{
    if (_sink == nullptr || !_sink->supported())
    {
        set_error(AudioError::UNSUPPORTED_HARDWARE);
        return;
    }

    AudioFormat format;
    format.sample_rate = 22050;
    format.channels = 1;
    format.bits_per_sample = 16;
    constexpr uint32_t total_frames = 22050;
    set_stream_status(AudioSourceKind::TEST_TONE, AudioCodec::PCM, format, 1000);

    if (!_sink->open(format))
    {
        set_error(AudioError::OUTPUT_INITIALIZATION_FAILURE);
        return;
    }
    _sink->set_volume(_requested_volume.load());
    set_state(AudioState::PLAYING);

    AudioTestTone tone(format);
    std::array<int16_t, 512> pcm = {};
    uint32_t remaining = total_frames;
    while (remaining != 0 && wait_if_paused(command.generation))
    {
        const size_t frames = std::min<size_t>(remaining, pcm.size());
        tone.generate(pcm.data(), frames);
        _sink->set_volume(_requested_volume.load());
        if (!write_frames(pcm.data(), frames, command.generation))
            break;
        update_progress(static_cast<uint32_t>(frames * sizeof(int16_t)), frames);
        remaining -= static_cast<uint32_t>(frames);
    }

    if (cancelled(command.generation))
    {
        _sink->close();
        finish_cancelled(command.generation);
        return;
    }
    if (status().state == AudioState::ERROR)
    {
        _sink->close();
        return;
    }

    _sink->drain();
    _sink->close();
    set_state(AudioState::FINISHED);
}

bool AudioService::write_frames(const int16_t *frames, size_t frame_count, uint32_t generation)
{
    if (cancelled(generation))
        return false;

    const int16_t *mixed_frames = nullptr;
    size_t mixed_frame_count = 0;
    if (!_mixer.mix_pcm(status().source_kind,
                        status().format,
                        frames,
                        frame_count,
                        &mixed_frames,
                        &mixed_frame_count))
    {
        set_error(AudioError::INVALID_ARGUMENT);
        return false;
    }

    const size_t written = _sink->write(mixed_frames, mixed_frame_count);
    if (written != mixed_frame_count)
    {
        if (!cancelled(generation))
        {
            {
#ifndef ESP_PLATFORM
                std::lock_guard<std::mutex> lock(_status_lock);
#else
                lock_status();
#endif
                _status.underrun_count++;
                _counters.underruns++;
#ifdef ESP_PLATFORM
                unlock_status();
#endif
            }
            set_error(AudioError::SOURCE_READ_FAILURE);
        }
        return false;
    }
    return true;
}

bool AudioService::process_seek(AudioSource &source,
                                AudioDecoderWav &decoder,
                                const AudioFormat &format,
                                uint32_t generation)
{
    if (!_seek_pending.load() || _seek_generation.load() != generation)
        return true;

    const uint32_t position_ms = _seek_position_ms.load();
    _seek_pending = false;

    _sink->close();
    AudioError error = AudioError::NONE;
    if (!decoder.seek_ms(source, position_ms, &error))
    {
        set_error(error);
        return false;
    }
    if (cancelled(generation))
        return false;

    if (!_sink->open(format))
    {
        set_error(AudioError::OUTPUT_INITIALIZATION_FAILURE);
        return false;
    }
    _sink->set_volume(_requested_volume.load());

    const uint32_t actual_position_ms = decoder.position_ms();
    {
#ifndef ESP_PLATFORM
        std::lock_guard<std::mutex> lock(_status_lock);
#else
        lock_status();
#endif
        _status.position_ms = actual_position_ms;
        _stream_frames_written = decoder.position_frames();
#ifdef ESP_PLATFORM
        unlock_status();
#endif
    }

    if (_pause_requested.load())
    {
        _sink->pause();
        set_state(AudioState::PAUSED);
    }
    else
    {
        set_state(AudioState::PLAYING);
    }
    return true;
}

bool AudioService::wait_if_paused(uint32_t generation)
{
    bool sink_paused = false;
    while (_pause_requested.load() && !cancelled(generation) &&
           !(_seek_pending.load() && _seek_generation.load() == generation))
    {
        if (!sink_paused)
        {
            _sink->pause();
            sink_paused = true;
        }
#ifndef ESP_PLATFORM
        std::unique_lock<std::mutex> lock(_lock);
        _wake.wait_for(lock, std::chrono::milliseconds(10));
#else
        vTaskDelay(pdMS_TO_TICKS(10));
#endif
    }

    if (sink_paused && !cancelled(generation) && !_pause_requested.load())
        _sink->resume();
    return !cancelled(generation);
}

bool AudioService::cancelled(uint32_t generation) const
{
    return _stop_worker.load() || generation != _generation.load();
}

void AudioService::finish_cancelled(uint32_t generation)
{
    (void)generation;
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#else
    lock_status();
#endif
    if (_status.state == AudioState::STOPPING)
    {
        _status.state = AudioState::IDLE;
        _status.source_kind = AudioSourceKind::NONE;
        _status.error = AudioError::NONE;
    }
#ifdef ESP_PLATFORM
    unlock_status();
#endif
}

void AudioService::set_stream_status(AudioSourceKind source_kind,
                                     AudioCodec codec,
                                     const AudioFormat &format,
                                     uint32_t duration_ms)
{
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#else
    lock_status();
#endif
    _status.state = AudioState::BUFFERING;
    _status.error = AudioError::NONE;
    _status.source_kind = source_kind;
    _status.codec = codec;
    _status.format = format;
    _status.position_ms = 0;
    _status.duration_ms = duration_ms;
    _status.buffered_ms = 0;
    _stream_frames_written = 0;
#ifdef ESP_PLATFORM
    unlock_status();
#endif
}

void AudioService::update_progress(uint32_t bytes_read, size_t frames_written)
{
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#else
    lock_status();
#endif
    _counters.bytes_read += bytes_read;
    _counters.decoded_frames += static_cast<uint32_t>(frames_written);
    _stream_frames_written += frames_written;
    if (_status.format.sample_rate != 0)
        _status.position_ms = static_cast<uint32_t>((_stream_frames_written * 1000) /
                                                    _status.format.sample_rate);
#ifdef ESP_PLATFORM
    unlock_status();
#endif
}

void AudioService::set_error(AudioError error)
{
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#else
    lock_status();
#endif
    _status.error = error;
    _status.state = error == AudioError::UNSUPPORTED_HARDWARE
                        ? AudioState::UNSUPPORTED
                        : AudioState::ERROR;
#ifdef ESP_PLATFORM
    unlock_status();
#endif
    AudioDebug_printf("audio error %u\n", static_cast<unsigned>(error));
}

void AudioService::set_state(AudioState state)
{
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#else
    lock_status();
#endif
    _status.state = state;
    if (state != AudioState::ERROR)
        _status.error = AudioError::NONE;
#ifdef ESP_PLATFORM
    unlock_status();
#endif
}

#ifndef ESP_PLATFORM
void AudioService::worker_loop()
{
    while (true)
    {
        AudioCommand command;
        {
            std::unique_lock<std::mutex> lock(_lock);
            _wake.wait(lock, [this]() { return _stop_worker || !_commands.empty(); });
            if (_stop_worker && _commands.empty())
                return;
            _commands.pop(&command);
        }

        process_command(command);
    }
}
#else
void AudioService::worker_task(void *arg)
{
    AudioService *service = static_cast<AudioService *>(arg);
    if (service == nullptr)
        vTaskDelete(nullptr);

    while (!service->_stop_worker)
    {
        bool processed = false;
        while (service->process_pending())
            processed = true;

        vTaskDelay(pdMS_TO_TICKS(processed ? 1 : 10));
    }

    service->_worker = nullptr;
    vTaskDelete(nullptr);
}
#endif
