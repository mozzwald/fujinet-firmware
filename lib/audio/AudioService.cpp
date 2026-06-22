#include "AudioService.h"

#include "AudioDebug.h"

AudioService::AudioService()
    : _commands(4)
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
        xTaskCreatePinnedToCore(worker_task, "audioSvc", 4096, this, 5, &_worker, 0);
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

bool AudioService::submit_pcm(AudioSourceKind source_kind,
                              const AudioFormat &format,
                              const int16_t *pcm_frames,
                              size_t frame_count,
                              AudioCodec codec)
{
    if (pcm_frames == nullptr && frame_count != 0)
    {
        set_error(AudioError::INVALID_ARGUMENT);
        return false;
    }

    AudioCommand command;
    command.kind = AudioCommandKind::SUBMIT_PCM;
    command.source_kind = source_kind;
    command.codec = codec;
    command.format = format;
    command.generation = _generation;

    if (pcm_frames != nullptr && frame_count != 0)
        command.pcm_frames.assign(pcm_frames, pcm_frames + frame_count);

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
    AudioCommand command;
    command.kind = AudioCommandKind::STOP;
    command.generation = ++_generation;
    {
#ifndef ESP_PLATFORM
        std::lock_guard<std::mutex> lock(_status_lock);
#else
        lock_status();
#endif
    _counters.cancellations++;
#ifdef ESP_PLATFORM
        unlock_status();
#endif
    }
    enqueue(command);
}

void AudioService::pause()
{
    AudioCommand command;
    command.kind = AudioCommandKind::PAUSE;
    command.generation = _generation;
    enqueue(command);
}

void AudioService::resume()
{
    AudioCommand command;
    command.kind = AudioCommandKind::RESUME;
    command.generation = _generation;
    enqueue(command);
}

void AudioService::set_volume(uint8_t percent)
{
    if (percent > 100)
        percent = 100;

    AudioCommand command;
    command.kind = AudioCommandKind::SET_VOLUME;
    command.volume = percent;
    command.generation = _generation;
    enqueue(command);
}

bool AudioService::enqueue(AudioCommand &command)
{
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_lock);
#else
    lock_queue();
#endif

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

        if (command.kind == AudioCommandKind::SUBMIT_PCM)
        {
            _status.state = AudioState::BUFFERING;
            _status.error = AudioError::NONE;
            _status.codec = command.codec;
            _status.source_kind = command.source_kind;
            _status.format = command.format;
            _status.position_ms = 0;
            _status.duration_ms = command.format.sample_rate == 0
                                      ? 0
                                      : static_cast<uint32_t>((static_cast<uint64_t>(command.pcm_frames.size()) * 1000) /
                                                              command.format.sample_rate);
        }
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
    if (command.generation != _generation && command.kind != AudioCommandKind::STOP)
        return;

    switch (command.kind)
    {
    case AudioCommandKind::SUBMIT_PCM:
        process_submit_pcm(command);
        break;
    case AudioCommandKind::STOP:
        if (_sink != nullptr)
            _sink->close();
    {
#ifndef ESP_PLATFORM
        std::lock_guard<std::mutex> lock(_lock);
#else
        lock_queue();
#endif
        _commands.clear();
#ifdef ESP_PLATFORM
        unlock_queue();
#endif
    }
    {
#ifndef ESP_PLATFORM
        std::lock_guard<std::mutex> status_lock(_status_lock);
#else
        lock_status();
#endif
        _status.state = AudioState::IDLE;
        _status.source_kind = AudioSourceKind::NONE;
#ifdef ESP_PLATFORM
        unlock_status();
#endif
    }
        break;
    case AudioCommandKind::PAUSE:
        if (_sink != nullptr)
            _sink->pause();
        set_state(AudioState::PAUSED);
        break;
    case AudioCommandKind::RESUME:
        if (_sink != nullptr)
            _sink->resume();
        set_state(AudioState::PLAYING);
        break;
    case AudioCommandKind::SET_VOLUME:
    {
#ifndef ESP_PLATFORM
        std::lock_guard<std::mutex> status_lock(_status_lock);
#else
        lock_status();
#endif
        _status.volume = command.volume;
#ifdef ESP_PLATFORM
        unlock_status();
#endif
    }
        if (_sink != nullptr)
            _sink->set_volume(command.volume);
        break;
    case AudioCommandKind::NONE:
        break;
    }
}

void AudioService::process_submit_pcm(const AudioCommand &command)
{
    if (_sink == nullptr || !_sink->supported())
    {
        {
#ifndef ESP_PLATFORM
            std::lock_guard<std::mutex> status_lock(_status_lock);
#else
            lock_status();
#endif
            _status.error = AudioError::UNSUPPORTED_HARDWARE;
            _status.state = AudioState::UNSUPPORTED;
#ifdef ESP_PLATFORM
            unlock_status();
#endif
        }
        return;
    }

    if (!_sink->open(command.format))
    {
        set_error(AudioError::OUTPUT_INITIALIZATION_FAILURE);
        return;
    }

    set_state(AudioState::PLAYING);

    const int16_t *mixed_frames = nullptr;
    size_t mixed_frame_count = 0;
    if (!_mixer.mix_pcm(command.source_kind,
                        command.format,
                        command.pcm_frames.empty() ? nullptr : command.pcm_frames.data(),
                        command.pcm_frames.size(),
                        &mixed_frames,
                        &mixed_frame_count))
    {
        set_error(AudioError::INVALID_ARGUMENT);
        return;
    }

    const size_t written = _sink->write(mixed_frames, mixed_frame_count);
    {
#ifndef ESP_PLATFORM
        std::lock_guard<std::mutex> status_lock(_status_lock);
#else
        lock_status();
#endif
    _counters.decoded_frames += static_cast<uint32_t>(written);
    _counters.bytes_read += static_cast<uint32_t>(written * command.format.channels * sizeof(int16_t));
    _status.position_ms = command.format.sample_rate == 0
                              ? 0
                              : static_cast<uint32_t>((static_cast<uint64_t>(written) * 1000) /
                                                      command.format.sample_rate);
#ifdef ESP_PLATFORM
    unlock_status();
#endif
    }

    if (written != mixed_frame_count)
    {
        {
#ifndef ESP_PLATFORM
            std::lock_guard<std::mutex> status_lock(_status_lock);
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
        return;
    }

    _sink->drain();

    if (command.generation == _generation)
        set_state(AudioState::FINISHED);
}

void AudioService::set_error(AudioError error)
{
#ifndef ESP_PLATFORM
    std::lock_guard<std::mutex> lock(_status_lock);
#else
    lock_status();
#endif
    _status.error = error;
    _status.state = AudioState::ERROR;
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
