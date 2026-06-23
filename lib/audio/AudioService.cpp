#include "AudioService.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>

#include "AudioDebug.h"
#include "AudioDecoderMp3.h"
#include "AudioDecoderWav.h"
#include "AudioRingBuffer.h"
#include "AudioSourceSD.h"
#include "AudioTestTone.h"

#ifdef ESP_PLATFORM
#include "debug.h"
#include <esp_heap_caps.h>
#define MP3_TRACE(...) Debug_printf(__VA_ARGS__)
#else
#define MP3_TRACE(...)
#endif

namespace
{
#ifdef ESP_PLATFORM
constexpr uint32_t AUDIO_WORKER_STACK_BYTES = 32768;
#endif
constexpr size_t MP3_DECODE_FRAMES = 2048;
constexpr size_t MP3_DRAIN_FRAMES = 512;
constexpr size_t MP3_PCM_QUEUE_FRAMES = 32768;
constexpr size_t MP3_PCM_PREFILL_FRAMES = 16384;
constexpr size_t MP3_PCM_REFILL_FRAMES = 16384;
#ifdef ESP_PLATFORM
constexpr uint32_t MP3_OUTPUT_STACK_BYTES = 3072;
constexpr UBaseType_t MP3_OUTPUT_STARTUP_PRIORITY = 4;
constexpr UBaseType_t MP3_OUTPUT_PLAY_PRIORITY = 6;
#define MP3_USE_OUTPUT_TASK 0
#endif

static void free_pcm_buffer(int16_t *buffer)
{
#ifdef ESP_PLATFORM
    heap_caps_free(buffer);
#else
    std::free(buffer);
#endif
}

static int16_t *allocate_pcm_buffer(size_t frame_count)
{
    if (frame_count == 0)
        return nullptr;

    const size_t bytes = frame_count * sizeof(int16_t);
#ifdef ESP_PLATFORM
#if CONFIG_SPIRAM
    int16_t *buffer = static_cast<int16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (buffer != nullptr)
        return buffer;
#endif
    return static_cast<int16_t *>(heap_caps_malloc(bytes, MALLOC_CAP_DEFAULT));
#else
    return static_cast<int16_t *>(std::malloc(bytes));
#endif
}

bool source_is_mp3(const AudioSource &source)
{
    const std::string content_type = source.content_type();
    if (content_type == "audio/mpeg" || content_type == "audio/mp3")
        return true;

    std::string source_path = source.metadata("source");
    if (source_path.size() < 4)
        return false;

    const size_t offset = source_path.size() - 4;
    return (source_path[offset] == '.') &&
           (source_path[offset + 1] == 'm' || source_path[offset + 1] == 'M') &&
           (source_path[offset + 2] == 'p' || source_path[offset + 2] == 'P') &&
           (source_path[offset + 3] == '3');
}
}

AudioService::AudioService()
    : _commands(8)
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
        if (xTaskCreatePinnedToCore(worker_task, "audioSvc", AUDIO_WORKER_STACK_BYTES, this, 5, &_worker, 0) != pdPASS)
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

    std::shared_ptr<char> source_buffer(new (std::nothrow) char[AudioCommand::MAX_SOURCE_LENGTH + 1],
                                        std::default_delete<char[]>());
    if (!source_buffer)
    {
        set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
        return false;
    }

    AudioCommand command;
    command.kind = AudioCommandKind::PLAY_SOURCE;
    command.source_length = static_cast<uint16_t>(source.size());
    command.source = source_buffer;
    std::memcpy(command.source.get(), source.data(), source.size());
    command.source.get()[source.size()] = '\0';
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

bool AudioService::play_pcm(const int16_t *frames,
                            size_t frame_count,
                            const AudioFormat &format,
                            AudioSourceKind source_kind)
{
    return enqueue_pcm_command(frames, frame_count, format, source_kind, false);
}

bool AudioService::enqueue_pcm_command(const int16_t *frames,
                                       size_t frame_count,
                                       const AudioFormat &format,
                                       AudioSourceKind source_kind,
                                       bool append_to_current)
{
    if (frames == nullptr || frame_count == 0 || format.sample_rate == 0 ||
        format.channels != 1 || format.bits_per_sample != 16)
    {
        set_error(AudioError::INVALID_ARGUMENT);
        return false;
    }

    int16_t *pcm_buffer = allocate_pcm_buffer(frame_count);
    if (pcm_buffer == nullptr)
    {
        set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
        return false;
    }
    std::memcpy(pcm_buffer, frames, frame_count * sizeof(int16_t));
    std::shared_ptr<int16_t> pcm(pcm_buffer, free_pcm_buffer);

    AudioCommand command;
    command.kind = AudioCommandKind::PLAY_PCM;
    command.pcm = pcm;
    command.pcm_frame_count = frame_count;
    command.format = format;
    command.source_kind = source_kind;
    command.append_to_current = append_to_current;
    if (append_to_current)
    {
        const AudioStatusSnapshot snapshot = status();
        const bool same_active_source =
            snapshot.source_kind == source_kind &&
            (snapshot.state == AudioState::OPENING ||
             snapshot.state == AudioState::BUFFERING ||
             snapshot.state == AudioState::PLAYING ||
             snapshot.state == AudioState::PAUSED);
        command.generation = same_active_source ? _generation.load() : ++_generation;
    }
    else
    {
        command.generation = ++_generation;
    }
    _pause_requested = false;
    _seek_pending = false;
    return enqueue(command);
}

bool AudioService::play_u8_pcm(const uint8_t *frames,
                               size_t frame_count,
                               uint32_t sample_rate,
                               AudioSourceKind source_kind)
{
    return enqueue_u8_pcm_command(frames, frame_count, sample_rate, source_kind, false);
}

bool AudioService::append_u8_pcm(const uint8_t *frames,
                                 size_t frame_count,
                                 uint32_t sample_rate,
                                 AudioSourceKind source_kind)
{
    return enqueue_u8_pcm_command(frames, frame_count, sample_rate, source_kind, true);
}

bool AudioService::enqueue_u8_pcm_command(const uint8_t *frames,
                                          size_t frame_count,
                                          uint32_t sample_rate,
                                          AudioSourceKind source_kind,
                                          bool append_to_current)
{
    if (frames == nullptr || frame_count == 0 || sample_rate == 0)
    {
        set_error(AudioError::INVALID_ARGUMENT);
        return false;
    }

    int16_t *pcm_buffer = allocate_pcm_buffer(frame_count);
    if (pcm_buffer == nullptr)
    {
        set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
        return false;
    }

    for (size_t i = 0; i < frame_count; ++i)
        pcm_buffer[i] = static_cast<int16_t>((static_cast<int32_t>(frames[i]) - 128) << 8);

    std::shared_ptr<int16_t> pcm(pcm_buffer, free_pcm_buffer);

    AudioFormat format;
    format.sample_rate = sample_rate;
    format.channels = 1;
    format.bits_per_sample = 16;

    AudioCommand command;
    command.kind = AudioCommandKind::PLAY_PCM;
    command.pcm = pcm;
    command.pcm_frame_count = frame_count;
    command.format = format;
    command.source_kind = source_kind;
    command.append_to_current = append_to_current;
    if (append_to_current)
    {
        const AudioStatusSnapshot snapshot = status();
        const bool same_active_source =
            snapshot.source_kind == source_kind &&
            (snapshot.state == AudioState::OPENING ||
             snapshot.state == AudioState::BUFFERING ||
             snapshot.state == AudioState::PLAYING ||
             snapshot.state == AudioState::PAUSED);
        command.generation = same_active_source ? _generation.load() : ++_generation;
    }
    else
    {
        command.generation = ++_generation;
    }
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

    // Interrupting commands supersede pending work. Appended PCM commands are
    // ordered chunks of the same generated source and must remain FIFO.
    if (!command.append_to_current)
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
        _status.error = AudioError::NONE;
        if (!command.append_to_current ||
            (_status.source_kind != command.source_kind ||
             (_status.state != AudioState::OPENING &&
              _status.state != AudioState::BUFFERING &&
              _status.state != AudioState::PLAYING &&
              _status.state != AudioState::PAUSED)))
        {
            _status.state = AudioState::OPENING;
            _status.source_kind = command.source_kind;
            _status.codec = command.kind == AudioCommandKind::PLAY_PCM ? AudioCodec::PCM : AudioCodec::UNKNOWN;
            _status.format = command.format;
            _status.position_ms = 0;
            _status.duration_ms = 0;
            _status.buffered_ms = 0;
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
    case AudioCommandKind::PLAY_PCM:
        process_pcm(command);
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

    if (!command.source || command.source_length == 0)
    {
        set_error(AudioError::INVALID_ARGUMENT);
        return;
    }

    const std::string source_uri(command.source.get(), command.source_length);
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

    AudioError error = AudioError::NONE;
    if (source_is_mp3(source))
    {
        AudioDecoderMp3 decoder;
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
        MP3_TRACE("audio mp3 format rate=%lu ch=%u bits=%u\r\n",
                  static_cast<unsigned long>(format.sample_rate),
                  format.channels,
                  format.bits_per_sample);
        set_stream_status(AudioSourceKind::URI_STREAM, AudioCodec::MP3, format, decoder.duration_ms());
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

        AudioRingBuffer pcm_queue;
        if (!pcm_queue.reset(MP3_PCM_QUEUE_FRAMES * sizeof(int16_t)))
        {
            source.close();
            _sink->close();
            set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
            return;
        }

#if defined(ESP_PLATFORM) && MP3_USE_OUTPUT_TASK
        SemaphoreHandle_t pcm_mutex = xSemaphoreCreateMutex();
        if (pcm_mutex == nullptr)
        {
            source.close();
            _sink->close();
            set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
            return;
        }
#endif

        auto lock_pcm_queue = [&]() {
#if defined(ESP_PLATFORM) && MP3_USE_OUTPUT_TASK
            xSemaphoreTake(pcm_mutex, portMAX_DELAY);
#endif
        };
        auto unlock_pcm_queue = [&]() {
#if defined(ESP_PLATFORM) && MP3_USE_OUTPUT_TASK
            xSemaphoreGive(pcm_mutex);
#endif
        };

        auto set_buffered_frames = [this, &format](size_t frames) {
#ifndef ESP_PLATFORM
            std::lock_guard<std::mutex> lock(_status_lock);
#else
            lock_status();
#endif
            _status.buffered_ms = format.sample_rate == 0
                                      ? 0
                                      : static_cast<uint32_t>((frames * 1000) / format.sample_rate);
#ifdef ESP_PLATFORM
            unlock_status();
#endif
        };

        auto pcm = std::unique_ptr<std::array<int16_t, MP3_DECODE_FRAMES>>(new (std::nothrow) std::array<int16_t, MP3_DECODE_FRAMES>());
        auto drain_pcm = std::unique_ptr<std::array<int16_t, MP3_DRAIN_FRAMES>>(new (std::nothrow) std::array<int16_t, MP3_DRAIN_FRAMES>());
        if (!pcm || !drain_pcm)
        {
            source.close();
            _sink->close();
#if defined(ESP_PLATFORM) && MP3_USE_OUTPUT_TASK
            vSemaphoreDelete(pcm_mutex);
#endif
            set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
            return;
        }
        bool decoder_finished = false;
        uint16_t decode_log_count = 0;

        auto drain_queue = [&]() -> bool {
            const size_t available_bytes = pcm_queue.size() & ~(sizeof(int16_t) - 1);
            if (available_bytes == 0)
                return true;

            const size_t bytes_to_read = std::min(available_bytes, drain_pcm->size() * sizeof(int16_t));
            const size_t bytes_read = pcm_queue.read(reinterpret_cast<uint8_t *>(drain_pcm->data()), bytes_to_read);
            const size_t frames_read = bytes_read / sizeof(int16_t);
            set_buffered_frames(pcm_queue.size() / sizeof(int16_t));
            if (frames_read == 0)
                return true;

            _sink->set_volume(_requested_volume.load());
            if (!write_frames(drain_pcm->data(), frames_read, command.generation))
                return false;
            update_progress(0, frames_read);
            return true;
        };

        auto fill_queue = [&](size_t target_frames) -> bool {
            const size_t target_bytes = std::min(target_frames * sizeof(int16_t), pcm_queue.capacity());
            while (!decoder_finished && pcm_queue.size() < target_bytes && wait_if_paused(command.generation))
            {
                size_t frames_produced = 0;
                uint32_t bytes_consumed = 0;
                if (!decoder.decode(source, pcm->data(), pcm->size(), &frames_produced, &bytes_consumed, &error))
                {
                    if (!cancelled(command.generation))
                        set_error(error);
                    return false;
                }
                update_progress(bytes_consumed, 0);
                if (decode_log_count < 8)
                {
                    MP3_TRACE("audio mp3 prefill frames=%u bytes=%lu queue=%u\r\n",
                              static_cast<unsigned>(frames_produced),
                              static_cast<unsigned long>(bytes_consumed),
                              static_cast<unsigned>(pcm_queue.size()));
                    decode_log_count++;
                }
                if (frames_produced == 0)
                {
                    decoder_finished = true;
                    break;
                }

                const uint8_t *bytes = reinterpret_cast<const uint8_t *>(pcm->data());
                size_t remaining = frames_produced * sizeof(int16_t);
                while (remaining != 0 && wait_if_paused(command.generation))
                {
                    const size_t written = pcm_queue.write(bytes, remaining);
                    bytes += written;
                    remaining -= written;
                    set_buffered_frames(pcm_queue.size() / sizeof(int16_t));

                    if (remaining != 0 && !drain_queue())
                        return false;
                    if (written == 0 && pcm_queue.full() && !drain_queue())
                        return false;
                }
            }
            return !cancelled(command.generation);
        };

        auto decode_one_chunk = [&]() -> bool {
            if (decoder_finished)
                return true;

            const size_t max_decode_bytes = pcm->size() * sizeof(int16_t);
            lock_pcm_queue();
            const bool has_space = pcm_queue.free_space() >= max_decode_bytes;
            unlock_pcm_queue();
            if (!has_space)
                return true;

            size_t frames_produced = 0;
            uint32_t bytes_consumed = 0;
            if (!decoder.decode(source, pcm->data(), pcm->size(), &frames_produced, &bytes_consumed, &error))
            {
                if (!cancelled(command.generation))
                    set_error(error);
                return false;
            }
            update_progress(bytes_consumed, 0);
            if (decode_log_count < 16)
            {
                MP3_TRACE("audio mp3 decode frames=%u bytes=%lu free=%u\r\n",
                          static_cast<unsigned>(frames_produced),
                          static_cast<unsigned long>(bytes_consumed),
                          static_cast<unsigned>(pcm_queue.free_space()));
                decode_log_count++;
            }
            if (frames_produced == 0)
            {
                decoder_finished = true;
                return true;
            }

            const size_t frame_bytes = frames_produced * sizeof(int16_t);
            lock_pcm_queue();
            if (pcm_queue.write(reinterpret_cast<const uint8_t *>(pcm->data()), frame_bytes) != frame_bytes)
            {
                unlock_pcm_queue();
                set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
                return false;
            }
            set_buffered_frames(pcm_queue.size() / sizeof(int16_t));
            unlock_pcm_queue();
            return !cancelled(command.generation);
        };

#if defined(ESP_PLATFORM) && MP3_USE_OUTPUT_TASK
        struct Mp3ConsumerContext
        {
            AudioService *service = nullptr;
            AudioRingBuffer *queue = nullptr;
            SemaphoreHandle_t mutex = nullptr;
            std::atomic<bool> start_requested{false};
            std::atomic<bool> producer_done{false};
            std::atomic<bool> consumer_done{false};
            std::atomic<bool> consumer_ok{true};
            uint32_t generation = 0;
            AudioFormat format;
            std::array<int16_t, MP3_DRAIN_FRAMES> output_pcm = {};
            uint16_t write_log_count = 0;
        };

        Mp3ConsumerContext *consumer_context = new Mp3ConsumerContext();
        if (consumer_context == nullptr)
        {
            source.cancel();
            source.close();
            _sink->close();
            vSemaphoreDelete(pcm_mutex);
            set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
            return;
        }

        consumer_context->service = this;
        consumer_context->queue = &pcm_queue;
        consumer_context->mutex = pcm_mutex;
        consumer_context->generation = command.generation;
        consumer_context->format = format;

        auto consumer_entry = +[](void *arg) {
            Mp3ConsumerContext *ctx = static_cast<Mp3ConsumerContext *>(arg);
            AudioService *service = ctx->service;

            while (!ctx->start_requested.load() && !service->cancelled(ctx->generation))
            {
                if (ctx->producer_done.load())
                    break;
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            MP3_TRACE("audio mp3 consumer start\r\n");

            while (!service->cancelled(ctx->generation))
            {
                if (!service->wait_if_paused(ctx->generation))
                    break;

                xSemaphoreTake(ctx->mutex, portMAX_DELAY);
                const size_t available_bytes = ctx->queue->size() & ~(sizeof(int16_t) - 1);
                const bool producer_done = ctx->producer_done.load();
                if (available_bytes == 0)
                {
                    xSemaphoreGive(ctx->mutex);
                    if (producer_done)
                        break;
                    vTaskDelay(pdMS_TO_TICKS(1));
                    continue;
                }

                const size_t bytes_to_read = std::min(available_bytes, ctx->output_pcm.size() * sizeof(int16_t));
                const size_t bytes_read = ctx->queue->read(reinterpret_cast<uint8_t *>(ctx->output_pcm.data()), bytes_to_read);
                const size_t queued_frames = ctx->queue->size() / sizeof(int16_t);
                xSemaphoreGive(ctx->mutex);

                service->lock_status();
                service->_status.buffered_ms = ctx->format.sample_rate == 0
                                                   ? 0
                                                   : static_cast<uint32_t>((queued_frames * 1000) / ctx->format.sample_rate);
                service->unlock_status();

                const size_t frames_read = bytes_read / sizeof(int16_t);
                if (frames_read == 0)
                    continue;

                service->_sink->set_volume(service->_requested_volume.load());
                const size_t written = service->_sink->write(ctx->output_pcm.data(), frames_read);
                if (ctx->write_log_count < 16)
                {
                    MP3_TRACE("audio mp3 out frames=%u written=%u queued=%u\r\n",
                              static_cast<unsigned>(frames_read),
                              static_cast<unsigned>(written),
                              static_cast<unsigned>(queued_frames));
                    ctx->write_log_count++;
                }
                if (written != frames_read)
                {
                    MP3_TRACE("audio mp3 out short write frames=%u written=%u\r\n",
                              static_cast<unsigned>(frames_read),
                              static_cast<unsigned>(written));
                    if (!service->cancelled(ctx->generation))
                    {
                        service->lock_status();
                        service->_status.underrun_count++;
                        service->_counters.underruns++;
                        service->unlock_status();
                    }
                    ctx->consumer_ok = false;
                    break;
                }
                service->update_progress(0, written);
            }

            ctx->consumer_done = true;
            MP3_TRACE("audio mp3 consumer done ok=%u producer=%u\r\n",
                      ctx->consumer_ok.load() ? 1 : 0,
                      ctx->producer_done.load() ? 1 : 0);
            vTaskDelete(nullptr);
        };

        TaskHandle_t consumer_task = nullptr;
        MP3_TRACE("audio mp3 creating consumer task\r\n");
        BaseType_t create_result = xTaskCreatePinnedToCore(consumer_entry, "audioOut", MP3_OUTPUT_STACK_BYTES, consumer_context, MP3_OUTPUT_STARTUP_PRIORITY, &consumer_task, 0);
        MP3_TRACE("audio mp3 consumer create result=%ld handle=%p\r\n",
                  static_cast<long>(create_result),
                  consumer_task);
        if (create_result != pdPASS)
        {
            MP3_TRACE("audio mp3 consumer create failed heap=%lu internal=%lu largest=%lu\r\n",
                      static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_8BIT)),
                      static_cast<unsigned long>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)),
                      static_cast<unsigned long>(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT)));
            source.cancel();
            source.close();
            _sink->close();
            vSemaphoreDelete(pcm_mutex);
            delete consumer_context;
            set_error(AudioError::BUFFER_ALLOCATION_FAILURE);
            return;
        }

        MP3_TRACE("audio mp3 starting prefill\r\n");
        if (!fill_queue(MP3_PCM_PREFILL_FRAMES))
        {
            consumer_context->producer_done = true;
            consumer_context->start_requested = true;
            while (!consumer_context->consumer_done.load())
                vTaskDelay(pdMS_TO_TICKS(1));
            source.cancel();
            source.close();
            _sink->close();
            vSemaphoreDelete(pcm_mutex);
            delete consumer_context;
            return;
        }
        MP3_TRACE("audio mp3 prefill done queued=%u finished=%u\r\n",
                  static_cast<unsigned>(pcm_queue.size()),
                  decoder_finished ? 1 : 0);
        set_state(AudioState::PLAYING);
        vTaskPrioritySet(consumer_task, MP3_OUTPUT_PLAY_PRIORITY);
        consumer_context->start_requested = true;

        while (wait_if_paused(command.generation) && !decoder_finished && consumer_context->consumer_ok.load())
        {
            lock_pcm_queue();
            const bool has_decode_space = pcm_queue.free_space() >= (pcm->size() * sizeof(int16_t));
            unlock_pcm_queue();
            if (has_decode_space)
            {
                if (!decode_one_chunk())
                    break;
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(1));
            }
        }

        consumer_context->producer_done = true;
        while (!consumer_context->consumer_done.load())
            vTaskDelay(pdMS_TO_TICKS(1));
        vSemaphoreDelete(pcm_mutex);
        delete consumer_context;
#else
        if (!fill_queue(MP3_PCM_PREFILL_FRAMES))
        {
            source.cancel();
            source.close();
            _sink->close();
            return;
        }
        MP3_TRACE("audio mp3 prefill done queued=%u finished=%u\r\n",
                  static_cast<unsigned>(pcm_queue.size()),
                  decoder_finished ? 1 : 0);
        set_state(AudioState::PLAYING);

        while (wait_if_paused(command.generation) && (!decoder_finished || !pcm_queue.empty()))
        {
            if (!drain_queue())
                break;

            if (!decoder_finished && pcm_queue.size() <= MP3_PCM_REFILL_FRAMES * sizeof(int16_t) &&
                !decode_one_chunk())
                break;
        }
#endif

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
        return;
    }

    AudioDecoderWav decoder;
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

void AudioService::process_pcm(const AudioCommand &command)
{
    if (_sink == nullptr || !_sink->supported())
    {
        set_error(AudioError::UNSUPPORTED_HARDWARE);
        return;
    }

    if (!command.pcm || command.pcm_frame_count == 0 || command.format.sample_rate == 0 ||
        command.format.channels != 1 || command.format.bits_per_sample != 16)
    {
        set_error(AudioError::INVALID_ARGUMENT);
        return;
    }

    const uint32_t duration_ms = static_cast<uint32_t>(
        (static_cast<uint64_t>(command.pcm_frame_count) * 1000) / command.format.sample_rate);
    set_stream_status(command.source_kind, AudioCodec::PCM, command.format, duration_ms);

    if (!_sink->open(command.format))
    {
        set_error(AudioError::OUTPUT_INITIALIZATION_FAILURE);
        return;
    }
    _sink->set_volume(_requested_volume.load());
    set_state(AudioState::PLAYING);

    constexpr size_t PCM_CHUNK_FRAMES = 512;
    size_t offset = 0;
    while (offset < command.pcm_frame_count && wait_if_paused(command.generation))
    {
        const size_t frames = std::min(PCM_CHUNK_FRAMES, command.pcm_frame_count - offset);
        _sink->set_volume(_requested_volume.load());
        if (!write_frames(command.pcm.get() + offset, frames, command.generation))
            break;
        update_progress(static_cast<uint32_t>(frames * sizeof(int16_t)), frames);
        offset += frames;
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
