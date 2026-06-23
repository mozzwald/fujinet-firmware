#ifndef AUDIOSERVICE_H
#define AUDIOSERVICE_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#ifndef ESP_PLATFORM
#include <condition_variable>
#include <mutex>
#include <thread>
#else
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#endif

#include "AudioCommandQueue.h"
#include "AudioMixer.h"
#include "AudioSink.h"
#include "AudioTypes.h"

class AudioDecoderWav;
class AudioSource;

class AudioService
{
public:
    AudioService();
    ~AudioService();

    void start();
    void set_sink(AudioSink *sink);
    AudioStatusSnapshot status() const;
    AudioCounters counters() const;

    bool play_source(const std::string &source);
    bool play_test_tone();
    bool play_pcm(const int16_t *frames,
                  size_t frame_count,
                  const AudioFormat &format,
                  AudioSourceKind source_kind = AudioSourceKind::GENERATED_PCM);
    bool play_u8_pcm(const uint8_t *frames,
                     size_t frame_count,
                     uint32_t sample_rate,
                     AudioSourceKind source_kind = AudioSourceKind::GENERATED_PCM);
    bool append_u8_pcm(const uint8_t *frames,
                       size_t frame_count,
                       uint32_t sample_rate,
                       AudioSourceKind source_kind = AudioSourceKind::GENERATED_PCM);
    bool process_pending();
    void stop();
    void pause();
    void resume();
    void set_volume(uint8_t percent);
    bool seek(uint32_t position_ms);

private:
    bool enqueue(const AudioCommand &command);
    bool enqueue_pcm_command(const int16_t *frames,
                             size_t frame_count,
                             const AudioFormat &format,
                             AudioSourceKind source_kind,
                             bool append_to_current);
    bool enqueue_u8_pcm_command(const uint8_t *frames,
                                size_t frame_count,
                                uint32_t sample_rate,
                                AudioSourceKind source_kind,
                                bool append_to_current);
    void process_command(const AudioCommand &command);
    void process_source(const AudioCommand &command);
    void process_test_tone(const AudioCommand &command);
    void process_pcm(const AudioCommand &command);
    bool write_frames(const int16_t *frames, size_t frame_count, uint32_t generation);
    bool process_seek(AudioSource &source,
                      AudioDecoderWav &decoder,
                      const AudioFormat &format,
                      uint32_t generation);
    bool wait_if_paused(uint32_t generation);
    bool cancelled(uint32_t generation) const;
    void finish_cancelled(uint32_t generation);
    void set_stream_status(AudioSourceKind source_kind,
                           AudioCodec codec,
                           const AudioFormat &format,
                           uint32_t duration_ms);
    void update_progress(uint32_t bytes_read, size_t frames_written);
    void set_error(AudioError error);
    void set_state(AudioState state);

#ifndef ESP_PLATFORM
    void worker_loop();
#else
    void ensure_sync();
    void lock_queue();
    void unlock_queue();
    void lock_status();
    void unlock_status();
    static void worker_task(void *arg);
#endif

    AudioSink *_sink = nullptr;
    AudioMixer _mixer;
    AudioCommandQueue _commands;
    AudioStatusSnapshot _status;
    AudioCounters _counters;
    std::atomic<uint32_t> _generation{1};
    std::atomic<bool> _pause_requested{false};
    std::atomic<uint8_t> _requested_volume{100};
    std::atomic<bool> _seek_pending{false};
    std::atomic<uint32_t> _seek_position_ms{0};
    std::atomic<uint32_t> _seek_generation{0};
    std::atomic<bool> _stop_worker{false};
    uint64_t _stream_frames_written = 0;

#ifndef ESP_PLATFORM
    mutable std::mutex _lock;
    mutable std::mutex _status_lock;
    std::condition_variable _wake;
    std::thread _worker;
#else
    TaskHandle_t _worker = nullptr;
    SemaphoreHandle_t _queue_mutex = nullptr;
    SemaphoreHandle_t _status_mutex = nullptr;
#endif
};

#endif // AUDIOSERVICE_H
