#ifndef AUDIOSERVICE_H
#define AUDIOSERVICE_H

#include <cstddef>
#include <cstdint>

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

class AudioService
{
public:
    AudioService();
    ~AudioService();

    void start();
    void set_sink(AudioSink *sink);
    AudioStatusSnapshot status() const;
    AudioCounters counters() const;

    bool submit_pcm(AudioSourceKind source_kind,
                    const AudioFormat &format,
                    const int16_t *pcm_frames,
                    size_t frame_count,
                    AudioCodec codec = AudioCodec::PCM);
    bool process_pending();
    void stop();
    void pause();
    void resume();
    void set_volume(uint8_t percent);

private:
    bool enqueue(AudioCommand &command);
    void process_command(const AudioCommand &command);
    void process_submit_pcm(const AudioCommand &command);
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

    // Mutable service state is owned by AudioService. Public methods take the
    // lock long enough to update snapshots or enqueue work, then return.
    AudioSink *_sink = nullptr;
    AudioMixer _mixer;
    AudioCommandQueue _commands;
    AudioStatusSnapshot _status;
    AudioCounters _counters;
    uint32_t _generation = 1;
    bool _stop_worker = false;

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
