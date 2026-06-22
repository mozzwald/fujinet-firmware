#ifndef AUDIOSINKMINIAUDIO_H
#define AUDIOSINKMINIAUDIO_H

#include <cstddef>
#include <cstdint>

#include "AudioRingBuffer.h"
#include "AudioSink.h"

class AudioSinkMiniaudio : public AudioSink
{
public:
    AudioSinkMiniaudio();
    ~AudioSinkMiniaudio() override;

    bool open(const AudioFormat &format) override;
    size_t write(const int16_t *pcm_frames, size_t frame_count) override;
    void pause() override;
    void resume() override;
    void drain() override;
    void close() override;
    void set_volume(uint8_t percent) override;
    bool supported() const override;

    void fill_output(void *output, uint32_t frame_count, uint32_t channels);

private:
    struct Impl;

    Impl *_impl = nullptr;
    AudioFormat _format;
    AudioRingBuffer _queue;
    uint8_t _volume = 100;
    bool _open = false;
};

#endif // AUDIOSINKMINIAUDIO_H
