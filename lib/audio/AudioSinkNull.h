#ifndef AUDIOSINKNULL_H
#define AUDIOSINKNULL_H

#include "AudioSink.h"

class AudioSinkNull : public AudioSink
{
public:
    bool open(const AudioFormat &format) override;
    size_t write(const int16_t *pcm_frames, size_t frame_count) override;
    void pause() override;
    void resume() override;
    void drain() override;
    void close() override;
    void set_volume(uint8_t percent) override;
    bool supported() const override;

    size_t frames_written() const { return _frames_written; }
    bool is_open() const { return _open; }

private:
    AudioFormat _format;
    size_t _frames_written = 0;
    uint8_t _volume = 100;
    bool _open = false;
    bool _paused = false;
};

#endif // AUDIOSINKNULL_H
