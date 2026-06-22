#ifndef AUDIOSINK_H
#define AUDIOSINK_H

#include <cstddef>
#include <cstdint>

#include "AudioTypes.h"

class AudioSink
{
public:
    virtual ~AudioSink() = default;

    virtual bool open(const AudioFormat &format) = 0;
    virtual size_t write(const int16_t *pcm_frames, size_t frame_count) = 0;
    virtual void pause() = 0;
    virtual void resume() = 0;
    virtual void drain() = 0;
    virtual void close() = 0;
    virtual void set_volume(uint8_t percent) = 0;
    virtual bool supported() const = 0;
};

#endif // AUDIOSINK_H
