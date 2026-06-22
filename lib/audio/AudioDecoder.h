#ifndef AUDIODECODER_H
#define AUDIODECODER_H

#include <cstddef>
#include <cstdint>

#include "AudioTypes.h"

class AudioDecoder
{
public:
    virtual ~AudioDecoder() = default;

    virtual bool open(const AudioFormat &input_format) = 0;
    virtual size_t consume(const uint8_t *input, size_t input_length) = 0;
    virtual size_t produce(int16_t *output_frames, size_t frame_count) = 0;
    virtual void flush() = 0;
    virtual void reset() = 0;
    virtual AudioFormat output_format() const = 0;
    virtual AudioCodec codec() const = 0;
};

#endif // AUDIODECODER_H
