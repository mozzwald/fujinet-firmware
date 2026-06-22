#ifndef AUDIOTESTTONE_H
#define AUDIOTESTTONE_H

#include <cstddef>
#include <cstdint>

#include "AudioTypes.h"

class AudioTestTone
{
public:
    explicit AudioTestTone(const AudioFormat &format = AudioFormat());

    size_t generate(int16_t *output_frames, size_t frame_count);
    AudioFormat format() const { return _format; }

private:
    AudioFormat _format;
    uint32_t _phase = 0;
};

#endif // AUDIOTESTTONE_H
