#include "AudioTestTone.h"

AudioTestTone::AudioTestTone(const AudioFormat &format)
    : _format(format)
{
}

size_t AudioTestTone::generate(int16_t *output_frames, size_t frame_count)
{
    if (output_frames == nullptr)
        return 0;

    const uint32_t period = _format.sample_rate == 0 ? 1 : _format.sample_rate / 440;
    const uint32_t half_period = period < 2 ? 1 : period / 2;

    for (size_t i = 0; i < frame_count; ++i)
    {
        output_frames[i] = (_phase % period) < half_period ? 6000 : -6000;
        _phase++;
    }

    return frame_count;
}

