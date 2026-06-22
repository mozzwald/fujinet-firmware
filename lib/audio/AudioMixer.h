#ifndef AUDIOMIXER_H
#define AUDIOMIXER_H

#include <cstddef>
#include <cstdint>

#include "AudioTypes.h"

class AudioMixer
{
public:
    bool mix_pcm(AudioSourceKind source_kind,
                 const AudioFormat &format,
                 const int16_t *input_frames,
                 size_t frame_count,
                 const int16_t **output_frames,
                 size_t *output_frame_count);
};

#endif // AUDIOMIXER_H
