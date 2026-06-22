#include "AudioMixer.h"

bool AudioMixer::mix_pcm(AudioSourceKind source_kind,
                         const AudioFormat &format,
                         const int16_t *input_frames,
                         size_t frame_count,
                         const int16_t **output_frames,
                         size_t *output_frame_count)
{
    (void)source_kind;
    (void)format;

    if (output_frames == nullptr || output_frame_count == nullptr)
        return false;

    if (input_frames == nullptr && frame_count != 0)
        return false;

    *output_frames = input_frames;
    *output_frame_count = frame_count;
    return true;
}
