#ifndef AUDIODECODERWAV_H
#define AUDIODECODERWAV_H

#include <cstddef>
#include <cstdint>

#include "AudioSourceSD.h"
#include "AudioTypes.h"

class AudioDecoderWav
{
public:
    bool open(AudioSourceSD &source, AudioError *error);
    bool decode(AudioSourceSD &source,
                int16_t *output_frames,
                size_t output_capacity,
                size_t *frames_produced,
                uint32_t *bytes_consumed,
                AudioError *error);

    const AudioFormat &format() const { return _format; }
    uint32_t duration_ms() const { return _duration_ms; }
    uint32_t data_size() const { return _data_size; }
    uint32_t frames_remaining() const { return _frames_remaining; }

private:
    static uint16_t read_u16le(const uint8_t *data);
    static uint32_t read_u32le(const uint8_t *data);
    static int16_t sample_to_s16(const uint8_t *sample, uint16_t bits_per_sample);
    static bool read_exact(AudioSourceSD &source, uint8_t *buffer, size_t length);
    static bool skip(AudioSourceSD &source, uint32_t length, uint32_t file_size);

    AudioFormat _format;
    uint16_t _input_channels = 0;
    uint16_t _input_bits_per_sample = 0;
    uint16_t _bytes_per_frame = 0;
    uint32_t _data_size = 0;
    uint32_t _frames_remaining = 0;
    uint32_t _duration_ms = 0;
    bool _open = false;
};

#endif // AUDIODECODERWAV_H
