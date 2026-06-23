#ifndef AUDIODECODERMP3_H
#define AUDIODECODERMP3_H

#include <cstddef>
#include <cstdint>

#include "AudioSource.h"
#include "AudioTypes.h"

class AudioDecoderMp3
{
public:
    struct Impl;

    AudioDecoderMp3();
    ~AudioDecoderMp3();

    bool open(AudioSource &source, AudioError *error);
    bool decode(AudioSource &source,
                int16_t *output_frames,
                size_t output_capacity,
                size_t *frames_produced,
                uint32_t *bytes_consumed,
                AudioError *error);
    bool seek_ms(AudioSource &source, uint32_t position_ms, AudioError *error);
    void close();

    const AudioFormat &format() const { return _format; }
    uint32_t duration_ms() const { return 0; }
    uint32_t position_ms() const;
    uint32_t position_frames() const { return static_cast<uint32_t>(_frames_read); }

private:
    Impl *_impl = nullptr;
    AudioFormat _format;
    uint64_t _frames_read = 0;
    bool _open = false;
};

#endif // AUDIODECODERMP3_H
