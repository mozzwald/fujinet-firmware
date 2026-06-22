#ifndef AUDIOSINKESP32DAC_H
#define AUDIOSINKESP32DAC_H

#include <cstddef>
#include <cstdint>
#include <vector>

#include "AudioSink.h"

class AudioSinkEsp32Dac : public AudioSink
{
public:
    ~AudioSinkEsp32Dac() override;

    bool open(const AudioFormat &format) override;
    size_t write(const int16_t *pcm_frames, size_t frame_count) override;
    void pause() override;
    void resume() override;
    void drain() override;
    void close() override;
    void set_volume(uint8_t percent) override;
    bool supported() const override;

private:
    AudioFormat _format;
    uint8_t _volume = 100;
    bool _open = false;
    bool _paused = false;
    void *_handle = nullptr;
    std::vector<uint8_t> _dac_buffer;
};

#endif // AUDIOSINKESP32DAC_H
