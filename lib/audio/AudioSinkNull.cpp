#include "AudioSinkNull.h"

bool AudioSinkNull::open(const AudioFormat &format)
{
    _format = format;
    _frames_written = 0;
    _open = true;
    _paused = false;
    return true;
}

size_t AudioSinkNull::write(const int16_t *pcm_frames, size_t frame_count)
{
    if (!_open || _paused || pcm_frames == nullptr)
        return 0;

    _frames_written += frame_count;
    return frame_count;
}

void AudioSinkNull::pause()
{
    _paused = true;
}

void AudioSinkNull::resume()
{
    _paused = false;
}

void AudioSinkNull::drain()
{
}

void AudioSinkNull::close()
{
    _open = false;
    _paused = false;
}

void AudioSinkNull::set_volume(uint8_t percent)
{
    _volume = percent > 100 ? 100 : percent;
}

bool AudioSinkNull::supported() const
{
    return true;
}
