#include "AudioDecoderWav.h"

#include <algorithm>
#include <array>
#include <cstring>

namespace
{
constexpr size_t WAV_INPUT_BYTES = 2048;
}

uint16_t AudioDecoderWav::read_u16le(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t AudioDecoderWav::read_u32le(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

int16_t AudioDecoderWav::sample_to_s16(const uint8_t *sample, uint16_t bits_per_sample)
{
    if (bits_per_sample == 8)
        return static_cast<int16_t>((static_cast<int32_t>(sample[0]) - 128) << 8);

    return static_cast<int16_t>(read_u16le(sample));
}

bool AudioDecoderWav::read_exact(AudioSourceSD &source, uint8_t *buffer, size_t length)
{
    size_t offset = 0;
    while (offset < length)
    {
        const int got = source.read(buffer + offset, length - offset);
        if (got <= 0)
            return false;
        offset += static_cast<size_t>(got);
    }
    return true;
}

bool AudioDecoderWav::skip(AudioSourceSD &source, uint32_t length, uint32_t file_size)
{
    const uint64_t target = static_cast<uint64_t>(source.position()) + length;
    return target <= file_size && source.seek(static_cast<uint32_t>(target));
}

bool AudioDecoderWav::open(AudioSourceSD &source, AudioError *error)
{
    if (error != nullptr)
        *error = AudioError::NONE;

    _open = false;
    _frames_remaining = 0;

    const uint32_t file_size = source.content_length();
    uint8_t riff[12] = {};
    if (file_size < sizeof(riff) || !read_exact(source, riff, sizeof(riff)) ||
        std::memcmp(riff, "RIFF", 4) != 0 ||
        std::memcmp(&riff[8], "WAVE", 4) != 0)
    {
        if (error != nullptr)
            *error = AudioError::UNSUPPORTED_FORMAT;
        return false;
    }

    bool have_fmt = false;
    while (source.position() <= file_size && file_size - source.position() >= 8)
    {
        uint8_t header[8] = {};
        if (!read_exact(source, header, sizeof(header)))
        {
            if (error != nullptr)
                *error = AudioError::SOURCE_READ_FAILURE;
            return false;
        }

        const uint32_t chunk_size = read_u32le(&header[4]);
        const uint32_t chunk_start = source.position();
        const uint64_t padded_end = static_cast<uint64_t>(chunk_start) + chunk_size + (chunk_size & 1U);
        if (padded_end > file_size)
        {
            if (error != nullptr)
                *error = AudioError::UNSUPPORTED_FORMAT;
            return false;
        }

        if (std::memcmp(header, "fmt ", 4) == 0)
        {
            if (chunk_size < 16)
            {
                if (error != nullptr)
                    *error = AudioError::UNSUPPORTED_FORMAT;
                return false;
            }

            uint8_t fmt[16] = {};
            if (!read_exact(source, fmt, sizeof(fmt)))
            {
                if (error != nullptr)
                    *error = AudioError::SOURCE_READ_FAILURE;
                return false;
            }

            const uint16_t audio_format = read_u16le(&fmt[0]);
            _input_channels = read_u16le(&fmt[2]);
            _format.sample_rate = read_u32le(&fmt[4]);
            _input_bits_per_sample = read_u16le(&fmt[14]);
            if (audio_format != 1 || _format.sample_rate == 0 ||
                (_input_channels != 1 && _input_channels != 2) ||
                (_input_bits_per_sample != 8 && _input_bits_per_sample != 16))
            {
                if (error != nullptr)
                    *error = AudioError::UNSUPPORTED_FORMAT;
                return false;
            }

            if (!source.seek(static_cast<uint32_t>(padded_end)))
            {
                if (error != nullptr)
                    *error = AudioError::SOURCE_READ_FAILURE;
                return false;
            }
            have_fmt = true;
        }
        else if (std::memcmp(header, "data", 4) == 0)
        {
            if (!have_fmt)
            {
                if (error != nullptr)
                    *error = AudioError::UNSUPPORTED_FORMAT;
                return false;
            }

            _bytes_per_frame = static_cast<uint16_t>(_input_channels * (_input_bits_per_sample / 8));
            if (_bytes_per_frame == 0 || chunk_size == 0 || (chunk_size % _bytes_per_frame) != 0)
            {
                if (error != nullptr)
                    *error = AudioError::UNSUPPORTED_FORMAT;
                return false;
            }

            _data_size = chunk_size;
            _frames_remaining = chunk_size / _bytes_per_frame;
            _format.channels = 1;
            _format.bits_per_sample = 16;
            _duration_ms = static_cast<uint32_t>((static_cast<uint64_t>(_frames_remaining) * 1000) /
                                                 _format.sample_rate);
            _open = true;
            return true;
        }
        else if (!skip(source, chunk_size + (chunk_size & 1U), file_size))
        {
            if (error != nullptr)
                *error = AudioError::SOURCE_READ_FAILURE;
            return false;
        }
    }

    if (error != nullptr)
        *error = AudioError::UNSUPPORTED_FORMAT;
    return false;
}

bool AudioDecoderWav::decode(AudioSourceSD &source,
                             int16_t *output_frames,
                             size_t output_capacity,
                             size_t *frames_produced,
                             uint32_t *bytes_consumed,
                             AudioError *error)
{
    if (frames_produced != nullptr)
        *frames_produced = 0;
    if (bytes_consumed != nullptr)
        *bytes_consumed = 0;
    if (error != nullptr)
        *error = AudioError::NONE;

    if (!_open || output_frames == nullptr || output_capacity == 0 ||
        frames_produced == nullptr || bytes_consumed == nullptr)
    {
        if (error != nullptr)
            *error = AudioError::INVALID_ARGUMENT;
        return false;
    }

    if (_frames_remaining == 0)
        return true;

    std::array<uint8_t, WAV_INPUT_BYTES> input = {};
    const size_t max_frames_by_input = input.size() / _bytes_per_frame;
    const size_t wanted_frames = std::min<size_t>(_frames_remaining,
                                                   std::min(output_capacity, max_frames_by_input));
    const size_t wanted_bytes = wanted_frames * _bytes_per_frame;
    if (!read_exact(source, input.data(), wanted_bytes))
    {
        if (error != nullptr)
            *error = AudioError::SOURCE_READ_FAILURE;
        return false;
    }

    const uint16_t bytes_per_sample = _input_bits_per_sample / 8;
    for (size_t frame = 0; frame < wanted_frames; ++frame)
    {
        const uint8_t *input_frame = &input[frame * _bytes_per_frame];
        int32_t mixed = 0;
        for (uint16_t channel = 0; channel < _input_channels; ++channel)
            mixed += sample_to_s16(input_frame + channel * bytes_per_sample, _input_bits_per_sample);
        mixed /= _input_channels;
        output_frames[frame] = static_cast<int16_t>(std::max<int32_t>(-32768, std::min<int32_t>(32767, mixed)));
    }

    _frames_remaining -= static_cast<uint32_t>(wanted_frames);
    *frames_produced = wanted_frames;
    *bytes_consumed = static_cast<uint32_t>(wanted_bytes);
    return true;
}
