#include "AudioDecoderMp3.h"

#include <algorithm>
#include <cstring>

#include "AudioMiniaudioConfig.h"
#include "../../components_pc/miniaudio/miniaudio.h"

#ifdef ESP_PLATFORM
#include "debug.h"
#define MP3_DEC_TRACE(...) Debug_printf(__VA_ARGS__)
namespace
{
constexpr size_t MP3_MAX_SOURCE_READ_BYTES = 2048;
}
#else
#define MP3_DEC_TRACE(...)
#endif

struct AudioDecoderMp3::Impl
{
    ma_decoder decoder;
    AudioSource *source = nullptr;
    uint32_t last_position = 0;
    uint16_t read_log_count = 0;
    uint16_t decode_log_count = 0;
    bool initialized = false;
};

namespace
{
static ma_result mp3_read(ma_decoder *decoder, void *buffer, size_t bytes_to_read, size_t *bytes_read)
{
    if (bytes_read != nullptr)
        *bytes_read = 0;
    if (decoder == nullptr || decoder->pUserData == nullptr || buffer == nullptr)
        return MA_INVALID_ARGS;

    AudioDecoderMp3::Impl *impl = static_cast<AudioDecoderMp3::Impl *>(decoder->pUserData);
    if (impl->source == nullptr)
        return MA_INVALID_OPERATION;

#ifdef ESP_PLATFORM
    const size_t bounded_read = std::min(bytes_to_read, MP3_MAX_SOURCE_READ_BYTES);
#else
    const size_t bounded_read = bytes_to_read;
#endif
    const int got = impl->source->read(static_cast<uint8_t *>(buffer), bounded_read);
    if (got < 0)
        return MA_ERROR;

    if (bytes_read != nullptr)
        *bytes_read = static_cast<size_t>(got);
    impl->last_position = impl->source->position();
    if (impl->read_log_count < 16)
    {
        MP3_DEC_TRACE("audio mp3 read want=%u got=%d pos=%lu\r\n",
                      static_cast<unsigned>(bytes_to_read),
                      got,
                      static_cast<unsigned long>(impl->last_position));
        impl->read_log_count++;
    }
    if (got == 0)
        return MA_AT_END;
    return MA_SUCCESS;
}

static ma_result mp3_seek(ma_decoder *decoder, ma_int64 offset, ma_seek_origin origin)
{
    if (decoder == nullptr || decoder->pUserData == nullptr)
        return MA_INVALID_ARGS;

    AudioDecoderMp3::Impl *impl = static_cast<AudioDecoderMp3::Impl *>(decoder->pUserData);
    if (impl->source == nullptr || !impl->source->is_seekable())
        return MA_INVALID_OPERATION;

    int64_t target = 0;
    if (origin == ma_seek_origin_start)
        target = offset;
    else if (origin == ma_seek_origin_current)
        target = static_cast<int64_t>(impl->source->position()) + offset;
    else if (origin == ma_seek_origin_end)
        target = static_cast<int64_t>(impl->source->content_length()) + offset;
    else
        return MA_INVALID_ARGS;

    if (target < 0 || target > static_cast<int64_t>(impl->source->content_length()))
        return MA_INVALID_ARGS;

    if (!impl->source->seek(static_cast<uint32_t>(target)))
        return MA_ERROR;

    impl->last_position = static_cast<uint32_t>(target);
    return MA_SUCCESS;
}
}

AudioDecoderMp3::AudioDecoderMp3()
{
    _impl = new Impl();
}

AudioDecoderMp3::~AudioDecoderMp3()
{
    close();
    delete _impl;
    _impl = nullptr;
}

bool AudioDecoderMp3::open(AudioSource &source, AudioError *error)
{
    if (error != nullptr)
        *error = AudioError::NONE;
    if (_impl == nullptr)
    {
        if (error != nullptr)
            *error = AudioError::BUFFER_ALLOCATION_FAILURE;
        return false;
    }

    close();
    _impl->source = &source;
    _impl->last_position = source.position();
    _impl->read_log_count = 0;
    _impl->decode_log_count = 0;

    ma_decoder_config config = ma_decoder_config_init(ma_format_s16, 1, 0);
    config.encodingFormat = ma_encoding_format_mp3;

    ma_result result = ma_decoder_init(mp3_read, mp3_seek, _impl, &config, &_impl->decoder);
    if (result != MA_SUCCESS)
    {
        _impl->source = nullptr;
        if (error != nullptr)
            *error = AudioError::DECODER_INITIALIZATION_FAILURE;
        return false;
    }

    ma_format output_format = ma_format_unknown;
    ma_uint32 output_channels = 0;
    ma_uint32 output_sample_rate = 0;
    result = ma_decoder_get_data_format(&_impl->decoder, &output_format, &output_channels, &output_sample_rate, nullptr, 0);
    if (result != MA_SUCCESS || output_format != ma_format_s16 || output_channels != 1 || output_sample_rate == 0)
    {
        close();
        if (error != nullptr)
            *error = AudioError::UNSUPPORTED_FORMAT;
        return false;
    }

    _format.sample_rate = output_sample_rate;
    _format.channels = 1;
    _format.bits_per_sample = 16;
    _frames_read = 0;
    _impl->initialized = true;
    _open = true;
    return true;
}

bool AudioDecoderMp3::decode(AudioSource &source,
                             int16_t *output_frames,
                             size_t output_capacity,
                             size_t *frames_produced,
                             uint32_t *bytes_consumed,
                             AudioError *error)
{
    (void)source;
    if (frames_produced != nullptr)
        *frames_produced = 0;
    if (bytes_consumed != nullptr)
        *bytes_consumed = 0;
    if (error != nullptr)
        *error = AudioError::NONE;

    if (!_open || _impl == nullptr || !_impl->initialized || output_frames == nullptr ||
        output_capacity == 0 || frames_produced == nullptr || bytes_consumed == nullptr)
    {
        if (error != nullptr)
            *error = AudioError::INVALID_ARGUMENT;
        return false;
    }

    const uint32_t start_position = _impl->last_position;
    ma_uint64 frames_read = 0;
    if (_impl->decode_log_count < 16)
    {
        MP3_DEC_TRACE("audio mp3 decode enter capacity=%u pos=%lu\r\n",
                      static_cast<unsigned>(output_capacity),
                      static_cast<unsigned long>(start_position));
    }
    const ma_result result = ma_decoder_read_pcm_frames(&_impl->decoder, output_frames, output_capacity, &frames_read);
    if (_impl->decode_log_count < 16)
    {
        MP3_DEC_TRACE("audio mp3 decode leave result=%d frames=%u pos=%lu\r\n",
                      static_cast<int>(result),
                      static_cast<unsigned>(frames_read),
                      static_cast<unsigned long>(_impl->last_position));
        _impl->decode_log_count++;
    }
    if (result != MA_SUCCESS && result != MA_AT_END)
    {
        if (error != nullptr)
            *error = AudioError::DECODE_FAILURE;
        return false;
    }

    *frames_produced = static_cast<size_t>(frames_read);
    *bytes_consumed = _impl->last_position >= start_position ? _impl->last_position - start_position : 0;
    _frames_read += frames_read;
    return true;
}

bool AudioDecoderMp3::seek_ms(AudioSource &source, uint32_t position_ms, AudioError *error)
{
    (void)source;
    (void)position_ms;
    if (error != nullptr)
        *error = AudioError::SEEK_UNSUPPORTED;
    return false;
}

void AudioDecoderMp3::close()
{
    if (_impl != nullptr && _impl->initialized)
        ma_decoder_uninit(&_impl->decoder);

    if (_impl != nullptr)
    {
        std::memset(&_impl->decoder, 0, sizeof(_impl->decoder));
        _impl->source = nullptr;
        _impl->last_position = 0;
        _impl->initialized = false;
    }

    _frames_read = 0;
    _open = false;
}

uint32_t AudioDecoderMp3::position_ms() const
{
    if (!_open || _format.sample_rate == 0)
        return 0;

    return static_cast<uint32_t>((_frames_read * 1000) / _format.sample_rate);
}
