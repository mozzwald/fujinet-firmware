#include "test_audio_decoder_wav.h"

#include <cstdint>
#include <vector>

#include <unity.h>

#include "../lib/audio/AudioDecoderWav.h"
#include "../lib/audio/AudioSourceMemory.h"

namespace
{
void put_u16le(std::vector<uint8_t> &data, size_t offset, uint16_t value)
{
    data[offset] = static_cast<uint8_t>(value);
    data[offset + 1] = static_cast<uint8_t>(value >> 8);
}

void put_u32le(std::vector<uint8_t> &data, size_t offset, uint32_t value)
{
    data[offset] = static_cast<uint8_t>(value);
    data[offset + 1] = static_cast<uint8_t>(value >> 8);
    data[offset + 2] = static_cast<uint8_t>(value >> 16);
    data[offset + 3] = static_cast<uint8_t>(value >> 24);
}

std::vector<uint8_t> make_pcm_wav(uint16_t channels,
                                  uint16_t bits_per_sample,
                                  uint32_t sample_rate,
                                  const std::vector<uint8_t> &pcm)
{
    std::vector<uint8_t> wav(44 + pcm.size(), 0);
    wav[0] = 'R';
    wav[1] = 'I';
    wav[2] = 'F';
    wav[3] = 'F';
    put_u32le(wav, 4, static_cast<uint32_t>(wav.size() - 8));
    wav[8] = 'W';
    wav[9] = 'A';
    wav[10] = 'V';
    wav[11] = 'E';
    wav[12] = 'f';
    wav[13] = 'm';
    wav[14] = 't';
    wav[15] = ' ';
    put_u32le(wav, 16, 16);
    put_u16le(wav, 20, 1);
    put_u16le(wav, 22, channels);
    put_u32le(wav, 24, sample_rate);
    const uint16_t block_align = static_cast<uint16_t>(channels * (bits_per_sample / 8));
    put_u32le(wav, 28, sample_rate * block_align);
    put_u16le(wav, 32, block_align);
    put_u16le(wav, 34, bits_per_sample);
    wav[36] = 'd';
    wav[37] = 'a';
    wav[38] = 't';
    wav[39] = 'a';
    put_u32le(wav, 40, static_cast<uint32_t>(pcm.size()));
    for (size_t i = 0; i < pcm.size(); ++i)
        wav[44 + i] = pcm[i];
    return wav;
}

bool open_decoder(const std::vector<uint8_t> &wav,
                  AudioSourceMemory *source,
                  AudioDecoderWav *decoder,
                  AudioError *error)
{
    source->set_data(wav.data(), wav.size(), "audio/wav");
    if (!source->open("memory://"))
        return false;
    return decoder->open(*source, error);
}

void test_wav_decodes_and_seeks_on_frame_boundaries()
{
    const std::vector<uint8_t> wav = make_pcm_wav(1, 8, 1000, {0, 128, 255, 64});
    AudioSourceMemory source;
    AudioDecoderWav decoder;
    AudioError error = AudioError::NONE;
    TEST_ASSERT_TRUE(open_decoder(wav, &source, &decoder, &error));
    TEST_ASSERT_EQUAL_UINT32(4, decoder.duration_ms());

    int16_t frames[4] = {};
    size_t produced = 0;
    uint32_t consumed = 0;
    TEST_ASSERT_TRUE(decoder.decode(source, frames, 2, &produced, &consumed, &error));
    TEST_ASSERT_EQUAL_UINT32(2, produced);
    TEST_ASSERT_EQUAL_INT16(-32768, frames[0]);
    TEST_ASSERT_EQUAL_INT16(0, frames[1]);

    TEST_ASSERT_TRUE(decoder.seek_ms(source, 2, &error));
    TEST_ASSERT_EQUAL_UINT32(2, decoder.position_ms());
    TEST_ASSERT_TRUE(decoder.decode(source, frames, 2, &produced, &consumed, &error));
    TEST_ASSERT_EQUAL_UINT32(2, produced);
    TEST_ASSERT_EQUAL_INT16(32512, frames[0]);
    TEST_ASSERT_EQUAL_INT16(-16384, frames[1]);

    TEST_ASSERT_TRUE(decoder.seek_ms(source, 1000, &error));
    TEST_ASSERT_EQUAL_UINT32(4, decoder.position_ms());
    TEST_ASSERT_EQUAL_UINT32(0, decoder.frames_remaining());
}

void test_wav_downmixes_stereo_16_bit()
{
    const std::vector<uint8_t> wav = make_pcm_wav(2, 16, 8000,
                                                  {0xE8, 0x03, 0x18, 0xFC,
                                                   0xFF, 0x7F, 0xFF, 0x7F});
    AudioSourceMemory source;
    AudioDecoderWav decoder;
    AudioError error = AudioError::NONE;
    TEST_ASSERT_TRUE(open_decoder(wav, &source, &decoder, &error));

    int16_t frames[2] = {};
    size_t produced = 0;
    uint32_t consumed = 0;
    TEST_ASSERT_TRUE(decoder.decode(source, frames, 2, &produced, &consumed, &error));
    TEST_ASSERT_EQUAL_UINT32(2, produced);
    TEST_ASSERT_EQUAL_INT16(0, frames[0]);
    TEST_ASSERT_EQUAL_INT16(32767, frames[1]);
}

void test_wav_rejects_truncated_riff()
{
    std::vector<uint8_t> wav = make_pcm_wav(1, 8, 8000, {128, 128});
    put_u32le(wav, 4, static_cast<uint32_t>(wav.size() + 100));
    AudioSourceMemory source;
    AudioDecoderWav decoder;
    AudioError error = AudioError::NONE;
    TEST_ASSERT_FALSE(open_decoder(wav, &source, &decoder, &error));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AudioError::UNSUPPORTED_FORMAT),
                            static_cast<uint8_t>(error));
}

void test_wav_rejects_invalid_chunk_size()
{
    std::vector<uint8_t> wav = make_pcm_wav(1, 8, 8000, {128, 128});
    wav[12] = 'J';
    wav[13] = 'U';
    wav[14] = 'N';
    wav[15] = 'K';
    put_u32le(wav, 16, 0xFFFFFFFFU);
    AudioSourceMemory source;
    AudioDecoderWav decoder;
    AudioError error = AudioError::NONE;
    TEST_ASSERT_FALSE(open_decoder(wav, &source, &decoder, &error));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AudioError::UNSUPPORTED_FORMAT),
                            static_cast<uint8_t>(error));
}

void test_wav_rejects_inconsistent_format_fields()
{
    std::vector<uint8_t> wav = make_pcm_wav(1, 16, 8000, {0, 0});
    put_u16le(wav, 32, 1);
    AudioSourceMemory source;
    AudioDecoderWav decoder;
    AudioError error = AudioError::NONE;
    TEST_ASSERT_FALSE(open_decoder(wav, &source, &decoder, &error));
    TEST_ASSERT_EQUAL_UINT8(static_cast<uint8_t>(AudioError::UNSUPPORTED_FORMAT),
                            static_cast<uint8_t>(error));
}
}

void tests_audio_decoder_wav()
{
    RUN_TEST(test_wav_decodes_and_seeks_on_frame_boundaries);
    RUN_TEST(test_wav_downmixes_stereo_16_bit);
    RUN_TEST(test_wav_rejects_truncated_riff);
    RUN_TEST(test_wav_rejects_invalid_chunk_size);
    RUN_TEST(test_wav_rejects_inconsistent_format_fields);
}
