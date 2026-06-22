#ifndef AUDIOTYPES_H
#define AUDIOTYPES_H

#include <cstddef>
#include <cstdint>

enum class AudioState : uint8_t
{
    IDLE = 0,
    OPENING,
    BUFFERING,
    PLAYING,
    PAUSED,
    STOPPING,
    FINISHED,
    ERROR,
    UNSUPPORTED,
};

enum class AudioError : uint8_t
{
    NONE = 0,
    INVALID_COMMAND,
    INVALID_ARGUMENT,
    URI_TOO_LONG,
    UNSUPPORTED_URI_SCHEME,
    UNSUPPORTED_HARDWARE,
    NETWORK_UNAVAILABLE,
    DNS_FAILURE,
    CONNECTION_FAILURE,
    HTTP_ERROR,
    TLS_ERROR,
    SOURCE_NOT_FOUND,
    SOURCE_READ_FAILURE,
    UNSUPPORTED_FORMAT,
    DECODER_INITIALIZATION_FAILURE,
    DECODE_FAILURE,
    OUTPUT_INITIALIZATION_FAILURE,
    BUFFER_ALLOCATION_FAILURE,
    SEEK_UNSUPPORTED,
    CANCELLED,
    INTERNAL_ERROR,
};

enum class AudioCodec : uint8_t
{
    UNKNOWN = 0,
    PCM,
    WAV,
    MP3,
    AAC,
};

enum class AudioSourceKind : uint8_t
{
    NONE = 0,
    URI_STREAM,
    GENERATED_PCM,
    SAM,
    TEST_TONE,
};

enum class AudioCommandKind : uint8_t
{
    NONE = 0,
    SUBMIT_PCM,
    STOP,
    PAUSE,
    RESUME,
    SET_VOLUME,
};

struct AudioFormat
{
    uint32_t sample_rate = 22050;
    uint8_t channels = 1;
    uint8_t bits_per_sample = 16;
};

struct AudioStatusSnapshot
{
    AudioState state = AudioState::IDLE;
    AudioError error = AudioError::NONE;
    AudioCodec codec = AudioCodec::UNKNOWN;
    AudioSourceKind source_kind = AudioSourceKind::NONE;
    AudioFormat format;
    uint8_t volume = 100;
    uint32_t buffered_ms = 0;
    uint32_t position_ms = 0;
    uint32_t duration_ms = 0;
    uint16_t underrun_count = 0;
    uint16_t metadata_generation = 0;
};

struct AudioCounters
{
    uint32_t bytes_read = 0;
    uint32_t decoded_frames = 0;
    uint32_t underruns = 0;
    uint32_t reconnects = 0;
    uint32_t commands_queued = 0;
    uint32_t commands_rejected = 0;
    uint32_t cancellations = 0;
};

#endif // AUDIOTYPES_H
