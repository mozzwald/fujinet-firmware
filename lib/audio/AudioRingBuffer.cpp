#include "AudioRingBuffer.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>

#ifdef ESP_PLATFORM
#include "sdkconfig.h"
#include <esp_heap_caps.h>
#endif

AudioRingBuffer::AudioRingBuffer(size_t capacity)
{
    reset(capacity);
}

AudioRingBuffer::~AudioRingBuffer()
{
#ifdef ESP_PLATFORM
    heap_caps_free(_buffer);
#else
    std::free(_buffer);
#endif
    _buffer = nullptr;
    _capacity = 0;
}

bool AudioRingBuffer::reset(size_t capacity)
{
#ifdef ESP_PLATFORM
    heap_caps_free(_buffer);
#else
    std::free(_buffer);
#endif
    _buffer = nullptr;
    _capacity = 0;
    clear();

    if (capacity == 0)
        return true;

#ifdef ESP_PLATFORM
#if CONFIG_SPIRAM
    _buffer = static_cast<uint8_t *>(heap_caps_malloc(capacity, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
#endif
    if (_buffer == nullptr)
        _buffer = static_cast<uint8_t *>(heap_caps_malloc(capacity, MALLOC_CAP_DEFAULT));
#else
    _buffer = static_cast<uint8_t *>(std::malloc(capacity));
#endif

    if (_buffer == nullptr)
        return false;

    std::memset(_buffer, 0, capacity);
    _capacity = capacity;
    return true;
}

void AudioRingBuffer::clear()
{
    _read_pos = 0;
    _write_pos = 0;
    _size = 0;
}

size_t AudioRingBuffer::write(const uint8_t *data, size_t length)
{
    if (data == nullptr || length == 0 || _buffer == nullptr || _capacity == 0)
        return 0;

    size_t written = 0;
    while (written < length && _size < _capacity)
    {
        const size_t contiguous = std::min(length - written, _capacity - _write_pos);
        const size_t available = std::min(contiguous, _capacity - _size);

        std::memcpy(_buffer + _write_pos, data + written, available);
        _write_pos = (_write_pos + available) % _capacity;
        _size += available;
        written += available;
    }

    return written;
}

size_t AudioRingBuffer::read(uint8_t *data, size_t length)
{
    if (data == nullptr || length == 0 || _buffer == nullptr || _capacity == 0)
        return 0;

    size_t read_count = 0;
    while (read_count < length && _size > 0)
    {
        const size_t contiguous = std::min(length - read_count, _capacity - _read_pos);
        const size_t available = std::min(contiguous, _size);

        std::memcpy(data + read_count, _buffer + _read_pos, available);
        _read_pos = (_read_pos + available) % _capacity;
        _size -= available;
        read_count += available;
    }

    return read_count;
}
