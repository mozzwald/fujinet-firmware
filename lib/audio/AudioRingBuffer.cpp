#include "AudioRingBuffer.h"

#include <algorithm>

AudioRingBuffer::AudioRingBuffer(size_t capacity)
{
    reset(capacity);
}

bool AudioRingBuffer::reset(size_t capacity)
{
    _buffer.assign(capacity, 0);
    clear();
    return _buffer.size() == capacity;
}

void AudioRingBuffer::clear()
{
    _read_pos = 0;
    _write_pos = 0;
    _size = 0;
}

size_t AudioRingBuffer::write(const uint8_t *data, size_t length)
{
    if (data == nullptr || length == 0 || _buffer.empty())
        return 0;

    size_t written = 0;
    while (written < length && _size < _buffer.size())
    {
        const size_t contiguous = std::min(length - written, _buffer.size() - _write_pos);
        const size_t available = std::min(contiguous, _buffer.size() - _size);

        std::copy(data + written, data + written + available, _buffer.begin() + _write_pos);
        _write_pos = (_write_pos + available) % _buffer.size();
        _size += available;
        written += available;
    }

    return written;
}

size_t AudioRingBuffer::read(uint8_t *data, size_t length)
{
    if (data == nullptr || length == 0 || _buffer.empty())
        return 0;

    size_t read_count = 0;
    while (read_count < length && _size > 0)
    {
        const size_t contiguous = std::min(length - read_count, _buffer.size() - _read_pos);
        const size_t available = std::min(contiguous, _size);

        std::copy(_buffer.begin() + _read_pos, _buffer.begin() + _read_pos + available, data + read_count);
        _read_pos = (_read_pos + available) % _buffer.size();
        _size -= available;
        read_count += available;
    }

    return read_count;
}
