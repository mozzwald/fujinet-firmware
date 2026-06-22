#ifndef AUDIORINGBUFFER_H
#define AUDIORINGBUFFER_H

#include <cstddef>
#include <cstdint>
#include <vector>

class AudioRingBuffer
{
public:
    explicit AudioRingBuffer(size_t capacity = 0);

    bool reset(size_t capacity);
    void clear();

    size_t write(const uint8_t *data, size_t length);
    size_t read(uint8_t *data, size_t length);

    size_t size() const { return _size; }
    size_t capacity() const { return _buffer.size(); }
    size_t free_space() const { return capacity() - _size; }
    bool empty() const { return _size == 0; }
    bool full() const { return _size == _buffer.size() && !_buffer.empty(); }

private:
    std::vector<uint8_t> _buffer;
    size_t _read_pos = 0;
    size_t _write_pos = 0;
    size_t _size = 0;
};

#endif // AUDIORINGBUFFER_H
