#ifndef AUDIORINGBUFFER_H
#define AUDIORINGBUFFER_H

#include <cstddef>
#include <cstdint>

class AudioRingBuffer
{
public:
    explicit AudioRingBuffer(size_t capacity = 0);
    ~AudioRingBuffer();

    AudioRingBuffer(const AudioRingBuffer &) = delete;
    AudioRingBuffer &operator=(const AudioRingBuffer &) = delete;

    bool reset(size_t capacity);
    void clear();

    size_t write(const uint8_t *data, size_t length);
    size_t read(uint8_t *data, size_t length);

    size_t size() const { return _size; }
    size_t capacity() const { return _capacity; }
    size_t free_space() const { return capacity() - _size; }
    bool empty() const { return _size == 0; }
    bool full() const { return _size == _capacity && _capacity != 0; }

private:
    uint8_t *_buffer = nullptr;
    size_t _capacity = 0;
    size_t _read_pos = 0;
    size_t _write_pos = 0;
    size_t _size = 0;
};

#endif // AUDIORINGBUFFER_H
