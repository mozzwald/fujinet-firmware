#ifndef AUDIOCOMMANDQUEUE_H
#define AUDIOCOMMANDQUEUE_H

#include <array>
#include <cstddef>
#include <cstdint>

#include "AudioTypes.h"

struct AudioCommand
{
    static constexpr size_t MAX_SOURCE_LENGTH = 1024;

    AudioCommandKind kind = AudioCommandKind::NONE;
    std::array<char, MAX_SOURCE_LENGTH + 1> source = {};
    uint16_t source_length = 0;
    uint32_t generation = 0;
};

class AudioCommandQueue
{
public:
    explicit AudioCommandQueue(size_t capacity = 4);

    bool push(const AudioCommand &command);
    bool pop(AudioCommand *command);
    void clear();

    bool empty() const { return _count == 0; }
    bool full() const { return _count >= _capacity; }
    size_t size() const { return _count; }
    size_t capacity() const { return _capacity; }

private:
    static constexpr size_t MAX_CAPACITY = 1;

    size_t _capacity;
    std::array<AudioCommand, MAX_CAPACITY> _queue = {};
    size_t _head = 0;
    size_t _tail = 0;
    size_t _count = 0;
};

#endif // AUDIOCOMMANDQUEUE_H
