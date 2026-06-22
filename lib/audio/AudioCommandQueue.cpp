#include "AudioCommandQueue.h"

AudioCommandQueue::AudioCommandQueue(size_t capacity)
    : _capacity(capacity == 0 ? 1 : (capacity > MAX_CAPACITY ? MAX_CAPACITY : capacity))
{
}

bool AudioCommandQueue::push(const AudioCommand &command)
{
    if (full())
        return false;

    _queue[_tail] = command;
    _tail = (_tail + 1) % _capacity;
    ++_count;
    return true;
}

bool AudioCommandQueue::pop(AudioCommand *command)
{
    if (command == nullptr || empty())
        return false;

    *command = _queue[_head];
    _head = (_head + 1) % _capacity;
    --_count;
    return true;
}

void AudioCommandQueue::clear()
{
    _head = 0;
    _tail = 0;
    _count = 0;
}

