#include "AudioCommandQueue.h"

AudioCommandQueue::AudioCommandQueue(size_t capacity)
    : _capacity(capacity == 0 ? 1 : capacity)
{
}

bool AudioCommandQueue::push(const AudioCommand &command)
{
    if (full())
        return false;

    _queue.push_back(command);
    return true;
}

bool AudioCommandQueue::pop(AudioCommand *command)
{
    if (command == nullptr || _queue.empty())
        return false;

    *command = _queue.front();
    _queue.pop_front();
    return true;
}

void AudioCommandQueue::clear()
{
    _queue.clear();
}

