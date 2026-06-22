#ifndef AUDIOCOMMANDQUEUE_H
#define AUDIOCOMMANDQUEUE_H

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

#include "AudioTypes.h"

struct AudioCommand
{
    AudioCommandKind kind = AudioCommandKind::NONE;
    AudioSourceKind source_kind = AudioSourceKind::NONE;
    AudioFormat format;
    std::vector<int16_t> pcm_frames;
    uint8_t volume = 100;
    uint32_t generation = 0;
};

class AudioCommandQueue
{
public:
    explicit AudioCommandQueue(size_t capacity = 4);

    bool push(const AudioCommand &command);
    bool pop(AudioCommand *command);
    void clear();

    bool empty() const { return _queue.empty(); }
    bool full() const { return _queue.size() >= _capacity; }
    size_t size() const { return _queue.size(); }
    size_t capacity() const { return _capacity; }

private:
    size_t _capacity;
    std::deque<AudioCommand> _queue;
};

#endif // AUDIOCOMMANDQUEUE_H
