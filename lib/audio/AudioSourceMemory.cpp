#include "AudioSourceMemory.h"

#include <algorithm>

AudioSourceMemory::AudioSourceMemory()
{
}

AudioSourceMemory::AudioSourceMemory(const uint8_t *data, size_t length, const std::string &content_type)
{
    set_data(data, length, content_type);
}

void AudioSourceMemory::set_data(const uint8_t *data, size_t length, const std::string &content_type)
{
    _data.clear();
    if (data != nullptr && length != 0)
        _data.assign(data, data + length);

    _content_type = content_type;
    _position = 0;
    _open = false;
    _cancelled = false;
}

bool AudioSourceMemory::open(const std::string &uri)
{
    if (!uri.empty() && uri != "memory://")
        return false;

    _position = 0;
    _open = true;
    _cancelled = false;
    return true;
}

int AudioSourceMemory::read(uint8_t *buffer, size_t length)
{
    if (!_open || _cancelled || buffer == nullptr)
        return -1;

    if (length == 0 || _position >= _data.size())
        return 0;

    const size_t available = std::min(length, _data.size() - _position);
    std::copy(_data.begin() + _position, _data.begin() + _position + available, buffer);
    _position += available;
    return static_cast<int>(available);
}

void AudioSourceMemory::close()
{
    _open = false;
    _position = 0;
}

void AudioSourceMemory::cancel()
{
    _cancelled = true;
}

bool AudioSourceMemory::is_seekable() const
{
    return true;
}

bool AudioSourceMemory::seek(uint32_t position)
{
    if (!_open || position > _data.size())
        return false;

    _position = position;
    return true;
}

uint32_t AudioSourceMemory::position() const
{
    return static_cast<uint32_t>(_position);
}

std::string AudioSourceMemory::content_type() const
{
    return _content_type;
}

uint32_t AudioSourceMemory::content_length() const
{
    return static_cast<uint32_t>(_data.size());
}

std::string AudioSourceMemory::metadata(const std::string &field) const
{
    if (field == "source")
        return "memory";

    return "";
}
