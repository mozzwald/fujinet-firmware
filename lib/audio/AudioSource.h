#ifndef AUDIOSOURCE_H
#define AUDIOSOURCE_H

#include <cstddef>
#include <cstdint>
#include <string>

class AudioSource
{
public:
    virtual ~AudioSource() = default;

    virtual bool open(const std::string &uri) = 0;
    virtual int read(uint8_t *buffer, size_t length) = 0;
    virtual void close() = 0;
    virtual void cancel() = 0;
    virtual bool is_seekable() const = 0;
    virtual bool seek(uint32_t position) = 0;
    virtual uint32_t position() const = 0;
    virtual std::string content_type() const = 0;
    virtual uint32_t content_length() const = 0;
    virtual std::string metadata(const std::string &field) const = 0;
};

#endif // AUDIOSOURCE_H
