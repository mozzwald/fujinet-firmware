#ifndef AUDIOSOURCEMEMORY_H
#define AUDIOSOURCEMEMORY_H

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "AudioSource.h"

class AudioSourceMemory : public AudioSource
{
public:
    AudioSourceMemory();
    AudioSourceMemory(const uint8_t *data, size_t length, const std::string &content_type);

    void set_data(const uint8_t *data, size_t length, const std::string &content_type);

    bool open(const std::string &uri) override;
    int read(uint8_t *buffer, size_t length) override;
    void close() override;
    void cancel() override;
    bool is_seekable() const override;
    bool seek(uint32_t position) override;
    uint32_t position() const override;
    std::string content_type() const override;
    uint32_t content_length() const override;
    std::string metadata(const std::string &field) const override;

private:
    std::vector<uint8_t> _data;
    std::string _content_type = "application/octet-stream";
    size_t _position = 0;
    bool _open = false;
    bool _cancelled = false;
};

#endif // AUDIOSOURCEMEMORY_H
