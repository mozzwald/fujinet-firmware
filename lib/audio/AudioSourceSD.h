#ifndef AUDIOSOURCESD_H
#define AUDIOSOURCESD_H

#include <cstdio>
#include <string>

#include "AudioSource.h"

class AudioSourceSD : public AudioSource
{
public:
    ~AudioSourceSD() override;

    bool open(const std::string &uri) override;
    int read(uint8_t *buffer, size_t length) override;
    void close() override;
    void cancel() override;
    bool is_seekable() const override;
    bool seek(uint32_t position) override;
    std::string content_type() const override;
    uint32_t content_length() const override;
    std::string metadata(const std::string &field) const override;

    const std::string &path() const { return _path; }
    uint32_t position() const override;

private:
    static bool normalize_uri(const std::string &uri, std::string *path);
    static bool has_wav_extension(const std::string &path);
    static bool has_mp3_extension(const std::string &path);

    FILE *_file = nullptr;
    std::string _path;
    std::string _content_type;
    uint32_t _content_length = 0;
    bool _cancelled = false;
};

#endif // AUDIOSOURCESD_H
