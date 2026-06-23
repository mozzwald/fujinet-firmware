#include "AudioSourceSD.h"

#include <algorithm>
#include <cctype>
#include <cstring>

#include "../FileSystem/fnFsSD.h"

AudioSourceSD::~AudioSourceSD()
{
    close();
}

bool AudioSourceSD::open(const std::string &uri)
{
    close();

    if (!normalize_uri(uri, &_path))
        return false;

    if (!fnSDFAT.running())
        return false;

    _file = fnSDFAT.file_open(_path.c_str(), "rb");
    if (_file == nullptr)
    {
        _path.clear();
        return false;
    }

    const long size = fnSDFAT.filesize(_path.c_str());
    _content_length = size > 0 ? static_cast<uint32_t>(size) : 0;
    if (has_wav_extension(_path))
        _content_type = "audio/wav";
    else if (has_mp3_extension(_path))
        _content_type = "audio/mpeg";
    else
        _content_type = "application/octet-stream";
    _cancelled = false;
    return true;
}

int AudioSourceSD::read(uint8_t *buffer, size_t length)
{
    if (_file == nullptr || _cancelled || buffer == nullptr)
        return -1;

    if (length == 0)
        return 0;

    const size_t bytes_read = std::fread(buffer, 1, length, _file);
    if (bytes_read == 0 && std::ferror(_file))
        return -1;

    return static_cast<int>(bytes_read);
}

void AudioSourceSD::close()
{
    if (_file != nullptr)
    {
        std::fclose(_file);
        _file = nullptr;
    }

    _path.clear();
    _content_type.clear();
    _content_length = 0;
    _cancelled = false;
}

void AudioSourceSD::cancel()
{
    _cancelled = true;
}

bool AudioSourceSD::is_seekable() const
{
    return _file != nullptr;
}

bool AudioSourceSD::seek(uint32_t position)
{
    if (_file == nullptr)
        return false;

    return std::fseek(_file, static_cast<long>(position), SEEK_SET) == 0;
}

std::string AudioSourceSD::content_type() const
{
    return _content_type;
}

uint32_t AudioSourceSD::content_length() const
{
    return _content_length;
}

std::string AudioSourceSD::metadata(const std::string &field) const
{
    if (field == "source")
        return _path;
    if (field == "content-type")
        return _content_type;

    return "";
}

uint32_t AudioSourceSD::position() const
{
    if (_file == nullptr)
        return 0;

    const long pos = std::ftell(_file);
    return pos > 0 ? static_cast<uint32_t>(pos) : 0;
}

bool AudioSourceSD::normalize_uri(const std::string &uri, std::string *path)
{
    if (path == nullptr || uri.rfind("sd:", 0) != 0)
        return false;

    std::string raw = uri.substr(3);
    while (raw.rfind("//", 0) == 0)
        raw.erase(0, 1);
    if (raw.empty())
        return false;
    if (raw[0] != '/')
        raw.insert(raw.begin(), '/');

    std::string normalized;
    size_t pos = 0;
    while (pos < raw.size())
    {
        while (pos < raw.size() && raw[pos] == '/')
            ++pos;

        const size_t next = raw.find('/', pos);
        const std::string segment = raw.substr(pos, next == std::string::npos ? std::string::npos : next - pos);
        if (!segment.empty())
        {
            if (segment == "." || segment == "..")
                return false;

            normalized.push_back('/');
            normalized.append(segment);
        }

        if (next == std::string::npos)
            break;
        pos = next + 1;
    }

    if (normalized.empty())
        return false;

    *path = normalized;
    return true;
}

bool AudioSourceSD::has_wav_extension(const std::string &path)
{
    if (path.size() < 4)
        return false;

    const std::string ext = path.substr(path.size() - 4);
    return std::tolower(static_cast<unsigned char>(ext[0])) == '.' &&
           std::tolower(static_cast<unsigned char>(ext[1])) == 'w' &&
           std::tolower(static_cast<unsigned char>(ext[2])) == 'a' &&
           std::tolower(static_cast<unsigned char>(ext[3])) == 'v';
}

bool AudioSourceSD::has_mp3_extension(const std::string &path)
{
    if (path.size() < 4)
        return false;

    const std::string ext = path.substr(path.size() - 4);
    return std::tolower(static_cast<unsigned char>(ext[0])) == '.' &&
           std::tolower(static_cast<unsigned char>(ext[1])) == 'm' &&
           std::tolower(static_cast<unsigned char>(ext[2])) == 'p' &&
           std::tolower(static_cast<unsigned char>(ext[3])) == '3';
}
