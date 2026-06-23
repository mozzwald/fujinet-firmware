#include "AudioSourceSD.h"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <cstring>

#include "../FileSystem/fnFsSD.h"

#ifdef ESP_PLATFORM
#include <esp_heap_caps.h>
#include <fcntl.h>
#include <unistd.h>

#include "debug.h"

namespace
{
constexpr size_t SD_DMA_READ_BUFFER_BYTES = 512;
}
#endif

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

#ifdef ESP_PLATFORM
    const std::string full_path = std::string(fnSDFAT.basepath()) + _path;
    _fd = ::open(full_path.c_str(), O_RDONLY);
    Debug_printf("open = %s rb : %s\r\n", _path.c_str(), _fd >= 0 ? "ok" : "error");
    if (_fd < 0)
    {
        _path.clear();
        return false;
    }
    _dma_read_buffer = static_cast<uint8_t *>(heap_caps_malloc(SD_DMA_READ_BUFFER_BYTES,
                                                              MALLOC_CAP_INTERNAL |
                                                                  MALLOC_CAP_DMA |
                                                                  MALLOC_CAP_8BIT));
    if (_dma_read_buffer == nullptr)
    {
        ::close(_fd);
        _fd = -1;
        _path.clear();
        return false;
    }
    _dma_read_buffer_size = SD_DMA_READ_BUFFER_BYTES;
    _position = 0;
#else
    _file = fnSDFAT.file_open(_path.c_str(), "rb");
    if (_file == nullptr)
    {
        _path.clear();
        return false;
    }
#endif

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
    if (_cancelled || buffer == nullptr)
        return -1;

    if (length == 0)
        return 0;

#ifdef ESP_PLATFORM
    if (_fd < 0 || _dma_read_buffer == nullptr || _dma_read_buffer_size == 0)
        return -1;

    size_t total_read = 0;
    while (total_read < length)
    {
        const size_t chunk = std::min(length - total_read, _dma_read_buffer_size);
        const ssize_t bytes_read = ::read(_fd, _dma_read_buffer, chunk);
        if (bytes_read < 0)
        {
            Debug_printf("audio sd read failed errno=%d\r\n", errno);
            return total_read == 0 ? -1 : static_cast<int>(total_read);
        }
        if (bytes_read == 0)
            break;

        std::memcpy(buffer + total_read, _dma_read_buffer, bytes_read);
        total_read += static_cast<size_t>(bytes_read);
        _position += static_cast<uint32_t>(bytes_read);

        if (static_cast<size_t>(bytes_read) < chunk)
            break;
    }

    return static_cast<int>(total_read);
#else
    if (_file == nullptr)
        return -1;

    const size_t bytes_read = std::fread(buffer, 1, length, _file);
    if (bytes_read == 0 && std::ferror(_file))
        return -1;

    return static_cast<int>(bytes_read);
#endif
}

void AudioSourceSD::close()
{
#ifdef ESP_PLATFORM
    if (_fd >= 0)
    {
        ::close(_fd);
        _fd = -1;
    }
    _position = 0;
#else
    if (_file != nullptr)
    {
        std::fclose(_file);
        _file = nullptr;
    }
#endif

    _path.clear();
    _content_type.clear();
    _content_length = 0;
    _cancelled = false;
#ifdef ESP_PLATFORM
    heap_caps_free(_dma_read_buffer);
    _dma_read_buffer = nullptr;
    _dma_read_buffer_size = 0;
#endif
}

void AudioSourceSD::cancel()
{
    _cancelled = true;
}

bool AudioSourceSD::is_seekable() const
{
#ifdef ESP_PLATFORM
    return _fd >= 0;
#else
    return _file != nullptr;
#endif
}

bool AudioSourceSD::seek(uint32_t position)
{
#ifdef ESP_PLATFORM
    if (_fd < 0)
        return false;

    const off_t result = ::lseek(_fd, static_cast<off_t>(position), SEEK_SET);
    if (result < 0)
        return false;

    _position = static_cast<uint32_t>(result);
    return true;
#else
    if (_file == nullptr)
        return false;

    return std::fseek(_file, static_cast<long>(position), SEEK_SET) == 0;
#endif
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
#ifdef ESP_PLATFORM
    return _position;
#else
    if (_file == nullptr)
        return 0;

    const long pos = std::ftell(_file);
    return pos > 0 ? static_cast<uint32_t>(pos) : 0;
#endif
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
