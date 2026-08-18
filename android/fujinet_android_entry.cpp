//
// FujiNet Android entry shim
//
// On PC the runtime owns its process: main() parses argv, installs signal
// handlers, runs the service loop and exits. Inside an Android app none of
// that is available - there is no process to own, and exiting kills the app.
//
// This shim replaces main() with a start/stop pair the host can call. It runs
// main_setup() and fn_service_loop() on a thread of their own, blocks the
// caller until setup has finished (so the host knows the bus is listening
// before it connects to it), and stops the loop through the cooperative
// shutdown flag rather than by exiting.
//
// It also captures stdout/stderr, because Debug_printf writes there and
// nothing on Android would otherwise see a word of it.
//

#include <android/log.h>
#include <unistd.h>

#include <cstdio>
#include <algorithm>
#include <condition_variable>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <exception>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "fnSystem.h"

extern void main_setup(int argc, char* argv[]);
extern void fn_service_loop(void* param);
extern void main_shutdown_handler();

namespace {
constexpr const char* LOG_TAG = "FujiNetRuntime";

std::mutex g_mutex;
std::condition_variable g_setup_condition;
std::thread g_runtime_thread;
std::string g_last_error;
bool g_running = false;
bool g_setup_complete = false;
bool g_setup_failed = false;

std::mutex g_log_mutex;
std::thread g_log_thread;
std::string g_log_tail;
std::string g_log_file_path;
size_t g_log_file_size = 0;
int g_log_read_fd = -1;
int g_log_write_fd = -1;
int g_saved_stdout_fd = -1;
int g_saved_stderr_fd = -1;

constexpr size_t kMaxLogTailBytes = 1024 * 1024;
constexpr size_t kMaxLogFileBytes = 1024 * 1024;

void set_last_error_locked(const std::string& message) {
    g_last_error = message;
    __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, "%s", g_last_error.c_str());
}

void publish_setup_result_locked(bool complete, bool failed) {
    g_setup_complete = complete;
    g_setup_failed = failed;
    g_setup_condition.notify_all();
}

// FujiNet's debug output is not guaranteed to be printable - protocol traces
// carry raw bytes - and a stray control character corrupts logcat. Escape
// anything that is not plain text, keeping the line structure intact.
std::string sanitize_log_chunk(const char* data, size_t size) {
    if (data == nullptr || size == 0) {
        return {};
    }

    std::string sanitized;
    sanitized.reserve(size);

    char escaped[5];
    for (size_t index = 0; index < size; ++index) {
        const unsigned char byte = static_cast<unsigned char>(data[index]);
        switch (byte) {
            case '\n':
            case '\r':
            case '\t':
                sanitized.push_back(static_cast<char>(byte));
                break;
            default:
                if (byte >= 0x20 && byte <= 0x7e) {
                    sanitized.push_back(static_cast<char>(byte));
                } else {
                    std::snprintf(escaped, sizeof(escaped), "\\x%02X", byte);
                    sanitized.append(escaped);
                }
                break;
        }
    }

    return sanitized;
}

void trim_log_tail_locked() {
    if (g_log_tail.size() > kMaxLogTailBytes) {
        g_log_tail.erase(0, g_log_tail.size() - kMaxLogTailBytes);
    }
}

void rewrite_log_file_locked() {
    if (g_log_file_path.empty()) {
        return;
    }
    FILE* file = fopen(g_log_file_path.c_str(), "wb");
    if (file == nullptr) {
        return;
    }
    if (!g_log_tail.empty()) {
        fwrite(g_log_tail.data(), 1, g_log_tail.size(), file);
    }
    fclose(file);
    g_log_file_size = g_log_tail.size();
}

void append_log_chunk_locked(const char* data, size_t size) {
    if (data == nullptr || size == 0) {
        return;
    }
    g_log_tail.append(data, size);
    trim_log_tail_locked();

    if (!g_log_file_path.empty()) {
        FILE* file = fopen(g_log_file_path.c_str(), "ab");
        if (file != nullptr) {
            fwrite(data, 1, size, file);
            fclose(file);
            g_log_file_size += size;
        }
        if (g_log_file_size > kMaxLogFileBytes) {
            rewrite_log_file_locked();
        }
    }
}

void append_log_chunk(const char* data, size_t size) {
    std::lock_guard<std::mutex> lock(g_log_mutex);
    append_log_chunk_locked(data, size);
}

void forward_logcat_line(std::string line) {
    if (!line.empty() && line.back() == '\r') {
        line.pop_back();
    }
    if (line.empty()) {
        return;
    }
    __android_log_print(ANDROID_LOG_INFO, "FujiNetConsole", "%s", line.c_str());
}

void stop_log_capture() {
    fflush(stdout);
    fflush(stderr);

    int saved_stdout_fd = -1;
    int saved_stderr_fd = -1;
    int log_read_fd = -1;
    int log_write_fd = -1;
    std::thread log_thread;

    {
        std::lock_guard<std::mutex> lock(g_log_mutex);
        saved_stdout_fd = g_saved_stdout_fd;
        saved_stderr_fd = g_saved_stderr_fd;
        log_read_fd = g_log_read_fd;
        log_write_fd = g_log_write_fd;
        g_saved_stdout_fd = -1;
        g_saved_stderr_fd = -1;
        g_log_read_fd = -1;
        g_log_write_fd = -1;
        log_thread = std::move(g_log_thread);
    }

    if (saved_stdout_fd >= 0) {
        dup2(saved_stdout_fd, STDOUT_FILENO);
        close(saved_stdout_fd);
    }
    if (saved_stderr_fd >= 0) {
        dup2(saved_stderr_fd, STDERR_FILENO);
        close(saved_stderr_fd);
    }
    // Closing the write end is what ends the reader thread's blocking read().
    if (log_write_fd >= 0) {
        close(log_write_fd);
    }

    if (log_thread.joinable()) {
        log_thread.join();
    }
    if (log_read_fd >= 0) {
        close(log_read_fd);
    }

    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_file_path.clear();
    g_log_file_size = 0;
}

void start_log_capture(const std::string& runtime_root) {
    stop_log_capture();

    int log_pipe[2];
    if (pipe(log_pipe) != 0) {
        __android_log_print(ANDROID_LOG_WARN, LOG_TAG, "Unable to create FujiNet log pipe");
        return;
    }

    const int saved_stdout_fd = dup(STDOUT_FILENO);
    const int saved_stderr_fd = dup(STDERR_FILENO);
    if (saved_stdout_fd < 0 || saved_stderr_fd < 0) {
        if (saved_stdout_fd >= 0) close(saved_stdout_fd);
        if (saved_stderr_fd >= 0) close(saved_stderr_fd);
        close(log_pipe[0]);
        close(log_pipe[1]);
        __android_log_print(ANDROID_LOG_WARN, LOG_TAG, "Unable to duplicate FujiNet stdio handles");
        return;
    }

    {
        std::lock_guard<std::mutex> lock(g_log_mutex);
        g_log_read_fd = log_pipe[0];
        g_log_write_fd = log_pipe[1];
        g_saved_stdout_fd = saved_stdout_fd;
        g_saved_stderr_fd = saved_stderr_fd;
        g_log_tail.clear();
        g_log_file_size = 0;
        g_log_file_path = runtime_root + "/fujinet-console.log";
        rewrite_log_file_locked();
    }

    dup2(log_pipe[1], STDOUT_FILENO);
    dup2(log_pipe[1], STDERR_FILENO);
    setvbuf(stdout, nullptr, _IOLBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);

    std::thread log_thread([read_fd = log_pipe[0]]() {
        char buffer[1024];
        std::string pending_line;

        while (true) {
            const ssize_t bytes_read = read(read_fd, buffer, sizeof(buffer));
            if (bytes_read <= 0) {
                break;
            }

            const std::string sanitized = sanitize_log_chunk(
                    buffer,
                    static_cast<size_t>(bytes_read)
            );
            if (sanitized.empty()) {
                continue;
            }

            append_log_chunk(sanitized.data(), sanitized.size());
            pending_line.append(sanitized);
            size_t newline_index = 0;
            while ((newline_index = pending_line.find('\n')) != std::string::npos) {
                forward_logcat_line(pending_line.substr(0, newline_index));
                pending_line.erase(0, newline_index + 1);
            }
        }

        if (!pending_line.empty()) {
            forward_logcat_line(pending_line);
        }
    });

    std::lock_guard<std::mutex> lock(g_log_mutex);
    g_log_thread = std::move(log_thread);
}
}  // namespace

extern "C" int fujinet_android_copy_recent_log(char* output, int maxBytes) {
    if (output == nullptr || maxBytes <= 0) {
        return 0;
    }

    std::lock_guard<std::mutex> lock(g_log_mutex);
    if (g_log_tail.empty()) {
        output[0] = '\0';
        return 0;
    }

    const size_t copyable = std::min(
        g_log_tail.size(),
        static_cast<size_t>(maxBytes - 1)
    );
    const size_t offset = g_log_tail.size() - copyable;
    memcpy(output, g_log_tail.data() + offset, copyable);
    output[copyable] = '\0';
    return static_cast<int>(copyable);
}

extern "C" bool fujinet_android_is_running() {
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_running;
}

extern "C" bool fujinet_android_start_runtime(
        const char* runtimeRootPath,
        const char* configPath,
        const char* sdPath,
        const char* dataPath,
        int listenPort
) {
    // The bus transport is configured through fnconfig.ini's [BOIP] section,
    // which the host writes before starting us; these are accepted so the
    // host-side contract can stay stable, and logged rather than applied.
    (void)dataPath;
    (void)listenPort;

    std::unique_lock<std::mutex> lock(g_mutex);
    g_last_error.clear();

    if (g_running) {
        return true;
    }
    if (runtimeRootPath == nullptr || configPath == nullptr || sdPath == nullptr) {
        set_last_error_locked("FujiNet runtime arguments were missing");
        return false;
    }

    // Both of these are process globals that survive a previous run. Without
    // the reset, getopt() would resume mid-argv and the service loop would see
    // a shutdown that was requested last time and return immediately.
    fnSystem.clear_shutdown_request();
    optind = 1;

    const std::string runtimeRoot(runtimeRootPath);
    const std::string config(configPath);
    const std::string sd(sdPath);
    g_setup_complete = false;
    g_setup_failed = false;

    try {
        g_runtime_thread = std::thread([runtimeRoot, config, sd]() {
            start_log_capture(runtimeRoot);

            // FujiNet resolves a number of paths relative to the working
            // directory, which in an app process is "/".
            if (chdir(runtimeRoot.c_str()) != 0) {
                std::lock_guard<std::mutex> lock(g_mutex);
                set_last_error_locked("FujiNet failed to change into runtime root");
                publish_setup_result_locked(false, true);
                stop_log_capture();
                return;
            }

            std::vector<std::string> argsStorage = {
                "fujinet",
                "-c",
                config,
                "-s",
                sd,
            };
            std::vector<char*> argv;
            argv.reserve(argsStorage.size());
            for (std::string& arg : argsStorage) {
                argv.push_back(arg.data());
            }

            try {
                main_setup(static_cast<int>(argv.size()), argv.data());
                std::lock_guard<std::mutex> lock(g_mutex);
                g_running = true;
                publish_setup_result_locked(true, false);
            } catch (const std::exception& error) {
                std::lock_guard<std::mutex> lock(g_mutex);
                set_last_error_locked(
                    std::string("FujiNet main_setup failed: ") + error.what()
                );
                g_running = false;
                publish_setup_result_locked(false, true);
                stop_log_capture();
                return;
            } catch (...) {
                std::lock_guard<std::mutex> lock(g_mutex);
                set_last_error_locked("FujiNet main_setup failed with an unknown error");
                g_running = false;
                publish_setup_result_locked(false, true);
                stop_log_capture();
                return;
            }

            fn_service_loop(nullptr);
            main_shutdown_handler();
            fnSystem.clear_shutdown_request();
            stop_log_capture();

            std::lock_guard<std::mutex> lock(g_mutex);
            g_running = false;
        });
    } catch (const std::exception& error) {
        set_last_error_locked(
            std::string("FujiNet runtime thread could not start: ") + error.what()
        );
        g_setup_failed = true;
        return false;
    }

    // Hold the caller until the bus is actually up. The emulator connects to
    // FujiNet, so starting it before this returns would race the listener.
    g_setup_condition.wait(lock, []() {
        return g_setup_complete || g_setup_failed;
    });

    if (g_setup_complete) {
        return true;
    }

    std::thread failedThread = std::move(g_runtime_thread);
    lock.unlock();
    if (failedThread.joinable()) {
        failedThread.join();
    }

    return false;
}

extern "C" void fujinet_android_stop_runtime() {
    std::thread runtimeThread;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (!g_running && !g_runtime_thread.joinable()) {
            return;
        }
        fnSystem.request_for_shutdown();
        runtimeThread = std::move(g_runtime_thread);
    }

    if (runtimeThread.joinable()) {
        runtimeThread.join();
    }

    stop_log_capture();

    std::lock_guard<std::mutex> lock(g_mutex);
    fnSystem.clear_shutdown_request();
    g_running = false;
}

extern "C" const char* fujinet_android_last_error_message() {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_last_error.empty()) {
        return nullptr;
    }
    return g_last_error.c_str();
}
