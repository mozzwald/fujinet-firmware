# FujiNet Audio Device Implementation Plan

## Purpose

Add a new asynchronous, bus-neutral FujiNet audio device. Atari 8-bit is the
first platform and maps the shared device to:

- Atari device name: `A:`
- SIO device ID: `$65`
- Primary use cases:
  - Play internet radio streams.
  - Play audio files from HTTP/HTTPS URLs.
  - Play audio files from a FujiNet SD card.
  - Provide a generic audio subsystem that can later be used by other FujiNet
    platforms and by existing features such as SAM speech.
- Playback must not block normal SIO, disk, network, printer, configuration, or
  web-interface processing.
- FujiNet-PC must implement the same device protocol and provide host-system
  audio output.
- Other FujiNet platforms must be able to add the same shared device to their
  bus chain when prototype hardware gains audio output.

This document is the implementation checklist and design reference for the
experimental `audio` branch. Protocol values and structures are provisional
until the protocol-definition phase is completed.

## Scope

### Initial scope

- One active playback session.
- A shared audio command set in `include/fujiCommandID.h`.
- A bus-neutral device implementation in `lib/device/audioDevice.h` and
  `lib/device/audioDevice.cpp`.
- Mono output through the Atari SIO audio pin on the initial hardware target.
- HTTP and HTTPS direct audio streams.
- Local SD-card files.
- WAV/PCM and MP3 playback.
- Play, pause, resume, stop, volume, status, and metadata commands.
- Background buffering, decoding, and playback.
- ESP32, supported ESP32-S3 Atari hardware, and FujiNet-PC.

### Later scope

- AAC/ADTS playback.
- Configured FujiNet host slots and other file protocols.
- Playlists and HLS.
- Seeking in seekable files.
- Migration of SAM speech and rotation sounds to the generic audio engine.
- Additional generated-audio devices that submit PCM to `AudioService`, such as
  future sound-chip emulation for platforms that can push register or event data
  to FujiNet.
- Mixing multiple independent FujiNet audio sources through `AudioService`.
- Audio support on non-Atari platforms.

### Explicitly out of initial scope

- Mixing multiple independent FujiNet audio sources.
- Stereo output over the Atari SIO audio pin.
- Sending decoded audio over a vintage-computer bus.
- Recording audio.
- A full CIO handler included in firmware. Host-side handler/application work
  may be maintained in a separate Atari software repository.
- Guaranteed playback on boards where `PIN_DAC1` is `GPIO_NUM_NC` or where the
  SIO audio signal is not physically connected.

## Existing implementation constraints

- The main ESP32 FujiNet service loop runs at priority 17 on core 1. Wi-Fi and
  most system work run on core 0.
- Atari SIO is serviced from the main FujiNet loop and must not wait for audio
  network reads, decoding, buffering, or playback.
- Existing SAM playback is synchronous. On classic ESP32 it writes samples
  individually through the one-shot DAC API; on ESP32-S3 it uses I2S PDM; on
  FujiNet-PC it uses miniaudio.
- The current HTTP client uses a helper FreeRTOS task and a 512-byte transfer
  buffer. Audio requires a larger jitter buffer and explicit cancellation.
- Classic Atari FujiNet hardware maps `PIN_DAC1` to GPIO25.
- The ESP32-S3 WROOM Atari pinmap maps `PIN_DAC1` to GPIO21.
- The ESP32-S3 xDrive pinmap currently maps `PIN_DAC1` to `GPIO_NUM_NC`.
- FujiNet-PC already vendors miniaudio, but its implementation is currently
  included directly by the SAM implementation and must be made a shared
  backend safely.

## High-level architecture

The implementation should separate bus framing, shared device commands, and
audio playback:

```text
Vintage-computer application
             |
             | platform bus framing
             v
 Platform virtualDevice / bus chain
             |
             | audio-device command bytes
             v
        audioDevice
             |
             | short, non-blocking controller calls
             v
                    AudioService
          /             |             \
         v              v              v
 AudioSource/Decoder  PCM producers  AudioMixer -> AudioSink
       |                 |                         |
 HTTP / SD, MP3/WAV   SAM, test tone,             ESP32 DAC DMA
                      future AY-style              ESP32-S3 PDM
                      generated audio              PC miniaudio
```

### Shared device and bus integration

The repository's current device direction separates common device logic from
bus ownership:

- Shared audio-device command IDs belong in `include/fujiCommandID.h`.
  This header contains command-byte constants for multiple devices, so duplicate
  byte values are acceptable when they are scoped to different device IDs.
- Shared audio-device behavior belongs in:
  - `lib/device/audioDevice.h`
  - `lib/device/audioDevice.cpp`
- Platform audio-device wrappers are only required for buses whose
  `virtualDevice` does not yet expose generic `transaction_*` helpers. Initial
  Atari support needs an SIO wrapper, for example:
  - `lib/device/sio/audio.h`
  - `lib/device/sio/audio.cpp`
- Generic playback implementation belongs in `lib/audio/`.
- `lib/device/device.h` exposes or instantiates the platform adapter for
  platform builds as appropriate.
- Each platform bus is responsible for:
  - Adding the platform audio device instance to its device chain during setup.
  - Mapping its native device identifier or device type to the audio device.
  - Receiving and sending payloads using that bus's framing rules.
  - Dispatching the audio-device command operation to `audioDevice`.
  - Returning completion, error, status, and data using native bus semantics.

The shared audio device must not contain SIO ACK/NAK timing, SmartPort packets,
IEC channels, AdamNet framing, DriveWire framing, or other bus-specific logic.
If a bus requires a thin adapter class, it should only translate between the
bus transaction and the shared `audioDevice` command API.

The existing `fujiDevice` split is the relevant precedent: shared behavior is
implemented under `lib/device/`, while transaction handling is supplied by the
active platform bus or by a thin platform wrapper. `audioDevice` should follow
this generic-device pattern without becoming part of the Fuji control device at
`$70`.

### Shared transaction pattern

`audioDevice` should follow the refactored `fujiDevice` architecture. The common
base class owns command validation and shared command behavior. It declares
transaction hooks like `fujiDevice`, and the active build supplies those hooks
either through the platform `virtualDevice` or through a small platform wrapper.

The shared base should declare the same transaction surface used by
`fujiDevice`:

- `transaction_begin(expectMoreData)`
- `transaction_complete()`
- `transaction_error()`
- `transaction_get(data, len)`
- `transaction_put(data, len, err)`

The shared base should expose `audiocmd_*` public entry points that validate
inputs, call the matching `AudioService` controller method, and send responses
through `transaction_*`. It should also expose lower-level `audiocore_*` methods
when useful for tests or wrappers that need to compose a transaction
differently.

A platform wrapper should be used only when the target bus has not yet been
refactored to provide usable generic `virtualDevice::transaction_*` helpers. The
wrapper should:

- Derive from `audioDevice`.
- Implement the transaction hooks using that bus's native framing.
- Decode the native command frame, packet, channel, or AUX fields.
- Switch `AUDIOCMD_*` command bytes to the shared `audiocmd_*` methods.
- Keep all ACK/NAK, checksum, packet status, and length-field handling outside
  the shared base.

For Atari, this means a concrete SIO adapter such as `sioAudioDevice` registers
on device `$65` and calls shared `audioDevice` methods. The shared base must not
directly parse SIO AUX bytes or include headers from `lib/device/sio/`.

Do not refactor every bus to generic transactions as part of the initial audio
implementation. Future bus refactors should be able to remove or simplify
platform wrappers without changing `audioDevice` command semantics.

Bus dispatch review for protocol phase:

| Bus family | Current transaction status | Audio-device integration decision |
| --- | --- | --- |
| Atari SIO | SIO `virtualDevice` does not provide generic `transaction_*`; `sioFuji` supplies its own hooks. | Add `lib/device/sio/audio.h/.cpp` wrapper for `$65`. |
| RS232/FujiBus | `virtualDevice` has generic `transaction_*` helpers. | Can derive from shared `audioDevice` directly when this bus is enabled for audio. |
| ComLynx | `virtualDevice` has generic `transaction_*` helpers. | Can derive from shared `audioDevice` directly when this bus is enabled for audio. |
| IWM/SmartPort | Uses its own packet/device model. | Requires a thin adapter if/when audio is added. |
| IEC | Channel-oriented device model, not the FujiBus transaction helper model. | Requires a thin adapter if/when audio is added. |
| AdamNet | Distinct AdamNet device model and device IDs. | Requires a thin adapter and separate device-ID mapping if/when audio is added. |
| DriveWire | Distinct command model. | Requires a thin adapter if/when audio is added. |
| Other buses | Treat as unsupported until reviewed for generic transaction helpers or adapter requirements. | Keep shared command behavior in `audioDevice`; add only bus framing adapters. |

### Proposed source interfaces

`AudioSource` supplies compressed or PCM input bytes independently of the
decoder and output hardware.

Required operations:

- `open(uri, options)`
- `read(buffer, length)`
- `close()`
- `cancel()`
- `is_seekable()`
- `seek(position)`
- `content_type()`
- `content_length()`
- `metadata()`

Initial implementations:

- `AudioSourceHTTP`
- `AudioSourceSD`

Future implementations:

- Configured FujiNet host slot source.
- TNFS, SMB, NFS, FTP, and other existing FujiNet file abstractions.
- HLS playlist/segment source.
- In-memory PCM source for SAM.

### Proposed generated-audio producer interfaces

Some audio will not originate from a URI, file, or compressed byte stream.
SAM speech, rotation sounds, deterministic test tones, and future platform
sound-chip devices such as AY-3-8910 emulation are generated-audio producers.

Generated-audio producers must submit PCM to `AudioService` rather than opening
or writing an `AudioSink` directly. This keeps physical output ownership in one
place and allows later mixing between stream playback and generated sounds.

Required producer behavior:

- Declare a producer type, priority, sample format, and expected duration when
  known.
- Accept platform/device events such as text-to-speech requests, disk-swap
  notifications, register writes, or sound-chip frame updates.
- Generate or enqueue signed 16-bit PCM frames in bounded buffers.
- Support cancellation and completion notification.
- Never block the active platform bus while producing PCM.

Initial implementation may support only one active audible producer at a time,
but the API must not assume that URI playback is the only source of audio.
SAM should become the first real generated-audio producer when it is migrated
off direct DAC/I2S/miniaudio ownership.

### Proposed decoder interfaces

`AudioDecoder` accepts source bytes and emits normalized PCM frames.

Required operations:

- Probe or select a codec from content type, extension, and stream signature.
- Initialize from stream format information.
- Consume arbitrary input chunks.
- Produce signed 16-bit mono PCM.
- Report input/output consumption.
- Flush and reset.
- Report sample rate, channels, bitrate, position, and duration when known.

Initial implementations:

- WAV/PCM decoder.
- MP3 decoder using the vendored miniaudio/dr_mp3 decoder on ESP32 and PC.

Later implementations:

- AAC/ADTS.
- FLAC and Ogg/Vorbis if firmware size and performance permit.

### Proposed sink interfaces

`AudioSink` owns the configured physical or emulated audio output.

Required operations:

- `open(format)`
- `write(pcm_frames)`
- `pause()`
- `resume()`
- `drain()`
- `close()`
- `set_volume(percent)`
- `supported()`

Initial implementations:

- `AudioSinkEsp32Dac`
  - Use the continuous DAC/DMA API, not per-sample one-shot writes.
- `AudioSinkEsp32S3Pdm`
  - Use I2S PDM TX and DMA.
- `AudioSinkMiniaudio`
  - Use miniaudio on FujiNet-PC.
- `AudioSinkNull`
  - Consume PCM without hardware output for tests and unsupported boards.

### Proposed mixer and ownership model

`AudioService` owns the only path to `AudioSink`. Decoders and generated-audio
producers submit normalized PCM to the service, and the service decides whether
to play, reject, interrupt, queue, or mix each source.

Initial implementation can use single-source arbitration, because mixing is out
of initial scope. The service and buffer APIs should still represent sources as
independent audio clients so a later `AudioMixer` can combine them without
rewriting device command handling or sink backends.

The long-term target is additive mixing with saturation and per-source gain. A
representative future behavior is: internet radio is playing, the user presses
the disk-swap button, SAM speaks the disk-swap message, and the listener hears
both the stream and speech through the same output sink.

### Proposed task model

Audio work must run outside the SIO service loop.

Initial ESP32 task proposal:

- Source task:
  - Core 0.
  - Low priority, initially 4 or 5.
  - Performs network/file reads.
  - Writes compressed bytes into an input ring buffer.
- Decode/output task:
  - Core 0.
  - Priority above the source task but well below the SIO loop, initially 6.
  - Reads compressed bytes, decodes PCM, and feeds the sink/DMA buffers.
- Bus-facing audio device:
  - Does not perform network I/O or decoding.
  - Updates controller state and exchanges bounded command/status data.
  - Returns promptly regardless of the active platform bus.

The priorities, stack sizes, and pinning must be measured and adjusted during
hardware testing. Audio code must not assume that core 0 is otherwise idle.

FujiNet-PC should use equivalent worker threads, condition variables, and
bounded queues behind the same public `AudioService` API.

### Buffering proposal

- Compressed input ring buffer:
  - PSRAM on ESP32 is required for use.
  - Initial target: 64 KiB.
  - Configurable at build time via environment variable/define.
- PCM/output buffers:
  - Allocate from internal DMA-capable memory where required.
  - Keep bounded; do not place DMA buffers in PSRAM unless supported.
- Startup threshold:
  - Do not begin output until a configurable minimum is buffered.
  - Initial target: 1 second or a minimum byte threshold.
- Rebuffering:
  - Enter `BUFFERING` after an underrun.
  - Resume automatically after the threshold is restored.
- Backpressure:
  - Source waits when the compressed ring buffer is full.
  - Decoder waits when output/DMA capacity is unavailable.

## Shared audio-device command protocol

The audio operations are audio-device commands defined in
`include/fujiCommandID.h`. They are not Fuji control-device commands and are not
SIO-only command definitions.

`include/fujiCommandID.h` is the repository-wide command-byte registry for
multiple device classes. Command bytes are interpreted in the context of the
addressed device, so an audio-device command may reuse a byte value used by the
Fuji control device, disk device, network device, or another standalone device.
The important collision check is within the audio device's own command set and
within any native bus command namespace that cannot distinguish devices.

The common implementation should be named `audioDevice` and live in
`lib/device/audioDevice.h` and `lib/device/audioDevice.cpp`. Generic playback
code lives under `lib/audio/`.

For Atari, the platform maps the shared device to SIO device `$65` and exposes
it to software as `A:`. It is registered independently from the FujiNet
control device at `$70` and the existing voice device at `$43`.

Other platform buses will assign their own native device identifier, type,
channel, or discovery entry while retaining the same command meanings and
payload structures.

### Protocol goals

- All control operations complete quickly.
- Starting playback queues a request and returns; it does not wait for network
  connection, buffering, or decoded output.
- Status is binary, versioned, and fixed-size.
- Strings are transferred separately from control commands.
- Commands work without a CIO handler so assembly and C applications can call
  SIO directly on Atari.
- Commands are declared once as `AUDIOCMD_*` values and used by every bus.
- Payload definitions do not assume SIO AUX bytes, SIO checksums, or a
  particular platform structure-packing convention.
- Unknown commands or unsupported protocol versions receive a clear native
  bus error.

### Audio-device command set

Command byte assignments are final for protocol version 1. The values are added
to the `fujiCommandID_t` enumeration in `include/fujiCommandID.h`. They are
unique within the audio-device command set and do not need to be globally unique
across unrelated device IDs.

| Shared command | Proposed byte | Direction | Purpose |
| --- | ---: | --- | --- |
| `AUDIOCMD_STATUS` | `$53` | Device to host | Return basic device status. |
| `AUDIOCMD_SET_SOURCE` | `$4F` | Host to device | Set/open a URI or path without blocking for playback. |
| `AUDIOCMD_PLAY` | `$50` | Host to device | Start or restart the configured source. |
| `AUDIOCMD_PAUSE` | `$41` | Host to device | Pause output while retaining the session. |
| `AUDIOCMD_RESUME` | `$52` | Host to device | Resume a paused session. |
| `AUDIOCMD_STOP` | `$58` | Host to device | Stop playback and release source/decoder resources. |
| `AUDIOCMD_SET_VOLUME` | `$56` | Host to device | Set volume from 0 through 100. |
| `AUDIOCMD_GET_INFO` | `$49` | Device to host | Return extended playback status. |
| `AUDIOCMD_GET_METADATA` | `$4D` | Device to host | Return a requested metadata field/chunk. |
| `AUDIOCMD_SEEK` | `$4B` | Host to device | Seek a file source; reject for live streams. |
| `AUDIOCMD_CAPABILITIES` | `$3F` | Device to host | Return protocol and codec/output capabilities. |

The bus adapter may additionally map a native status operation onto
`AUDIOCMD_STATUS`, but the shared implementation must not depend on an
ASCII command-byte convention.

### Source Transfer

The source string may exceed one bus command frame and must not depend on a
fixed SIO or printer-style buffer.

Final shared request:

- `AUDIOCMD_SET_SOURCE` receives URI bytes plus an explicit length supplied by
  the bus adapter. If the bus supports a request body length, that length is the
  authoritative length. If not, the adapter maps native request fields to a
  length before calling shared `audioDevice` behavior.
- Maximum initial URI length: 1024 bytes.
- Payload is UTF-8 without a required trailing NUL.
- Firmware validates the length before allocating or receiving.
- A successful command means the source was syntactically accepted and stored as
  the pending source. It does not open the source, contact a remote server, or
  start buffering.
- Each bus adapter derives the length from its native packet, command, channel,
  or transaction mechanism. Atari SIO may use AUX1/AUX2 as a little-endian
  length, but that is an Atari adapter detail.

Supported initial URI forms:

- `http://host/path`
- `https://host/path`
- `sd:/path/file.wav`
- `sd:/path/file.mp3`

Future URI form:

- `host://<slot>/path/file.mp3`

### Playback State

The controller state machine should be explicit:

```text
IDLE
  |
  v
OPENING -> BUFFERING -> PLAYING <-> PAUSED
   |           ^           |
   |           |           v
   +--------> ERROR      BUFFERING
                |
                v
               IDLE

Any active state --STOP--> STOPPING --> IDLE
```

Final protocol version 1 states:

| Value | State |
| ---: | --- |
| 0 | `IDLE` |
| 1 | `OPENING` |
| 2 | `BUFFERING` |
| 3 | `PLAYING` |
| 4 | `PAUSED` |
| 5 | `STOPPING` |
| 6 | `FINISHED` |
| 7 | `ERROR` |
| 8 | `UNSUPPORTED` |

### Protocol Payloads And Byte Order

All multi-byte integers in protocol version 1 are unsigned little-endian values.
Shared request and response definitions are logical byte layouts, not C/C++
structures. Implementations must serialize fields explicitly and must not expose
compiler padding, native enum size, or native structure layout over any bus.

Fixed command payloads:

| Command | Request payload | Response payload |
| --- | --- | --- |
| `AUDIOCMD_STATUS` | none | 4 bytes: protocol version, state, error, flags. |
| `AUDIOCMD_SET_SOURCE` | URI bytes, length supplied by bus adapter. | none. |
| `AUDIOCMD_PLAY` | optional flags byte, omitted means zero. | none. |
| `AUDIOCMD_PAUSE` | none. | none. |
| `AUDIOCMD_RESUME` | none. | none. |
| `AUDIOCMD_STOP` | none. | none. |
| `AUDIOCMD_SET_VOLUME` | 1 byte: volume 0 through 100. | none. |
| `AUDIOCMD_GET_INFO` | none. | version 1 extended status structure below. |
| `AUDIOCMD_GET_METADATA` | field id, offset u16, requested length u16. | field id, offset u16, total length u16, returned length u16, bytes. |
| `AUDIOCMD_SEEK` | position milliseconds u32. | none. |
| `AUDIOCMD_CAPABILITIES` | optional requested protocol version byte. | version 1 capability structure below. |

`AUDIOCMD_PLAY` behavior:

- From `IDLE` with a pending source: queue open/buffer/play work and return.
- From `PAUSED`: resume playback, equivalent to `AUDIOCMD_RESUME`.
- From `FINISHED`: restart the current source from the beginning when possible.
- From `ERROR`: clear the error and retry the current source if one exists;
  otherwise return `INVALID_ARGUMENT`.
- From `OPENING`, `BUFFERING`, or `PLAYING`: leave the current operation active
  and return success.

`AUDIOCMD_SET_SOURCE` replaces the pending/current source and returns after
validation. It does not open the source. `AUDIOCMD_PLAY` is the command that
starts asynchronous open, buffering, decoding, and output.

### Atari SIO Mapping

Atari SIO registers device `$65` as the initial audio device. The SIO wrapper
maps command bytes from `AUDIOCMD_*` to shared `audioDevice` methods.

SIO-specific rules:

- `DAUX1/DAUX2` are little-endian request parameters when a command needs a
  short parameter and no separate request body length is available.
- For `AUDIOCMD_SET_SOURCE`, `DAUX1/DAUX2` carry the URI byte length.
- For `AUDIOCMD_GET_METADATA`, `DAUX1` carries the metadata field id and
  `DAUX2` carries the requested chunk length for simple direct-SIO callers; a
  longer bus request body may provide field id, offset, and length when the
  adapter supports it.
- SIO ACK/NAK, checksum, timeout, and transfer-direction details stay in
  `lib/device/sio/audio.cpp`, not in shared `audioDevice`.

Direct SIO callers do not need a CIO handler. Atari assembly or C callers set
the DCB device to `$65`, set `DCOMND` to an `AUDIOCMD_*` byte, set the transfer
direction and buffer fields for commands with payloads, and use `DAUX1/DAUX2`
only as the Atari adapter mapping described above.

### Capability Structure

Protocol version 1 capability response:

| Field | Size | Description |
| --- | ---: | --- |
| protocol version | 1 | Capability structure version. |
| minimum supported protocol | 1 | Lowest accepted protocol version. |
| maximum supported protocol | 1 | Highest accepted protocol version. |
| flags | 1 | Output available, seek supported, metadata supported, generated PCM supported. |
| max URI length | 2 | Initial value 1024. |
| max metadata chunk | 2 | Maximum bytes returned by one metadata response. |
| source mask | 2 | HTTP, HTTPS, SD, generated PCM. |
| codec mask | 2 | PCM, WAV, MP3 initially; AAC reserved for later. |
| output mask | 2 | ESP32 DAC, ESP32-S3 PDM, FujiNet-PC miniaudio, null/test. |

Protocol version 1 codec values:

| Value | Codec |
| ---: | --- |
| 0 | `UNKNOWN` |
| 1 | `PCM` |
| 2 | `WAV` |
| 3 | `MP3` |
| 4 | `AAC` |

### Extended Status Structure

The structure must use explicitly sized integer fields and a documented byte
order. Do not expose compiler-dependent C++ structure padding over any bus.

Final version 1 fields:

| Field | Size | Description |
| --- | ---: | --- |
| protocol version | 1 | Status structure version. |
| state | 1 | Playback state enum. |
| error | 1 | Stable audio error code. |
| flags | 1 | Live, seekable, metadata available, muted, rebuffering. |
| codec | 1 | Unknown, PCM, WAV, MP3, AAC, etc. |
| channels | 1 | Source channel count. |
| output channels | 1 | Normally 1 for the initial Atari SIO audio sink. |
| volume | 1 | 0 through 100. |
| sample rate | 4 | Decoded/output sample rate in Hz. |
| bitrate | 4 | Source bitrate when known. |
| buffered milliseconds | 4 | Estimated playable buffered time. |
| position milliseconds | 4 | Current decoded playback position. |
| duration milliseconds | 4 | Zero or sentinel when unknown/live. |
| underrun count | 2 | Number of output underruns. |
| metadata generation | 2 | Changes whenever metadata changes. |

The byte layout above is serialized exactly in table order using the byte order
defined in this section.

### Metadata

Initial metadata fields:

- Stream/station name.
- Current title.
- Content type.
- Codec name.
- Source URL/path.

Metadata should be retrieved in bounded chunks using a bus-neutral request
containing field type, offset, and requested length. The Atari adapter may map
small request fields onto AUX1/AUX2 where practical. The implementation must
not return unbounded strings or allocate from arbitrary host-provided sizes.

Metadata field ids for protocol version 1:

| Field id | Field |
| ---: | --- |
| 0 | Stream/station name. |
| 1 | Current title. |
| 2 | Content type. |
| 3 | Codec name. |
| 4 | Source URL/path. |

### Volume

- Public range is 0 through 100.
- Volume zero is mute.
- Initial implementation may use software scaling of signed 16-bit PCM.
- Scaling must saturate safely and avoid integer overflow.
- Future hardware sinks may implement native gain where available.
- Initial protocol behavior does not persist volume across reboots. Boot/default
  volume is 100 until a later configuration phase explicitly adds persisted
  default volume.

### Error model

Define stable device errors rather than exposing ESP-IDF or host OS errors:

| Value | Error |
| ---: | --- |
| 0 | `NONE` |
| 1 | `INVALID_COMMAND` |
| 2 | `INVALID_ARGUMENT` |
| 3 | `URI_TOO_LONG` |
| 4 | `UNSUPPORTED_URI_SCHEME` |
| 5 | `UNSUPPORTED_HARDWARE` |
| 6 | `NETWORK_UNAVAILABLE` |
| 7 | `DNS_FAILURE` |
| 8 | `CONNECTION_FAILURE` |
| 9 | `HTTP_ERROR` |
| 10 | `TLS_ERROR` |
| 11 | `SOURCE_NOT_FOUND` |
| 12 | `SOURCE_READ_FAILURE` |
| 13 | `UNSUPPORTED_FORMAT` |
| 14 | `DECODER_INITIALIZATION_FAILURE` |
| 15 | `DECODE_FAILURE` |
| 16 | `OUTPUT_INITIALIZATION_FAILURE` |
| 17 | `BUFFER_ALLOCATION_FAILURE` |
| 18 | `SEEK_UNSUPPORTED` |
| 19 | `CANCELLED` |
| 20 | `INTERNAL_ERROR` |

Detailed platform errors may be logged but must not become part of the public
protocol.

## Phase 0: Protocol and architecture definition

Goal: freeze enough of the public design that firmware and Atari client work
can proceed independently.

- [x] Confirm `$65` is available for the Atari SIO mapping.
- [x] Add the Atari `FUJI_DEVICEID_AUDIO = 0x65` mapping to the appropriate
      device-ID definition.
- [x] Confirm the shared files are named `lib/device/audioDevice.h` and
      `lib/device/audioDevice.cpp`.
- [x] Confirm the Atari adapter files are named `lib/device/sio/audio.h` and
      `lib/device/sio/audio.cpp`, unless the project chooses a different
      adapter naming convention before implementation.
- [x] Define `audioDevice` as a shared abstract base using the same transaction
      hook pattern as `fujiDevice`.
- [x] Define the initial Atari SIO wrapper as a concrete subclass because SIO
      `virtualDevice` does not yet provide generic `transaction_*` helpers.
- [x] Document that wrappers are temporary per-bus bridges and are not a reason
      to fork shared audio command behavior.
- [x] Do not require refactoring all buses to generic transactions before the
      initial Atari audio implementation.
- [x] Reserve and add all audio-device command values in
      `include/fujiCommandID.h`.
- [x] Confirm audio-device command bytes are unique within the audio-device
      command set, while documenting that reuse of bytes from other device
      classes is acceptable when scoped by device ID.
- [x] Finalize bus-neutral request and response payloads for every command.
- [x] Document Atari AUX1/AUX2 mappings separately from shared payload
      definitions.
- [x] Finalize maximum source URI length.
- [x] Finalize protocol capability structure.
- [x] Finalize extended status byte layout and byte order.
- [x] Finalize playback-state enumeration.
- [x] Finalize codec enumeration.
- [x] Finalize stable error enumeration.
- [x] Finalize metadata request and chunking protocol.
- [x] Decide whether `AUDIOCMD_SET_SOURCE` only stores a source or also
      begins opening it.
- [x] Decide exact `AUDIOCMD_PLAY` behavior after `FINISHED`, `ERROR`, and
      `PAUSED`.
- [x] Decide whether volume persists in FujiNet configuration.
- [x] Document direct SIO call examples for Atari assembly and C.
- [x] Document when a bus can use `audioDevice` directly through generic
      `virtualDevice::transaction_*` helpers and when it needs a wrapper.
- [x] Document the `audiocmd_*` and `audiocore_*` split, matching the current
      `fujicmd_*` and `fujicore_*` precedent.
- [x] Document the interface generated-audio producers use to submit PCM into
      `AudioService`.
- [x] Document initial single-source arbitration separately from the future
      mixer design.
- [x] Review command dispatch against SIO, IWM/SmartPort, IEC, AdamNet,
      DriveWire, RS232, and other current bus models.

Exit criteria:

- [ ] A versioned protocol specification is committed.
- [x] No protocol response depends on native structure packing.
- [x] No shared request or response depends on SIO AUX fields.
- [x] Host commands can return within normal bus timing without waiting for
      audio work.

## Phase 1: Generic audio core and test doubles

Goal: create platform-independent playback control, state management, and
bounded buffering before enabling physical audio.

- [x] Create `lib/audio/`.
- [x] Add `lib/audio/*.cpp` and any required include paths to the ESP32 build in
      `src/CMakeLists.txt`.
- [x] Add the new `lib/audio/` files to the FujiNet-PC source list in
      `fujinet_pc.cmake`.
- [x] Define `AudioService`.
- [x] Define `AudioSource`.
- [x] Define `AudioDecoder`.
- [x] Define `AudioSink`.
- [x] Define a generated-audio producer or PCM client interface for SAM,
      test tones, rotation sounds, and future sound-chip devices.
- [x] Define an internal source/client identifier so status, arbitration, and
      future mixing can distinguish stream playback from generated audio.
- [x] Define initial source arbitration rules without hard-coding the service to
      URI playback only.
- [x] Define immutable or synchronized status snapshots.
- [x] Implement the playback state machine.
- [x] Implement stable error conversion.
- [x] Implement command/event queues.
- [x] Implement compressed-data ring buffering.
- [x] Implement cancellation that unblocks source and decoder waits.
- [x] Implement clean stop and resource teardown.
- [x] Implement `AudioSinkNull`.
- [x] Implement an in-memory source for unit tests.
- [x] Implement a deterministic PCM/test-tone source or decoder.
- [x] Implement a bounded generated-PCM submission path into `AudioService`.
- [x] Add a placeholder mixer boundary even if the initial implementation only
      admits one audible source at a time.
- [x] Ensure public controller methods do not perform blocking I/O.
- [x] Add locking rules and document ownership of mutable state.
- [x] Add counters for bytes read, decoded frames, underruns, and reconnects.
- [x] Add structured debug logging guarded by an audio debug build flag.
- [x] Document manual smoke checks for state transitions.
- [x] Document manual smoke checks for ring-buffer wraparound and backpressure.
- [x] Document manual smoke checks for stop/cancel during opening, buffering,
      and playback.
- [x] Document manual smoke checks for malformed and oversized inputs.

Phase 1 implementation note:

- Desktop/FujiNet-PC uses an internal worker thread to drain the command queue.
  ESP32 firmware does not create a `std::thread`; the future audio device/task
  integration must call `AudioService::process_pending()` from a firmware-owned
  task or service pump.
- Public controller calls enqueue bounded commands and update status snapshots;
  sink I/O happens from the worker or explicit service pump.
- Stop/cancel is implemented with a generation counter and queue clear. Later
  streaming sources and decoders must check the same cancellation boundary while
  waiting on network reads, decode buffers, or sink capacity.
- The first arbitration rule is conservative: stream, generated speech, test
  tones, rotation sounds, and future sound-chip producers all identify their
  source kind before entering `AudioService`. Phase 1 still allows one active
  audible command at a time, but the mixer boundary preserves the path for
  additive mixing in later phases.

Manual Phase 1 smoke checks:

- Submit a memory-backed PCM/test-tone buffer to `AudioService` with
  `AudioSinkNull`, call `process_pending()` on firmware builds, and verify the
  status reaches `FINISHED`.
- Submit generated PCM using `AudioSourceKind::GENERATED_SPEECH` and verify the
  producer never opens an `AudioSink` directly.
- Fill `AudioCommandQueue` beyond capacity and verify the enqueue path returns
  false, records `BUFFER_ALLOCATION_FAILURE`, and increments
  `commands_rejected`.
- Exercise `AudioRingBuffer` by writing across the end of the buffer, reading
  back across the wrap point, and verifying the expected byte order and
  available/free byte counts.
- Submit, stop, and resubmit PCM repeatedly; verify counters remain coherent,
  state returns to `IDLE` or `FINISHED`, and no sink is left open.
- Submit invalid PCM arguments and oversized queued command loads; verify stable
  `AudioError` values instead of transport-specific failures.

Exit criteria:

- [x] A memory-backed PCM stream can run through the engine into the null sink.
- [x] A generated PCM producer can run through `AudioService` into the null
      sink without direct `AudioSink` access.
- [x] Repeated play/stop cycles do not leak resources.
- [x] Status polling is race-free.
- [x] Cancellation completes within a documented bounded time.

## Phase 2: Platform audio sinks

Goal: produce continuous audio through each supported output backend without
blocking the active platform bus service loop.

### Classic ESP32 DAC

- [x] Implement `AudioSinkEsp32Dac`.
- [x] Add any required ESP-IDF DAC component dependency, such as
      `esp_driver_dac`, to `src/CMakeLists.txt` for ESP builds.
- [x] Use ESP-IDF continuous DAC/DMA output.
- [x] Convert signed 16-bit PCM to the DAC's unsigned output format.
- [ ] Add mono downmix support for stereo source material.
- [x] Implement software volume scaling.
- [x] Allocate required buffers from DMA-capable internal memory.
- [x] Keep DAC resources open for the active playback session.
- [x] Test clean shutdown and restart of the DAC channel.
- [x] Verify GPIO25 output on Atari FujiNet v1 hardware.

### ESP32-S3 PDM

Deferred: keep this section as the future implementation target for the rare
ESP32-S3 Atari hardware, but do not block Phase 2 completion on this backend.

- [ ] Implement `AudioSinkEsp32S3Pdm`.
- [ ] Add any required ESP-IDF I2S component dependency, such as
      `esp_driver_i2s`, to `src/CMakeLists.txt` for ESP32-S3 builds.
- [ ] Move reusable PDM setup logic out of `samlib.cpp`.
- [ ] Use I2S PDM TX with DMA.
- [ ] Validate supported sample rates.
- [ ] Verify output on the S3 WROOM Atari design using GPIO21.
- [ ] Detect `PIN_DAC1 == GPIO_NUM_NC` as unsupported.
- [ ] Verify the xDrive build remains functional with audio reported as
      unsupported.

### FujiNet-PC

- [x] Implement `AudioSinkMiniaudio`.
- [x] Move miniaudio implementation inclusion into one dedicated translation
      unit.
- [x] Ensure SAM and the new audio sink do not define miniaudio twice.
- [x] Use a callback-safe PCM queue.
- [x] Support Linux.
- [ ] Support macOS where the existing FujiNet-PC build supports it.
- [ ] Support Windows where the existing FujiNet-PC build supports it.
- [x] Handle systems with no output device without terminating FujiNet-PC.

### Shared sink verification

- [x] Add sink capability detection.
- [ ] Return `UNSUPPORTED` through A: on boards without audio output.
- [ ] Verify sample-rate changes between consecutive files.
- [ ] Verify pause/resume does not produce uncontrolled stale output.
- [ ] Verify stop quickly silences output.
- [ ] Measure CPU use, task stack high-water marks, and internal-memory use.

Phase 2 implementation note:

- Classic ESP32 firmware now uses `AudioSinkEsp32Dac` for device `$65` when
  `PIN_DAC1` maps to built-in DAC GPIO25 or GPIO26. ESP32-S3 remains on the
  deferred PDM path.
- `AudioService::start()` creates a firmware-owned FreeRTOS pump task on ESP32,
  pinned to core 0, so command handlers enqueue work and return without draining
  audio from the high-priority SIO service loop on core 1.
- ESP32 `AudioService` queue and status access are protected with FreeRTOS
  mutexes because SIO command handlers and the audio pump now run concurrently.
- The generated tone command path has been confirmed manually on classic ESP32
  hardware through status, source selection, play command completion, and audible
  GPIO25 DAC output.
- Repeated play/stop/play on classic ESP32 hardware completes without command
  errors and produces the tone both times.
- Stop-while-playing verification needs a longer source than the current
  one-second generated tone and is deferred to Phase 3 WAV/SD playback.

Exit criteria:

- [x] A generated PCM tone plays continuously on classic ESP32 hardware.
- [ ] A generated PCM tone plays continuously on supported ESP32-S3 hardware.
      Deferred with the ESP32-S3 PDM backend.
- [x] A generated PCM tone plays through FujiNet-PC.
- [ ] SIO disk and status operations continue while the tone plays. Deferred to
      longer Phase 3 playback sources because the current generated tone is too
      short for meaningful manual overlap testing.

## Phase 3: WAV/PCM and SD-card file playback

Goal: validate the complete engine using a seekable local source before adding
network timing and codec complexity.

- [x] Implement `AudioSourceSD`.
- [x] Normalize and validate `sd:` paths.
- [x] Prevent paths from escaping the configured SD root.
- [x] Implement file open, read, EOF, close, cancel, and seek.
- [x] Implement SD reads in small bounded chunks.
- [x] Check cancellation between bounded read/decode blocks. The current local
      `FileHandler` path still cannot interrupt an individual active `fread()`.
- [x] Implement WAV RIFF parsing.
- [x] Support PCM WAV input.
- [x] Support at least 8-bit unsigned and 16-bit signed PCM.
- [x] Support mono input.
- [x] Downmix stereo input to mono.
- [x] Reject unsupported WAV encodings with a stable error.
- [x] Implement position and duration reporting.
- [x] Replace whole-file WAV decoding with incremental, bounded decoding.
- [x] Move SD open, WAV parsing, decoding, and sink writes out of the SIO
      command handler and into the core-0 audio worker.
- [x] Make `AUDIOCMD_PLAY` enqueue only a lightweight source request and return
      promptly after accepting it.
- [x] Remove whole-recording PCM storage from `AudioCommand`; queued control
      commands must not own or copy an entire decoded file.
- [x] Add an incremental `AudioDecoderWav` that retains only parser state and
      bounded source bytes between calls.
- [x] Decode and downmix WAV input into bounded signed 16-bit mono blocks,
      initially 512 frames per block.
- [x] Feed decoded blocks to the DAC sink incrementally instead of calling the
      sink once with the complete recording.
- [x] Keep DAC/DMA buffers in internal DMA-capable memory. WAV playback no
      longer requires memory proportional to file duration.
- [x] Check stop, pause, generation, and cancellation state between every
      source read, decode block, and sink write.
- [x] Use bounded sink-write waits so control commands can be observed during
      playback.
- [x] Update bytes-read, decoded-frame, position, duration, and state fields
      incrementally during playback.
- [x] Validate RIFF chunk sizes against the source length and reject malformed
      sizes before seeking, reading, or allocating.
- [x] Remove file-size-dependent allocations and use a fixed-capacity command
      queue so allocation failure cannot be triggered by WAV duration.
- [x] Implement asynchronous, frame-aligned seek for WAV/PCM files.
- [x] Test truncated and malformed WAV headers.
- [ ] Test removal or failure of the SD card during playback. Deferred at the
      user's request so Phase 3 does not block on remaining manual tests.
- [ ] Test multiple sample rates representative of speech and music. Deferred
      at the user's request so Phase 3 does not block on remaining manual
      tests.
- [x] Test repeated EOF/replay/stop behavior.
- [x] Test a WAV larger than internal ESP32 RAM; the 1,323,044-byte hardware
      test file plays through the bounded streaming path.
- [x] Test pause, resume, and stop during active SD reads and DAC writes.
- [ ] Monitor heap before, during, and after repeated playback for leaks and
      loss of the largest free block. Deferred for now; do not block Phase 3
      exit on heap measurement.
- [ ] Measure the audio worker stack high-water mark during WAV playback.
      Deferred for now; do not block Phase 3 exit on stack measurement.

Exit criteria:

- [x] `sd:/...wav` plays from A: on ESP32.
- [x] The same command plays the file on FujiNet-PC.
- [x] Disk and configuration SIO requests remain responsive during playback.
- [x] Position, duration, pause, resume, stop, volume, and seek work.
- [ ] A multi-hour WAV uses essentially the same bounded working memory as a
      short WAV with the same format. Deferred with heap/stack measurement.
- [x] `AUDIOCMD_PLAY` performs no SD open, file read, WAV decode, or sink write
      on the SIO service loop.
- [x] Allocation, malformed-input, SD-read, and output failures reach `ERROR`
      without aborting or resetting FujiNet.

Phase 3 implementation note:

- The current SD/WAV path streams `sd:/...wav` from the core-0 worker, converts
  bounded PCM blocks to mono signed 16-bit frames, and writes them incrementally
  through `AudioService`.
- Supported WAV input is RIFF/WAVE PCM format tag 1, mono or stereo, 8-bit
  unsigned or 16-bit signed. Stereo is downmixed to mono.
- The Atari manual test app has a fixed `W` command for `sd:/fnaudio.wav` and a
  `K` command that seeks to 1000 milliseconds.
- The original whole-file implementation was not true streaming playback. On
  classic ESP32, testing a 1,323,044-byte WAV aborted in
  `AudioService::submit_pcm()` while `std::vector<int16_t>::assign()` attempted
  to allocate a second complete PCM buffer. The original decoded vector was
  still live, and the command queue would have made further full-buffer copies.
  FujiNet-PC succeeds because host memory can accommodate those copies.
- The crash also confirmed that the original SD open, WAV parsing, whole-file
  decoding, and PCM submission ran synchronously from `AUDIOCMD_PLAY` on the
  core-1 SIO service loop. The implemented fix is a core-0 worker-side streaming
  pipeline:

  ```text
  lightweight PLAY request
          |
          v
  open/parse WAV on audio worker
          |
          v
  bounded SD read -> bounded decode/downmix -> bounded DAC DMA write
  ```

- Moving vectors into the queue may remove individual copies, but is not the
  architectural fix because memory would still scale with recording duration.
  The streaming path must maintain constant bounded working memory regardless
  of WAV size.
- Seek is now accepted as a four-byte little-endian millisecond payload. The
  worker aligns it to an input frame, flushes and reopens the sink, and resumes
  at the clamped file position without blocking SIO. Real-hardware seek has
  been confirmed.
- Decoder tests cover valid 8-bit mono and 16-bit stereo input, downmixing,
  frame-aligned seeking and clamping, truncated RIFF lengths, oversized chunks,
  and inconsistent format fields.

## Phase 4: Shared audio device and Atari bus registration

Goal: implement the shared FujiNet audio device, then expose it through the
Atari bus as dedicated device `$65` / `A:`.

- [x] Add all shared `AUDIOCMD_*` command IDs to
      `include/fujiCommandID.h`.
- [x] Add `FUJI_DEVICEID_AUDIO = 0x65` to the non-ADAM Atari/SIO device ID
      range in `include/fujiDeviceID.h`.
- [x] Create `lib/device/audioDevice.h` as the shared abstract base.
- [x] Create `lib/device/audioDevice.cpp` for shared `audiocmd_*` and
      `audiocore_*` behavior.
- [x] Define transaction hooks on `audioDevice` matching the `fujiDevice`
      transaction model.
- [x] Implement bus-neutral command handling and payload validation in
      `audioDevice` methods that use only `transaction_*` calls for bus I/O.
- [x] Create the Atari SIO adapter, initially `lib/device/sio/audio.h` and
      `lib/device/sio/audio.cpp`, unless naming is changed before coding.
- [x] Keep the Atari SIO adapter as a thin wrapper whose only purpose is to
      supply missing SIO transaction hooks and SIO command-frame dispatch.
- [x] Implement SIO transaction hooks in the Atari wrapper using
      `sio_ack()`, `sio_late_ack()`, `sio_complete()`, `sio_nak()`,
      `sio_error()`, `bus_to_peripheral()`, and `bus_to_computer()` as
      appropriate.
- [x] Implement the SIO wrapper's `sio_process()` to decode the command frame,
      derive Atari-specific AUX1/AUX2 lengths or request parameters, and call
      shared `audioDevice` command methods.
- [x] Connect `audioDevice` to `AudioService`.
- [x] Add the shared device files to ESP32 and FujiNet-PC build inputs.
- [x] Add the SIO adapter files to the ESP32 globbed build and to the explicit
      FujiNet-PC Atari source list.
- [x] Expose/instantiate the Atari SIO audio adapter through
      `lib/device/device.h`, similar to other Atari global device instances.
- [x] Add the Atari `$65` device ID mapping.
- [x] Add the SIO audio adapter instance to the Atari setup path in
      `src/main.cpp` using `SYSTEM_BUS.addDevice(&sioAudio,
      FUJI_DEVICEID_AUDIO)` before `SYSTEM_BUS.setup()`.
- [x] Keep device registration in the platform bus setup path rather than in
      `audioDevice` itself.
- [x] Ensure Atari registration is available in ESP32 and FujiNet-PC builds.
- [x] Do not add SIO-specific fields or shortcuts to the shared `audioDevice`
      base. A typed `_audioDev` shortcut in `systemBus` is optional and not
      required for normal daisy-chain dispatch.
- [x] Do not refactor all platform buses as part of this phase. When a future
      bus gains generic `virtualDevice::transaction_*` helpers, it should be
      able to use `audioDevice` directly or with a much thinner wrapper.
- [x] Keep SIO transaction framing, ACK/NAK, checksum, AUX interpretation, and
      bus data transfer outside the shared audio logic.
- [x] Implement capability inquiry.
- [x] Implement source-string transfer with strict length validation.
- [x] Implement play for the initial generated test-tone source.
- [x] Implement pause.
- [x] Implement resume.
- [x] Implement stop.
- [x] Implement volume.
- [x] Implement shared basic status and map it to Atari's standard four-byte
      SIO status where required.
- [x] Implement extended status.
- [x] Implement initial metadata retrieval for the active generated/test source.
- [x] Implement seek for seekable sources.
- [x] Return an appropriate native bus error for unsupported commands; Atari
      maps this to the required SIO NAK/error response.
- [x] Ensure no command handler performs source open, DNS, network read,
      decoding, or output draining synchronously. SD/WAV open, parse, decode,
      and output now run from the core-0 audio worker.
- [ ] Add shared device-command tests.
- [ ] Add Atari SIO adapter/transaction tests where practical.
- [x] Add a small Atari test program or documented test sequence using direct
      SIO calls. Initial app: `atari-audio-test/`, built with cc65 as
      `build/fnaudio-test.xex`.
- [x] Verify the existing P3:/SAM printer-interface device remains compatible.
      Atari SAM now queues generated PCM through `AudioService`; manual P3:
      speech regression remains listed in Phase 9.

Implementation notes:

- The `$65` implementation supports generated test tones and bounded streaming
  playback of PCM WAV files from `sd:` sources.
- FujiNet-PC and ESP32 Atari builds register device `$65`; real classic ESP32
  hardware playback through the GPIO25 DAC has been verified.
- FujiNet-PC uses the miniaudio sink. Classic ESP32 uses the continuous DAC/DMA
  sink from a core-0 audio worker while SIO remains responsive on core 1.
- `SET_VOLUME` currently uses the payload form exercised by the Atari test
  program. Decide later whether an AUX-only shortcut is needed for convenience
  or compatibility.

Exit criteria:

- [x] An Atari can start SD WAV playback through `$65`.
- [x] A host can poll status during generated test-tone playback.
- [x] Other FujiNet SIO devices continue operating during audio.
- [ ] Invalid lengths and commands cannot corrupt state or exhaust memory.
- [x] The shared `audioDevice` compiles without depending on headers from
      `lib/device/sio/`.
- [x] The Atari SIO adapter is the only audio-device code that depends on SIO
      ACK/NAK, checksum, AUX, or `bus_to_*` helpers.

## Phase 5: MP3 decoding

Goal: support the most common file and internet-radio format.

- [x] Use the existing vendored miniaudio/dr_mp3 decoder for the first ESP32 and
      FujiNet-PC MP3 implementation. No new external component is introduced in
      this pass.
- [x] Keep the decoder version pinned to the vendored miniaudio snapshot already
      present in `components_pc/miniaudio`.
- [ ] Record component license and source/version information.
- [x] Implement the ESP32 MP3 decoder adapter.
- [x] Implement the FujiNet-PC MP3 decoder adapter using miniaudio.
- [ ] Probe MP3 by signature in addition to URL extension.
- [x] Support ID3v2 tags before audio frames through the miniaudio/dr_mp3
      decoder.
- [x] Skip or parse ID3 metadata without passing it to the output path through
      the miniaudio/dr_mp3 decoder.
- [x] Handle arbitrary source chunk boundaries.
- [ ] Handle bitrate and sample-rate changes where supported.
- [x] Downmix stereo MP3 output to mono.
- [ ] Report decoded sample rate and bitrate. Decoded sample rate is reported;
      bitrate remains TBD.
- [ ] Test constant-bitrate MP3 files.
- [ ] Test variable-bitrate MP3 files.
- [ ] Test files with and without ID3 tags.
- [ ] Test truncated and corrupt MP3 data.
- [ ] Measure decoder CPU use on classic ESP32 and ESP32-S3.
- [ ] Measure minimum safe task stack sizes.
      Initial ESP32 MP3 hardware testing overflowed the 8192-byte `audioSvc`
      stack during MP3 startup. A follow-up test also overflowed at 16384
      bytes before the output-task context and service-owned MP3 scratch buffers
      were moved off the worker stack. Direct testing then showed the vendored
      miniaudio/dr_mp3 frame decoder still allocates internal decode scratch on
      the caller stack, so the worker stack is provisionally set back to 32768
      bytes and still needs high-water-mark measurement.
      Initial successful playback showed repeat-like skipping consistent with
      DAC starvation. A 16-descriptor queue with 2048-byte DMA buffers failed
      to allocate at DAC open on real ESP32 hardware. A smaller 12-descriptor
      queue with 1024-byte DMA buffers opened, but SDMMC later failed to
      allocate DMA-capable transfer memory during playback. The DAC queue is
      therefore restored to the original 8 descriptors of 1024 bytes so SDMMC
      retains internal DMA-capable heap. MP3 decode blocks remain at 2048
      frames.
- [x] Add a PCM ring-buffer queue between decoders and sinks for compressed
      streams.
      Implementation constraints:
      - The DAC sink keeps only a small DMA-capable queue. Do not grow DAC DMA
        descriptors to hide decode or SD jitter because SDMMC also needs
        internal DMA-capable heap.
      - The PCM queue stores decoded 16-bit mono frames and must not request
        `MALLOC_CAP_DMA`.
      - On ESP32, allocate the PCM queue from PSRAM first
        (`MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`) and fall back to default heap
        only when PSRAM is unavailable.
      - The MP3 path should prebuffer decoded PCM before entering `PLAYING`,
        then refill the queue when it drops below a low-water mark while
        draining smaller chunks to `AudioSink::write()`.
      - Status `buffered_ms` should reflect queued decoded PCM, not DAC DMA
        descriptors.
      Initial implementation uses a 32768-frame decoded-PCM queue, starts
      playback after 16384 frames or EOF, drains 512-frame chunks to the active
      sink, and performs runtime refill incrementally with at most one
      2048-frame MP3 decode chunk per drain cycle once the queue drops to
      16384 frames. Runtime refill must not fill the queue back to full in one
      burst because that starves the DAC at a timed interval.
      A dedicated ESP32 `audioOut` consumer task was prototyped so the audio
      worker could decode from SD while the consumer independently drained the
      PCM ring buffer to the DAC. Hardware testing showed that the 32768-byte
      worker stack required by the vendored MP3 decoder leaves too little
      internal heap for a second FreeRTOS task on classic ESP32, so that path is
      disabled for now. Re-enable it only after decoder stack use is reduced or
      an alternate stack allocation strategy is proven on hardware.
- [ ] Remove or formally retire the disabled ESP32 `audioOut` prototype
      (`MP3_USE_OUTPUT_TASK`) once the single-worker MP3 path has survived
      broader testing, or replace it with a proven low-stack/alternate-stack
      producer-consumer implementation.
      With the output task disabled, real-hardware testing showed each audible
      skip coincided with a large miniaudio source read. The ESP32 MP3 read
      callback now bounds each SD read to 2048 bytes while still reporting the
      decoder's requested size in debug output. This resolved the repeated skip
      on real Atari/FujiNet hardware.
      Later SAM/rotation-sound testing exposed a related SDMMC constraint:
      passing miniaudio's destination buffer through stdio `fread()` can make
      the ESP32 SDMMC layer allocate a late DMA bounce buffer or use an
      internal stdio buffer that is not suitable for SDMMC DMA. When internal
      heap is already tight, that allocation fails with
      `esp_dma_capable_malloc` / `sdmmc_read_blocks failed`. On ESP32,
      `AudioSourceSD` therefore opens the SD file through the VFS file
      descriptor API and reads through one 512-byte internal DMA-capable bounce
      buffer, then copies into the decoder's destination. Do not switch this
      path back to stdio without retesting MP3 playback on real hardware.
      Hardware testing also confirmed that MP3 playback continues while the
      Atari resets and reloads the test XEX, while the WebUI mounts a different
      disk, and while the newly booted Atari program connects to a network TCP
      port and streams data.
- [ ] Measure firmware size and heap impact.

Exit criteria:

- [x] SD-card MP3 files play on ESP32.
- [ ] SD-card MP3 files play on FujiNet-PC.
- [ ] Corrupt MP3 input reaches `ERROR` without crashing or wedging playback.
- [x] SIO service remains reliable during sustained decoding.

## Phase 6: HTTP/HTTPS streaming and internet radio

Goal: play direct HTTP audio files and continuous Icecast/Shoutcast streams.

- [ ] Implement `AudioSourceHTTP`.
- [ ] Implement `AudioSourceHTTP` with explicit `open()`, bounded `read()`,
      `close()`, and `cancel()` semantics behind the `AudioSource` interface.
- [ ] Prefer lower-level streaming reads over the current `fnHttpClient`
      `GET()`/`read()` transaction model when required for continuous streams.
- [ ] Reuse or refactor FujiNet HTTP/TLS infrastructure rather than creating a
      second unrelated network stack.
- [ ] Do not depend on `Content-Length` or `available()` for live stream
      progress; support unknown-length and infinite bodies as a normal case.
- [ ] Avoid deleting an in-flight HTTP task as the primary cancellation
      mechanism; cancellation must close or abort network waits cleanly where
      the underlying HTTP/TLS stack permits it.
- [ ] Ensure source reads are cancellable.
- [ ] Increase buffering beyond the existing 512-byte HTTP handoff.
- [ ] Support responses with known content length.
- [ ] Support chunked responses.
- [ ] Support responses with unknown/infinite content length.
- [ ] Follow redirects with a bounded redirect count.
- [ ] Capture HTTP status and selected response headers.
- [ ] Use `Content-Type` as one codec-detection input.
- [ ] Request `Icy-MetaData: 1`.
- [ ] Parse `icy-metaint`.
- [ ] Remove ICY metadata blocks before decoder input.
- [ ] Parse `StreamTitle` safely.
- [ ] Capture `icy-name`, `icy-br`, and relevant stream headers.
- [ ] Increment metadata generation when metadata changes.
- [ ] Implement startup buffering.
- [ ] Implement underrun detection and transition back to `BUFFERING`.
- [ ] Implement bounded reconnect/backoff policy.
- [ ] Distinguish end-of-file from a dropped live stream.
- [ ] Ensure stop interrupts DNS, connect, TLS, and read waits where possible.
- [ ] Test plain HTTP MP3 files.
- [ ] Test HTTPS MP3 files.
- [ ] Test Icecast MP3 streams.
- [ ] Test redirects.
- [ ] Test chunked transfer.
- [ ] Test streams without `Content-Length`.
- [ ] Test slow and bursty servers.
- [ ] Test Wi-Fi loss and reconnection.
- [ ] Test invalid certificates according to the firmware's existing TLS
      policy.
- [ ] Test continuous playback for at least several hours.

Exit criteria:

- [ ] A direct MP3 URL plays through the Atari mapping of the shared audio
      command set.
- [ ] At least one representative Icecast/Shoutcast station plays reliably.
- [ ] Current stream title can be retrieved through A:.
- [ ] Wi-Fi interruption does not crash or block SIO.

## Phase 7: Atari client API and A: user experience

Goal: make the device practical for Atari software without embedding protocol
details in every application.

- [ ] Add protocol constants and structures to the appropriate Atari client
      library.
- [ ] Provide assembly-callable wrappers.
- [ ] Provide C-callable wrappers.
- [ ] Implement source/open helper.
- [ ] Implement play, pause, resume, stop, and volume helpers.
- [ ] Implement status decoding.
- [ ] Implement metadata retrieval.
- [ ] Implement capability negotiation.
- [ ] Provide a minimal command-line or menu-driven player.
- [ ] Display opening, buffering, playing, paused, finished, and error states.
- [ ] Display station/title metadata when available.
- [ ] Document direct SIO usage for software not using the helper library.
- [ ] Decide whether a resident CIO `A:` handler is desirable and define its
      semantics separately.
- [ ] If a CIO handler is added, ensure its text/device operations map cleanly
      to the binary SIO protocol rather than changing firmware semantics.

Exit criteria:

- [ ] A sample Atari application can enter or select a URL and control
      playback.
- [ ] Applications can remain responsive while FujiNet plays audio.
- [ ] Protocol-version mismatch produces a useful user-visible result.

## Phase 8: AAC and additional source support

Goal: broaden practical internet-radio compatibility after the MP3 path is
stable.

- [ ] Add AAC/ADTS decoder support on ESP32.
- [ ] Select and document the FujiNet-PC AAC decoder strategy.
- [ ] Support AAC content-type detection.
- [ ] Test common AAC-LC radio streams.
- [ ] Evaluate HE-AAC support and CPU requirements.
- [ ] Evaluate FLAC and Ogg/Vorbis based on firmware size and demand.
- [ ] Add configured-host-slot URI support.
- [ ] Reuse existing FujiNet filesystem abstractions where thread safety and
      cancellation behavior are adequate.
- [ ] Test SD, TNFS, SMB, NFS, and FTP sources independently before enabling
      each.
- [ ] Add per-source capability reporting.

Exit criteria:

- [ ] Newly enabled codecs and source types pass the same stop/cancel,
      malformed-input, and concurrency tests as MP3/HTTP.
- [ ] Features that exceed smaller-board flash or RAM can be disabled cleanly
      at build time.

## Phase 9: SAM integration and shared audio ownership

Goal: eliminate competing implementations that directly own the audio pin and
make SAM the first real generated-audio producer for `AudioService`.

This is the next implementation target after the initial ESP32 SD-card MP3
playback milestone. HTTP streaming, AAC, HLS, and broad FujiNet-PC MP3 testing
remain deferred while SAM is migrated onto the shared audio path.

- [x] Refactor SAM so speech generation can emit PCM without directly opening
      an output device.
- [x] Add a SAM PCM source or enqueue generated SAM PCM into `AudioService`.
- [ ] Separate SAM synthesis from `OutputSound()` so generated samples can be
      handed to `AudioService` without blocking the SIO voice-device command
      path.
- [x] Replace or wrap SAM's current global generated-sample buffer lifecycle
      with an owned PCM buffer or producer object that can be queued, cancelled,
      and freed deterministically.
- [x] Normalize SAM's current 8-bit generated samples into the signed 16-bit PCM
      format expected by `AudioService`, or document an explicit conversion path
      in the SAM producer.
- [x] Use SAM as the reference implementation for future generated-audio
      devices that push PCM into `AudioService`.
- [x] Queue appended SAM PCM chunks so printer-split utterances are played in
      order instead of each chunk cancelling the previous chunk.
- [x] Preserve existing P3: command behavior.
- [x] Preserve SAM parameter and preset behavior.
- [x] Route rotation sounds through the shared audio service if appropriate.
- [x] Define arbitration behavior when SAM is requested during music playback.
- [x] Implement the initial SAM/music behavior using the Phase 1 arbitration
      rules if the mixer is not ready.
- [ ] Define the future mixed behavior where SAM speech overlays active music or
      radio playback instead of requiring direct sink access.
- [x] Ensure only one component owns DAC/I2S/miniaudio output resources.
- [ ] Remove the classic ESP32 per-sample one-shot DAC output loop.
- [ ] Remove duplicated S3 sink initialization from SAM.
- [ ] Remove duplicated FujiNet-PC miniaudio output setup from SAM.
- [ ] Add a test or manual scenario for disk-swap speech during active stream
      playback.
- [ ] Add regression tests for P3: speech.

Implementation note:

Atari SAM now hands each generated 8-bit sample buffer to `audioDev`, which
copies and converts it into owned signed 16-bit mono PCM before queueing a
`PLAY_PCM` command on `AudioService`. SAM uses the generated-PCM append path:
the first SAM chunk interrupts any active non-SAM source using the current Phase
1 generation/cancel behavior, while later SAM chunks for the same utterance
reuse the active SAM generation and remain FIFO queued instead of cancelling the
chunk currently playing. `AudioService` then opens the active `AudioSink` and
drains each PCM chunk in the same worker path used by other audio commands. This
keeps Atari SAM from directly owning DAC/I2S/miniaudio output. True overlay
mixing remains a future mixer task. SAM synthesis itself still runs
synchronously before each PCM chunk is queued, so the remaining blocking item
above is about moving generation off the SIO command path, not about playback
duration. Legacy SAM direct-output code is still present for non-Atari paths and
should be removed or wrapped once those targets are migrated.

Disk rotation announcements now use the same path. Generic `fujiDevice`
rotation logic calls a platform hook after rotating IDs and locating the slot
now assigned to D1:. Atari SIO overrides that hook and speaks "disk N" with the
existing SAM phonetic helpers, so the generated PCM enters `AudioService`
through the SAM append queue instead of writing directly to DAC/I2S/miniaudio.

Exit criteria:

- [ ] SAM speech uses the generic sink.
- [ ] SAM speech reaches the sink only through `AudioService`.
- [ ] SAM no longer blocks the SIO service loop for the duration of playback.
- [ ] Audio-resource arbitration is deterministic and documented.
- [ ] Future generated-audio devices can follow the SAM producer pattern without
      taking direct ownership of output hardware.

## Phase 10: HLS and playlists

Goal: support stations that do not expose a direct continuous MP3/AAC URL.

- [ ] Implement M3U and PLS parsing for direct stream playlists.
- [ ] Resolve relative playlist URLs.
- [ ] Add bounded playlist sizes and entry counts.
- [ ] Evaluate HLS requirements separately from simple playlists.
- [ ] Implement HLS master/media playlist parsing if approved.
- [ ] Implement segment fetching and sequencing.
- [ ] Implement live-window refresh.
- [ ] Handle discontinuities and codec changes.
- [ ] Add MPEG-TS demuxing if required by selected streams.
- [ ] Test AAC-in-TS and MP3-in-TS if supported.
- [ ] Define stop/cancel behavior across segment fetches.
- [ ] Measure RAM, flash, CPU, and network impact before enabling by default.

Exit criteria:

- [ ] Playlist and HLS support does not destabilize direct-stream playback.
- [ ] Unsupported playlist features produce explicit errors.

## Phase 11: Configuration, observability, and hardening

Goal: prepare the feature for broader testing and eventual release.

- [ ] Add build-time audio enable/disable option if firmware-size constraints
      require it.
- [ ] Add board capability detection for physical audio output.
- [ ] Add configurable default volume if approved.
- [ ] Add configurable input-buffer size and startup threshold.
- [ ] Add optional web UI controls only after the core protocol is stable.
- [ ] Add console diagnostics for audio state, buffers, tasks, and errors.
- [ ] Expose task stack high-water marks in debug builds.
- [ ] Expose underrun and reconnect counters in debug/status output.
- [ ] Audit all source length and metadata parsing.
- [ ] Fuzz WAV, MP3 framing, playlist, and ICY metadata parsers where practical.
- [ ] Run static analysis on the new subsystem.
- [ ] Verify cleanup during firmware reboot and shutdown.
- [ ] Verify repeated Wi-Fi reconnects.
- [ ] Verify playback while mounting disks and accessing network devices.
- [ ] Verify playback while the web UI is active.
- [ ] Verify behavior under low-memory conditions.
- [ ] Verify behavior on boards with and without PSRAM.
- [ ] Document codec and third-party library licenses.
- [ ] Document supported boards, codecs, URI schemes, and known limitations.

Exit criteria:

- [ ] Long-running stress tests pass without leaks, watchdog resets, or SIO
      failures.
- [ ] Unsupported hardware fails gracefully.
- [ ] Build and licensing implications are documented.

## Phase 12: Additional platform bus registration

Goal: add the already-shared `audioDevice` to additional platform bus chains as
audio-capable prototype hardware becomes available.

- [ ] Keep shared command handling in `lib/device/audioDevice.*`.
- [ ] Keep audio service, source, decoder, and sink APIs bus-neutral.
- [ ] Keep platform packet framing and transport responses in each bus layer.
- [ ] Identify audio-capable hardware for each platform.
- [ ] Add null/unsupported behavior where no output exists.
- [ ] Assign each bus's native device ID, type, channel, or discovery entry.
- [ ] Add the shared audio device to each supported bus chain during setup.
- [ ] Dispatch the shared `AUDIOCMD_*` commands from each bus.
- [ ] Map shared request/response payloads onto each bus transaction model.
- [ ] For platforms that want emulated sound chips, define separate device-layer
      commands that accept chip state, register writes, or frame updates and
      submit generated PCM to `AudioService`.
- [ ] Treat future AY-3-8910-style support as a generated-audio producer, not as
      a direct `AudioSink` user.
- [ ] Add platform-specific host libraries and sample applications.

Exit criteria:

- [ ] At least one non-SIO platform registers and controls `audioDevice`
      without including Atari-specific headers or protocol code.

## Testing matrix

The following matrix should be maintained as implementation proceeds.

| Target | Output | WAV/SD | MP3/SD | MP3/HTTP | Radio/ICY | SAM regression |
| --- | --- | --- | --- | --- | --- | --- |
| Atari FujiNet v1 ESP32 | GPIO25 DAC | [ ] | [ ] | [ ] | [ ] | [ ] |
| Atari ESP32-S3 WROOM | GPIO21 PDM | [ ] | [ ] | [ ] | [ ] | [ ] |
| Atari ESP32-S3 xDrive | Unsupported/null | [ ] | [ ] | [ ] | [ ] | [ ] |
| FujiNet-PC Linux | miniaudio | [ ] | [ ] | [ ] | [ ] | [ ] |
| FujiNet-PC macOS | miniaudio | [ ] | [ ] | [ ] | [ ] | [ ] |
| FujiNet-PC Windows | miniaudio | [ ] | [ ] | [ ] | [ ] | [ ] |

For the xDrive and other boards without an audio output, a successful test
means the build succeeds, capabilities report no output, and commands fail
gracefully without affecting other FujiNet functions.

## Performance and reliability targets

These are initial engineering targets and should be revised from measurements:

- Bus command handlers perform no unbounded waits.
- Stop request begins cancellation immediately and silences output within
  250 ms under normal conditions.
- Status remains available while opening, buffering, reconnecting, or failing.
- No audio worker priority equals or exceeds the SIO service-loop priority.
- No sustained audio work runs on core 1 in the initial ESP32 design.
- No network or decoder buffer grows without a fixed maximum.
- Continuous radio playback runs for at least 8 hours without a memory leak,
  watchdog reset, or unrecoverable underrun.
- Disk reads, directory operations, and network-device commands remain usable
  during playback.
- Unsupported or malformed input cannot crash firmware or FujiNet-PC.

## Open design questions

- [x] `AUDIOCMD_SET_SOURCE` only validates and stores the pending source; all
      opening, buffering, decoding, and output waits for `AUDIOCMD_PLAY`.
- [x] `AUDIOCMD_PLAY` restarts a finished source when possible, resumes from
      `PAUSED`, and retries the current source from `ERROR` when one exists.
- [x] Pause and full-buffer handling must apply backpressure to source reads
      rather than allowing unbounded buffering.
- [x] Default boot volume is 100 for protocol version 1. Physical SIO output
      calibration remains part of sink validation, not the public protocol.
- [x] Volume is not persisted in protocol version 1; persisted default volume is
      deferred to the configuration phase if approved.
- [ ] Until the mixer exists, should SAM interrupt audio, pause it, queue, or
      reject speech requests?
- [ ] What mixer policy should apply when generated audio overlays active
      playback: per-source gain, ducking, clipping prevention, and priority?
- [ ] Is software resampling required initially, or should unsupported sample
      rates be rejected?
- [ ] Which sample rates are reliable through each physical sink?
- [ ] Should the decoder always output a fixed sink rate to simplify hardware?
- [ ] Is the existing HTTP client suitable after refactoring, or should an
      audio-specific adapter use the lower-level HTTP client directly?
- [ ] Can configured-host `FileHandler` implementations be safely called from
      an audio task while their related bus devices are active?
- [ ] Which codec features fit all supported flash layouts?
- [ ] Is AAC support required before the first usable experimental release?
- [ ] Should A: expose a small text command mode in addition to the binary SIO
      command protocol?

## Definition of experimental success

The initial experiment is successful when:

- [ ] An Atari application sends an HTTP MP3 URL to the shared audio device
      through its SIO `$65` mapping.
- [ ] FujiNet returns promptly and begins playback asynchronously.
- [ ] Audio is emitted through the Atari SIO audio pin and mixed by the Atari
      hardware with POKEY output.
- [ ] The Atari can continue disk, network, and configuration operations while
      audio plays.
- [ ] The application can query state and current stream title.
- [ ] Pause, resume, stop, and volume controls work.
- [ ] The same shared audio commands work in FujiNet-PC through its emulated
      Atari `$65` mapping with host-system audio output.
- [ ] Failure or loss of the stream produces a recoverable status/error rather
      than blocking or resetting FujiNet.
