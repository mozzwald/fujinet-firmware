# FujiNet Audio Atari Test

This is a small Atari 8-bit `.xex` test app for the FujiNet audio device.
It sends direct SIO commands to device `$65`, the planned Atari audio-device
ID, and prints the returned SIO status and protocol fields.

The app is intentionally separate from the firmware implementation so it can
evolve into the manual regression tool for the audio feature.

## Build

```sh
make
```

The output is:

```text
build/fnaudio-test.xex
```

## Run With FujiNet-PC

Build FujiNet-PC for Atari:

```sh
./build.sh -cgp ATARI
```

Start FujiNet-PC and an Atari emulator using the normal NetSIO workflow, then
boot `build/fnaudio-test.xex`.

Expected behavior with the `$65` audio device registered:

- Capabilities returns a protocol/version block.
- Status returns the four-byte basic audio status. The initial idle response is
  expected to begin with protocol version `01`.
- Set source accepts the initial source string `gen:test-tone`.
- Play submits the generated test tone to the FujiNet audio service. On
  FujiNet-PC this should be audible through the host output device when one is
  available.
- `W` sets the source to `sd:/fnaudio.wav` for SD/WAV playback testing. Put a
  WAV file named `fnaudio.wav` at the SD root, then press `W` and `P`.
- `M` sets the source to `sd:/fnaudio.mp3` for SD/MP3 playback testing. Put an
  MP3 file named `fnaudio.mp3` at the SD root, then press `M` and `P`.
- Pause, resume, stop, volume, info, and metadata can be selected from the menu.
- Seek is expected to fail until service-level seek support is wired to SD/HTTP
  sources.

If FujiNet-PC logs `SIO CMD ignored` for command frames beginning with
`65`, rebuild FujiNet-PC for Atari and confirm the `$65` audio-device
registration is present in the build being run.

## Menu Coverage

- `C`: capabilities
- `S`: basic status
- `I`: extended status
- `O`: set generated tone source
- `W`: set `sd:/fnaudio.wav`
- `M`: set `sd:/fnaudio.mp3`
- `P`: play
- `A`: pause
- `R`: resume
- `X`: stop
- `V`: volume
- `T`: metadata title
- `K`: seek
- `Q`: quit

The generated tone source is `gen:test-tone`. The fixed SD test sources are
`sd:/fnaudio.wav` and `sd:/fnaudio.mp3` so the same bootable app can compare
WAV and MP3 playback behavior.
