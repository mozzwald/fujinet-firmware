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
- Pause, resume, stop, volume, info, and metadata can be selected from the menu.
- Seek is expected to fail until seekable SD/HTTP sources are implemented.

If FujiNet-PC logs `SIO CMD ignored` for command frames beginning with
`65`, rebuild FujiNet-PC for Atari and confirm the `$65` audio-device
registration is present in the build being run.

## Menu Coverage

- `C`: capabilities
- `S`: basic status
- `I`: extended status
- `O`: set source
- `P`: play
- `A`: pause
- `R`: resume
- `X`: stop
- `V`: volume
- `M`: metadata
- `K`: seek
- `Q`: quit

The initial source string is `gen:test-tone`. This gives the firmware side a
simple generated-audio target before SD, HTTP, WAV, or MP3 sources exist.
