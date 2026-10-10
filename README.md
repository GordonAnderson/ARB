# ARB

Firmware for the GAA Custom Electronics **ARB module**, an 8-channel arbitrary waveform
generator for the MIPS system. It is used to drive traveling wave (Twave) ion devices.
This is the firmware for the **original ARB hardware**, the module built around a plug-in
Arduino Due (rev 2.0 and 3.0). The newer ARB with the integrated SAM3X8E (rev 4.1 and
later) runs the sibling **ARB2** firmware.

Current version: **1.25**. The revision history is at the top of [src/ARB.cpp](src/ARB.cpp),
and open issues are in [TODO.md](TODO.md).

Version 1.25 is the Arduino IDE firmware (1.24) ported to PlatformIO and brought up to the
feature level of ARB2 2.27. The code is based on ARB2 with the ARB2-only hardware removed.

## What it does

The firmware has two operating modes:

- **TWAVE mode.** All 8 outputs play the same waveform, each shifted 1/8 of a period (45°)
  from the one before. The waveform can be sine, ramp, triangle, pulse, or a user-defined
  ARB vector. Each period has 4 to 128 points (32 by default), and the module supports
  direction reversal, phase shift, compression, an alternate waveform, and
  frequency/amplitude sweeps.
- **ARB mode.** A conventional arbitrary waveform generator with a buffer of up to 8000
  samples per channel. It plays the buffer a set number of times on each sync trigger.
  A channel can be loaded with a value, a value over a range, or one sine cycle.

The waveform buffer is streamed by DMA to two memory-mapped quad 8-bit DACs on the 8-bit
parallel bus (SMC). The DMA clock comes from the timer, or from an external clock through
the CPLD. An AD5625 on `Wire1` (I2C) sets the voltage range, offset, aux and reference. The
processor's own DACs set the two board bias outputs.

## What differs from ARB2

| | ARB (this firmware) | ARB2 |
|---|---|---|
| Hardware | Plug-in Arduino Due, rev 2.0 and 3.0 | Integrated SAM3X8E, rev 4.1 and later |
| DAC bus | 8 bit, channels in order 0 to 7 (`Ch2Index(i)` is `i`) | 16 bit, channels interleaved |
| Range, offset, aux, reference | AD5625 on `Wire1` | AD5592 on SPI |
| Sync / compress lines | A0 / pin 22 | A6 / pin 22 |
| Max ARB sample rate | 1.28 MHz (`MAXARBRATE`) | 2.56 MHz |
| CPLD programming (`JTAG`), option jumpers | No | Yes |
| Readbacks (`RAUX`, `ROFF`, `RTW`) and the reference multiplier | No | Yes |
| Old-board only | `TMCRT`, `SLEGACY`, `SARBSINE`, SMC timing commands | |

## Interfaces

| Interface | Use |
|---|---|
| TWI (I2C) slave, `Wire`, default address `0x40` | Control from the MIPS controller. Command codes are the `TWI_*` constants in [include/ARB.h](include/ARB.h). |
| Native USB serial | ASCII host commands, calibration, FLASH upload and download. |
| `Wire1` (I2C master, 400 kHz) | The AD5625 DAC at address `0x12`. |
| Line 1 (A0) / Line 2 (pin 22) | Hardware sync and compress inputs. Either function can be moved to either line with `SSYNCLINE` / `SCOMPLINE`. |

### Serial commands

Commands are comma-delimited and end with `\n` or `;`. Commands starting with `S` set a
value and reply with ACK (`0x06`). Commands starting with `G` reply with ACK followed by
the value. An invalid command or argument returns NAK (`0x15`) and sets the error code
read by `GERR`. `GCMDS` lists every command. The full table, with a comment on each entry,
is `CmdArray` in [src/Serial.cpp](src/Serial.cpp).

Commonly used commands:

| Command | Description |
|---|---|
| `GVER` | Firmware version |
| `SMODE,TWAVE` / `SMODE,ARB` | Select the mode |
| `SWFENA` / `SWFDIS` | Start / stop waveform generation |
| `SWFREQ,f` | Frequency in Hz |
| `SWFRM,SIN` | Waveform type: `SIN`, `RAMP`, `TRIANGLE`, `PULSE`, `ARB` |
| `SWFVEC,v1,v2,...` | ARB waveform points, -100 to 100 |
| `SWFVRNG,v` / `SWFVOFF,v` / `SWFVAUX,v` | Range, offset and aux voltages |
| `SWFDIR,TRUE` | Twave direction |
| `SFWDPS,deg` / `SREVPS,deg` | Phase shift for the forward / reverse direction |
| `SWFCMP,TRUE` / `SWFORD,n` | Compression on/off, compression order |
| `SALTENA,TRUE` and the other `SALT*` commands | Alternate waveform |
| `SARBSINE,ch,phase` | ARB mode: one sine cycle on a channel |
| `SAVE` / `RESTORE` | Save settings to FLASH / load them |
| `CALAUX`, `CALOFF`, `CALRNG`, `CALCHN,n`, `CALBIAS,n` | Interactive calibration |

## Building

This is a [PlatformIO](https://platformio.org) project (`env:dueUSB`, `atmelsam`, Arduino
framework).

```sh
pio run                 # build
pio run -t upload       # build and upload over the native USB port
```

Every build also copies the binary to `firmware/ARB-V<version>.bin`. The version number
comes from the `Version` string in `src/ARB.cpp`, and the copy is made by
[rename_firmware.py](rename_firmware.py). Change the `Version` string and `fVersion`
together when you release.

`platformio.ini` links with `-Wl,--allow-multiple-definition`. This lets
[src/MemoryFix.cpp](src/MemoryFix.cpp) replace the core's `_sbrk`, so `malloc` and `new`
return NULL when memory runs out. With the Arduino IDE this needed a hand-edited
`syscalls_sam3.c`.

### Libraries

No libraries are taken from the build machine. Everything is either in `lib/` or pulled
from GitHub:

| Library | Source |
|---|---|
| `MIPStimer`, `SerialBuffer`, `DueFlashStorage` | `lib/`, copied from [GordonAnderson/MIPS](https://github.com/GordonAnderson/MIPS) `lib/` (same copies as ARB2) |
| `Parallel` | [delsauce/ArduinoDueParallel](https://github.com/delsauce/ArduinoDueParallel), fetched through `lib_deps` |
| `Wire` | Arduino SAM core |

## Project layout

| Path | Contents |
|---|---|
| `src/ARB.cpp` | `setup()` / `loop()`, waveform generation, clocking, the TWI command handler, and the serial command handlers |
| `src/Hardware.cpp` | DMA engine, AD5625 driver, board bias and supply monitors, SMC timing commands, and FLASH upload/download |
| `src/Serial.cpp` | Serial command table and parser, plus the input ring buffer |
| `src/Calibration.cpp` | Interactive calibration routines |
| `src/MemoryFix.cpp` | `_sbrk` replacement and `freeMemory()` |
| `include/Shared.h` | Declarations shared between the source files (the Arduino IDE generated these) |
| `include/ARB.h` | TWI command codes and `ARB_PARMS`, the saved parameter block |
| `include/Hardware.h` | Pin assignments, AD5625 channels, and DMA helpers |

## Settings storage

`ARB_PARMS` is written as raw bytes to the last 1000 bytes of FLASH (`NonVolStorage`).
Settings are restored at boot if the stored signature matches. `GETFLASH` / `PUTFLASH`
transfer this block to and from the MIPS host app as hex with an 8-bit CRC. Run-time state
(enable, sync, compression and alternate waveform flags) is cleared in the saved copy, so
the module always powers up idle.

Version 1.25 added members to the end of `ARB_PARMS`. The members before them are laid out
exactly as in 1.24, so settings saved by 1.24 or earlier restore with their calibration
intact, and the new settings take their defaults until the next `SAVE`. The last member
(`layout`) tells the two apart. **Only add new members at the end of the structure.**

## Firmware updates

`ARBPGM,address,size` writes a hex image into FLASH bank 1 (the firmware rejects any
address outside bank 1). The ARB image goes to `0xD0000` and the mover application to
`0xC0000`. `MOVEIT,5410` then boots the mover, which copies the image into bank 0 and
restarts.

## Hardware flags

- `ARBparms.Legacy` (`SLEGACY,TRUE`): for the very first ARB rev. It enables `TWI_SET_REF`,
  which writes the Due DAC that set the amplitude on that board.
- `ARBparms.TimingCorrect` (`TMCRT,TRUE`): works around a CPLD logic bug by moving channel 1
  back one sample.
- `ARBparms.CPLD` / `ARBparms.XPtiming` (`CPLD`, `XPT`): select the SMC bus timing. The
  `CYCLETM`, `PULSETM`, `ADDSETUP` and `SMCMODE` commands change the timing live, for
  testing.
- `ARBparms.rev` (`SREV`) is a spare behaviour flag. It is not the firmware version, and
  nothing reads it on this hardware.
