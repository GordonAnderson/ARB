# ARB TODO

Open bugs, things to verify on hardware, and planned improvements. Each entry names the
function so it can be found quickly. Move an item to **Done** with the version number when
it is fixed. Most of the open bugs were found during the ARB2 review and are shared with ARB2
(both firmware versions are the same code below the hardware layer), fix them in both.

## Verify on hardware (version 1.25)

Version 1.25 is a port. It was built, but it has **not been run on an ARB**. The first four
items could stop the module working, check them first.

- [ ] **DAC bus.** The 8 bit bus, `Ch2Index(i) = i` and the double byte write in `UpdateDACs()`
      come from the 1.24 code. Check all 8 outputs with `SDACV`, then TWAVE mode at a few
      frequencies, and check the channel order and the 45° shift.
- [ ] **Sync input.** ARB2's `ARBline1ISR()` decodes the pin as a short pulse or a level change.
      1.24 just took a rising edge on A0. Check that `SSYNCENA,TRUE` plus a sync from MIPS
      restarts the waveform, in TWAVE mode and ARB mode. If the MIPS sync pulse is wide this
      will not trigger. Attach A0 with `RISING` in `setup()` instead.
- [ ] **AD5625 writes.** The driver now saves a write that arrives while another is in progress
      (`DMAC_Handler` can change the range) and sends it afterwards. Check range, offset and aux
      with `SWFVRNG`, `SWFVOFF`, `SWFVAUX`, and the alternate waveform range change.
- [ ] **Restore from 1.24 settings.** Load the new firmware on a calibrated module, run `RESTORE`
      and check `GWFVRNG` etc. and the calibration values match what the old firmware had. Then
      `SAVE` and `RESTORE` again.
- [ ] **Phase shift.** `SFWDPS` / `SREVPS` now move the waveform (see **Done**). ARB2 2.27 has the
      same code and the shift has no effect there, check which behavior MIPS expects.
- [ ] **Alternate waveform, `SSYNCLINE` / `SCOMPLINE`, `TWI_CMD`, sweep.** New to this board.
      Test them with MIPS.
- [ ] **`_sbrk` override.** Check that `new` returns NULL when memory runs out.
- [ ] **`TMCRT` and `SLEGACY`.** Their code moved. `TMCRT` now follows the direction and phase
      shift. Test on a board that needs them.
- [ ] Full regression with MIPS: Twave, compression, ARB mode with a sync trigger, calibration,
      `SAVE` / `RESTORE`, firmware update with `ARBPGM` and `MOVEIT`.

## Open bugs (shared with ARB2)

- [ ] **Power enable polarity is inconsistent.** `Hardware.h` says logic high disables
      `PowerEnable`, and `SPWR ON` drives it LOW. But `TWI_SET_PWR` with 1 drives it HIGH,
      and the end of `setup()` drives it HIGH under the comment "Turn on power supplies".
      Find the correct polarity for this hardware and make all three agree.
- [ ] **Heavy work in the TWI interrupt.** `receiveEvent()` runs inside the TWI ISR, but it
      rebuilds waveform tables (`SetWaveformType`, many `sin()` calls), writes FLASH
      (`TWI_SAVE`), and runs serial commands (`TWI_CMD`). Set flags here and do the work in
      `loop()`, the way `FreqUpdate` and `AuxDACupdate` already work.
- [ ] **`TWI_CMD` busy-waits up to 250 ms in the ISR.** `onReceive` already has the whole
      message, so waiting on `Wire.available()` can never get more bytes. Without a `\n` it
      always waits the full timeout.
- [ ] **Alternate frequency forces internal clock.** On entering the alternate waveform,
      `DMAC_Handler` drives `ClockMode` LOW after `SetFrequency`, so external clock mode is
      dropped until the alternate waveform ends. Check whether this is intended.
- [ ] **`SetFrequency()` called from `DMAC_Handler`.** It stops and restarts the timers inside
      an interrupt. Only the AD5625 write is protected against re-entry.
- [ ] **`SWFVEC` can be processed half-received.** `ProcessSerial()` runs
      `ProcessVectorString()` whenever `VectorString` is non-empty. That can happen while a
      vector is still arriving, if another command line completes first.
- [ ] **Sweep does not update `ActualFreq`.** `GWFREQ` reports the pre-sweep value during a
      sweep. The sweep also changes only the DMA clock, so it does nothing in ISR mode.
- [ ] **Mode change while running.** `SMODE` and `TWI_SET_MODE` change `ARBparms.Mode`, but
      the DMA/clock stays set up for the old mode until the next enable.
- [ ] **Polled alternate hardware ignores edge modes.** With `ISRcompress` false, `loop()`
      treats the compress line as a level for the alternate waveform, even when
      `AltHwdMode` is 1 or 2 (edge with delay and duration).
- [ ] **`Status` is never set.** `TWI_READ_STATUS` always returns 0.
- [ ] **Voltage test / shutdown not implemented.** `TWI_SET_TST_ENABLE` sets
      `VoltageTestEnable`, which nothing reads (rev 3.0 feature list in `ARB.cpp`).
- [ ] **Missing range checks** on `TWI_SET_RANGE`, `TWI_SET_FREQ`, `TWI_SET_AFRQ`,
      `STWIADD`, `SARBNUM`, `SWFORD` and `SALTFRQ`.
- [ ] **Saved settings checks.** `RestoreSettings()` tells the 1.24 and 1.25 layouts apart with
      `ARB_LAYOUT`, but any later change still needs a new marker value and a restore path.
      `ValidateParms()` only catches the fields that size buffers.
- [ ] **Ramp restarts badly.** A new `SWFVRNG` / `TWI_SET_RANGE` during a ramp keeps the
      old start time and base, so the output can jump.
- [ ] **`MeasureVoltages()` reads without settling.** It reads the supply monitors every
      pass of `loop()` that is not ramping.
- [ ] **Sweep voltage and alternate range use `WriteWFrange()` directly** while a ramp can
      be running, so the two can fight.

## Questions / cleanup

- [ ] **Direction naming.** In the code, `Direction == true` plays `WaveformR` and uses
      `fwdPS`. The `SWFDIR` comment said "forward = TRUE", while `ARB.h` said "true is
      reversed". Confirm the intended meaning and rename `fwdPS` / `revPS` or the flag.
- [ ] **Interrupt-driven waveform mode.** It (`RealTimeTWAVEISR` / `RealTimeARBISR`) is only
      used when the DAC clock is at or below `MINDMAFREQ` (0), so in practice never. It also
      needs `ppp` to be a power of two. Remove it or set a real crossover frequency.
- [ ] **Reverse alternate waveform (type 2).** It mirrors raw DAC counts between channels,
      which ignores the per-channel calibration.
- [ ] **`receiveEvent()` is very long.** Consider splitting it into a table of handlers like
      the serial command table.
- [ ] **Share the code with ARB2.** The two projects differ only in `Hardware.cpp`, `Hardware.h`,
      `Ch2Index`, the pins and a few commands. Moving the common code into a shared library
      or one project with two environments would stop fixes being made twice.
- [ ] **`SerialBuffer` is shared with MIPS** (`lib/SerialBuffer`, copied from the MIPS repo).
      `peek()` always returns 0 and the header has no include guard. Fix it upstream in the
      MIPS repo first.
- [ ] **`ARBparms.rev`** is not read by anything on this hardware. Remove it or give it a
      use. Removing it changes the saved layout.

## Done

### 1.25 (PlatformIO port, ARB2 features)
- Ported from the Arduino IDE to PlatformIO. Added `include/Shared.h` for shared
  declarations, and libraries are now taken from `lib/` and GitHub (see README).
- Added the ARB2 features: alternate waveform (all `SALT*` commands and the TWI codes),
  selectable sync / compress lines, ISR compress option, forward and reverse phase shift,
  reverse aux voltage, `TWI_CMD`, ARB buffer length and TWI vector checks.
- Kept the features that only this firmware has: `SARBSINE` / `TWI_SET_SINE`, `TMCRT`,
  `SLEGACY`, the SMC timing commands, the AD5625 driver with the 1.24 reference fix.
- Settings saved by 1.24 or earlier still restore (`ARB_LAYOUT`).
- The AD5625 driver is safe to call from the DMA interrupt. A write that arrives while another
  is in progress is saved and sent afterwards.
- Fixed in ARB2 2.27 and carried over, see the ARB2 TODO.md for the details: `TWI_SET_FWDPS` /
  `TWI_SET_REVPS` not stored, `AD5625()` returned no value, `TWI_CMD` switched the serial port
  to `Serial`, buffer overflows (TWI vector, ARB buffer length, `SetARBchannelRange`, `SWFVEC`,
  `TWI_SET_PPP`), the triangle waveform in reverse, `SWFRM` not setting `wft`, `SWFDIR`,
  `SFWDPS`, `SREVPS` and `SWFVEC` not taking effect until a waveform change, alternate
  frequency restore drifting, `ProcessSweep()` using an uninitialized variable, `SAVE` turning
  off the outputs and replying ACK on failure, `PUTFLASH` leaking and not checking the size,
  `ARBPGM` writing anywhere in FLASH, `SCALDAC` and `CALBIAS` problems, `SDACV` not clamping,
  divide by zero at 0 Hz, `SACHRNG` missing the last sample, `TWI_SWPGO` accepting bad states,
  and the `EOF`, `Ch2Index` and `CMDint` slips in `Serial.h` / `Serial.cpp`.
- **The phase shift commands did nothing.** In `FillDACbuffer()` (ARB2 2.27 has the same
  code) the shifted index was used both for where a sample is stored and for which sample is
  read, so every sample landed back where it started. It now reads sample `k` and stores it at
  `k + shift`. The sign of the shift is unchanged from the ARB2 intent, check it.
- `TMCRT` now follows the direction and the phase shift. Before it always used the forward waveform.
- `AD5625_EnableRef()` is back to the 1.24 fix (external reference). ARB2 2.27 still has
  the old value, which only matters on the old board.
- The DAC range limit in `WriteWF*()` was 65565, a typo for 65535 in 1.24 (ARB2 has it right).
