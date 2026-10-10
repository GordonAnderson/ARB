//
// File: ARB
//
// This program enables the 8 channel ARB using the arduino due connected to two quad
// 8 bit dacs. The dacs are memory mapped for performance reasons.
// This module was desiged to support the PNNL Twave project. The max arb clock
// rate is ~1.2MHz. This will support maximum frequency per channel of ~40KHz when using
// 32 points per cycle. The waveforms are shifted 45 degrees from channel to channel.
// Below are the features of this application:
//  * The arduino due becomes a TWI device that is mounted on a module and communicates
//    with MIPS
//  * This module support two mode, ARB and TWAVE
//    TWAVE - generates the same waveform on each of the 8 outputs each shifed 45 degrees. A number
//            of definded waveforms are supported
//    ARB   - This is a more conventional ARB mode. Commands have been developed to allow you to use
//            this mode to generate pulse sequences.
//  * 8 waveformes total, each shifted 45 degrees
//  * 32 points per waveform
//  * Adjustable frequency in Hz, 1000 to 40000
//  * This output frequency requres a DAC clock 32 times higher
//  * Use DMA mode from MINDMAFREQ and up, interrupts below MINDMAFREQ
//  * The Twave data is buffered in period array, so the 32 points. This buffer is followed by one more
//    period of values that are held at the first period's last value. This is used for compression.
//  * In dma mode the list mode is used to enable continous DMA
//  * Both TWI and serial commands allow control of module
//  * Serial commands also allow calibration and configuration of module
//
//  This firmware is for the original ARB hardware (rev 2.0 and 3.0), the ARB module built around a plug in
//  Arduino Due. The newer integrated ARB (rev 4.1 and later) is supported by the ARB2 firmware. What differs:
//   - The DACs are on an 8 bit memory mapped bus and the channels are in order, 0 to 7.
//   - The range, offset, aux and reference voltages are set by an AD5625 DAC on the I2C bus (Wire1).
//   - Sync is on pin A0 and compress is on pin 22. The Twave mode sync and compress lines can still
//     be swapped with the SSYNCLINE and SCOMPLINE commands.
//   - No CPLD JTAG programming, option jumpers or voltage readback muxes.
//  The ARBparms.Legacy flag selects the very first ARB rev behavior and ARBparms.TimingCorrect works
//  around a CPLD logic bug, see the rev 1.18 and 1.22 history below.
//
// Revision history:
//
// Version 1.1, July 30, 2016
//    Added the following new capability, bacically a new mode of operation that will largely be
//    controlled through the serial interface. July 21, 2016
//     1.) Add new mode, traditional ARB mode. Add a mode parameter to define / switch between modes
//     2.) Add commands and variable to support the following:
//         a.) vector length, maximum of 100k
//         b.) digitization rate, Hz
//         c.) clear array, Set all to mid range
//         d.) set channel to value
//         e.) set channel to value over interval
//         f.) continous, true or false
//         g.) software start and stop commands
// Version 1.2, October 23, 2016
//    Cleanded up a lot of issues related to waveform loading and calibration. Waveform are now
//    generated using calibration parameters.
//      1.) Added all the calibration functions
//      2.) Fixed a bug with the output channel order
//      3.) Added the sync enable function
//      4.) Added save and restore functions
// Version 1.3, November 4, 2016
//      1.) added compression logic
//            -- Order command
//            -- Compression enable command
//            -- External compression logic input signal enable
//            -- External clock capability
//      2.) Changed the frequency logic so that when the frequency is changed we do not re-init the waveform generation
//          only do this if the mode changes from software to DMA hardware.
//      3.) DMA mode used for all frequencies of clock generation
//      4.) Added commands to set the ref voltage and aux offset ref values
//      5.) Added the ability to set the TWI address for the ARB module
// Version 1.4, December 15, 2016
//      1.) The DMA controller has an undocumented BTSIZE limmit of 4095, the code has been redesigned to never exceed this value.
// Version 1.5, January 6, 2017
//      1.) Updated the ARB mode and fixed the external trigger function. The system would accept an external trigger but then it was
//          messed up!
// Version 1.6, January 24, 2017
//      1.) Fixed phase error on the waveforms, only the sine was properly calculating the phase shift.
// Version 1.7, July 27, 2017
//      1.) Added support for the rev 3.0 bias or offset for individual boards.
//      2.) Fixed order = 0 to stop the Twave generation
// Version 1.8, August 4, 2017
//      1.) Added the order ramping, cramp
//      2.) Fixed bugs in the TWAVE mode ARB waveform download
// Version 1.9, September 3, 2017
//      1.) Added Cramp order
// Version 1.10, Dec 2, 2017
//      1.) Added FLASH read and write from MIPS app for all the parameters, includes calibration data
//      2.) Added the TWI commands for the table functions
// Version 1.11, Jan 27, 2018
//      1.) Moved the flash storange area from the second flash page into this ARB app's code space
//      2.) Add capability to assign the serial communications to the TWI Wire port. This allows the
//          MIPS controller to use the serial commands for calibration and setup. Enable with a TWI
//          command and disable with the serial escape character.
// Version 1.12, Feb 25, 2018
//      1.) Debugged the serial communications through the TWI interface. This requires the MIPS app
//          and you issue the TWITALK command to MIPS with the board address and TWI address.
//      2.) Added the ARBPGM command that allow uploading programs to the FLASH memory.
//      3.) Added the MOVEIT command to move an uploaded ARB app from 0xd0000 to 0x80000 and then
//          restart. Note! The ARB app has to be loaded at 0xd0000 and the MOVER app has to be loaded
//          at 0xc0000 for this to work.
//      4.) Moved the parameter storage area to the very top of FLASH.
// Version 1.13, Mar 27, 2018
//      1.) Added extended order command
// Version 1.13a, May 24, 2018
//      1.) Added CPLD command to allow changing timing of new ARB version
// Version 1.14, September 12, 2018
//      1.) Added the sweep commands
//      2.) Added the ability to change the points per waveform value
//      3.) Added TWI command to read version as a float
//      4.) Added TWI command to save parameters
//      5.) Added TWI commands to read and write points per waveform
//      6.) Added support for external clock module, required 8/16/2018 Xilinx version
// Version 1.15, October 9, 2018
//      1.) Adjusted the frequency calculation to reduce the error and never exceed 40KHz, 40KHz was 41.3KHz and its
//          now 39.7KHz
// Version 1.16, November 3, 2018
//      1.) Implemented an option for ISR processing of the compression mode request via hardware
//      2.) Added addition USB host commands and cleaned up a few minor issues.
// Version 1.17, December 7, 2018
//      1.) Implemended the ramp rate
// Version 1.18, June 30, 2019
//      1.) Added a flag to shift channel 1 waveform to correct in firmware a bug in the CPLD logic
// Version 1.19, July 6, 2019
//      1.) Added sine wave generation to conventinal ARB mode, for thermo project. Generates
//          one cycle for defined buffer length.
//          parameters; channel, initial phase in degrees
// Version 1.20, Sept 17, 2019
//      1.) Increased the maximum PPP to 128.
// Version 1.21, Dec 4, 2020
//          - Added new DMA restart function to remove jitter. I now use a timer to restart at a fixed delay
//          - after the controller has reported it has stopped
//          - EnablePower pin was not set to output, fixed that bug
// Version 1.22, Sept 17, 2021
//          - Added the legacy flag to fix a problem with the A/B offset dual ARB amp mode
// Version 1.23, Jan 20, 2022
//          - Sweep stop freq was written to start value
// Version 1.24, Dec 7, 2022
//      1.) Expanded the range of the bias offset from +/- 10V to +/- 30 volts.
//          This uncovered a bug in the refrence set up of the on the ARB module.
//          Upgrading to to this version will require a full recalibration of the module.
//          The reference should be set to 1.5 volts when using the extended bias offset
//          range. This version sets the default at 1.5 volts from 1.39 volts.
// Version 1.25, Oct 10, 2026
//      Ported from the Arduino IDE to PlatformIO and brought up to the feature level of the ARB2
//      firmware (2.27) for the original hardware. The code is based on ARB2, with the ARB2 only hardware
//      support removed. New in this version:
//          - Alternate waveform mode (compress, reverse, arb, fixed profile, current) with TWI, host and
//            hardware trigger control, optional range and frequency change, delayed and timed triggers
//          - Selectable sync and compress hardware lines, ISR processing of the compress line
//          - Forward and reverse direction phase shifts, unique reverse direction aux voltage
//          - TWI_CMD, runs a serial command through the TWI interface
//          - Phase shift of the waveform for each direction
//          - Bug fixes, see TODO.md. Most were found in ARB2 and fixed here too: buffer overflow checks on the
//            TWI vector, ARB buffer and serial long string commands, triangle waveform in reverse direction,
//            SWFDIR / SWFVEC / SFWDPS / SREVPS take effect immediately, SAVE no longer turns off the
//            outputs, ARBPGM is limited to FLASH bank 1, the phase shift commands now shift the waveform.
//      Settings saved by rev 1.24 or earlier are restored, including the calibration. The new
//      settings take their defaults until the module is saved again.
//
// Implementation notes for programming through the TWI interface, all items are done
//      1.) Write a mover application that is located at the start of flash bank 1. This app, when called
//          Will move the ARB image from flash bank 1 to start of flash bank 0 then start it up. The app
//          is located in the second half of flash bank 1 so its max size is 128k. This should be plenty
//          because the ARB app is small. If the mover is small enough then we can allow a bigger ARB app.
//      2.) Add functions to the ARB app to program a file to flash and to verify.
//      3.) Add function to ARB amp to start the mover.
//
// June 15, 2017
//      Harware rev 3.0 has been developed and uses the Xilinx CPLD for the control logic. This hardware revision adds the following
//      capabilities:
//        - +- 12V power monitors
//        - +- HV opamp voltage monitors
//        - Offset output control supporting two channels
//        - Power supply enable output
//      Add the following firmware updates to support the new hardware
//        1.) Calibration functions for the new voltage monitoring functions.
//            4 total, +12,-12,+50,-50. This may not require a calibration procedure.
//            Just calculate using the ideal parameters, does not need to be accurate.
//        2.) Calibration function for bias channels. Set the zero point and the 5 volt bias point then calculate
//        2.) Enable voltage testing / shutdown
//        3.) Offset control using DAC 0 and DAC 1, The output voltage range of the DAC is only 2.75-0.55 = 2.2 V, with a resolution
//            of 2.2 /4095 = 0.5372 mV.
//        4.) Add voltage reporting function.
//        5.) Power supply ON/OFF command. (pin 25)
//
// Gordon Anderson
//
#include "Shared.h"
#include <efc.h>
#include <flash_efc.h>
#include <Parallel.h>
#include <stddef.h>

// The TWAVE DAC buffer holds two waveform periods. The first is the waveform; the second
// is the "alternate" period used for compression and the alternate waveform modes (by
// default it holds the last value of the first period).

#define TIOA0 2             // DUE pin number of the TIOA0 signal, used for LDAC control

#define DACref    DAC1      // DAC used to set the reference voltage
#define DACoffset DAC0      // DAC used to set the output amplifier offset

MIPStimer DMAclk(0);        // This timer is used to generate the clock for DMA
MIPStimer ResetTMR(2);      // This timer is used to remove jitter on the sync function
MIPStimer ARBclk(3);        // This timer is used to generate the clock in interrupt mode
MIPStimer ALTtmr(4);        // This timer is used to det the delay and duration of the alternut waveform.
                            // User 10.5MHz, delay and playtime are in mS.

char Version[] = "\nARB Version 1.25, Oct 10, 2026";
float fVersion = 1.25;

SerialBuffer sb;

// General buffers and waveform generation parameters
// Each DAC sample is 8 bytes (one per channel, see Ch2Index) written as two 32 bit words.
uint32_t ARBbuffer[MAXBUFFER * 2];           // ARB mode buffer, MAXBUFFER samples
uint32_t buffer[PPP * NP * 2 * (CHANS / 4)]; // TWAVE mode buffer, two periods of PPP samples
uint8_t  *bufBytes = (uint8_t *)buffer;      // Byte pointer to the TWAVE buffer
int      ptr = 0;                            // ISR mode DAC output word index
uint32_t *DACadd = (uint32_t *)0x60000000;   // Memory mapped base address of DACs
uint8_t  *DACchan = (uint8_t *)0x60000000;   // Memory mapped base address of single DAC channel

float   DACchanValues[8] = {0,0,0,0,0,0,0,0};
uint8_t DACchanBits[8]   = {127,127,127,127,127,127,127,127};

// Writes DACchanBits to the eight DAC channels, one 8 bit bus write per channel. Each
// write is done twice, the second write makes sure the DAC latches the value.
static void UpdateDACs(void)
{
  for(int i=0;i<8;i++)
  {
    DACchan[i] = DACchanBits[i];
    DACchan[i] = DACchanBits[i];
  }
}

uint8_t Waveform[CHANS][PPP];                // Active waveform forward, this is one cycle of the active waveform for each channel.
uint8_t WaveformR[CHANS][PPP];               // Active waveform reverse, this is one cycle of the active waveform for each channel.
int8_t  ARBwaveform[PPP];                    // This waveform is downloaded via TWI and is the user defined arbitrary waveform
char    VectorString[PPP * 5 + 2] = "";      // SWFVEC vector from the serial port, up to PPP values of "-100,". Processed when not empty.

#define UpdateVoltageRange      1
#define UpdateVoltageOffset     2
#define UpdateVoltageAux        4
#define UpdateBias0             8
#define UpdateBias1             16
#define UpdateAltRange          32
#define UpdateAll               (UpdateVoltageRange | UpdateVoltageOffset | UpdateVoltageAux | UpdateBias0 | UpdateBias1)

#define CLKSperMS               656.25     // ALTtmr clocks per millisecond (TIMER_CLOCK4)

volatile byte    AuxDACupdate = UpdateAll; // Update* flags, DAC writes requested from ISRs and done in loop()

bool    FreqUpdate = true;                  // Request loop() to apply ARBparms.RequestedFreq
bool    VoltageTestEnable = false;          // Set by TWI_SET_TST_ENABLE, not used yet (see TODO.md)
byte    Status;                             // Reported by TWI_READ_STATUS, never set (see TODO.md)
int     WorkingOrder = 1;                   // Current compresssion order
int     CrampCounter = 0;                   // Order ramping counter

bool    RangeUpdate = false;                // Request loop() to move the range to VoltageRangeRequest
float   VoltageRangeRequest;

int     DACaddBias[2] = {DAC0, DAC1};

float   PS12v[2] = {0, 0};
float   PS50v[2] = {0, 0};

static  bool enabled = false;               // Output state last applied by loop()
static  bool ReadyForTrigger = false;       // ARB mode is armed and waiting for a sync trigger
static  bool ForceUpdate = false;           // Re-apply the enable state even if it has not changed

FreqSweep fSweep = {10000, 10000, 20000, 20, 30, 30, 10, 0, 0, SS_IDLE};
bool SweepUpdateRequest = false;

ARB_PARMS ARBparms;

void  (*ARBline1pulseISR)(void) = NULL;
void  (*ARBline1stateISR)(int)  = NULL;
void  (*ARBline2pulseISR)(void) = NULL;
void  (*ARBline2stateISR)(int)  = NULL;

DueFlashStorage dueFlashStorage;

// Clears the run time state flags so saved settings always start up idle
static void ClearRunTimeFlags(ARB_PARMS *ap)
{
  ap->Enabled = false;
  ap->SyncEnable = false;
  ap->CompressEnable = false;
  ap->CompressHardware = false;
  ap->AlternateEnable = false;
  ap->AlternateHardware = false;
  ap->AlternateRngEna = false;
  ap->AlternateFreqEna = false;
}

// Forces the parameters that size buffers into their valid ranges. Used after a
// restore so a corrupt or out of date FLASH image can not overrun the buffers.
static void ValidateParms(void)
{
  if ((ARBparms.ppp < 4) || (ARBparms.ppp > PPP)) ARBparms.ppp = 32;
  if ((ARBparms.Bufferlength < 1) || (ARBparms.Bufferlength > MAXBUFFER)) ARBparms.Bufferlength = 1024;
  if (ARBparms.NumBuffers < 0) ARBparms.NumBuffers = 1;
}

// Saves the ARB parameters to FLASH. The run time state flags are cleared in the
// saved copy only, the outputs keep running.
void SaveSettings(void)
{
  ARB_PARMS ap = ARBparms;
  bool      ok;

  ClearRunTimeFlags(&ap);
  noInterrupts();
  ok = dueFlashStorage.writeAbs((uint32_t)NonVolStorage, (byte *)&ap, sizeof(ARB_PARMS));
  interrupts();
  if (ok)
  {
    serial->println("Flash updated!");
    SendACK;
    return;
  }
  serial->println("Flash updated failed!");
  SendNAK;
}

// Loads the ARB parameters from FLASH if the saved signature matches. Settings saved by
// firmware before rev 1.25 end after the Legacy flag and do not carry the layout marker.
// Only that part is restored in that case, the newer settings keep their defaults.
void RestoreSettings(void)
{
  ARB_PARMS ap;
  byte      *bp = (byte *)&ap;

  for (unsigned int i = 0; i < sizeof(ARB_PARMS); i++)
  {
    bp[i] = dueFlashStorage.readAbs(((uint32_t)NonVolStorage) + i);
  }
  if (ARBparms.signature != ap.signature)
  {
    serial->println("Restore from flash failed!");
    SendNAK;
    return;
  }
  if (ap.layout != ARB_LAYOUT)
  {
    ARB_PARMS old;                           // Defaults for the members not saved
    memcpy(&old, &ap, offsetof(ARB_PARMS, AlternateEnable));
    ap = old;
  }
  ARBparms = ap;
  ClearRunTimeFlags(&ARBparms);
  ARBparms.ApplyRevAuxV = false;
  ARBparms.RevAuxV = 0.0;
  ValidateParms();
  serial->println("Restored from flash!");
  SendACK;
}

// Converts a channel value in percent of full scale (-100 to 100) to DAC counts
// using that channel's calibration.
int ARBchannelValue2Counts(int ch, float value)
{
  int  i;

  i = ARBparms.DACoffsets[ch] + ARBparms.DACgains[ch] * value;
  if (i < 0) i = 0;
  if (i > 255) i = 255;
  return (i);
}

// ARB buffer channel range set function. Sets the channel to the value defined over the
// defined range.
// Sample indexes run from startI up to, but not including, stopI.
void SetARBchannelRange(int ch, int startI, int stopI, float fval)
{
  uint8_t  *bp = (uint8_t *)ARBbuffer;
  int i, val;

  if ((ch < 0) || (ch >= CHANS)) return;
  if (startI < 0) startI = 0;
  if (stopI > MAXBUFFER) stopI = MAXBUFFER;
  val = ARBchannelValue2Counts(ch, fval);
  for (i = startI; i < stopI; i++)
  {
    bp[(i * 8) + Ch2Index(ch)] = val;
  }
}

// This function will add one sine wave cycle to the selected channel, spread over the
// buffer length. iPhase is the starting phase in degrees.
void SetARBchannelSine(int ch, float iPhase)
{
  uint8_t *bp = (uint8_t *)ARBbuffer;
  float   fval;
  int     i;

  if ((ch < 0) || (ch >= CHANS)) return;
  for (i = 0; i < ARBparms.Bufferlength; i++)
  {
    fval = 100 * sin(((float)i / (float)ARBparms.Bufferlength) * 2.0 * PI + 2.0 * PI * iPhase / 360.0);
    if (fval > 100) fval = 100;
    if (fval < -100) fval = -100;
    bp[(i * 8) + Ch2Index(ch)] = ARBchannelValue2Counts(ch, fval);
  }
}

// ARB buffer set function. Sets all the channels to the value defined over the
// full buffer range.
void SetARBchannels(float fval)
{
  for (int i = 0; i < 8; i++)
  {
    SetARBchannelRange(i, 0, ARBparms.Bufferlength, fval);
  }
}

// Copies the active waveform (Waveform or WaveformR, by direction) into the TWAVE DAC
// buffer, applies the direction's phase shift, then rebuilds the alternate period.
void FillDACbuffer(void)
{
  int i, j, k, ps;
  uint8_t (*wf)[PPP] = ARBparms.Direction ? WaveformR : Waveform;

  // Phase shift in points, wrapped into 0 to ppp-1 so negative shifts work
  ps = (int)(((ARBparms.Direction ? ARBparms.fwdPS : ARBparms.revPS) * ARBparms.ppp) / 360) % ARBparms.ppp;
  if (ps < 0) ps += ARBparms.ppp;
  // Fill the first period
  for (k = 0; k < ARBparms.ppp; k++)
  {
    i = (k + ps) % ARBparms.ppp;
    for (j = 0; j < CHANS; j++) bufBytes[i * CHANS + Ch2Index(j)] = wf[j][k];
  }
  // Work around the CPLD logic bug by moving the first channel back one sample
  if (ARBparms.TimingCorrect) for (k = 0; k < ARBparms.ppp; k++)
  {
    i = (k + ps) % ARBparms.ppp;
    bufBytes[i * CHANS + Ch2Index(0)] = wf[0][(k + ARBparms.ppp - 1) % ARBparms.ppp];
  }
  // Fill the next period with the last value of the previous period for compression
  for (i = ARBparms.ppp; i < ARBparms.ppp * 2; i++)
  {
    for (j = 0; j < CHANS; j++) bufBytes[i * CHANS + Ch2Index(j)] = wf[j][ARBparms.ppp - 1];
  }
  // If direction is reverse and the apply reverse AUX voltage flag is set then apply the voltage
  if(ARBparms.ApplyRevAuxV)
  {
    if (ARBparms.Direction) WriteWFaux(ARBparms.RevAuxV);
    else WriteWFaux(ARBparms.VoltageAux);
  }
  SetAlternateWaveform(ARBparms.AltWaveform);
}

// Builds one period of the selected waveform type for every channel, in both the
// forward (Waveform) and reverse (WaveformR) direction, then fills the DAC buffer.
// Each channel is shifted by 1/8 of a period from the one before. The tables are only
// rebuilt when the type changes, pass force = true after changing anything they depend
// on (ppp, the ARB vector or calibration).
void SetWaveformType(int wft, bool force)
{
  float      f1,f2;
  int        i;
  int        ch;
  static int last_wft=-1;

  if(force) last_wft = -1;
  if(wft != last_wft) switch (wft)
  {
    case TWI_WAVEFORM_SIN:
      f2 = (2.0 * PI) / (float)ARBparms.ppp;
      for (ch = 0; ch < CHANS; ch++ )
      {
        f1 = (2.0 * PI * ch) / CHANS;
        for (i = 0; i < ARBparms.ppp; i++)  Waveform[ch][i]  = ARBchannelValue2Counts(ch, 100 * sin((float)i * f2 - f1));
        for (i = 0; i < ARBparms.ppp; i++)  WaveformR[ch][i] = ARBchannelValue2Counts(ch, 100 * sin((float)i * f2 + f1));
      }
      break;
    case TWI_WAVEFORM_RAMP:
      for (ch = 0; ch < CHANS; ch++ )
      {
        for (i = 0; i < ARBparms.ppp; i++)  Waveform[ch][(i + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100.0 + (200.0 * (float)i) / (float)(ARBparms.ppp - 1));
        for (i = 0; i < ARBparms.ppp; i++)  WaveformR[ch][(i + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100.0 + (200.0 * (float)i) / (float)(ARBparms.ppp - 1));
      }
      break;
    case TWI_WAVEFORM_TRI:
      for (ch = 0; ch < CHANS; ch++ )
      {
        for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100.0 + (200.0 * (float)i) / (float)(ARBparms.ppp / 2 - 1));
        for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + ARBparms.ppp / 2 + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, 100.0 - (200.0 * (float)i) / (float)(ARBparms.ppp / 2 - 1));
        for (i = 0; i < ARBparms.ppp / 2; i++) WaveformR[ch][(i + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100.0 + (200.0 * (float)i) / (float)(ARBparms.ppp / 2 - 1));
        for (i = 0; i < ARBparms.ppp / 2; i++) WaveformR[ch][(i + ARBparms.ppp / 2 + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, 100.0 - (200.0 * (float)i) / (float)(ARBparms.ppp / 2 - 1));
      }
      break;
    case TWI_WAVEFORM_PULSE:
      for (ch = 0; ch < CHANS; ch++ )
      {
        for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, 100);
        for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + ARBparms.ppp / 2 + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100);
        for (i = 0; i < ARBparms.ppp / 2; i++) WaveformR[ch][(i + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, 100.0);
        for (i = 0; i < ARBparms.ppp / 2; i++) WaveformR[ch][(i + ARBparms.ppp / 2 + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100);
      }
      break;
    case TWI_WAVEFORM_ARB:
      for (ch = 0; ch < CHANS; ch++ )
      {
        for (i = 0; i < ARBparms.ppp; i++)  Waveform[ch][(i + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, ARBwaveform[i]);
        for (i = 0; i < ARBparms.ppp; i++)  WaveformR[ch][(i + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, ARBwaveform[i]);
      }
      break;
    default:
      break;
  }
  last_wft = wft;
  FillDACbuffer();
}

// Sets the points per waveform period and rebuilds the waveform tables. The value must
// fit the PPP sized tables and leave at least two points for each half of the triangle
// and pulse waveforms. Returns false if the value is out of range.
bool SetPointsPerPeriod(int ppp)
{
  if ((ppp < 4) || (ppp > PPP)) return false;
  ARBparms.ppp = ppp;
  SetWaveformType(ARBparms.wft, true);
  return true;
}

// Turns compression on or off. Turning it on restarts the order ramp at the programmed order.
void SetCompression(bool enable)
{
  if (enable && !ARBparms.CompressEnable)
  {
    WorkingOrder = ARBparms.Order;
    CrampCounter = 0;
  }
  ARBparms.CompressEnable = enable;
}

// Sets one DAC channel directly, channel number 0 through 7, value in percent
void SetDACchannelValue(int ch, float fval)
{
  if ((ch < 0) || (ch > 7)) return;
  if ((fval > 100) || (fval < -100)) return;
  DACchanValues[ch] = fval;
  DACchanBits[Ch2Index(ch)] = ARBchannelValue2Counts(ch, fval);
  UpdateDACs();
}

// ISR used to output the DAC values for DAC clock rates under MINDMAFREQ, one sample per call.
// The index mask requires ppp to be a power of two.
void RealTimeTWAVEISR()
{
  uint32_t *Cbuffer = &buffer[ARBparms.ppp * NP * CHANS / 4];   // Alternate (compress) period

  if (!ARBparms.CompressEnable)
  {
    if (ptr == 0) Bcount++;
    DACadd[0] = buffer[ptr++];
    DACadd[1] = buffer[ptr++];
    ptr &= (ARBparms.ppp * NP * CHANS / 4) - 1;
    return;
  }
  if (Bcount >= ARBparms.Order)
  {
    DACadd[0] = buffer[ptr++];
    DACadd[1] = buffer[ptr++];
    ptr &= (ARBparms.ppp * NP * CHANS / 4) - 1;
    if (ptr == 0) Bcount = 1;
  }
  else
  {
    DACadd[0] = Cbuffer[ptr++];
    DACadd[1] = Cbuffer[ptr++];
    ptr &= (ARBparms.ppp * NP * CHANS / 4) - 1;
    if (ptr == 0) Bcount++;
  }
}

// ARB mode version of RealTimeTWAVEISR, stops after NumBuffers passes through the buffer
void RealTimeARBISR()
{
  DACadd[0] = ARBbuffer[ptr++];
  DACadd[1] = ARBbuffer[ptr++];
  if (ptr >= ARBparms.Bufferlength * 2)
  {
    ptr = 0;
    Bcount++;
    if (ARBparms.Mode == ARBmode) if ((Bcount == ARBparms.NumBuffers) && (ARBparms.NumBuffers != 0)) ARBclk.stop();
  }
}

// Works out the timer divider for a requested frequency and returns the frequency
// that divider really gives. In TWAVE mode freq is the waveform frequency and the DAC
// clock is freq * ppp, in ARB mode freq is the DAC sample rate.
//   actual = VARIANT_MCK / (2 * ppp * clkdiv), VARIANT_MCK = 84 MHz
static int ComputeClock(int freq, int *clkdiv)
{
  int ppp = (ARBparms.Mode == TWAVEmode) ? ARBparms.ppp : 1;

  if (freq < 1) freq = 1;            // Avoid a divide by zero
  *clkdiv = VARIANT_MCK / (2 * ppp * freq) + 1;
  return VARIANT_MCK / (2 * ppp * *clkdiv);
}

// DAC sample clock rate in Hz for a waveform frequency returned by ComputeClock
static int DACclockRate(int actualF)
{
  return (ARBparms.Mode == TWAVEmode) ? actualF * ARBparms.ppp : actualF;
}

// Sets the DAC output clock for the requested frequency without restarting the DMA
// transfer, and returns the actual frequency. Rates above MINDMAFREQ use the DMA clock,
// lower rates use the interrupt driven ARBclk.
int SetFrequency(int freq)
{
  int clkdiv, actualF, rate, CM;

  CM = digitalRead(ClockMode);
  digitalWrite(ClockMode,HIGH);

  DMAclk.stop();                     // Stop the current clock
  ARBclk.stop();                     // Stop the current clock
  actualF = ComputeClock(freq, &clkdiv);
  rate = DACclockRate(actualF);
  if (rate > MINDMAFREQ)
  {
    DMAclk.setFrequency(rate);
    DMAclk.setTIOAeffect(clkdiv - 2, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
    DMAclk.start(-1, 0, true);
  }
  else
  {
    ARBclk.setFrequency(rate);
    ARBclk.start(-1, 0, false);
  }
  digitalWrite(ClockMode,CM);
  return actualF;
}

// Starts waveform generation at ARBparms.RequestedFreq. Above MINDMAFREQ the DAC buffer is
// streamed by DMA, paced by the external wait states driven from the DMA clock; below it the
// ARBclk interrupt writes one sample per tick. NoTrigger sets up the DMA without starting it
// (ARB mode then waits for a sync trigger).
void SetEnable(bool NoTrigger)
{
  int clkdiv, actualF, rate;

  actualF = ComputeClock(ARBparms.RequestedFreq, &clkdiv);
  rate = DACclockRate(actualF);
  dmac_channel_disable(DMAC_MEMCH); // Stop any current DMA ops
  while (!dmac_channel_transfer_done(DMAC_MEMCH));
  DMAclk.stop();                     // Stop the current clock
  ARBclk.stop();                     // Stop the current clock
  if (rate > MINDMAFREQ)
  {
    DMAclk.setFrequency(rate);
    DMAclk.setTIOAeffect(clkdiv - 2, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
    DMAclk.start(-1, 0, true);
    Parallel.setMode((ReadModeFlags_t)0, (WriteModeFlags_t)0x20); // Enables the external wait states
    if (ARBparms.Mode == TWAVEmode) DMAbuffer2DAC((uint32_t *)0x60000000, buffer, ARBparms.ppp * NP * CHANS / 4, NoTrigger);
    if (ARBparms.Mode == ARBmode) DMAbuffer2DAC((uint32_t *)0x60000000, ARBbuffer, ARBparms.Bufferlength * 2, NoTrigger);
  }
  else
  {
    Bcount = 0;
    Parallel.setMode((ReadModeFlags_t)0, (WriteModeFlags_t)0x00); // Disables the external wait states
    if (ARBparms.Mode == TWAVEmode) ARBclk.attachInterrupt(RealTimeTWAVEISR);
    if (ARBparms.Mode == ARBmode) ARBclk.attachInterrupt(RealTimeARBISR);
    ARBclk.setFrequency(rate);
    pinMode(TIOA0, OUTPUT);
    digitalWrite(TIOA0, LOW);
    ARBclk.start(-1, 0, false);
  }
}

// Stops waveform generation, sets all outputs to 0 and arms the DMA (without starting
// it) so an ARB mode sync trigger can start it.
void SetDisable(void)
{
  Parallel.setMode((ReadModeFlags_t)0, (WriteModeFlags_t)0x00); // Disables the external wait states
  dmac_channel_disable(DMAC_MEMCH);  // Stop any current DMA ops
  while (!dmac_channel_transfer_done(DMAC_MEMCH));
  DMAclk.stop();                     // Stop the current clock
  ARBclk.stop();                     // Stop the current clock
  for (int i = 0; i < 8; i++) DACchanBits[Ch2Index(i)] = ARBchannelValue2Counts(i, DACchanValues[i] = 0.0);
  UpdateDACs();
  SetEnable(true);
  ReadyForTrigger = true;
}

//
// TWI (I2C) slave interface to the MIPS controller. Multi-byte values are little endian.
//

// Reads a 16 bit value from the TWI interface, return -1 if two bytes
// were not avalibale
int ReadUnsignedWord(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  if (Wire.available() == 0) return -1;
  i |= Wire.read() << 8;
  return i & 0xFFFF;
}

// Reads a 32 bit int, returns false if four bytes were not available
bool ReadInt(int *i)
{
  if (Wire.available() == 0) return false;
  *i = Wire.read();
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 8;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 16;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 24;
  return true;
}

// Reads a 8 bit value from the TWI interface, return -1 if a byte
// was not avalibale
int ReadUnsignedByte(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  return i & 0xFF;
}

// Reads a 8 bit signed value from the TWI interface, return false if a byte
// was not avalibale or true if ok
bool ReadByte(int8_t *b)
{
  if (Wire.available() == 0) return false;
  *b = Wire.read();
  return true;
}

// Reads a signed 16 bit value, returns false if two bytes were not available
bool Read16bitInt(int16_t *shortint)
{
  uint8_t *bp = (uint8_t *)shortint;

  if (Wire.available() == 0) return false;
  bp[0] = Wire.read();
  if (Wire.available() == 0) return false;
  bp[1] = Wire.read();
  return true;
}

// Reads a float value from the TWI interface, return false if float
// was not avalibale
bool ReadFloat(float *fval)
{
  uint8_t *bp = (uint8_t *)fval;

  for (int j = 0; j < 4; j++)
  {
    if (Wire.available() == 0) return false;
    bp[j] = Wire.read();
  }
  return true;
}

// Reads a byte flag, 1 = true and anything else = false. Returns false if no byte was available.
static bool ReadBool(bool *flag)
{
  int i = ReadUnsignedByte();

  if (i == -1) return false;
  *flag = (i == 1);
  return true;
}

// The Send functions queue a response in sb, the master collects it with a read (requestEvent)
void SendByte(byte bval)
{
  sb.write(bval);
}

void SendInt24(int ival)
{
  uint8_t *bp = (uint8_t *)&ival;

  sb.write(bp[0]);
  sb.write(bp[1]);
  sb.write(bp[2]);
}

void SendFloat(float fval)
{
  uint8_t *bp = (uint8_t *)&fval;

  sb.write(bp[0]);
  sb.write(bp[1]);
  sb.write(bp[2]);
  sb.write(bp[3]);
}

// This function is called when the master asks for data.
// Send up to 32 bytes from the sb structure
void requestEvent(void)
{
  int num = sb.available();
  for (int i = 0; i < num; i++)
  {
    if (i >= 30) break;
    Wire.write(sb.read());
  }
}

// Called from the TWI interrupt when the master writes. Each message is a command byte
// (TWI_* in ARB.h) followed by its arguments. While the serial port is redirected to TWI
// (TWI_SERIAL) all bytes go to the serial command processor until an ESC is received.
// Note this runs in interrupt context, see TODO.md.
void receiveEvent(int howMany)
{
  uint8_t cmd;
  int i, j, off, count, startI, stopI;
  int8_t b;
  int16_t shortint;
  float fval;
  unsigned long tm;
  Stream *prevSerial;

  while (Wire.available() != 0)
  {
    cmd = Wire.read();
    if (serial == &sb)
    {
      if (cmd == ESC) serial = &SerialUSB;
      else PutCh(cmd);
    }
    else switch (cmd)
      {
        case TWI_SERIAL:
          serial = &sb;
          break;
        case TWI_CMD:
          // Process command using the serial processor, the response is returned through sb
          sb.clear();
          prevSerial = serial;
          serial = &sb;
          tm = millis();
          // Read the ascii string and place in the serial processor ring buffer.
          while(true)
          {
            if(Wire.available() != 0)
            {
              cmd = Wire.read();
              PutCh(cmd);
              if(cmd == '\n') break;
            } 
            if((tm+250) < millis())
            {
              // Timeout, so put a \n in the command ring buffer and exit
              PutCh('\n');
              break;
            }
          }
          // Process any commands fill sb buffer with results
          while (RB_Commands(&RB) > 0) ProcessSerial();
          serial = prevSerial;
          break;
        case TWI_SET_MODE:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 0) ARBparms.Mode = TWAVEmode;
            if (i == 1) ARBparms.Mode = ARBmode;
          }
          break;
        case TWI_SET_ENABLE:
          if (ReadBool(&ARBparms.Enabled)) ForceUpdate = true;
          break;
        case TWI_SET_FREQ:
          i = ReadUnsignedWord();
          j = ReadUnsignedByte();
          if(FreqUpdate) break;              // Ignore while a change is still being applied
          if ((i != -1) && (j != -1))
          {
            ARBparms.RequestedFreq = i | (j << 16);
            FreqUpdate = true;
          }
          break;
        case TWI_SET_REF:
          i = ReadUnsignedWord();
          if (!ARBparms.Legacy) break;         // Only the first ARB rev has this DAC output
          if (i != -1) analogWrite(DACref, i);
          break;
        case TWI_SET_OFFSET:
          i = ReadUnsignedWord();
          if (i != -1) analogWrite(DACoffset, i);
          break;
        case TWI_SET_DIR:
          if (ReadBool(&ARBparms.Direction)) SetWaveformType(ARBparms.wft);
          break;
        case TWI_SET_WAVEFORM:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            ARBparms.wft = i;
            SetWaveformType(i);
          }
          break;
        case TWI_SET_VECTOR:
          // Get offset
          if ((off = ReadUnsignedByte()) == -1) break;
          // Get count
          if ((count = ReadUnsignedByte()) == -1) break;
          for (j = 0; j < count; j++)
          {
            if (!ReadByte(&b)) break;
            if ((j + off) < PPP) ARBwaveform[j + off] = b;
          }
          // If the mode is TWAVE and the waveform is ARB then reload the ARB buffers
          if ((ARBparms.Mode == TWAVEmode) && (ARBparms.wft == TWI_WAVEFORM_ARB))
          {
            SetWaveformType(ARBparms.wft,true);
          }
          break;
        case TWI_SET_RANGE:
          if (!ReadFloat(&fval)) break;
          VoltageRangeRequest = fval;
          RangeUpdate = true;
          break;
        case TWI_SET_RAMP:
          if (!ReadFloat(&fval)) break;
          ARBparms.RampRate = fval;
          break;
        case TWI_SET_OFFSETV:
          if (!ReadFloat(&fval)) break;
          ARBparms.VoltageOffset = fval;
          AuxDACupdate |= UpdateVoltageOffset;
          break;
        case TWI_SET_AUX:
          if (!ReadFloat(&fval)) break;
          ARBparms.VoltageAux = fval;
          AuxDACupdate |= UpdateVoltageAux;
          break;
        case TWI_SET_BUFFER_LEN:
          if ((i = ReadUnsignedWord()) == -1) break;
          if ((i >= 1) && (i <= MAXBUFFER)) ARBparms.Bufferlength = i;
          break;
        case TWI_SET_NUM_BUFFER:
          if ((i = ReadUnsignedWord()) == -1) break;
          ARBparms.NumBuffers = i;
          break;
        case TWI_SET_SET_BUFFER:
          if (!ReadFloat(&fval)) break;
          SetARBchannels(fval);
          break;
        case TWI_SET_SET_CHANNEL:
          if ((i = ReadUnsignedByte()) == -1) break;
          if (!ReadFloat(&fval)) break;
          SetARBchannelRange(i, 0, ARBparms.Bufferlength, fval);
          break;
        case TWI_SET_SET_CHN_RNG:
          if ((i = ReadUnsignedByte()) == -1) break;
          if ((startI = ReadUnsignedWord()) == -1) break;
          if ((stopI = ReadUnsignedWord()) == -1) break;
          if (!ReadFloat(&fval)) break;
          SetARBchannelRange(i, startI, stopI, fval);
          break;
        case TWI_SET_SINE:
          if ((i = ReadUnsignedByte()) == -1) break;
          if (!ReadFloat(&fval)) break;
          SetARBchannelSine(i, fval);
          break;
        case TWI_SET_DAC:
          if ((i = ReadUnsignedByte()) == -1) break;
          if (!ReadFloat(&fval)) break;
          SetDACchannelValue(i, fval);
          break;
        case TWI_SET_SYNC_ENA:
          ReadBool(&ARBparms.SyncEnable);
          break;
        case TWI_SET_COMP_ENA:
          if ((i = ReadUnsignedByte()) != -1) SetCompression(i == 1);
          break;
        case TWI_SET_COMP_EXT:
          ReadBool(&ARBparms.CompressHardware);
          break;
        case TWI_SET_COMP_ORDER:
          i = ReadUnsignedByte();
          if (i != -1) ARBparms.Order = i;
          break;
        case TWI_SET_COMP_ORDER_EX:
          i = ReadUnsignedWord();
          if (i != -1) ARBparms.Order = i;
          break;
        case TWI_SET_CRAMP:
          if (Read16bitInt(&shortint)) ARBparms.CompressRamp = shortint;
          break;
        case TWI_SET_CRAMPORDER:
          if (Read16bitInt(&shortint)) ARBparms.CrampOrder = shortint;
          break;
        case TWI_SET_EXT_CLOCK:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 1) digitalWrite(ClockMode, HIGH);
            else digitalWrite(ClockMode, LOW);
          }
          break;
        case TWI_SET_SEXTSRC:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 1) digitalWrite(ExtClockSel, HIGH);
            else digitalWrite(ExtClockSel, LOW);
          }
          break;
        case TWI_SET_BRD_BIAS:
          i = ReadUnsignedByte();
          if (!ReadFloat(&fval)) break;
          if ((i < 0) || (i > 1)) break;
          ARBparms.Bias[i] = fval;
          if(i==0) AuxDACupdate |= UpdateBias0;
          else AuxDACupdate |= UpdateBias1; 
          break;
        case TWI_SET_PWR:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 1) digitalWrite(PowerEnable, HIGH);
            else digitalWrite(PowerEnable, LOW);
          }
          break;
        case TWI_SET_TST_ENABLE:
          ReadBool(&VoltageTestEnable);
          break;
        case TWI_SET_FWDPS:
          if (!ReadFloat(&fval)) break;
          ARBparms.fwdPS = fval;
          FillDACbuffer();
          break;
        case TWI_SET_REVPS:
          if (!ReadFloat(&fval)) break;
          ARBparms.revPS = fval;
          FillDACbuffer();
          break;
        case TWI_UPDATE_AUX:
          if (!ReadFloat(&fval)) break;
          ARBparms.VoltageAux = fval;
          // Load DAC registers but do not output, wait for load updates command
          break;
        case TWI_UPDATE_BRD_BIAS:
          i = ReadUnsignedByte();
          if (!ReadFloat(&fval)) break;
          if ((i < 0) || (i > 1)) break;
          ARBparms.Bias[i] = fval;
          break;
        case TWI_LOAD_UPDATES:
          // Load AUX and board offsets
          WriteWFaux(ARBparms.VoltageAux);
          WriteBoardBias(0, ARBparms.Bias[0]);
          WriteBoardBias(1, ARBparms.Bias[1]);
          break;
        // Sweep commands
        case TWI_SWPSTARTFREQ:
          if (!ReadInt(&i)) break;
          fSweep.StartFreq = i;
          break;
        case TWI_SWPSTOPFREQ:
          if (!ReadInt(&i)) break;
          fSweep.StopFreq = i;
          break;
        case TWI_SWPSTARTV:
          if (!ReadFloat(&fval)) break;
          fSweep.StartVoltage = fval;
          break;
        case TWI_SWPSTOPV:
          if (!ReadFloat(&fval)) break;
          fSweep.StopVoltage = fval;
          break;
        case TWI_SWPTIME:
          if (!ReadFloat(&fval)) break;
          fSweep.SweepTime = fval;
          break;
        case TWI_SWPGO:
          if (!ReadByte(&b)) break;
          if ((b >= SS_IDLE) && (b <= SS_SWEEPING)) fSweep.State = (SweepState)b;
          break;
        case TWI_SET_PPP:
          if ((i = ReadUnsignedByte()) == -1) break;
          SetPointsPerPeriod(i);
          break;
        case TWI_SAVE:
          SaveSettings();
          break;
        case TWI_SET_AMODE:  // Set the alternate waveform mode
          ReadBool(&ARBparms.AlternateEnable);
          break;
        case TWI_SET_ATMODE:// Defines the alternate waveform trigger mode. 0 = level trigger, 1 = pos edge, 2 = neg edge
          if((i = ReadUnsignedByte()) != -1)
          {
            if((i>=0)&&(i<=2)) ARBparms.AltHwdMode = i;
          }
          break;
        case TWI_SET_AEXT:  //Set the external control via hardware of alternate waveform mode
          ReadBool(&ARBparms.AlternateHardware);
          break;
        case TWI_SET_AWFRM:   // Define alternate waveform, byte: 1=compress (default),2=reverse,3=arb,4=fixed
          i = ReadUnsignedByte();
          if((i>=1) && (i<=5))
          {
            // Set the alternate waveform in the buffer
            ARBparms.AltWaveform = i;
            SetAlternateWaveform(i);
          }
          break;
        case TWI_SET_ARNGENA: // Enable range change in compress or alternate waveform mode, byte: true if enabled
          ReadBool(&ARBparms.AlternateRngEna);
          break;
        case TWI_SET_ARNG:    // Compress or alternate range, float.
          if (!ReadFloat(&fval)) break;
          ARBparms.AlternateRng = fval;
          break;
        case TWI_SET_AFRQENA:  // Enable alternate waveform frequency
          ReadBool(&ARBparms.AlternateFreqEna);
          break;
        case TWI_SET_AFRQ:    // Set alternate waveform frequency
          i = ReadUnsignedWord();
          j = ReadUnsignedByte();
          if ((i != -1) && (j != -1)) ARBparms.AltFreq = i | (j << 16);
          break;
        case TWI_SET_FIXED:   // Set fixed voltage profile value, byte,float. byte=index, 0 thru 7. float = value in %
          i = ReadUnsignedByte();
          if (!ReadFloat(&fval)) break;
          if((i>=0)&&(i<=7)) 
          {
            ARBparms.Fixed[i] = fval;
            if(ARBparms.AltWaveform == 4) SetAlternateWaveform(ARBparms.AltWaveform);
          }
          break;
        case TWI_SET_TRGDELAY:  // Applies in pos or neg alternate waveform trigger mode. defines trigger delay, mS
          if (!ReadFloat(&fval)) break;
          ARBparms.TriggerDly = fval;
          break;
        case TWI_SET_PLYTIME:  // Applies in pos or neg alternate waveform trigger mode. defines alternate waveform active time, mS
          if (!ReadFloat(&fval)) break;
          ARBparms.PlayDuration = fval;
          break;
        case TWI_SET_REVAV: 
          if (!ReadFloat(&fval)) break;
          ARBparms.RevAuxV = fval;
          ARBparms.ApplyRevAuxV = true;
          break;
        case TWI_SET_CLRRAV:
          ARBparms.ApplyRevAuxV = false;
          break;
        case TWI_SET_HWDISR:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i != 0) ARBparms.ISRcompress = true;
            else ARBparms.ISRcompress = false;
          }
          break;
        case TWI_SET_SYNCLINE:
          i = ReadUnsignedByte();
          if(i == 1)
          {
             ARBline2pulseISR = NULL;
             ARBline1pulseISR = ARBsyncISR;
          }
          if(i == 2)
          {
             ARBline1pulseISR = NULL;
             ARBline2pulseISR = ARBsyncISR;         
          }
          break;
        case TWI_SET_COMPLINE:
          i = ReadUnsignedByte();
          if(i == 1)
          {
             ARBline2stateISR = NULL;
             if(ARBparms.ISRcompress) ARBline1stateISR = compressISR;
             CompressPin = ARBline1;
          }
          if(i == 2)
          {
              ARBline1stateISR = NULL;
              if(ARBparms.ISRcompress) ARBline2stateISR = compressISR;
              CompressPin = ARBline2;
          }
          break;
        //
        // The following commands result in a response being sent to the master
        //
        case TWI_READ_REQ_FREQ:
          SendInt24(ARBparms.RequestedFreq);
          break;
        case TWI_READ_ACT_FREQ:
          SendInt24(ARBparms.ActualFreq);
          break;
        case TWI_READ_STATUS:
          SendByte(Status);
          break;
        case TWI_READ_PPP:
          SendByte(ARBparms.ppp);
          break;
        case TWI_READ_VERSION:
          SendFloat(fVersion);
          break;
        case TWI_READ_SWEEP_STATUS:
          SendByte(fSweep.State);
          break;
        default:
          break;
      }
  }
}

// Sets the alternate waveform type used in the compress mode of the alternate waveform mode
// valid type are:
//                1 = compress
//                2 = reverse
//                3 = arb
//                4 = fixed
//                5 = Current waveform, no change
void SetAlternateWaveform(int type)
{
  int   i, j, windex;

   switch(type)
   {
      case 1:
         // Fill the next period with the last value of the previous period for compression
         for (i = ARBparms.ppp; i < ARBparms.ppp * 2; i++)
         {
            for (j = 0; j < CHANS; j++) bufBytes[i * CHANS + Ch2Index(j)] = bufBytes[(ARBparms.ppp-1) * CHANS + Ch2Index(j)];
         }
         break;
      case 2:
         // Fill the next period with the reverse dir of the previous period by mirroring the
         // channel order (0 stays, 1<->7, 2<->6, 3<->5, 4 stays).
         // This code violates the calibration and needs to be rewritten
         for (i = 0; i < ARBparms.ppp; i++)
         {
            for (j = 0; j < CHANS; j++)
              bufBytes[(i + ARBparms.ppp) * CHANS + Ch2Index((CHANS - j) % CHANS)] = bufBytes[i * CHANS + Ch2Index(j)];
         }
         break;
      case 3:
         // Move arb waveform into the second period buffer
         for (i = 0; i < ARBparms.ppp; i++)
         {
            for (j = 0; j < CHANS; j++) 
            {
              if(ARBparms.Direction) windex = i + j * (ARBparms.ppp / 8);
              else windex = i - j * (ARBparms.ppp / 8);
              if(windex >= ARBparms.ppp) windex = windex - ARBparms.ppp;
              if(windex < 0) windex = ARBparms.ppp + windex;
              bufBytes[(i + ARBparms.ppp) * CHANS + Ch2Index(j)] = ARBchannelValue2Counts(j, ARBwaveform[windex]);
            }
         }
         break;
      case 4:
         // Move fixed voltage profile into the second period buffer
         for (i = 0; i < ARBparms.ppp; i++)
         {
            for (j = 0; j < CHANS; j++) bufBytes[(i + ARBparms.ppp) * CHANS + Ch2Index(j)] = ARBchannelValue2Counts(j, ARBparms.Fixed[j]);
         }
         break;
      case 5:
         // Copy the first period, so the alternate waveform is the current waveform
         memcpy(&bufBytes[ARBparms.ppp * CHANS], bufBytes, ARBparms.ppp * CHANS);
         break;
      default:
         break;
   }
}

// Hardware interrupt line 1, pin A0. Default usage for this pin is sync.
// This interrupt fires on change.
// This ISR detectes two types of events, state change from high to low or low to high
// and a pulse event.
void ARBline1ISR(void)
{
  static int PinState=-1;
  static Pio *pio = g_APinDescription[ARBline1].pPort;
  static uint32_t pin =g_APinDescription[ARBline1].ulPin;

  // If this is a detection of a pulse event it will only be 1uS wide, so we delay 1uS
  // and if there is no pin state change then this must be a pulse event
  delayMicroseconds(1);
  (void)pio->PIO_ISR;         // Reading the status clears a pending ISR from the other edge in the case
                              // of 1uS pulse. Added 12/3/2020
  if(digitalRead(ARBline1) == PinState) 
  {
    if(pio->PIO_FRLHSR & pin) return; // Exit if falling edge
    // If here process the pulse
    if(ARBline1pulseISR != NULL) ARBline1pulseISR();
    PinState = digitalRead(ARBline1);
    return;
  }
  PinState = digitalRead(ARBline1);
  // If here then the state of ARBline1 has changed.
  if(ARBline1stateISR != NULL) ARBline1stateISR(PinState);
}

// Hardware interrupt line 2, pin 22. Default usage for this pin is compress line.
// This interrupt fires on change.
// This ISR detectes two types of events, state change from high to low or low to high
// and a pulse event.
void ARBline2ISR(void)
{
  static int PinState=-1;
  static Pio *pio = g_APinDescription[ARBline2].pPort;
  static uint32_t pin =g_APinDescription[ARBline2].ulPin;

  // If this is a detection of a pulse event it will only be 1uS wide, so we delay 1uS
  // and if there is no pin state change then this must be a pulse event
  delayMicroseconds(1);
  (void)pio->PIO_ISR;         // Reading the status clears a pending ISR from the other edge in the case
                              // of 1uS pulse. Added 12/3/2020
  if(digitalRead(ARBline2) == PinState) 
  {
    if(pio->PIO_FRLHSR & pin) return; // Exit if falling edge
    // If here process the pulse
    if(ARBline2pulseISR != NULL) ARBline2pulseISR();
    PinState = digitalRead(ARBline2);
    return;
  }
  PinState = digitalRead(ARBline2);
  // If here then the state of ARBline2 has changed.
  if(ARBline2stateISR != NULL) ARBline2stateISR(PinState);
}

// This interrupt fires when the resync trigger is received. In TWAVE mode it restarts the
// waveform at the start of the buffer, in ARB mode it starts playing the buffer.
void ARBsyncISR(void)
{
  if (!ARBparms.SyncEnable) return;
  if (ARBparms.Mode == TWAVEmode)
  {
    Bcount = 1;
    ptr = 0;  // This will resync the interrupt mode
    if ((ARBparms.ActualFreq * ARBparms.ppp) > MINDMAFREQ) DMArestart();
  }
  else
  {
    // Here if ARB mode so enable!
    if (ARBparms.ActualFreq > MINDMAFREQ)
    {
      if ((!ARBparms.Enabled) && (!ReadyForTrigger))
      {
        ARBparms.Enabled = true;
        enabled = true;
        SetEnable();
        return;
      }
      Bcount = 0;
      DMAclk.start(-1, 0, true);
      DMAbuffer2DAC((uint32_t *)0x60000000, ARBbuffer, ARBparms.Bufferlength * 2, false);
    }
    else
    {
      Bcount = 0;
      ptr = 0;
      ARBclk.start(-1, 0, false);
    }
  }
}

// Interrupt fires after trigger play time and disables alternate waveform
void ALTwaveformStopISR(void)
{
  ARBparms.AlternateEnable = false;  
}

// Interrupt fires after trigger delay, sets the alternate waveform
void ALTwaveformStartISR(void)
{
  if(ARBparms.PlayDuration == 0) return;
  ARBparms.AlternateEnable = true;
  ALTtmr.attachInterrupt(ALTwaveformStopISR);
  ALTtmr.setRC(ARBparms.PlayDuration * CLKSperMS);
  ALTtmr.softwareTrigger();
}

// This interrupt fires when a change is detected on the compressors hardware line.
// This line can be programmed for compress or alternate waveform.
void compressISR(int state)
{
  if (ARBparms.AlternateHardware)
  {
     if(ARBparms.AltHwdMode == 0)
     {
        if (digitalRead(CompressPin) == HIGH) ARBparms.AlternateEnable = true;
        else ARBparms.AlternateEnable = false;
     }
     else if(((ARBparms.AltHwdMode == 1)&&(digitalRead(CompressPin) == HIGH)) || ((ARBparms.AltHwdMode == 2)&&(digitalRead(CompressPin) == LOW)))  // pos or neg edge detection
     {
        if(ARBparms.TriggerDly > 0)
        {
           ALTtmr.attachInterrupt(ALTwaveformStartISR);
           ALTtmr.setRC(ARBparms.TriggerDly * CLKSperMS);
           ALTtmr.softwareTrigger();
        }
        else ALTwaveformStartISR();
     }
     return;
  }
  if (!ARBparms.CompressHardware) return;
  SetCompression(digitalRead(CompressPin) == HIGH);
}

void setup()
{
  // Diable the power supply
  pinMode(PowerEnable, OUTPUT);
  digitalWrite(PowerEnable, HIGH);
  // Set the external clock source the MIPS
  pinMode(ExtClockSel,OUTPUT);
  digitalWrite(ExtClockSel,LOW);
  // Setup the resync timer
  ResetTMR.stop();
  ResetTMR.begin();
  ResetTMR.attachInterrupt(DMAstartISR);
  ResetTMR.setTrigger(TC_CMR_EEVTEDG_NONE);
  ResetTMR.setClock(TC_CMR_TCCLKS_TIMER_CLOCK2);   // 10.5 MHz clock
  ResetTMR.stopOnRC();
  // Setup the serial port, used for debug operations
  SerialInit();
  // Restore from flash
  RestoreSettings();
  // Configure parallel bus to drive the memory mapped DAC
  pmc_enable_periph_clk(ID_DMAC);
  dmac_disable();
  DMAC->DMAC_GCFG = DMAC_GCFG_ARB_CFG_FIXED;
  dmac_enable();
  if (ARBparms.XPtiming)
  {
    // This timing option is for the CPLD logic upgrade for external clock operation. 8/21/18
    Parallel.begin(PARALLEL_BUS_WIDTH_8, PARALLEL_CS_0, 3, 1, 1);
    Parallel.setCycleTiming(4, 4);
    Parallel.setPulseTiming(3, 3, 3, 3);
    Parallel.setAddressSetupTiming(0, 0, 0, 0);
  }
  else if (!ARBparms.CPLD)
  {
    Parallel.begin(PARALLEL_BUS_WIDTH_8, PARALLEL_CS_0, 3, 1, 1);
    Parallel.setCycleTiming(6, 6);                  // ARB PLD based design
    Parallel.setPulseTiming(5, 5, 5, 5);
    Parallel.setAddressSetupTiming(0, 0, 0, 0);
  }
  else
  {
    Parallel.begin(PARALLEL_BUS_WIDTH_8, PARALLEL_CS_0, 3, 1, 1);
    Parallel.setCycleTiming(6, 6);                // ARB rev 3.0 (CPLD based) timing
    Parallel.setPulseTiming(5, 5, 5, 5);
    Parallel.setAddressSetupTiming(1, 1, 1, 1);
  }
  // Drives the LDAC signal low, active state
  pinMode(TIOA0, OUTPUT);
  pinMode(DAC1, INPUT);
  pinMode(DAC0, INPUT);
  digitalWrite(TIOA0, LOW);
  // Set mode low, this is the external clock mode control line
  pinMode(ClockMode, OUTPUT);
  digitalWrite(ClockMode, LOW);
  // Set the default waveform to a sine wave
  SetWaveformType(TWI_WAVEFORM_SIN);
  // The set frequency call will start the waveform generation
  ARBparms.ActualFreq = SetFrequency(ARBparms.RequestedFreq);
  // Setup reference voltage and offset voltage DACS
  analogWriteResolution(12);
  analogReadResolution(12);
  analogWrite(DACref, 2048);
  analogWrite(DACoffset, 2048);
  // Setup TWI as slave to communicate with MIPS
  Wire.begin(ARBparms.TWIarbAdd);      // join i2c bus
  Wire.onReceive(receiveEvent);        // register event
  Wire.onRequest(requestEvent);
  Wire1.begin();
  Wire1.setClock(400000);           // AD5625 DAC on Wire1
  sb.begin();
  // Default set to ARB disabled
  ARBparms.Enabled = false;
  DMAclk.stop();                     // Stop the current clock
  ARBclk.stop();                     // Stop the current clock
  for (int i = 0; i < 8; i++) DACchanBits[Ch2Index(i)] = ARBchannelValue2Counts(i, DACchanValues[i] = 0.0);
  UpdateDACs();
  // Setup the defaults on interrupt line 1, this is the ARBsync line
  pinMode(ARBsync, INPUT);
  attachInterrupt(ARBsync, ARBline1ISR, CHANGE);
  ARBline1pulseISR = ARBsyncISR;
  // Setup the defaults on interrupt line 2
  pinMode(CompressPin, INPUT);
  attachInterrupt(CompressPin, ARBline2ISR, CHANGE);
  // Apply the saved output levels on the first pass through loop()
  AuxDACupdate = UpdateAll;
  RangeUpdate  = true;
  VoltageRangeRequest = ARBparms.VoltageRange;
  // Use the external reference for the AD5625 and set the aux and offset reference level
  AD5625_EnableRef(DACadr);
  ARBparms.RefDAC = (ARBparms.AuxOffRef / ARBparms.DACrefVoltage) * 65535;
  AD5625(DACadr, DACrefCH, ARBparms.RefDAC, 3);
  // Set interrupt priorities
  NVIC_SetPriority(TWI0_IRQn, 8);
  NVIC_SetPriority(TWI1_IRQn, 8);
  NVIC_SetPriority(DMAC_IRQn, 1);
  // Set voltages to zero
  WriteWFrange(0);
  WriteWFoffset(0);
  WriteWFaux(0);
  WriteBoardBias(0, 0);
  WriteBoardBias(1, 0);
  if(ARBparms.ISRcompress) ARBline2stateISR = compressISR;
  // Setup the timer and ISRs for the alternate waveform system
  ALTtmr.begin();
  ALTtmr.setTrigger(TC_CMR_EEVTEDG_NONE);
  ALTtmr.setClock(TC_CMR_TCCLKS_TIMER_CLOCK4);    // 656,250 clock, 656.25 clocks per mS
  ALTtmr.stopOnRC();
  ALTtmr.enableTrigger();
  // Turn on power supplies for ARB_AMP
  digitalWrite(PowerEnable, HIGH);
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan)
{
  // Put serial received characters in the input ring buffer
  if (SerialUSB.available() > 0)
  {
    PutCh(SerialUSB.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
  if (strlen(VectorString) > 0) ProcessVectorString();
}

// Moves the voltage range towards VoltageRangeRequest, stepping at ARBparms.RampRate V/s,
// or in one step if the ramp rate is 0. Returns true while a range update is in progress.
bool ProcessRamp(void)
{
  static bool      ramping = false;
  static uint32_t  starttime;
  float            Vstep;
  
  if(!RangeUpdate) return false;
  if(ARBparms.RampRate == 0.0) 
  {
    ARBparms.VoltageRange = VoltageRangeRequest;
    WriteWFrange(ARBparms.VoltageRange);
    RangeUpdate = false;
  }
  else
  {
    if(!ramping)
    {
      ramping   = true;
      starttime = millis();
      return false;
    }
    else
    {
      Vstep = (float)(millis() - starttime) * ARBparms.RampRate / 1000.0;
      if(Vstep < abs(ARBparms.VoltageRange - VoltageRangeRequest))
      {
        if(ARBparms.VoltageRange < VoltageRangeRequest) WriteWFrange(ARBparms.VoltageRange + Vstep);
        else WriteWFrange(ARBparms.VoltageRange - Vstep);
      }
      else
      {
        ARBparms.VoltageRange = VoltageRangeRequest;
        WriteWFrange(ARBparms.VoltageRange);
        ramping = false;  
        RangeUpdate = false;   
      }
    }
  }
  return true;
}

// Runs the frequency / voltage sweep state machine, linear from the start to the stop
// values over SweepTime seconds. Called from loop(), does nothing until millis() changes.
void ProcessSweep(void)
{
  static uint32_t RightNow = 0;
  float           rnf, rnv;

  if (fSweep.State == SS_IDLE) return;
  if(RightNow == millis()) return;
  RightNow = millis();
  if (fSweep.State == SS_START)
  {
    fSweep.OrginalFreq = ARBparms.RequestedFreq;
    fSweep.OrginalVoltage = ARBparms.VoltageRange;
    ARBparms.RequestedFreq = fSweep.StartFreq;
    ARBparms.VoltageRange = fSweep.StartVoltage;
    fSweep.SweepStartTime = RightNow;
    fSweep.CurrentSweepTime = RightNow;
    fSweep.State = SS_SWEEPING;
    SweepUpdateRequest = true;
  }
  if (fSweep.State == SS_STOP)
  {
    ARBparms.RequestedFreq = fSweep.OrginalFreq;
    ARBparms.VoltageRange = fSweep.OrginalVoltage;
    fSweep.State = SS_IDLE;
    SweepUpdateRequest = true;
  }
  if (fSweep.State == SS_SWEEPING)
  {
    // Calculate the right now frequency.
    rnv = rnf = ((float)RightNow - (float)fSweep.SweepStartTime) / (fSweep.SweepTime * 1000);
    rnf = fSweep.StartFreq + (float)(fSweep.StopFreq - fSweep.StartFreq) * rnf;
    // Calculate the right now voltage
    rnv = fSweep.StartVoltage + (float)(fSweep.StopVoltage - fSweep.StartVoltage) * rnv;
    if (RightNow >= (fSweep.SweepStartTime + fSweep.SweepTime * 1000))
    {
      fSweep.State = SS_STOP;
    }
    ARBparms.RequestedFreq = rnf;
    ARBparms.VoltageRange = rnv;
    fSweep.CurrentSweepTime = RightNow;
    SweepUpdateRequest = true;
  }
}

// ARB main processing loop. Requests made by the TWI and serial commands (flags set
// in interrupt context) are applied here.
void loop()
{
  int          clkdiv;
  byte         b;

  // If ISR processing of the compress line is off, poll the line here
  if(!ARBparms.ISRcompress)
  {
     if (ARBparms.AlternateHardware) ARBparms.AlternateEnable = (digitalRead(CompressPin) == HIGH);
     if (ARBparms.CompressHardware) SetCompression(digitalRead(CompressPin) == HIGH);
  }
  // process any serial commands
  ProcessSerial();
  // Process any TWI commands
  if ((ARBparms.Enabled != enabled) || (ForceUpdate))
  {
    if (ARBparms.Enabled) SetEnable();
    else SetDisable();
    enabled = ARBparms.Enabled;
    ForceUpdate = false;
  }
  if (FreqUpdate)
  {
    ARBparms.ActualFreq = SetFrequency(ARBparms.RequestedFreq);
    FreqUpdate = false;
  }
  if (AuxDACupdate != 0)
  {
    noInterrupts();
    b = AuxDACupdate;
    interrupts();
    if((b & UpdateVoltageRange) != 0)  RangeUpdate = true;
    if((b & UpdateAltRange) != 0)      WriteWFrange(ARBparms.AlternateRng);
    if((b & UpdateVoltageOffset) != 0) WriteWFoffset(ARBparms.VoltageOffset);
    if((b & UpdateVoltageAux) != 0)    WriteWFaux(ARBparms.VoltageAux);
    if((b & UpdateBias0) != 0)         WriteBoardBias(0, ARBparms.Bias[0]);
    if((b & UpdateBias1) != 0)         WriteBoardBias(1, ARBparms.Bias[1]);
    noInterrupts();
    AuxDACupdate &= ~b;
    interrupts();
  }
  if(!ProcessRamp()) MeasureVoltages();
  ProcessSweep();
  if (SweepUpdateRequest)
  {
    SweepUpdateRequest = false;
    WriteWFrange(ARBparms.VoltageRange);
    if (fSweep.StartFreq != fSweep.StopFreq)
    {
      // Change the DMA clock divider in place so the waveform is not restarted
      ComputeClock(ARBparms.RequestedFreq, &clkdiv);
      DMAclk.setRC(clkdiv);
      DMAclk.setRA(clkdiv - 2);
      DMAclk.softwareTrigger();
    }
  }
}

//
// ARB Twave serial host commands
//
void SetMode(char *mode)
{
  String smode;

  smode = mode;
  if ((smode == String("TWAVE")) || (smode == String("ARB")))
  {
    if (smode == String("TWAVE")) ARBparms.Mode = TWAVEmode;
    else ARBparms.Mode = ARBmode;
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void GetMode(void)
{
  SendACKonly;
  if (ARBparms.Mode == TWAVEmode) serial->println("TWAVE");
  else serial->println("ARB");
}

void SetWFfreq(int freq)
{
  float MaxARBfreq = MAXARBRATE / ARBparms.ppp;
  if (((ARBparms.Mode == TWAVEmode) && (freq >= 0) && (freq <= MaxARBfreq)) || ((ARBparms.Mode == ARBmode) && (freq >= 0) && (freq <= 1500000)))
  {
    // If enabled then call set frequency, else save the frequency in data structure only
    ARBparms.RequestedFreq = freq;
    FreqUpdate = true;
//    ARBparms.ActualFreq = SetFrequency(freq);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void GetWFfreq(void)
{
  SendACKonly;
  serial->println(ARBparms.ActualFreq);
}

void SetWFref(int count)
{
  if ((count >= 0) && (count <= 4095))
  {
    analogWrite(DACref, count);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetWFoffset(int count)
{
  if ((count >= 0) && (count <= 4095))
  {
    analogWrite(DACoffset, count);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetWFdisable(void)
{
  ARBparms.Enabled = false;
  SendACK;
}

void SetWFenable(void)
{
  ARBparms.Enabled = true;
  SendACK;
}

// Sets the Twave waveform type, SIN, RAMP, TRIANGLE, PULSE or ARB
void SetWaveform(char *wavefrm)
{
  int wft;

  if (strcmp(wavefrm, "SIN") == 0) wft = TWI_WAVEFORM_SIN;
  else if (strcmp(wavefrm, "RAMP") == 0) wft = TWI_WAVEFORM_RAMP;
  else if (strcmp(wavefrm, "TRIANGLE") == 0) wft = TWI_WAVEFORM_TRI;
  else if (strcmp(wavefrm, "PULSE") == 0) wft = TWI_WAVEFORM_PULSE;
  else if (strcmp(wavefrm, "ARB") == 0) wft = TWI_WAVEFORM_ARB;
  else
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  ARBparms.wft = wft;
  SetWaveformType(wft);
  SendACK;
}

// Sets the Twave direction, TRUE or FALSE, and rebuilds the DAC buffer
void SetWFdir(char *dir)
{
  if ((strcmp(dir, "TRUE") != 0) && (strcmp(dir, "FALSE") != 0))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  ARBparms.Direction = (strcmp(dir, "TRUE") == 0);
  SetWaveformType(ARBparms.wft);
  SendACK;
}

// Set the forward / reverse direction phase shift in degrees and rebuild the DAC buffer
void SetFwdPS(char *val)
{
  ARBparms.fwdPS = String(val).toFloat();
  FillDACbuffer();
  SendACK;
}

void SetRevPS(char *val)
{
  ARBparms.revPS = String(val).toFloat();
  FillDACbuffer();
  SendACK;
}

// Sets the Twave points per period
void SetPPP(int ppp)
{
  if (SetPointsPerPeriod(ppp))
  {
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// Sets the ARB mode buffer length in samples, 1 to MAXBUFFER
void SetBufferLength(int len)
{
  if ((len >= 1) && (len <= MAXBUFFER))
  {
    ARBparms.Bufferlength = len;
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetDACchannelR(int chan, int val)
{
  if ((chan >= 0) && (chan <= 7) && (val >= 0) && (val <= 255))
  {
    DACchanBits[Ch2Index(chan)]  = val;
    UpdateDACs();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetBoardBias(char *sboard, char *sval)
{
  String sToken;
  int    board;
  float  val;

  sToken = sboard;
  board = sToken.toInt();
  board--;
  sToken = sval;
  val = sToken.toFloat();
  if (((board == 0) || (board == 1)) && (val >= -10) && (val <= 10))
  {
    WriteBoardBias(board, val);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetPowerEnable(char *state)
{
  String sToken;

  sToken = state;
  if ((sToken == "ON") || (sToken == "OFF"))
  {
    if (sToken == "ON") digitalWrite(PowerEnable, LOW);
    else  digitalWrite(PowerEnable, HIGH);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetDACchannelV(char *schan, char *sval)
{
  String sToken;
  int    chan;
  float  val;

  sToken = schan;
  chan = sToken.toInt() - 1;
  sToken = sval;
  val = sToken.toFloat();
  if ((chan >= 0) && (chan <= 7) && (val >= -100) && (val <= 100))
  {
    DACchanValues[chan] = val;
    DACchanBits[Ch2Index(chan)]  = ARBchannelValue2Counts(chan, val);
    UpdateDACs();
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// Loads the comma separated SWFVEC values (-100 to 100) into the ARB waveform and, if the
// ARB waveform is playing in TWAVE mode, rebuilds the waveform tables.
void ProcessVectorString(void)
{
  int  i;
  String cmd, Token;

  cmd = VectorString;
  for (i = 0; i < ARBparms.ppp; i++)
  {
    if ((Token = GetToken(cmd, i + 1)) == "") break;
    ARBwaveform[i] = constrain(Token.toInt(), -100, 100);
  }
  if ((ARBparms.Mode == TWAVEmode) && (ARBparms.wft == TWI_WAVEFORM_ARB)) SetWaveformType(ARBparms.wft, true);
  VectorString[0] = 0;
}

// Commands that are supported on rev 2

// This function sets the ARB DAC voltage range
void SetWFrange(char *srange)
{
  float  range;
  String spar;

  spar = srange;
  range = spar.toFloat();
  if ((range >= 0) && (range <= 100))
  {
    VoltageRangeRequest = range;
    RangeUpdate = true;
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetWFoffsetV(char *srange)
{
  float  range;
  String spar;

  spar = srange;
  range = spar.toFloat();
  if ((range >= -50) && (range <= 50))
  {
    ARBparms.VoltageOffset = range;
    WriteWFoffset(range);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void SetWFaux(char *srange)
{
  float  range;
  String spar;

  spar = srange;
  range = spar.toFloat();
  if ((range >= -50) && (range <= 50))
  {
    ARBparms.VoltageAux = range;
    WriteWFaux(range);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

//
// ARM serial commands
//
void SetARBchns(char *sval)
{
  String sToken;
  float  val;

  sToken = sval;
  val = sToken.toFloat();
  if ((val < -100) || (val > 100))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SetARBchannels(val);
  SendACK;
}

// SARBSINE command, channel 0 to 7 and the starting phase in degrees
void SetARBsine(char *Chan, char *iPhase)
{
  String sToken;
  int    ch;

  sToken = Chan;
  ch = sToken.toInt();
  if ((ch < 0) || (ch > 7))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  sToken = iPhase;
  SetARBchannelSine(ch, sToken.toFloat());
  SendACK;
}

void SetARBchannel(char * sch, char *sval)
{
  String sToken;
  int    ch;
  float  val;

  while (1)
  {
    sToken = sch;
    ch = sToken.toInt();
    if ((ch < 0) || (ch > 7)) break;
    sToken = sval;
    val = sToken.toFloat();
    if ((val < -100) || (val > 100)) break;
    SetARBchannelRange(ch, 0, ARBparms.Bufferlength, val);
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// This command will set an ARB channel to a defined value over a defined range.
// All parameters are pulled from the ring buffer.
//  Channel,Start index,Stop index,value
void SetARBchanRange(void)
{
  char   *Token;
  String sToken;
  int    ch, startI, stopI;
  float  val;

  while (1)
  {
    // Read all the arguments
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sToken = Token;
    ch = sToken.toInt();
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sToken = Token;
    startI = sToken.toInt();
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sToken = Token;
    stopI = sToken.toInt();
    GetToken(true);
    if ((Token = GetToken(true)) == NULL) break;
    sToken = Token;
    val = sToken.toFloat();
    if ((Token = GetToken(true)) == NULL) break;
    if (Token[0] != '\n') break;
    // Test the range
    if ((ch < 0) || (ch > 7)) break;
    if ((startI < 0) || (startI >= ARBparms.Bufferlength)) break;
    if ((stopI < 0) || (stopI > ARBparms.Bufferlength)) break;
    if (startI > stopI) break;
    if ((val < -100) || (val > 100)) break;
    // Now we can call the function!
    SetARBchannelRange(ch, startI, stopI, val);
    SendACK;
    return;
  }
  // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void ReportSupplyVoltages(void)
{
  serial->print("+12 volt supply = "); serial->println(PS12v[0]);
  serial->print("-12 volt supply = "); serial->println(PS12v[1]);
  serial->print("+50 volt supply = "); serial->println(PS50v[0]);
  serial->print("-50 volt supply = "); serial->println(PS50v[1]);
}

// DEBUG command, currently forces a DMA restart (same as a TWAVE sync)
void DebugFunction(void)
{
  DMArestart();
}

// Sweep commands
void StartSweep(void)
{
  fSweep.State = SS_START;
  SendACK;
  return;
}
void StopSweep(void)
{
  fSweep.State = SS_STOP;
  SendACK;
  return;
}
void GetStatus(void)
{
  SendACKonly;
  switch (fSweep.State)
  {
    case SS_IDLE:
      serial->println("IDLE");
      break;
    case SS_START:
      serial->println("STARTING");
      break;
    case SS_STOP:
      serial->println("STOPPING");
      break;
    case SS_SWEEPING:
      serial->println("SWEEPING");
      break;
    default:
      break;
  }
}

void SetCompressor(char *value)
{
  String sToken;

  sToken = value;
  if((sToken != "TRUE") && (sToken != "FALSE"))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
     return;
  }
  SendACK;
  SetCompression(sToken == "TRUE");
}

// True = external clock
void SetExternalClock(char *value)
{
  String sToken;

  sToken = value;
  if((sToken != "TRUE") && (sToken != "FALSE"))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
     return;
  }
  SendACK;
  if(sToken == "TRUE") digitalWrite(ClockMode, HIGH);
  else digitalWrite(ClockMode, LOW);
}

// MIPS or EXT
void SetExternalClockSource(char *value)
{
  String sToken;

  sToken = value;
  if((sToken != "MIPS") && (sToken != "EXT"))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
     return;
  }
  SendACK;
  if(sToken == "MIPS")   digitalWrite(ExtClockSel,LOW);
  else digitalWrite(ExtClockSel, HIGH);  
}

void SetAltTrigMode(int mode)
{
  if((mode < 0) || (mode > 2))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
     return;
  }
  SendACK;
  ARBparms.AltHwdMode = mode;
}

void GetAltTrigMode(void)
{
  SendACKonly;
  serial->println(ARBparms.AltHwdMode);
}

void SetFixedValue(char *index, char *val)
{
  String sToken;
  int    i;
  float  fval;

  sToken = index;
  i = sToken.toInt();
  if((i < 0) || (i > 7))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
     return;
  }
  sToken = val;
  fval = sToken.toFloat();
  if((fval < -100) || (fval > 100))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
     return;
  }
  SendACK;
  ARBparms.Fixed[i] = fval;
  if(ARBparms.AltWaveform == 4) SetAlternateWaveform(ARBparms.AltWaveform);
}

void GetFixedValue(int index)
{
  if((index < 0) || (index > 7))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
     return;
  }
  SendACKonly;
  serial->println(ARBparms.Fixed[index]);
}

void SetAltWaveFrm(int wfrm)
{
  if((wfrm < 1) || (wfrm > 5))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
     return;
  }
  SendACK;
  ARBparms.AltWaveform = wfrm;
  SetAlternateWaveform(wfrm);
}

void SetSyncLine(int line)
{
  // line = 1 or 2, if not exit
  if((line != 1) && (line !=2))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
     return;
  }
  SendACK;
  // Set the select interrupt line
  if(line == 1)
  {
    ARBline2pulseISR = NULL;
    ARBline1pulseISR = ARBsyncISR;
  }
  if(line == 2)
  {
    ARBline1pulseISR = NULL;
    ARBline2pulseISR = ARBsyncISR;
  }
}

void SetCompLine(int line)
{
  // line = 1 or 2, if not exit
  if((line != 1) && (line !=2))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
     return;
  }
  SendACK;
  // Set the select interrupt line
  if(line == 1)
  {
    ARBline2stateISR = NULL;
    if(ARBparms.ISRcompress) ARBline1stateISR = compressISR;
    CompressPin = ARBline1;
  }
  if(line == 2)
  {
    ARBline1stateISR = NULL;
    if(ARBparms.ISRcompress) ARBline2stateISR = compressISR;
    CompressPin = ARBline2;
  }
}
