//
// File: ARB
//
// This program enables the 8 channel ARB using the arduino due connected to two quad
// 8 bit dacs. The dacs are memory mapped for performance reasons.
// This module was desiged to support the PNNL Twave project. The max arb clock
// rate is ~1.2MHz. This will support maximum frequency per channel of ~40KHz when using
// 32 pointsGV per cycle. The waveforms are shifted 45 degrees from channel to channel.
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
#include <DueFlashStorage.h>
#include <efc.h>
#include <flash_efc.h>
#include "Arduino.h"
#include "Wire.h"
#include "arb.h"
#include "serial.h"
#include "Errors.h"
#include "hardware.h"
#include <MIPStimer.h>
#include <Parallel.h>
#include <SerialBuffer.h>

// Constants
#define PPP   32            // Number of points per waveform period
#define NP    1             // Number of periods in buffer, has to be 1 for compresion. The actual buffer
// is twice this or two periods. The second period holds at the last value
// of the first period and is used for compresion
#define CHANS 8             // Total number of DAC channels
#define MAXBUFFER 8000

#define TIOA0 2             // DUE pin number of the TIOA0 signal, used for LDAC control

#define DACref    DAC1      // DAC used to set the reference voltage
#define DACoffset DAC0      // DAC used to set the output amplifier offset

MIPStimer DMAclk(0);        // This timer is used to generate the clock for DMA
MIPStimer ARBclk(3);        // This timer is used to generate the clock in interrupt mode

char Version[] = "\nARB Version 1.17, Dec 7, 2018";
float fVersion = 1.17;

SerialBuffer sb;

// General buffers and waveform generation parameters
uint32_t ARBbuffer[MAXBUFFER * 2];
uint32_t buffer[PPP * NP * 2 * (CHANS / 4)]; // Data packed in 32 bit unsigned words
uint32_t *Cbuffer = &(buffer[64]);
uint8_t  *b = (uint8_t *)buffer;             // Byte pointer to dac buffer
int      ptr = 0;                            // dac output data pointer
uint32_t *DACadd = (uint32_t *)0x60000000;   // Memory mapped base address of DACs
uint8_t  *DACchan = (uint8_t *)0x60000000;   // Memory mapped base address of single DAC channel

uint8_t Waveform[CHANS][PPP];                // Active waveform, this is one cycle of the active waveform for each channel.
int8_t  ARBwaveform[PPP];                    // This waveform is downloaded via TWI and is the user defined arbitrary waveform
char    VectorString[200] = "";              // Contains a vector received from the serial port if length is greator than 0;
bool    AuxDACupdate = true;
bool    FreqUpdate = true;
bool    VoltageTestEnable = false;
byte    Status;
int     WorkingOrder = 1;                   // Current compresssion order
int     CrampCounter = 0;                   // Order ramping counter

bool    RangeUpdate = false;
float   VoltageRangeRequest;

int     DACaddBias[2] = {DAC0, DAC1};

float   PS12v[2] = {0, 0};
float   PS50v[2] = {0, 0};

static  bool enabled = false;
static  bool ReadyForTrigger = false;
static  bool ForceUpdate = false;

FreqSweep fSweep = {10000, 10000, 20000, 20, 30, 30, 10, 0, 0, SS_IDLE};
bool SweepUpdateRequest = false;

ARB_PARMS ARBparms;

DueFlashStorage dueFlashStorage;

// Storage used to save arb settings
//const PROGMEM byte NonVolStorage[1000] = {};
#define NonVolStorage (0x100000 - 1000)

// Save the ARB data structure to flash
void SaveSettings(void)
{
  ARBparms.Enabled = false;
  ARBparms.SyncEnable = false;
  ARBparms.CompressEnable = false;
  ARBparms.CompressHardware = false;

  noInterrupts();
  if (dueFlashStorage.writeAbs((uint32_t)NonVolStorage, (byte *)&ARBparms, sizeof(ARB_PARMS)))
  {
    interrupts();
    serial->println("Flash updated!");
    SendACK;
    return;
  }
  else
  {
    interrupts();
    serial->println("Flash updated failed!");
    SendACK;
    return;
  }
}

void RestoreSettings(void)
{
  ARB_PARMS ap;
  byte *b;
  int i;

  b = (byte *)&ap;
  for (i = 0; i < sizeof(ARB_PARMS); i++)
  {
    b[i] = dueFlashStorage.readAbs(((uint32_t)NonVolStorage) + i);
  }
  // Check signature, if correct then update
  if (ARBparms.signature == ap.signature)
  {
    ARBparms = ap;
    serial->println("Restored from flash!");
    SendACK;
    return;
  }
  else
  {
    serial->println("Restore from flash failed!");
    SendACK;
    return;
  }
}

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
void SetARBchannelRange(int ch, int startI, int stopI, float fval)
{
  uint8_t  *b = (uint8_t *)ARBbuffer;
  int i, val;

  val = ARBchannelValue2Counts(ch, fval);
  for (i = startI; i < stopI; i++)
  {
    b[(i * 8) + ch] = val;
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

// This function uses the waveform buffer to fill the DAC buffer and determine the phase shifts
void FillDACbuffer(void)
{
  int   i, j;

  // Fill the first period
  for (i = 0; i < ARBparms.ppp; i++)
  {
    for (j = 0; j < CHANS; j++)
    {
      b[i * CHANS + j] = Waveform[j][i];
    }
  }
  // Fill the next period with the last value of the previous period for compression
  for (i = ARBparms.ppp; i < ARBparms.ppp * 2; i++)
  {
    for (j = 0; j < CHANS; j++)
    {
      b[i * CHANS + j] = Waveform[j][ARBparms.ppp - 1];
    }
  }
}

// This function fills the buffer with the selected waveform type
void SetWaveformType(int wft)
{
  int i;
  int ch;

  switch (wft)
  {
    case TWI_WAVEFORM_SIN:
      for (ch = 0; ch < CHANS; ch++ )
      {
        if (ARBparms.Direction)
          for (i = 0; i < ARBparms.ppp; i++)  Waveform[ch][i] = ARBchannelValue2Counts(ch, 100 * sin(((float)i / (float)ARBparms.ppp) * 2.0 * PI + 2.0 * PI * ch / CHANS));
        else
          for (i = 0; i < ARBparms.ppp; i++)  Waveform[ch][i] = ARBchannelValue2Counts(ch, 100 * sin(((float)i / (float)ARBparms.ppp) * 2.0 * PI - 2.0 * PI * ch / CHANS));
      }
      break;
    case TWI_WAVEFORM_RAMP:
      for (ch = 0; ch < CHANS; ch++ )
      {
        if (!ARBparms.Direction)
          for (i = 0; i < ARBparms.ppp; i++)  Waveform[ch][(i + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100.0 + (200.0 * (float)i) / (float)(ARBparms.ppp - 1));
        else
          for (i = 0; i < ARBparms.ppp; i++)  Waveform[ch][(i + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100.0 + (200.0 * (float)i) / (float)(ARBparms.ppp - 1));
      }
      break;
    case TWI_WAVEFORM_TRI:
      for (ch = 0; ch < CHANS; ch++ )
      {
        if (!ARBparms.Direction)
        {
          for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100.0 + (200.0 * (float)i) / (float)(ARBparms.ppp / 2 - 1));
          for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + ARBparms.ppp / 2 + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, 100.0 - (200.0 * (float)i) / (float)(ARBparms.ppp / 2 - 1));
        }
        else
        {
          for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100.0 + (200.0 * (float)i) / (float)(ARBparms.ppp / 2 - 1));
          for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + ARBparms.ppp / 2 + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, 100.0 - (200.0 * (float)i) / (float)(ARBparms.ppp / 2 - 1));
        }
      }
      break;
    case TWI_WAVEFORM_PULSE:
      for (ch = 0; ch < CHANS; ch++ )
      {
        if (!ARBparms.Direction)
        {
          for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, 100);
          for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + ARBparms.ppp / 2 + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100);
        }
        else
        {
          for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, 100.0);
          for (i = 0; i < ARBparms.ppp / 2; i++) Waveform[ch][(i + ARBparms.ppp / 2 + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, -100);
        }
      }
      break;
    case TWI_WAVEFORM_ARB:
      for (ch = 0; ch < CHANS; ch++ )
      {
        if (!ARBparms.Direction)
          for (i = 0; i < ARBparms.ppp; i++)  Waveform[ch][(i + ch * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, ARBwaveform[i]);
        else
          for (i = 0; i < ARBparms.ppp; i++)  Waveform[ch][(i + (CHANS - 1 - ch) * ARBparms.ppp / (CHANS)) % ARBparms.ppp] = ARBchannelValue2Counts(ch, ARBwaveform[i]);
      }
      break;
    default:
      break;
  }
  FillDACbuffer();
}

// Channel number 0 through 7
void SetDACchannelValue(int ch, float fval)
{
  if ((ch < 0) || (ch > 7)) return;
  if ((fval > 100) || (fval < -100)) return;
  int j = ARBchannelValue2Counts(ch, fval);
  DACchan[ch] = j;
}

// ISR used to output the DAC values for frequncies under MINDMAFREQ
void RealTimeTWAVEISR()
{
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

// This function sets the DAC output clock needed to generate the requested
// frequency for the waveform period. This is the frequency of the waveform
// on each of the 8 output channels.
// VARIANT_MCK is the timer clock source, currently 42,000,000 Hz
// This function returns the actually frequency set.
// Frequency = VARIANT_MCK/(div * PPP)
//  Where:
//        VARIANT_MCK = 42,000,000 Hz
//        PPP = 32, points per period
//        div = clock divider, 26 to 1312
int SetFrequency(int freq)
{
  int clkdiv;
  int actualF;

  DMAclk.stop();                     // Stop the current clock
  ARBclk.stop();                     // Stop the current clock
  if (ARBparms.Mode == TWAVEmode)
  {
    clkdiv = VARIANT_MCK / (2 * ARBparms.ppp * freq) + 1;
    actualF = VARIANT_MCK / (2 * ARBparms.ppp * clkdiv);
  }
  if (ARBparms.Mode == ARBmode)
  {
    clkdiv = VARIANT_MCK / (2 * freq) + 1;
    actualF = VARIANT_MCK / (2 * clkdiv);
  }
  if ((((actualF * ARBparms.ppp) > MINDMAFREQ) && (ARBparms.Mode == TWAVEmode)) || (((actualF) > MINDMAFREQ) && (ARBparms.Mode == ARBmode)))
  {
    // Here if above MINDMAFREQ so use DMA
    if (ARBparms.Mode == TWAVEmode) DMAclk.setFrequency(actualF * ARBparms.ppp);
    if (ARBparms.Mode == ARBmode) DMAclk.setFrequency(actualF);
    DMAclk.setTIOAeffect(clkdiv - 2, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET); 
    DMAclk.start(-1, 0, true);
  }
  else
  {
    if (ARBparms.Mode == TWAVEmode)
    {
      ARBclk.setFrequency(actualF * ARBparms.ppp);
    }
    if (ARBparms.Mode == ARBmode)
    {
      ARBclk.setFrequency(actualF);
    }
    ARBclk.start(-1, 0, false);
  }
  return actualF;
}

void SetEnable(bool NoTrigger = false)
{
  int clkdiv;
  int actualF;

  if (ARBparms.Mode == TWAVEmode)
  {
    clkdiv = VARIANT_MCK / (2 * ARBparms.ppp * ARBparms.RequestedFreq) + 1;
    actualF = VARIANT_MCK / (2 * ARBparms.ppp * clkdiv);
  }
  if (ARBparms.Mode == ARBmode)
  {
    clkdiv = VARIANT_MCK / (2 * ARBparms.RequestedFreq) + 1;
    actualF = VARIANT_MCK / (2 * clkdiv);
  }
  // If the clock is below MINDMAFREQ then the waveform is generated using an ISR to
  // output each point. Above MINDMAFREQ is done with DMA and the clock used wait states
  // in hardware to control the rate.
  dmac_channel_disable(DMAC_MEMCH); // Stop any current DMA ops
  while (!dmac_channel_transfer_done(DMAC_MEMCH));
  DMAclk.stop();                     // Stop the current clock
  ARBclk.stop();                     // Stop the current clock
  if ((((actualF * ARBparms.ppp) > MINDMAFREQ) && (ARBparms.Mode == TWAVEmode)) || (((actualF) > MINDMAFREQ) && (ARBparms.Mode == ARBmode)))
  {
    // Here if above MINDMAFREQ so use DMA
    if (ARBparms.Mode == TWAVEmode) DMAclk.setFrequency(actualF * ARBparms.ppp);
    if (ARBparms.Mode == ARBmode) DMAclk.setFrequency(actualF);
    DMAclk.setTIOAeffect(clkdiv - 2, TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
    DMAclk.start(-1, 0, true);
    //ARBclk.attachInterrupt(RealTimeISR);
    Parallel.setMode((ReadModeFlags_t)0, (WriteModeFlags_t)0x20); // Enables the external wait states
    if (ARBparms.Mode == TWAVEmode) DMAbuffer2DAC((uint32_t *)0x60000000, buffer, ARBparms.ppp * NP * CHANS / 4, NoTrigger);
    if (ARBparms.Mode == ARBmode) DMAbuffer2DAC((uint32_t *)0x60000000, ARBbuffer, ARBparms.Bufferlength * 2, NoTrigger);
  }
  else
  {
    Bcount = 0;
    Parallel.setMode((ReadModeFlags_t)0, (WriteModeFlags_t)0x00); // Disables the external wait states
    // Here is below 400KHz so use the ISR
    if (ARBparms.Mode == TWAVEmode)
    {
      ARBclk.attachInterrupt(RealTimeTWAVEISR);
      ARBclk.setFrequency(actualF * ARBparms.ppp);
    }
    if (ARBparms.Mode == ARBmode)
    {
      ARBclk.attachInterrupt(RealTimeARBISR);
      ARBclk.setFrequency(actualF);
    }
    pinMode(TIOA0, OUTPUT);
    digitalWrite(TIOA0, LOW);
    ARBclk.start(-1, 0, false);
  }
}

void SetDisable(void)
{
  Parallel.setMode((ReadModeFlags_t)0, (WriteModeFlags_t)0x00); // Disables the external wait states
  dmac_channel_disable(DMAC_MEMCH);  // Stop any current DMA ops
  while (!dmac_channel_transfer_done(DMAC_MEMCH));
  DMAclk.stop();                     // Stop the current clock
  ARBclk.stop();                     // Stop the current clock
  for (int i = 0; i < 8; i++) DACchan[i] = ARBchannelValue2Counts(i, 0.0);
  SetEnable(true);
  ReadyForTrigger = true;
}

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

bool Read16bitInt(int16_t *shortint)
{
  uint8_t *b = (uint8_t *)shortint;

  if (Wire.available() == 0) return false;
  b[0] = Wire.read();
  if (Wire.available() == 0) return false;
  b[1] = Wire.read();
  return true;
}

// Reads a float value from the TWI interface, return false if float
// was not avalibale
bool ReadFloat(float *fval)
{
  int i;
  uint8_t *b;

  b = (uint8_t *)fval;
  for (int j = 0; j < 4; j++)
  {
    if (Wire.available() == 0) return false;
    b[j] = Wire.read();
  }
  return true;
}

void SendByte(byte bval)
{
  sb.write(bval);
}

void SendInt24(int ival)
{
  uint8_t *b;

  b = (uint8_t *)&ival;
  // Send the 24 bit word to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
}

void SendFloat(float fval)
{
  uint8_t *b;

  b = (uint8_t *)&fval;
  // Send the float to the ARB module
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
  sb.write(b[3]);
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

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  uint8_t cmd;
  int i, j, off, count, startI, stopI;
  int8_t b;
  int16_t shortint;
  float fval;

  while (Wire.available() != 0)
  {
    cmd = Wire.read();
    //SerialUSB.println(cmd);
    //PutCh(cmd);
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
        case TWI_SET_MODE:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 0) ARBparms.Mode = TWAVEmode;
            if (i == 1) ARBparms.Mode = ARBmode;
          }
          break;
        case TWI_SET_ENABLE:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 1) ARBparms.Enabled = true;
            else ARBparms.Enabled = false;
            ForceUpdate = true;
          }
          break;
        case TWI_SET_FREQ:
          i = ReadUnsignedWord();
          j = ReadUnsignedByte();
          if ((i != -1) && (j != -1))
          {
            ARBparms.RequestedFreq = i | (j << 16);
            FreqUpdate = true;
          }
          break;
        case TWI_SET_REF:
          i = ReadUnsignedWord();
          if (i != -1) analogWrite(DACref, i);
          break;
        case TWI_SET_OFFSET:
          i = ReadUnsignedWord();
          if (i != -1) analogWrite(DACoffset, i);
          break;
        case TWI_SET_DIR:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 1) ARBparms.Direction = true;
            else ARBparms.Direction = false;
            SetWaveformType(ARBparms.wft);
          }
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
            if (ReadByte(&b)) ARBwaveform[j + off] = b;
            else break;
          }
          // If the mode is TWAVE and the waveform is ARB then reload the ARB buffers
          if ((ARBparms.Mode == TWAVEmode) && (ARBparms.wft == TWI_WAVEFORM_ARB))
          {
            SetWaveformType(ARBparms.wft);
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
          AuxDACupdate = true;
          break;
        case TWI_SET_AUX:
          if (!ReadFloat(&fval)) break;
          ARBparms.VoltageAux = fval;
          AuxDACupdate = true;
          break;
        case TWI_SET_BUFFER_LEN:
          if ((i = ReadUnsignedWord()) == -1) break;
          ARBparms.Bufferlength = i;
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
        case TWI_SET_DAC:
          if ((i = ReadUnsignedByte()) == -1) break;
          if (!ReadFloat(&fval)) break;
          SetDACchannelValue(i, fval);
          break;
        case TWI_SET_SYNC_ENA:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 1) ARBparms.SyncEnable = true;
            else ARBparms.SyncEnable = false;
          }
          break;
        case TWI_SET_COMP_ENA:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 1)
            {
              if (ARBparms.CompressEnable == false)
              {
                WorkingOrder = ARBparms.Order;
                CrampCounter = 0;
              }
              ARBparms.CompressEnable = true;
            }
            else ARBparms.CompressEnable = false;
          }
          break;
        case TWI_SET_COMP_EXT:
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 1) ARBparms.CompressHardware = true;
            else ARBparms.CompressHardware = false;
          }
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
          AuxDACupdate = true;
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
          i = ReadUnsignedByte();
          if (i != -1)
          {
            if (i == 1) VoltageTestEnable = true;
            else VoltageTestEnable = false;
          }
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
          fSweep.StartFreq = i;
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
          fSweep.State = (SweepState)b;
          break;
        case TWI_SET_PPP:
          if (!ReadByte(&b)) break;
          ARBparms.ppp = b;
          break;
        case TWI_SAVE:
          SaveSettings();
          break;
        // The following commands result in a response being sent to the master
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

// This interrupt fires when the resync trigger is received
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

void compressISR(void)
{
  if (!ARBparms.CompressHardware) return;
  if (digitalRead(CompressPin) == HIGH)
  {
      WorkingOrder = ARBparms.Order;
      CrampCounter = 0;  
      ARBparms.CompressEnable = true;
      return;
  }
  ARBparms.CompressEnable = false;  
}

void setup()
{
  int   i;

  // Diable the power supply
  pinMode(PowerEnable, INPUT);
  digitalWrite(PowerEnable, HIGH);
  // Set the external clock source the MIPS
  pinMode(ExtClockSel,OUTPUT);
  digitalWrite(ExtClockSel,LOW);
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
    Parallel.begin(PARALLEL_BUS_WIDTH_8, PARALLEL_CS_0, 3, 1, 1);
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
  // Set default frequency to 10KHz
  //  TWIfreqRequest = 10000;
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
  Wire1.setClock(400000);
  sb.begin();
  // Default set to ARB disabled
  ARBparms.Enabled = false;
  DMAclk.stop();                     // Stop the current clock
  ARBclk.stop();                     // Stop the current clock
  for (int i = 0; i < 8; i++) DACchan[i] = ARBchannelValue2Counts(i, 0.0);
  pinMode(ARBsync, INPUT);
  attachInterrupt(ARBsync, ARBsyncISR, RISING) ;
  AuxDACupdate = true;
  RangeUpdate  = true;
  VoltageRangeRequest = ARBparms.VoltageRange;
  AD5625_EnableRef(DACadr);
  ARBparms.RefDAC = (ARBparms.AuxOffRef / ARBparms.DACrefVoltage) * 65535;
  AD5625(DACadr, DACrefCH, ARBparms.RefDAC , 3);
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
  if(ARBparms.ISRcompress) attachInterrupt(CompressPin, compressISR, CHANGE);
  // Turn on power supplies for ARB_AMP
  digitalWrite(PowerEnable, HIGH);
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan = true)
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

void ProcessRamp(void)
{
  static bool      ramping = false;
  static uint32_t  starttime;
  float            Vstep;
  
  if(!RangeUpdate) return;
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
}

void ProcessSweep(void)
{
  uint32_t      RightNow;
  float         rnf, rnv;

  if (fSweep.State == SS_IDLE) return;
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

// ARB main processing loop
void loop()
{
  int          j;
  int          clkdiv;
  int          actualF;

  // If the compress hardware flag is true then look at the
  // compress hardware line and control the enable. 
  if(!ARBparms.ISRcompress)
  {
     if (ARBparms.CompressHardware)
     {
       if (digitalRead(CompressPin) == HIGH)
       {
         if (ARBparms.CompressEnable == false)
         {
           WorkingOrder = ARBparms.Order;
           CrampCounter = 0;
         }
         ARBparms.CompressEnable = true;
       }
       else ARBparms.CompressEnable = false;
     }
     else ARBparms.CompressEnable = false;
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
    FreqUpdate = false;
    ARBparms.ActualFreq = SetFrequency(ARBparms.RequestedFreq);
  }
  if (AuxDACupdate)
  {
    AuxDACupdate = false;
    WriteWFoffset(ARBparms.VoltageOffset);
    WriteWFaux(ARBparms.VoltageAux);
    WriteBoardBias(0, ARBparms.Bias[0]);
    WriteBoardBias(1, ARBparms.Bias[1]);
  }
  ProcessRamp();
  ProcessSweep();
  if (SweepUpdateRequest)
  {
    SweepUpdateRequest = false;
    WriteWFrange(ARBparms.VoltageRange);
    if (fSweep.StartFreq != fSweep.StopFreq)
    {
      if (ARBparms.Mode == TWAVEmode)
      {
        clkdiv = VARIANT_MCK / (2 * ARBparms.ppp * ARBparms.RequestedFreq) + 1;
        actualF = VARIANT_MCK / (2 * ARBparms.ppp * clkdiv);
      }
      else
      {
        clkdiv = VARIANT_MCK / (2 * ARBparms.RequestedFreq) + 1;
        actualF = VARIANT_MCK / (2 * clkdiv);
      }
      DMAclk.setRC(clkdiv);
      DMAclk.setRA(clkdiv - 2);
      DMAclk.softwareTrigger();
    }
  }
  MeasureVoltages();
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

void SetWaveform(char *wavefrm)
{
  String wf;

  wf = wavefrm;
  if (wf == String("SIN")) SetWaveformType(TWI_WAVEFORM_SIN);
  else if (wf == String("RAMP")) SetWaveformType(TWI_WAVEFORM_RAMP);
  else if (wf == String("TRIANGLE")) SetWaveformType(TWI_WAVEFORM_TRI);
  else if (wf == String("PULSE")) SetWaveformType(TWI_WAVEFORM_PULSE);
  else if (wf == String("ARB")) SetWaveformType(TWI_WAVEFORM_ARB);
  else
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  FillDACbuffer();
  SendACK;
}

void SetDACchannelR(int chan, int val)
{
  if ((chan >= 0) && (chan <= 7) & (val >= 0) && (val <= 255))
  {
    DACchan[chan] = val;
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
    DACchan[chan]  = ARBparms.DACgains[chan] * val + ARBparms.DACoffsets[chan];
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

void ProcessVectorString(void)
{
  int  i;
  String cmd, Token;

  cmd = VectorString;
  //  for (i = 0; i < PPP; i++) Waveform[i] = 0;
  for (i = 0; i < ARBparms.ppp; i++)
  {
    if ((Token = GetToken(cmd, i + 1)) == "") break;
    ARBwaveform[i] = Token.toInt();
  }
  FillDACbuffer();
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
    if ((stopI < 0) || (stopI >= ARBparms.Bufferlength)) break;
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

void DebugFunction(void)
{
  int i;

  for (i = 0; i < 32; i++)
  {
    serial->println(ARBwaveform[i]);
  }
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

// True = external clock
void SetExternalClock(char *value)
{
  String sToken;

  sToken = value;
  if((sToken != "TRUE") && (sToken != "FALSE"))
  {
     SetErrorCode(ERR_BADARG);
     SendNAK;    
  }
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
  }
  if(sToken == "MIPS")   digitalWrite(ExtClockSel,LOW);
  else digitalWrite(ExtClockSel, HIGH);  
}
