//
// File: Calibration
//
// This file contains functions to enable calibration of the ARB outputs. These functions are
// called by the serial host interface system. Calibration is performed using a USB connection
// to the ARB module Due. The offset jumper should be moved to the non offset mode position (JP1 pins 3 to 4) for
// this procedure or the PC/MAC will need to float. The following steps outline the calibration process:
//  1.) Calibrate the offset, monitor the voltage at JP1 pin 1
//  2.) Calibrate the aux output
//  3.) Calibrate the voltage range control, monitoring the first ARB output channel
//  4.) Calibrate the ARB output channels 1 through 8
//  5.) Calibrate the A output bias, optional for rev 3 only with dual outputs, A and B
//  6.) Calibrate the B output bias, optional for rev 3 only with dual outputs, A and B
//  7.) Save the results
//
// The clock source needs to be MIPS to perform the calibration, SEXTCLK,FALSE
//
// Gordon Anderson
// October 22, 2016
//

#include "Shared.h"

// This function enables calibration of the Aux output channel.
void CalibrateAux(void)
{
  char   *Token;
  String sToken;
  float  val1,val2,m,b;

  serial->println("Calibrate Aux output, monitor with a voltmeter.");
  serial->println("Enter values when prompted.");
  // Set output to 0
  WriteWFaux(0);
  // Ask for actual value
  serial->print("Enter actual output value: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  serial->println(Token);
  sToken = Token;
  val1 = sToken.toFloat();
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);  
  // Set output to 25
  WriteWFaux(25);
  // Ask for actual value
  serial->print("Enter actual output value: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  sToken = Token;
  serial->println(Token);
  val2 = sToken.toFloat();
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);  
  // Calculate calibration parameters and apply
  m = (25.0 * ARBparms.AuxDACm) / (val2-val1);
  b = ARBparms.AuxDACb - val1 * ARBparms.AuxDACm;
  serial->println("Calibration parameters:");
  serial->print("m = "); serial->println(m);
  serial->print("b = "); serial->println(b);
  ARBparms.AuxDACm = m;
  ARBparms.AuxDACb = b;
}

// This function enables calibration of the offset channel.
void CalibrateOffset(void)
{
  char   *Token;
  String sToken;
  float  val1,val2,m,b;

  serial->println("Calibrate offset output, monitor with a voltmeter.");
  serial->println("Enter values when prompted.");
  // Set output to 0
  WriteWFoffset(0);
  // Ask for actual value
  serial->print("Enter actual output value: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  serial->println(Token);
  sToken = Token;
  val1 = -sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);  
  // Set output to 25
  WriteWFoffset(25);
  // Ask for actual value
  serial->print("Enter actual output value: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  sToken = Token;
  serial->println(Token);
  val2 = -sToken.toFloat();
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);  
  // Calculate calibration parameters and apply
  m = (25.0 * ARBparms.OffsetDACm) / (val1-val2);
  b = ARBparms.OffsetDACb + val1 * ARBparms.OffsetDACm;
  serial->println("Calibration parameters:");
  serial->print("m = "); serial->println(m);
  serial->print("b = "); serial->println(b);
  ARBparms.OffsetDACm = m;
  ARBparms.OffsetDACb = b;  
}

// Calibration of the range output uses the first ARB output channel. Monitor
// this channel and enter the values requested from the application.
void CalibrateRange(void)
{
  char   *Token;
  String sToken;
  float  val1,m,b;

  serial->println("Calibrate ARB range, monitor ARB channel 1 with a voltmeter.");
  serial->println("Enter values when prompted.");
  // Set output channel 0 to full scale, 100%
  // Set to 0 volts and ask for actual value
  for(int i=0;i<8;i++) DACchanBits[i] = 127;
  SetDACchannelR(0,245);
  WriteWFrange(50.0);
  // Ask for actual value
  serial->print("Enter actual output value: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  serial->println(Token);
  sToken = Token;
  val1 = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);  
  // Calculate calibration parameters and apply
  m = (25.0 * ARBparms.GainDACm) / val1;
  b = 0;
  serial->print("m = ");
  serial->println(m);
  serial->print("b = ");
  serial->println(b);
  ARBparms.GainDACm = m;
  ARBparms.GainDACb = b;
  
}

// This function is used to calibrate each DAC channel, 1 through 8.
void CalibrateChannel(int channel)
{ 
  char   *Token;
  String sToken;
  float  val1,val2,m,b;

  // Test input channel range
  if((channel < 1) || (channel > 8))
  {
    serial->println("Invalid channel selected, valid range 1 thru 8.");
    return;
  }
  channel--;
  serial->println("Calibrate ARB channel, monitor chan with voltmeter.");
  serial->println("Enter values when prompted.");
  WriteWFrange(50.0);
  // Set to 0 volts and ask for actual value
  for(int i=0;i<8;i++) DACchanBits[i] = 127;
  SetDACchannelR(channel,127);
  serial->print("Enter actual output value: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  serial->println(Token);
  sToken = Token;
  val1 = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);  
  // Set to 25 volts and ask for actual value
  SetDACchannelR(channel,245);
  serial->print("Enter actual output value: ");
  while((Token = GetToken(true)) == NULL) ProcessSerial(false);
  serial->println(Token);
  sToken = Token;
  val2 = sToken.toFloat(); 
  while((Token = GetToken(true)) != NULL) ProcessSerial(false);  
  // Calculate calibration parameters and apply
  m = 118.0 / ((val2-val1) * 4.0);
  b = 127 - val1 * m * 4.0;
  serial->println("Calibration parameters:");
  serial->print("m = "); serial->println(m);
  serial->print("b = "); serial->println(b);
  ARBparms.DACgains[channel] = m;
  ARBparms.DACoffsets[channel] = b;
}

// SCALDAC,channel,gain,offset. Writes the calibration parameters for a DAC channel, 0 to 7.
void SetCalDAC(void)
{
   char   *Token;
   String sToken;
   int    ch;
   float  gain,offset;

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     ch = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     gain = sToken.toFloat();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     offset = sToken.toFloat();
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     if((ch < 0) || (ch > 7)) break;
     // Set DAC parms
     ARBparms.DACgains[ch] = gain;
     ARBparms.DACoffsets[ch] = offset;
     SendACK;
     return;
   }
   SetErrorCode(ERR_BADARG);
   SendNAK;
}

// This function is used to calibrate the output boards bias and gain and offset.
// The capability is avalible on Rev 3 of the arb and arb_amp and supports bias 
// offset for each board. Supports board 1 and 2.
void CalibrateBias(int board)
{
   char   *Token;
   String sToken;
   int    DACzero,DAC5v,DACchan;

  // Test input board range
  if((board < 1) || (board > 2))
  {
    serial->println("Invalid board selected, valid range 1 thru 2.");
    return;
  }
  board--;
  DACchan = DACaddBias[board];
  // Start from the current calibration in case -1 is entered without adjusting
  DACzero = ARBparms.BiasCalB[board];
  DAC5v   = ARBparms.BiasCalB[board] + 5.0 * ARBparms.BiasCalM[board];
  // Set the DAC output channel 1 to zero and the range to 0.
  WriteWFrange(0);
  SetDACchannelR(0,127);
  // Adjust DAC to set output channels at zero. Enter DAC counts to
  // adjust, when finished enter -1 to advance. Monitor channel 1 output.
  serial->println("Calibrate ARB amp board bias output, monitor ARB channel 1 with a voltmeter.");
  serial->println("Enter DAC counts to adjust channel 1 to 0 volts, enter -1 when done.");
  while(1)
  {
     serial->print("Enter DAC counts: ");
     GetToken(true);
     while((Token = GetToken(true)) == NULL) ProcessSerial(false);
     serial->println(Token);
     sToken = Token;
     if(sToken.toInt() <= -1) break;
     DACzero = sToken.toInt();
     analogWrite(DACchan, DACzero);
  }
  // Adjust DAC to set output channels at 5 volts. Enter DAC counts to
  // adjust, when finished enter -1 to advance. Monitor channel 1 output.
  serial->println("Calibrate ARB amp board bias output, monitor ARB channel 1 with a voltmeter.");
  serial->println("Enter DAC counts to adjust channel 1 to 5 volts, enter -1 when done.");
  while(1)
  {
     serial->print("Enter DAC counts: ");
     GetToken(true);
     while((Token = GetToken(true)) == NULL) ProcessSerial(false);
     serial->println(Token);
     sToken = Token;
     if(sToken.toInt() <= -1) break;
     DAC5v = sToken.toInt();
     analogWrite(DACchan, DAC5v);
  }
  // Calculate cal parameters and save. 
  // DAC counts = mx + b
  ARBparms.BiasCalB[board] = (float)DACzero;
  ARBparms.BiasCalM[board] = (float)(DAC5v - DACzero) / 5.0;
  serial->print("m = ");
  serial->println(ARBparms.BiasCalM[board]);
  serial->print("b = ");
  serial->println(ARBparms.BiasCalB[board]);
  // Set to value defined in data structure
  WriteBoardBias(board, ARBparms.Bias[board]);
}
