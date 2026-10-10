/*
 * Serial.cpp
 *
 * This file contains the code for the serial IO hardware including the ring buffer
 * code and the serial command processor.
 *
 * Developed by: Gordon Anderson
 */
#include "Shared.h"
#include "string.h"

Stream *serial = &SerialUSB;
bool SerialMute = false;

#define MaxToken 20

char Token[MaxToken];
char Sarg1[MaxToken];
char Sarg2[MaxToken];
unsigned char Tptr;

int ErrorCode = 0;   // Last communication error that was logged

Ring_Buffer  RB;     // Receive ring buffer

// ACK only string, two options. Need a comma when in the echo mode
const char *ACKonlyString1 = "\x06";
const char *ACKonlyString2 = ",\x06";
const char *SelectedACKonlyString = ACKonlyString1;

bool echoMode = false;

// Serial command table. Each entry is the command token, how it is handled (CmdTypes in
// Serial.h), the number of arguments and the variable or function it acts on. Commands
// starting with S set a value and reply with ACK; commands starting with G return a value.
Commands  CmdArray[] =   {
// General commands
  {"GVER",  CMDstr, 0, (char *)Version},                 // Report version
  {"GREV",  CMDint, 0, (char *)&ARBparms.rev},           // Report data structure rev value
  {"SREV",  CMDint, 1, (char *)&ARBparms.rev},           // Set data structure rev value
  {"GERR",  CMDint, 0, (char *)&ErrorCode},              // Report the last error code
  {"SERIAL", CMDfunction, 0, (char *)&SerialIO},         // Reassigns the serial IO port to SerialUSB
  {"DEBUG",  CMDfunction, 0, (char *)&DebugFunction},    // Debug, forces a DMA restart
  {"MUTE",  CMDfunctionStr, 1, (char *)Mute},            // Turns on and off the serial response from the MIPS system
  {"ECHO",  CMDbool, 1, (char *)&echoMode},              // Turns on and off the serial echo mode where the command is echoed to host, TRUE or FALSE
  {"DELAY", CMDfunction, 1, (char *)DelayCommand},       // Generates a delay in milliseconds. This is used by the macro functions
                                                         // to define delays in voltage ramp up etc.
  {"GCMDS",  CMDfunction, 0, (char *)GetCommands},       // Send a list of all commands
  {"RESET",  CMDfunction, 0, (char *)Software_Reset},    // System reboot
  {"SAVE",   CMDfunction, 0, (char *)SaveSettings},      // Save settings
  {"RESTORE", CMDfunction, 0, (char *)RestoreSettings},  // Restore settings
  {"STWIADD", CMDint, 1, (char *)&ARBparms.TWIarbAdd},   // Sets this ARB modules TWI address
  {"GTWIADD", CMDint, 0, (char *)&ARBparms.TWIarbAdd},   // Returns this ARB modules TWI address
  {"GETFLASH", CMDfunction, 0, (char *)FLASHtoSerial},   // Sends the FLASH data to the host
  {"PUTFLASH", CMDfunction, 0, (char *)SerialtoFLASH},   // Receives data from the host and writes to FLASH
  {"ARBPGM", CMDfunctionStr, 2, (char *)ProgramFLASH},   // Upload program to ARB FLASH
  {"MOVEIT", CMDfunction, 1, (char *)MoveIt},            // Restart in Flash1 to run moveit program
  {"CPLD",  CMDbool, 1, (char *)&ARBparms.CPLD},         // TRUE to select timing options for CPLD logic
  {"XPT",  CMDbool, 1, (char *)&ARBparms.XPtiming},      // TRUE to select timing options for XPtiming logic
  {"TMCRT", CMDbool, 1, (char *)&ARBparms.TimingCorrect},// TRUE to correct the timing error from the CPLD logic bug on channel 1
  {"SPPP", CMDfunction, 1, (char *)SetPPP},              // Sets ARB Twave mode point per period, 4 to 128
  {"GPPP", CMDint, 0, (char *)&ARBparms.ppp},            // Returns ARB Twave mode point per period  
// ARB general commands
  {"SMODE", CMDfunctionStr, 1, (char *)SetMode},         // Sets the ARB mode
  {"GMODE", CMDfunction, 0, (char *)GetMode},            // Reports the ARB mode
  {"SWFREQ", CMDfunction, 1, (char *)SetWFfreq},         // Sets waveform frequency in Hz, TWAVE max 1.28MHz/ppp, ARB max 1.28MHz
  {"GWFREQ", CMDfunction, 0, (char *)GetWFfreq},         // Returns the actual waveform frequency in Hz
  {"SWFVRNG", CMDfunctionStr, 1, (char *)SetWFrange},    // Sets waveform voltage range, rev 2.0
  {"GWFVRNG", CMDfloat, 0, (char *)&ARBparms.VoltageRange},
  {"SWFVRAMP", CMDfloat, 1, (char *)&ARBparms.RampRate}, // Sets waveform voltage range ramp rate, 0 = no ramp
  {"GWFVRAMP", CMDfloat, 0, (char *)&ARBparms.RampRate},
  {"SWFVOFF", CMDfunctionStr, 1, (char *)SetWFoffsetV},  // Sets waveform offset voltage, rev 2.0
  {"GWFVOFF", CMDfloat, 0, (char *)&ARBparms.VoltageOffset},
  {"SWFVAUX", CMDfunctionStr, 1, (char *)SetWFaux},       // Sets waveform aux voltage, rev 2.0
  {"GWFVAUX", CMDfloat, 0, (char *)&ARBparms.VoltageAux},
  {"SWFDIS", CMDfunction, 0, (char *)SetWFdisable},        // Stops waveform generation
  {"SWFENA", CMDfunction, 0, (char *)SetWFenable},         // Starts waveform generation
  {"SSYNCENA",  CMDbool, 1, (char *)&ARBparms.SyncEnable}, // TRUE to enable externl sync enable
  {"SEXTCLK", CMDfunctionStr, 1, (char *)SetExternalClock},         // Set External clock mode, TRUE / FALSE
  {"SEXTSRC", CMDfunctionStr, 1, (char *)SetExternalClockSource},   // Set External clock source, MIPS / EXT 
  {"SHWDCMP",  CMDbool, 1, (char *)&ARBparms.ISRcompress},          // Set to true to enable ISR processing of compress signal
  {"GHWDCMP",  CMDbool, 0, (char *)&ARBparms.ISRcompress},          // Returns the status of the ISR compress option flag
  // ARB Twave mode commands
  {"SWFOFF", CMDfunction, 1, (char *)SetWFoffset},       // Set waveform offset, rev 1 function only
  {"SWFREF", CMDfunction, 1, (char *)SetWFref},          // Set waveform ref (sets gain), rev 1 function only
  {"SWFVEC", CMDlongStr, PPP * 5 + 2, (char *)VectorString}, // Set waveform vector, up to ppp comma separated values -100 to 100, for Twave ARB mode
  {"SWFRM", CMDfunctionStr, 1, (char *)SetWaveform},     // Set waveform type
  {"SDACR", CMDfunction, 2, (char *)SetDACchannelR},     // Sets the selected DAC channel value, channel is 0-7 and value is raw counts 0-255
  {"SDACV", CMDfunctionStr, 2, (char *)SetDACchannelV},  // Sets the selected DAC channel value, channel is 1-8 and value is percent of FS, +-100
  {"SWFDIR", CMDfunctionStr, 1, (char *)SetWFdir},      // Set waveform direction, TRUE or FALSE
  {"GWFDIR", CMDbool, 0, (char *)&ARBparms.Direction},   // Returns direction flag
  {"SFWDPS", CMDfunctionStr, 1, (char *)SetFwdPS},      // Set forward direction phase shift, degrees
  {"GFWDPS", CMDfloat, 0, (char *)&ARBparms.fwdPS},      // Returns forward direction phase shift
  {"SREVPS", CMDfunctionStr, 1, (char *)SetRevPS},      // Set reverse direction phase shift, degrees
  {"GREVPS", CMDfloat, 0, (char *)&ARBparms.revPS},      // Returns reverse direction phase shift
  {"SWFCMP", CMDfunctionStr, 1, (char *)SetCompressor},              // Enables or disables compression mode, TRUE = enable
  {"GWFCMP", CMDbool, 0, (char *)&ARBparms.CompressEnable},          // Return compression flag
  {"SWFORD", CMDint, 1, (char *)&ARBparms.Order},                    // Set the compression order
  {"GWFORD", CMDint, 0, (char *)&ARBparms.Order},                    // Returns the compression order
  {"SWFEXTCMP", CMDbool, 1 , (char *)&ARBparms.CompressHardware},    // Enable external compression control, TRUE OR FALSE  
  {"GWFEXTCMP", CMDbool, 0 , (char *)&ARBparms.CompressHardware},    // Returns external compression control flag 
  {"GBCOUNT", CMDint, 0, (char *)&Bcount},                  // Debug, returns the DMA buffer count
  // ARB conventional ARB mode commands
  {"SARBBUF", CMDfunction, 1, (char *)SetBufferLength},     // Sets ARB buffer length, 1 to 8000 samples
  {"GARBBUF", CMDint, 0, (char *)&ARBparms.Bufferlength},   // Reports ARB buffer length
  {"SARBNUM", CMDint, 1, (char *)&ARBparms.NumBuffers},     // Sets number of ARB buffer repeats per trigger
  {"GARBNUM", CMDint, 0, (char *)&ARBparms.NumBuffers},     // Reports number of ARB buffer repeats per trigger
  {"SARBCHS", CMDfunctionStr, 1, (char *)SetARBchns},       // Sets all ARB channels in the full buffer to a defined value
  {"SARBCH", CMDfunctionStr, 1, (char *)SetARBchannel},     // Sets a defined ARB channel in the full buffer to a defined value
  {"SACHRNG", CMDfunctionLine, 0, (char *)SetARBchanRange}, // Sets an ARB channel to a value over a defined range
  {"SARBSINE", CMDfunctionStr, 2, (char *)SetARBsine},      // Sets an ARB channel to one sine wave cycle, arguments are channel 0 to 7 and starting phase in degrees
  // Commands supporting hardware rev 3.0
  {"RSUPPLY", CMDfunction, 0, (char *)ReportSupplyVoltages},// Reports the measured +-12V and HV opamp supplies
  {"SBIAS", CMDfunctionStr, 2, (char *)SetBoardBias},       // Sets a board (1 or 2) DC bias value (-10 to 10)
  {"SPWR", CMDfunctionStr, 1, (char *)SetPowerEnable},      // Turns power supply ON and OFF
  // Calibration functions and commands, changing the reference values requires a reboot.
  {"SCALDAC", CMDfunctionLine, 0, (char *)SetCalDAC},       // Sets the slope and offset for a specific DAC channel
  {"SDACREF", CMDfloat, 1, (char *)&ARBparms.DACrefVoltage},// Sets the reference voltage used on the DAC, 3.0 or 2.5
  {"GDACREF", CMDfloat, 0, (char *)&ARBparms.DACrefVoltage},// Returns the reference voltage used on the DAC, 3.0 or 2.5
  {"SAXOFREF", CMDfloat, 1, (char *)&ARBparms.AuxOffRef},   // Sets the reference voltage used for aux and offset outputs
  {"GAXOFREF", CMDfloat, 0, (char *)&ARBparms.AuxOffRef},   // Returns the reference voltage used for aux and offset outputs
  {"CALAUX", CMDfunction, 0, (char *)CalibrateAux},         // Interactive calibration, see Calibration.cpp
  {"CALOFF", CMDfunction, 0, (char *)CalibrateOffset},
  {"CALRNG", CMDfunction, 0, (char *)CalibrateRange},
  {"CALCHN", CMDfunction, 1, (char *)CalibrateChannel},     // Channel 1 to 8
  {"CALBIAS", CMDfunction, 1, (char *)CalibrateBias},       // Board 1 or 2
  {"SLEGACY", CMDbool, 1, (char *)&ARBparms.Legacy},        // TRUE to support the first ARB rev (enables TWI_SET_REF)
  {"GLEGACY", CMDbool, 0, (char *)&ARBparms.Legacy},        // Returns the legacy flag
  // SMC bus timing commands, used to tune the memory mapped DAC interface
  {"SMCMODE", CMDfunctionStr, 1, (char *)SetSMCmode},       // Sets the SMC mode register for chip select 0, hex
  {"CYCLETM", CMDfunction, 1, (char *)SetCycleTiming},      // Sets the read and write cycle length
  {"PULSETM", CMDfunction, 2, (char *)SetPulseTiming},      // Sets the read/write pulse and chip select pulse width
  {"ADDSETUP", CMDfunction, 1, (char *)SetAddressSetup},    // Sets the address setup time
  // Sweep commands
  // Twave and ARB frequency/voltage sweep commands
  {"STWSSTRT",CMDint, 1, (char *)&fSweep.StartFreq},           // Set the sweep start frequency
  {"GTWSSTRT",CMDint, 0, (char *)&fSweep.StartFreq},           // Return the sweep start frequency
  {"STWSSTP",CMDint, 1, (char *)&fSweep.StopFreq},             // Set the sweep stop frequency
  {"GTWSSTP",CMDint, 0, (char *)&fSweep.StopFreq},             // Return the sweep stop frequency
  {"STWSSTRTV",CMDfloat, 1, (char *)&fSweep.StartVoltage},     // Set the sweep start voltage
  {"GTWSSTRTV",CMDfloat, 0, (char *)&fSweep.StartVoltage},     // Return the sweep start voltage
  {"STWSSTPV",CMDfloat, 1, (char *)&fSweep.StopVoltage},       // Set the sweep stop voltage
  {"GTWSSTPV",CMDfloat, 0, (char *)&fSweep.StopVoltage},       // Return the sweep stop voltage
  {"STWSTM",CMDfloat, 1, (char *)&fSweep.SweepTime},           // Set the sweep time
  {"GTWSTM",CMDfloat, 0, (char *)&fSweep.SweepTime},           // Return the sweep time
  {"STWSGO",CMDfunction, 0, (char *)StartSweep},               // Start the sweep
  {"STWSHLT",CMDfunction, 0, (char *)StopSweep},               // Stop the sweep
  {"GTWSTA",CMDfunction, 0, (char *)GetStatus},                // Return the sweep status
  // Alternate waveform commands
  {"SALTENA", CMDbool, 1, (char *)&ARBparms.AlternateEnable},   // Enables alternate waveform, TRUE or FALSE
  {"GALTENA", CMDbool, 0, (char *)&ARBparms.AlternateEnable},   // Returns alternate waveform enable status
  {"SALTHWD", CMDbool, 1, (char *)&ARBparms.AlternateHardware}, // Enables alternate waveform harware triggering. TRUE or FALSE
  {"GALTHWD", CMDbool, 0, (char *)&ARBparms.AlternateHardware}, // Returns alternate waveform harware triggering status
  {"SALTTMODE",CMDfunction, 1, (char *)SetAltTrigMode},         // Sets the alternate trigger mode, 0 = level, 1= pos, 2= neg
  {"GALTTMODE",CMDfunction, 0, (char *)GetAltTrigMode},         // Return the alternate trigger mode, 0 = level, 1= pos, 2= neg

  {"SALTFVAL",CMDfunctionStr, 2, (char *)SetFixedValue},        // Sets the alternate waveform fixed value in percent, -100 to 100. first argument is index 0 to 7
  {"GALTFVAL",CMDfunction, 1, (char *)GetFixedValue},           // Returns the alternate waveform fixed value in percent, -100 to 100. first argument is index 0 to 7

  {"SALTWFM",CMDfunction, 1, (char *)SetAltWaveFrm},            // Sets the alternate waveform type, 1=compress (default),2=reverse,3=arb,4=fixed,5=current
 
  {"SALTDLY", CMDfloat,1, (char *)&ARBparms.TriggerDly},        // Sets the alternate waveform trigger delay for pos or neg edge trigging only, in mS
  {"GALTDLY", CMDfloat,0, (char *)&ARBparms.TriggerDly},        // Returns the alternate waveform trigger delay for pos or neg edge trigging only, in mS
  {"SALTPLY", CMDfloat,1, (char *)&ARBparms.PlayDuration},      // Sets the alternate waveform play or apply time for pos or neg edge trigging only, in mS
  {"GALTPLY", CMDfloat,0, (char *)&ARBparms.PlayDuration},      // Returns the alternate waveform play or apply time for pos or neg edge trigging only, in mS
  
  {"SALTRENA", CMDbool, 1, (char *)&ARBparms.AlternateRngEna},  // Enables alternate waveform range. TRUE or FALSE
  {"GALTRENA", CMDbool, 0, (char *)&ARBparms.AlternateRngEna},  // Returns alternate waveform range enable flag
  {"SALTRNG",  CMDfloat,1, (char *)&ARBparms.AlternateRng},     // Sets the alternate waveform range in volts
  {"GALTRNG",  CMDfloat,0, (char *)&ARBparms.AlternateRng},     // Returns the alternate waveform range in volts

  {"SALTFENA", CMDbool, 1, (char *)&ARBparms.AlternateFreqEna},  // Enables alternate waveform frequency. TRUE or FALSE
  {"GALTFENA", CMDbool, 0, (char *)&ARBparms.AlternateFreqEna},  // Returns alternate waveform frequency enable flag
  {"SALTFRQ",  CMDint,1, (char *)&ARBparms.AltFreq},             // Sets the alternate waveform frequency in Hz
  {"GALTFRQ",  CMDint,0, (char *)&ARBparms.AltFreq},             // Returns the alternate waveform frequency in Hz

  // Commands to redefine the hardware control lines used for Sync and Compression
  {"SSYNCLINE",CMDfunction, 1, (char *)SetSyncLine},            // Sets the hardware line used for sync, 1 = default, 2 = default compress line
  {"SCOMPLINE",CMDfunction, 1, (char *)SetCompLine},            // Sets the hardware line used for compress, 1 = default for sync line, 2 = default  
  // End of table marker
  {0},
};

// MOVEIT command. Sets the GPNVM boot bits to boot from FLASH bank 1, where the mover
// application copies a new ARB image (uploaded with ARBPGM) into bank 0, then resets.
// Runs from RAM because it changes the FLASH controller. passcode must be 5410.
__attribute__ ((long_call, section (".ramfunc")))
void MoveIt(int passcode)
{
  if (passcode != 5410) return;
  // Send ESC to TWI port to end TWITALK session if in in progress
  sb.write(ESC);
  delay(500);      // Give it time to happen
  // Reboot to Flash1
  __disable_irq();
  const int EEFC_FCMD_SGPB = 0x0B;
  const int EEFC_KEY = 0x5A;
  while ((EFC0->EEFC_FSR & EEFC_FSR_FRDY) == 0);
  // Set bootflag to run from FLASH instead of ROM
  EFC0->EEFC_FCR =
    EEFC_FCR_FCMD(EEFC_FCMD_SGPB) |
    EEFC_FCR_FARG(1) |
    EEFC_FCR_FKEY(EEFC_KEY);
  while ((EFC0->EEFC_FSR & EEFC_FSR_FRDY) == 0);
  // Set bootflag to run from FLASH1
  EFC0->EEFC_FCR =
    EEFC_FCR_FCMD(EEFC_FCMD_SGPB) |
    EEFC_FCR_FARG(2) |
    EEFC_FCR_FKEY(EEFC_KEY);
  while ((EFC0->EEFC_FSR & EEFC_FSR_FRDY) == 0);
  // Force a hard reset
  const int RSTC_KEY = 0xA5;
  RSTC->RSTC_CR =
    RSTC_CR_KEY(RSTC_KEY) |
    RSTC_CR_PROCRST |
    RSTC_CR_PERRST;
  while (true); //bye cruel world!
}

// Sends a list of all commands
void GetCommands(void)
{
  int  i;

  SendACKonly;
  // Loop through the commands array and send all the command tokens
  for (i = 0; CmdArray[i].Cmd != 0; i++)
  {
    serial->println((char *)CmdArray[i].Cmd);
  }
}

// Delay command, delay is in millisecs
void DelayCommand(int dtime)
{
  delay(dtime);
  SendACK;
}

// Turns on and off responses from the MIPS system
void Mute(char *cmd)
{
  if (strcmp(cmd, "ON") == 0)
  {
    SerialMute = true;
    SendACK;
    return;
  }
  else if (strcmp(cmd, "OFF") == 0)
  {
    SerialMute = false;
    SendACK;
    return;
  }
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// SERIAL command, returns the serial port to USB
void SerialIO(void)
{
  serial = &SerialUSB;
}

void SerialInit(void)
{
  SerialUSB.begin(115200);
  RB_Init(&RB);
}

// This function builds a token string from the characters passed.
void Char2Token(char ch)
{
  Token[Tptr++] = ch;
  if (Tptr >= MaxToken) Tptr = MaxToken - 1;
}

// Get token from string object, delimited with comma
// Token are numbered 1 through n, returns empty string
// at end of tokens
String GetToken(String cmd, int TokenNum)
{
  int i, j, k;
  String Token;

  // If TokenNum is 0 or 1 then return the first token.
  // Return up to delimiter of the whole string.
  cmd.trim();
  if (TokenNum <= 1)
  {
    if ((i = cmd.indexOf(',')) == -1) return cmd;
    Token = cmd.substring(0, i);
    return Token;
  }
  // Find the requested token
  k = 0;
  for (i = 2; i <= TokenNum; i++)
  {
    if ((j = cmd.indexOf(',', k)) == -1) return "";
    k = j + 1;
  }
  Token = cmd.substring(k);
  Token.trim();
  if ((j = Token.indexOf(',')) == -1) return Token;
  Token = Token.substring(0, j);
  Token.trim();
  return Token;
}

// This function reads the serial input ring buffer and returns a pointer to a ascii token.
// Tokens are comma delimited. Commands strings end with a semicolon or a \n.
// The returned token pointer points to a standard C null terminated string.
// This function does not block. If there is nothing in the input buffer null is returned.
char *GetToken(bool ReturnComma)
{
  unsigned char ch;

  // Exit if the input buffer is empty
  while (1)
  {
    ch = RB_Next(&RB);
    if (ch == 0xFF) return NULL;
    if (Tptr >= MaxToken) Tptr = MaxToken - 1;
    if ((ch == '\n') || (ch == ';') || (ch == ':') || (ch == ',') || (ch == ']') || (ch == '['))
    {
      if (Tptr != 0) ch = 0;
      else
      {
        Char2Token(RB_Get(&RB));
        ch = 0;
      }
    }
    else RB_Get(&RB);
    // Place the character in the input buffer and advance pointer
    Char2Token(ch);
    if (ch == 0)
    {
      Tptr = 0;
      if ((Token[0] == ',') && !ReturnComma) return NULL;
      return Token;
    }
  }
}

void ExecuteCommand(Commands *cmd, int arg1, int arg2, char *args1, char *args2, float farg1)
{
  if (echoMode) SelectedACKonlyString = ACKonlyString2;
  else SelectedACKonlyString = ACKonlyString1;
  switch (cmd->Type)
  {
    case CMDbool:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute)
        {
          if (*(cmd->pointers.boolPtr)) serial->println("TRUE");
          else serial->println("FALSE");
        }
      }
      if (cmd->NumArgs == 1)  // If true then read the value
      {
        if ((strcmp(args1, "TRUE") == 0) || (strcmp(args1, "FALSE") == 0))
        {
          if (strcmp(args1, "TRUE") == 0) *(cmd->pointers.boolPtr) = true;
          else *(cmd->pointers.boolPtr) = false;
          SendACK;
          break;
        }
        SetErrorCode(ERR_BADARG);
        SendNAK;
      }
      break;
    case CMDstr:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute) serial->println(cmd->pointers.charPtr);
        break;
      }
      if (cmd->NumArgs == 1)  // If true then read the value
      {
        strcpy(cmd->pointers.charPtr, args1);
        SendACK;
        break;
      }
      break;
    case CMDint:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute) serial->println(*(cmd->pointers.intPtr));
        break;
      }
      if (cmd->NumArgs == 1)
      {
        *(cmd->pointers.intPtr) = arg1;
        SendACK;
        break;
      }
      break;
    case CMDfloat:
      if (cmd->NumArgs == 0)   // If true then write the value
      {
        SendACKonly;
        if (!SerialMute) serial->println(*(cmd->pointers.floatPtr));
      }
      if (cmd->NumArgs == 1)
      {
        *(cmd->pointers.floatPtr) = farg1;
        SendACK;
        break;
      }
      break;
    case CMDfunction:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      if (cmd->NumArgs == 1) cmd->pointers.func1int(arg1);
      if (cmd->NumArgs == 2) cmd->pointers.func2int(arg1, arg2);
      break;
    case CMDfunctionStr:
      if (cmd->NumArgs == 0) cmd->pointers.funcVoid();
      if (cmd->NumArgs == 1) cmd->pointers.func1str(args1);
      if (cmd->NumArgs == 2) cmd->pointers.func2str(args1, args2);
      break;
    case CMDfun2int1flt:
      if (cmd->NumArgs == 3) cmd->pointers.func2int1flt(arg1, arg2, farg1);
      break;
    default:
      SendNAK;
      break;
  }
}

// This function processes serial commands.
// This function does not block and returns -1 if there was nothing to do.
int ProcessCommand(void)
{
  String sToken;
  char   *Token, ch;
  int    i;
  static int   arg1, arg2;
  static float farg1;
  static enum  PCstates state;
  static int   CmdNum;
  static char  delimiter = 0;
  // The following variables are used for the long string reading mode
  static char *lstrptr = NULL;
  static int  lstrindex;
  static bool lstrmode = false;
  static int  lstrmax;

  // Wait for line in ringbuffer
  if (state == PCargLine)
  {
    if (RB.Commands <= 0) return -1;
    CmdArray[CmdNum].pointers.funcVoid();
    state = PCcmd;
    return 0;
  }
  if (lstrmode)
  {
    ch = RB_Get(&RB);
    if (ch == 0xFF) return (-1);
    if (ch == ',') return (0);
    if (ch == '\r') return (0);
    if (ch == '\n')
    {
      lstrptr[lstrindex++] = 0;
      lstrmode = false;
      return (0);
    }
    if (lstrindex < lstrmax - 1) lstrptr[lstrindex++] = ch;   // Drop characters that do not fit
    return (0);
  }
  Token = GetToken(false);
  if (Token == NULL) return (-1);
  if (Token[0] == 0) return (-1);
  if ((echoMode) && (!SerialMute))
  {
    if (strcmp(Token, "\n") != 0)
    {
      if (delimiter != 0) serial->write(delimiter);
      serial->print(Token);
    }
    if (strcmp(Token, "\n") == 0) delimiter = 0;
    else delimiter = ',';
  }
  switch (state)
  {
    case PCcmd:
      if (strcmp(Token, ";") == 0) break;
      if (strcmp(Token, "\n") == 0) break;
      CmdNum = -1;
      // Look for command in command table
      for (i = 0; CmdArray[i].Cmd != 0; i++) if (strcmp(Token, CmdArray[i].Cmd) == 0)
        {
          CmdNum = i;
          break;
        }
      if (CmdNum == -1)
      {
        SetErrorCode(ERR_BADCMD);
        SendNAK;
        break;
      }
      // If the type CMDfunctionLine then we will wait for a full line in the ring buffer
      // before we call the function. Function has not args and must pull tokens from ring buffer.
      if (CmdArray[i].Type == CMDfunctionLine)
      {
        state = PCargLine;
        break;
      }
      // If this is a long string read command type then init the vaiable to support saving the
      // string directly to the provided pointer and exit. This function must not block
      if (CmdArray[i].Type == CMDlongStr)
      {
        lstrptr = CmdArray[i].pointers.charPtr;
        lstrindex = 0;
        lstrmax = CmdArray[i].NumArgs;
        lstrmode = true;
        break;
      }
      if (CmdArray[i].NumArgs > 0) state = PCarg1;
      else state = PCend;
      break;
    case PCarg1:
      Sarg1[0] = 0;
      sToken = Token;
      sToken.trim();
      arg1 = sToken.toInt();
      farg1 = sToken.toFloat();
      strcpy(Sarg1, sToken.c_str());
      if (CmdArray[CmdNum].NumArgs > 1) state = PCarg2;
      else state = PCend;
      break;
    case PCarg2:
      Sarg2[0] = 0;
      sToken = Token;
      sToken.trim();
      arg2 = sToken.toInt();
      strcpy(Sarg2, sToken.c_str());
      if (CmdArray[CmdNum].NumArgs > 2) state = PCarg3;
      else state = PCend;
      break;
    case PCarg3:
      sToken = Token;
      sToken.trim();
      farg1 = sToken.toFloat();
      state = PCend;
      break;
    case PCend:
      if ((strcmp(Token, "\n") != 0) && (strcmp(Token, ";") != 0))
      {
        state = PCcmd;
        SendNAK;
        break;
      }
      i = CmdNum;
      CmdNum = -1;
      state = PCcmd;
      ExecuteCommand(&CmdArray[i], arg1, arg2, Sarg1, Sarg2, farg1);
      break;
    default:
      state = PCcmd;
      break;
  }
  return (0);
}

void RB_Init(Ring_Buffer *rb)
{
  rb->Head = 0;
  rb->Tail = 0;
  rb->Count = 0;
  rb->Commands = 0;
}

int RB_Size(Ring_Buffer *rb)
{
  return (rb->Count);
}

int RB_Commands(Ring_Buffer *rb)
{
  return (rb->Commands);
}

// Put character in ring buffer, return 0xFF if buffer is full and can't take a character.
// Return 0 if character is processed.
char RB_Put(Ring_Buffer *rb, char ch)
{
  if (rb->Count >= RB_BUF_SIZE) return (0xFF);
  rb->Buffer[rb->Tail] = ch;
  if (rb->Tail++ >= RB_BUF_SIZE - 1) rb->Tail = 0;
  rb->Count++;
  if (ch == ';') rb->Commands++;
  if (ch == '\r') rb->Commands++;
  if (ch == '\n') rb->Commands++;
  return (0);
}

// Get character from ring buffer, return 0xFF if empty. \r is returned as \n.
char RB_Get(Ring_Buffer *rb)
{
  char ch;

  if (rb->Count == 0)
  {
    rb->Commands = 0;  // This has to be true if the buffer is empty...
    return (0xFF);
  }
  ch = rb->Buffer[rb->Head];
  if (rb->Head++ >= RB_BUF_SIZE - 1) rb->Head = 0;
  rb->Count--;
  // Map \r to \n
  //  if(Recording) MacroFile.write(ch);
  if (ch == '\r') ch = '\n';
  if (ch == ';') rb->Commands--;
  if (ch == '\n') rb->Commands--;
  if (rb->Commands < 0) rb->Commands = 0;
  return (ch);
}

// Return the next character in the ring buffer but do not remove it, return 0xFF if empty.
char RB_Next(Ring_Buffer *rb)
{
  char ch;

  if (rb->Count == 0) return (0xFF);
  ch = rb->Buffer[rb->Head];
  if (ch == '\r') ch = '\n';
  return (ch);
}

void PutCh(char ch)
{
  RB_Put(&RB, ch);
}
