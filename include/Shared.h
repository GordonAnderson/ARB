//
// Shared.h
//
// Shared declarations for the ARB firmware. When this project was built with the
// Arduino IDE all the .ino files were concatenated into one translation unit and the
// IDE generated function prototypes automatically. Under PlatformIO each .cpp file is
// compiled on its own, so the globals and functions shared between files are declared here.
//
#ifndef SHARED_H_
#define SHARED_H_

#include <Arduino.h>
#include <Wire.h>
#include <DueFlashStorage.h>
#include <MIPStimer.h>
#include <SerialBuffer.h>
#include "ARB.h"
#include "Hardware.h"
#include "Serial.h"
#include "Errors.h"

// Buffer sizes and storage location, shared by ARB.cpp and Hardware.cpp
#define PPP   128           // Number of points per waveform period
#define NP    1             // Number of periods in buffer, has to be 1 for compresion. The actual buffer
#define CHANS 8             // Total number of DAC channels
#define MAXBUFFER 8000
#define NonVolSize    1000                         // Bytes reserved at the top of FLASH for settings
#define NonVolStorage (0x100000 - NonVolSize)      // Absolute FLASH address of the saved ARB_PARMS

// Globals defined in ARB.cpp
extern ARB_PARMS        ARBparms;
extern DueFlashStorage  dueFlashStorage;
extern SerialBuffer     sb;
extern MIPStimer        DMAclk;
extern MIPStimer        ResetTMR;
extern MIPStimer        ARBclk;
extern MIPStimer        ALTtmr;
extern float            fVersion;
extern uint32_t         ARBbuffer[];
extern uint32_t         buffer[];
extern uint32_t         *DACadd;
extern uint8_t          *DACchan;
extern float            DACchanValues[8];
extern char             VectorString[];
extern volatile byte    AuxDACupdate;
extern bool             FreqUpdate;
extern bool             VoltageTestEnable;
extern byte             Status;
extern float            PS12v[2];
extern float            PS50v[2];
extern FreqSweep        fSweep;
extern int              WorkingOrder;
extern int              CrampCounter;
extern int              DACaddBias[2];

// ARB.cpp
void SaveSettings(void);
void RestoreSettings(void);
int  ARBchannelValue2Counts(int ch, float value);
void SetARBchannelRange(int ch, int startI, int stopI, float fval);
void SetARBchannels(float fval);
void SetARBchannelSine(int ch, float iPhase);
void FillDACbuffer(void);
void SetWaveformType(int wft, bool force = false);
bool SetPointsPerPeriod(int ppp);
void SetCompression(bool enable);
void SetDACchannelValue(int ch, float fval);
void RealTimeTWAVEISR();
void RealTimeARBISR();
int  SetFrequency(int freq);
void SetEnable(bool NoTrigger = false);
void SetDisable(void);
int  ReadUnsignedWord(void);
bool ReadInt(int *i);
int  ReadUnsignedByte(void);
bool ReadByte(int8_t *b);
bool Read16bitInt(int16_t *shortint);
bool ReadFloat(float *fval);
void SendByte(byte bval);
void SendInt24(int ival);
void SendFloat(float fval);
void requestEvent(void);
void receiveEvent(int howMany);
void SetAlternateWaveform(int type);
void ARBline1ISR(void);
void ARBline2ISR(void);
void ARBsyncISR(void);
void ALTwaveformStopISR(void);
void ALTwaveformStartISR(void);
void compressISR(int state);
bool ProcessRamp(void);
void ProcessSweep(void);
void SetMode(char *mode);
void GetMode(void);
void SetWFfreq(int freq);
void GetWFfreq(void);
void SetWFref(int count);
void SetWFoffset(int count);
void SetWFdisable(void);
void SetWFenable(void);
void SetWFdir(char *dir);
void SetFwdPS(char *val);
void SetRevPS(char *val);
void SetPPP(int ppp);
void SetWaveform(char *wavefrm);
void SetDACchannelR(int chan, int val);
void SetBoardBias(char *sboard, char *sval);
void SetPowerEnable(char *state);
void SetDACchannelV(char *schan, char *sval);
void ProcessVectorString(void);
void SetWFrange(char *srange);
void SetWFoffsetV(char *srange);
void SetWFaux(char *srange);
void SetARBchns(char *sval);
void SetARBchannel(char *sch, char *sval);
void SetARBsine(char *Chan, char *iPhase);
void SetARBchanRange(void);
void SetBufferLength(int len);
void ReportSupplyVoltages(void);
void DebugFunction(void);
void StartSweep(void);
void StopSweep(void);
void GetStatus(void);
void SetCompressor(char *value);
void SetExternalClock(char *value);
void SetExternalClockSource(char *value);
void SetAltTrigMode(int mode);
void GetAltTrigMode(void);
void SetFixedValue(char *index, char *val);
void GetFixedValue(int index);
void SetAltWaveFrm(int wfrm);
void SetSyncLine(int line);
void SetCompLine(int line);

// Hardware.cpp
void DMArestart(void);
void DMAstartISR(void);
void SetSMCmode(char *val);
void SetCycleTiming(int val);
void SetPulseTiming(int rw, int csrw);
void SetAddressSetup(int val);
void Software_Reset();
void WriteWFrange(float value);
void WriteWFoffset(float value);
void WriteWFaux(float value);
void WriteBoardBias(int board, float value);
void MeasureVoltages(void);
int  AD5625(int8_t adr, uint8_t chan, uint16_t val);
int  AD5625(int8_t adr, uint8_t chan, uint16_t val, int8_t Cmd);
int  AD5625_EnableRef(int8_t adr);
void ComputeCRCbyte(byte *crc, byte by);
byte ComputeCRC(byte *buf, int bsize);
void FLASHtoSerial(void);
void SerialtoFLASH(void);
void ProgramFLASH(char *Faddress, char *Fsize);

// Calibration.cpp
void CalibrateAux(void);
void CalibrateOffset(void);
void CalibrateRange(void);
void CalibrateChannel(int channel);
void SetCalDAC(void);
void CalibrateBias(int board);

// Serial.cpp
void MoveIt(int passcode);
void SerialIO(void);
String GetToken(String cmd, int TokenNum);

#endif /* SHARED_H_ */
