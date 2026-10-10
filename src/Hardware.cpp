//
// File: Hardware
//
// Low level hardware support for the ARB: the DMA engine that streams the DAC buffer, the
// AD5625 I2C DAC that sets the range, offset, aux and reference voltages, the board bias
// DACs and supply monitors, the SMC bus timing commands, and the FLASH upload / download
// commands.
//
#include "Shared.h"
#include <Parallel.h>

int ARBsync     = ARBline1;   // Sync input pin, A0 on this hardware. See SetSyncLine()
int CompressPin = ARBline2;   // Compress input pin, 22 on this hardware. See SetCompLine()

// Two structure arrays used to enable continous DMA of the buffer.
#define NumLLI  4
volatile LLI lliA[NumLLI];   
volatile LLI lliB[NumLLI]; 
volatile int lliMax = 0;
volatile int Bcount;    // Used to count the number of buffers transfered to the DACs

//
// DMA routines
//

// The DMA plays the DAC buffer as two linked lists, lliA and lliB, that point to each
// other so the transfer runs continuously. Each time one completes this interrupt picks
// the source for the next pass, the waveform period or the alternate period (compression
// and the alternate waveform modes), then clears the done bits to keep the process going.
// This routine only fires in the DMA mode.
void DMAC_Handler(void)
{
  uint32_t        i;
  static bool     lastAlt = false;
  static int      CM;

  if(Bcount == -1)
  {
    // Ignore any pending buffer loaded interrupts.
    Bcount = 2;
    return;
  }
  (void)DMAC->DMAC_EBCISR;   // Reading the status clears the interrupt
  Bcount++;
  // ARB mode, leave the done bits set after NumBuffers passes so the transfer stops
  if(ARBparms.Mode == ARBmode) if((Bcount >= ARBparms.NumBuffers-1) && (ARBparms.NumBuffers != 0))
  {
    return;
  }
  // Compression: play the waveform period once every WorkingOrder passes and the alternate
  // (held) period in between. CompressRamp steps WorkingOrder every |CompressRamp| cycles.
  if((ARBparms.Mode == TWAVEmode) && (ARBparms.CompressEnable)  && (WorkingOrder != 1))
  {
    if(Bcount > WorkingOrder)
    {
      lliA[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]);
      lliB[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]);
      Bcount = 1;
    }
    else if(Bcount == WorkingOrder)
    {
      lliA[0].SADDR = (uint32_t)&(buffer[0]);
      lliB[0].SADDR = (uint32_t)&(buffer[0]);
      Bcount = 0;
      CrampCounter++;
      if((CrampCounter >= abs(ARBparms.CompressRamp)) && (ARBparms.CompressRamp != 0))
      {
        CrampCounter = 0;
        if(ARBparms.CompressRamp < 0) WorkingOrder -= ARBparms.CrampOrder;
        else WorkingOrder += ARBparms.CrampOrder;
        if(WorkingOrder < 2) WorkingOrder = 2;
      }
    }
    else
    {
      lliA[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]);
      lliB[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]);      
    }
  }
  else if (ARBparms.Mode == TWAVEmode)
  {
    if(ARBparms.AlternateEnable)
    { 
      lliA[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]);
      lliB[0].SADDR = (uint32_t)&(buffer[ARBparms.ppp * CHANS / 4]); 
      if((!lastAlt) && (ARBparms.AlternateRngEna))  WriteWFrange(ARBparms.AlternateRng); // AuxDACupdate |= UpdateAltRange;, 11/11/22
      if((!lastAlt) && (ARBparms.AlternateFreqEna))
      {
        // If here set the frequency to the alternate value
        CM = digitalRead(ClockMode);
        digitalWrite(ClockMode,HIGH);
        SetFrequency(ARBparms.AltFreq);
        digitalWrite(ClockMode,LOW);
      }
      lastAlt = true;   
    }
    else
    {
       lliA[0].SADDR = (uint32_t)&(buffer[0]);
       lliB[0].SADDR = (uint32_t)&(buffer[0]);
       if((lastAlt) && (ARBparms.AlternateRngEna)) WriteWFrange(ARBparms.VoltageRange); //AuxDACupdate |= UpdateVoltageRange;, 11/11/22
      if((lastAlt) && (ARBparms.AlternateFreqEna))
      {
        // If here reset the frequency to the orginal value. Use the requested frequency, the
        // actual frequency would round down one divider step each time.
        digitalWrite(ClockMode,HIGH);
        SetFrequency(ARBparms.RequestedFreq);
        digitalWrite(ClockMode,CM);
      }
      lastAlt = false;          
    }
  }
  for(i=0;i<NumLLI;i++)
  {
     lliA[i].CTRLA &= ~DMAC_CTRLA_DONE;
     lliB[i].CTRLA &= ~DMAC_CTRLA_DONE;    
  }
}

// ResetTMR interrupt, restarts the TWAVE DMA transfer at a fixed delay after DMArestart
// stopped it. The fixed delay removes jitter on the sync.
void DMAstartISR(void)
{
  (void)DMAC->DMAC_EBCIER; //11-3-18
  DMAbuffer2DAC((uint32_t *)0x60000000, buffer, ARBparms.ppp * NP * CHANS / 4);
  Bcount = 2;
  if(dmac_channel_fifo_empty(DMAC_MEMCH)) Bcount = -1;    // This will only be true when there is a pending interrupt, can't clear it!
                                                          // Bcount is always -1 due to semi-colon, 11-3-18 removed semi-colon  
}

// Resyncs the TWAVE waveform: stops the DMA and starts ResetTMR, which restarts it from
// the start of the buffer in DMAstartISR.
void DMArestart(void)
{
  // Set RC count to delay time plus width
  ResetTMR.setRC(150);
  // Start the timer
  ResetTMR.enableTrigger();
  ResetTMR.softwareTrigger();

  // Reset the Cramp parameters
  WorkingOrder = ARBparms.Order;
  CrampCounter = 0;
  // Stop the DMA process
  dmac_channel_disable(DMAC_MEMCH);
  while(!dmac_channel_transfer_done(DMAC_MEMCH));
}

// This function starts the DMA continous transfer of n 32 bit words from src to the
// memory mapped DACs at dst. The DMA controller can move at most 4095 words per
// descriptor, so the buffer is split across up to NumLLI descriptors per list.
// NoTrigger sets everything up without enabling the channel.
void DMAbuffer2DAC(uint32_t *dst, uint32_t *src, uint32_t n, bool NoTrigger)
{
  uint32_t i;

  dmac_channel_disable(DMAC_MEMCH);
  while(!dmac_channel_transfer_done(DMAC_MEMCH));
  // Calculate lliMax based on buffer size, each DMA BTSIZE can not exceed 4095.
  // Setup each sent of LLI arrays
  lliMax = 0;
  for(i=0;i<NumLLI;i++)
  {
    if(i < NumLLI - 1)
    {
      lliA[i].DSCR = (uint32_t)&lliA[i+1];
      lliB[i].DSCR = (uint32_t)&lliB[i+1];
    }
    lliA[i].SADDR = (uint32_t)src + (4095 * 4 * i);
    lliA[i].DADDR = (uint32_t)dst + (4095 * 4 * i);
    lliB[i].SADDR = (uint32_t)src + (4095 * 4 * i);
    lliB[i].DADDR = (uint32_t)dst + (4095 * 4 * i);
    if(n <= 4095)
    {
       lliA[i].CTRLA = n | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_WORD;
       lliB[i].CTRLA = n | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_WORD;
       lliA[i].CTRLB =  DMAC_CTRLB_FC_MEM2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_INCREMENTING;
       lliB[i].CTRLB =  DMAC_CTRLB_FC_MEM2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_INCREMENTING;
       break;
    }
    else
    {
       lliA[i].CTRLA = 4095 | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_WORD;
       lliB[i].CTRLA = 4095 | DMAC_CTRLA_SRC_WIDTH_WORD | DMAC_CTRLA_DST_WIDTH_WORD;     
       lliA[i].CTRLB =  DMAC_CTRLB_FC_MEM2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_INCREMENTING | DMAC_CTRLB_IEN;
       lliB[i].CTRLB =  DMAC_CTRLB_FC_MEM2MEM_DMA_FC | DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_INCREMENTING | DMAC_CTRLB_IEN;
       n -= 4095;
       lliMax++; 
    }
  }
  // Setup the source and destination addresses and link the ping-pong  buffers
  lliA[0].SADDR = (uint32_t)src;
  lliA[0].DADDR = (uint32_t)dst;
  lliA[lliMax].DSCR =  (uint32_t)&lliB[0];
  lliB[0].SADDR = (uint32_t)src;
  lliB[0].DADDR = (uint32_t)dst;
  lliB[lliMax].DSCR =  (uint32_t)&lliA[0];
  // Setup the source registers when in compress mode. 
  if((ARBparms.CompressEnable) && (WorkingOrder == 2))
  {
     lliA[0].SADDR = (uint32_t)&(src[ARBparms.ppp * CHANS / 4]);   // Compress waveform
     lliB[0].SADDR = (uint32_t)src;                       // Normal waveform
  }
  if((ARBparms.CompressEnable) && ((WorkingOrder > 2) || (WorkingOrder == 0)))
  {
     lliA[0].SADDR = (uint32_t)&(src[ARBparms.ppp * CHANS / 4]);
     lliB[0].SADDR = (uint32_t)&(src[ARBparms.ppp * CHANS / 4]);
  }
  
  DMAC->DMAC_CH_NUM[DMAC_MEMCH].DMAC_SADDR = 0;
  DMAC->DMAC_CH_NUM[DMAC_MEMCH].DMAC_DADDR = 0;

  if((ARBparms.NumBuffers == 1) && (ARBparms.Mode == ARBmode)) lliB[0].CTRLA |= DMAC_CTRLA_DONE;

  DMAC->DMAC_CH_NUM[DMAC_MEMCH].DMAC_DSCR =  (uint32_t) & (lliA[0]);

  DMAC->DMAC_CH_NUM[DMAC_MEMCH].DMAC_CTRLB = DMAC_CTRLB_FC_MEM2MEM_DMA_FC |
      DMAC_CTRLB_SRC_INCR_INCREMENTING | DMAC_CTRLB_DST_INCR_INCREMENTING;

  DMAC->DMAC_CH_NUM[DMAC_MEMCH].DMAC_CFG = DMAC_CFG_SOD | DMAC_CFG_FIFOCFG_ASAP_CFG;

  DMAC->DMAC_EBCIER |= 1 << (DMAC_MEMCH);

  NVIC_ClearPendingIRQ(DMAC_IRQn);
  NVIC_EnableIRQ(DMAC_IRQn);

  (void)DMAC->DMAC_EBCISR;

  Bcount = 0;
  if(!NoTrigger) dmac_channel_enable(DMAC_MEMCH);
}

// ******* End DMA routines *********

// Resets the processor and peripherals
void Software_Reset()
{
  const int RSTC_KEY = 0xA5;
  RSTC->RSTC_CR = RSTC_CR_KEY(RSTC_KEY) | RSTC_CR_PROCRST | RSTC_CR_PERRST;
  while (true);
}

// The Write functions convert a voltage to DAC counts using the calibration in ARBparms.
// Do not disable interrupts around these, it stalls the DMA (removed 3/11/23).

// Sets the waveform voltage range (peak)
void WriteWFrange(float value)
{
  int i = ARBparms.GainDACm * value + ARBparms.GainDACb;
  if(i > 65535) i = 65535;
  if(i < 0) i = 0;
  AD5625(DACadr, DACrangeCH, i, 3);
}

// Sets the offset output voltage
void WriteWFoffset(float value)
{
  int i = ARBparms.OffsetDACm * value + ARBparms.OffsetDACb;
  if(i > 65535) i = 65535;
  if(i < 0) i = 0;
  AD5625(DACadr, DACoffsetCH, i, 3);
}

// Sets the aux (guard) output voltage
void WriteWFaux(float value)
{
  int i = ARBparms.AuxDACm * value + ARBparms.AuxDACb;
  if(i > 65535) i = 65535;
  if(i < 0) i = 0;
  AD5625(DACadr, DACauxCH, i, 3);
}

// Sets a board's DC bias using the processor DAC, board 0 or 1
void WriteBoardBias(int board, float value)
{
  int i = ARBparms.BiasCalM[board] * value + ARBparms.BiasCalB[board];
  if(i > 4095) i = 4095;
  if(i < 0) i = 0;
  analogWrite(DACaddBias[board],i);
  dacc_set_timing(DACC_INTERFACE, 0x01, 0, 0x10);
}

// Reads the +-12 V and +-50 V supplies into PS12v and PS50v
void MeasureVoltages(void)
{
  int   Counts;
  float Vp, Vn;

  // Calculate the +- 12 Volt supples from the readback channels
  analogRead(10);
  Counts = analogRead(10);
  Vp = ((float)Counts / 4095.0) * 19.8;
  analogRead(11);
  Counts = analogRead(11);
  Vn = 1.4 * Vp - (((float)Counts / 4095.0) * 3.3) * 2.4;
  PS12v[0] = Vp;
  PS12v[1] = Vn;
  // Calculate the +- 50 Volt supples from the readback channels
  analogRead(8);
  Counts = analogRead(8);
  Vp = ((float)Counts / 4095.0) * 80.85;
  analogRead(9);
  Counts = analogRead(9);
  Vn = 1.1064 * Vp - (((float)Counts / 4095.0) * 3.3) * 2.1064;
  PS50v[0] = Vp;
  PS50v[1] = Vn;
}

// AD5625 is a 4 channel DAC on Wire1.
//
// The range, offset and aux outputs are written from loop() but also from the DMA interrupt
// (range change on entering and leaving the alternate waveform), and a write is a multi byte
// I2C transaction. A write requested while one is in progress is saved here, and the write
// that is in progress sends it when it finishes. Only the newest value for each channel is kept.
// Interrupts are not disabled, doing that stalls the DMA.
static volatile int   ad5625Busy = 0;
static volatile int32_t ad5625Pending[4] = {-1, -1, -1, -1};   // -1 = nothing waiting
static volatile int8_t  ad5625PendingAdr = 0;

// Sends one write to the AD5625. adr = TWI address of device, chan = channel number,
// 0,1,2, or 3, val = binary value to output to the DAC, Cmd = AD5625 command (3 =
// write and update). Returns the status of the TWI transaction, 0 if no errors.
static int AD5625send(int8_t adr, uint8_t chan, uint16_t val, int8_t Cmd)
{
  Wire1.beginTransmission(adr);
  Wire1.write((Cmd << 3) | chan);
  Wire1.write((val >> 8) & 0xFF);
  Wire1.write(val & 0xFF);
  return Wire1.endTransmission();
}

static bool AD5625anyPending(void)
{
  for(int i = 0; i < 4; i++) if(ad5625Pending[i] >= 0) return true;
  return false;
}

// Outputs a value to the selected channel of the AD5625, see AD5625send(). Returns -1 if the
// bus is busy and the request could not be saved (only write and update requests are saved).
int AD5625(int8_t adr, uint8_t chan, uint16_t val)
{
  return AD5625(adr, chan, val, 0);
}

int AD5625(int8_t adr, uint8_t chan, uint16_t val, int8_t Cmd)
{
  int iStat;

  if(__sync_lock_test_and_set(&ad5625Busy, 1))
  {
    // Called from an interrupt while a write was in progress, save it
    if((Cmd != 3) || (chan > 3)) return -1;
    ad5625PendingAdr = adr;
    ad5625Pending[chan] = val;
    return 0;
  }
  iStat = AD5625send(adr, chan, val, Cmd);
  for(;;)
  {
    // Send anything saved by an interrupt while we were busy
    for(int i = 0; i < 4; i++)
    {
      int32_t v = __atomic_exchange_n(&ad5625Pending[i], (int32_t)-1, __ATOMIC_SEQ_CST);
      if(v >= 0) AD5625send(ad5625PendingAdr, i, v, 3);
    }
    __sync_lock_release(&ad5625Busy);
    // An interrupt may have saved a request after the check above, if so go around again
    if(!AD5625anyPending()) break;
    if(__sync_lock_test_and_set(&ad5625Busy, 1)) break;
  }
  return iStat;
}

// This function enables the external voltage reference in the AD5625.
// This was fixed 12/7/2022 in rev 1.24, it was enabling the internal reference with an
// external ref of 3 volts applied.
int AD5625_EnableRef(int8_t adr)
{
  Wire1.beginTransmission(adr);
  Wire1.write(0x38);
  Wire1.write(0);
  Wire1.write(0);    // was 1, 12/7/22
  return Wire1.endTransmission();
}

// Adds one byte to an 8 bit CRC, polynomial 0x1D
void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0)
    {
      *crc = ((*crc << 1) ^ generator);
    }
    else
    {
      *crc <<= 1;
    }
  }
}

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte crc = 0;

  for(int i=0; i<bsize; i++) ComputeCRCbyte(&crc, buf[i]);
  return crc;
}

// GETFLASH command. Sends the FLASH saved settings to the active serial port as an ASCII
// hex block. The MIPS host app is designed to use this function.
// The FLASH  buffer is assumed to
// be binary and its contents are converted to hex and send. after the ACK is sent.
// After the ACK, the files size is sent as an ascii string with a EOL termination, then
// the data block is sent as ascii hex followed by a EOL, and finally the 8 bit CRC is
// sent as a byte followed by a EOL.
void FLASHtoSerial(void)
{
  char sbuf[3];
  byte by;
  byte crc=0;

  SendACK;
  // Send the filesize
  serial->println(sizeof(ARB_PARMS));
  // Send the data as hex
  for(unsigned int i=0; i<sizeof(ARB_PARMS); i++)
  {
    by = dueFlashStorage.readAbs((uint32_t)NonVolStorage + i);
    ComputeCRCbyte(&crc,by);
    sprintf(sbuf,"%02x",by);
    serial->print(sbuf);
  }
  serial->println("");
  // Send the CRC then exit
  serial->println(crc);
}

// PUTFLASH command, the reverse of GETFLASH. Receives the byte count, the hex data block
// and the CRC, then writes the block to the settings area of FLASH. Use RESTORE to load it.
void SerialtoFLASH(void)
{
  char sbuf[3],*Token,c;
  int  numBytes = -1, val, crc;
  byte buf[NonVolSize];
  uint32_t start;

  SendACK;
  start = millis();
  // Receive the number of bytes
  while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
  sscanf(Token,"%d",&numBytes);
  GetToken(true); // Get the \n and toss
  if((numBytes <= 0) || (numBytes > NonVolSize))
  {
    serial->println("Invalid FLASH data size!");
    SendNAK;
    return;
  }
  // Read the data block
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
    sbuf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
    sbuf[1] = c;
    sbuf[2] = 0;
    sscanf(sbuf,"%x",&val);
    buf[i] = val;
  }
  start = millis();
  // Now we should see an EOL, \n
  while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
  if(c == '\n')
  {
    // Get CRC and test
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
    sscanf(Token,"%d",&crc);
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
    if((Token[0] == '\n') && (crc == ComputeCRC(buf,numBytes)))
    {
       noInterrupts();
       if(dueFlashStorage.writeAbs((uint32_t)NonVolStorage, buf, numBytes))
       {
         interrupts();
         serial->println("FLASH data written!");
         SendACK;
         return;
      }
      interrupts();
    }
  }
  serial->println("Unable to write to FLASH!");
  SendNAK;
  return;
TimeoutS2F:
  serial->println("\nFLASH data receive from host timedout!");
  SendNAK;
  return;
}

// The function will program the FLASH memory by receiving a file from the USB connected host. 
// The file must be sent in hex and use the following format:
// First the FLASH address in hex and file size, in bytes (decimal) are sent. If the file can
// be burned to FLASH an ACK is sent to the host otherwise a NAK is sent. The process stops
// if a NAK is sent. 
// If an ACK is sent to the host then the host will send the data for the body of the 
// file in hex. After all the data is sent then a 8 bit CRC is sent, in decimal. If the
// crc is correct and ACK is returned.
// Only FLASH bank 1 below the settings area can be written, the running program is in bank 0.
void ProgramFLASH(char * Faddress,char *Fsize)
{
  String sToken;
  uint32_t FlashAddress;
  int    numBytes,fi,val,tcrc;
  char   c,buf[3],*Token;
  byte   fbuf[256],crc=0;
  uint32_t start;

  FlashAddress = strtol(Faddress, 0, 16);
  sToken = Fsize;
  numBytes = sToken.toInt();
  if((numBytes <= 0) || (FlashAddress < IFLASH1_ADDR) || ((FlashAddress + numBytes) > (uint32_t)NonVolStorage))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  SendACK;
  fi = 0;
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[1] = c;
    buf[2] = 0;
    sscanf(buf,"%x",&val);
    fbuf[fi++] = val;
    ComputeCRCbyte(&crc,val);
    WDT_Restart(WDT);
    if(fi == 256)
    {
      fi = 0;
      // Write the block to FLASH
      noInterrupts();
      if(!dueFlashStorage.writeAbs((uint32_t)FlashAddress, fbuf, 256))
      {
        interrupts();
        serial->println("FLASH data write error!");
        SendNAK;
        return;
      }
      interrupts();
      FlashAddress += 256;
      serial->println("Next");
    }
  }
  // If fi is > 0 then write the last partial block to FLASH
  if(fi > 0)
  {
    noInterrupts();
    if(!dueFlashStorage.writeAbs((uint32_t)FlashAddress, fbuf, fi))
    {
      interrupts();
      serial->println("FLASH data write error!");
      SendNAK;
      return;
    }
    interrupts();
  }
  // Now we should see an EOL, \n
  start = millis();
  while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
  if(c == '\n')
  {
    // Get CRC and test, if ok exit else delete file and exit
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    sscanf(Token,"%d",&tcrc);
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    if((Token[0] == '\n') && (crc == tcrc)) 
    {
       serial->println("File received from host and written to FLASH.");
       SendACK;
       return;
    }
  }
  serial->println("\nError during file receive from host!");
  SendNAK;
  return;
TimeoutExit:
  serial->println("\nFile receive from host timedout!");
  SendNAK;
  return;
}

//
// SMC (static memory controller) commands. These tune the bus timing of the memory mapped
// DACs from the serial port, to help when developing a new hardware revision.
//

// SMCMODE command, sets the SMC mode register for chip select 0, value in hex
void SetSMCmode(char *val)
{
  int i;

  sscanf(val, "%x", &i);
  serial->println(i, 16);
  smc_set_mode(SMC, PARALLEL_CS_0, i);
}

// CYCLETM command, read and write cycle length
void SetCycleTiming(int val)
{
  Parallel.setCycleTiming(val, val);
}

// PULSETM command, read/write pulse width and chip select pulse width
void SetPulseTiming(int rw, int csrw)
{
  Parallel.setPulseTiming(rw, csrw, rw, csrw);
}

// ADDSETUP command, address setup time
void SetAddressSetup(int val)
{
  Parallel.setAddressSetupTiming(val, val, val, val);
}
