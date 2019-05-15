#include "hardware.h"

// Two structure arrays used to enable continous DMA of the buffer.
#define NumLLI  4
volatile LLI lliA[NumLLI];   
volatile LLI lliB[NumLLI]; 
volatile int lliMax = 0;
volatile int Bcount;    // Used to count the number of buffers transfered to the DACs

//
// DMA routines
//

/** Disable DMA Controller. */
inline void dmac_disable()
{
  DMAC->DMAC_EN &= (~DMAC_EN_ENABLE);
}
/** Enable DMA Controller. */
inline void dmac_enable()
{
  DMAC->DMAC_EN = DMAC_EN_ENABLE;
}
/** Disable DMA Channel. */
inline void dmac_channel_disable(uint32_t ul_num)
{
  DMAC->DMAC_CHDR = DMAC_CHDR_DIS0 << ul_num;
}
/** Enable DMA Channel. */
inline void dmac_channel_enable(uint32_t ul_num)
{
  DMAC->DMAC_CHER = DMAC_CHER_ENA0 << ul_num;
}
/** Suspend DMA Channel. */
inline void dmac_channel_suspend(uint32_t ul_num)
{
  DMAC->DMAC_CHER = DMAC_CHER_SUSP0 << ul_num;
}
/** Poll for FIFO empty. */
inline bool dmac_channel_fifo_empty(uint32_t ul_num)
{
  return (DMAC->DMAC_CHSR & (DMAC_CHSR_EMPT0 << ul_num)) ? false : true;
}
/** Poll for transfer complete. */
inline bool dmac_channel_transfer_done(uint32_t ul_num)
{
  return (DMAC->DMAC_CHSR & (DMAC_CHSR_ENA0 << ul_num)) ? false : true;
}

// DMA ISR is used to reenable the completed buffers. This interrupt fires when
// a buffer completes and this code clears the done bit to keep the process
// going.
// This routine only files in the DMA mode
void DMAC_Handler(void)
{
  static uint32_t i;

if(Bcount == -1) 
{
  // Ignore any pending buffer loaded interrupts.
  Bcount = 2;
  return;
}
  i = DMAC->DMAC_EBCISR;
  Bcount++;
  if(ARBparms.Mode == ARBmode) if((Bcount >= ARBparms.NumBuffers-1) && (ARBparms.NumBuffers != 0))
  {
//    dmac_channel_disable(DMAC_MEMCH);
    return;
  }
//  if((ARBparms.Mode == TWAVEmode) && (ARBparms.CompressEnable)  && (ARBparms.Order > 1))
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
    lliA[0].SADDR = (uint32_t)&(buffer[0]);
    lliB[0].SADDR = (uint32_t)&(buffer[0]);    
  }
  for(i=0;i<NumLLI;i++)
  {
     lliA[i].CTRLA &= ~DMAC_CTRLA_DONE;
     lliB[i].CTRLA &= ~DMAC_CTRLA_DONE;    
  }
}

// This function restarts the DMA buffer transfer. This function will stop the DMA process, reset the done
// bits and then restart the DMA process. This is used in the Twave mode to sync the waveform generation
void DMArestart(void)
{
  uint32_t i;

  // Reset the Cramp parameters
  WorkingOrder = ARBparms.Order;
  CrampCounter = 0;
  // Stop the DMA process
  dmac_channel_disable(DMAC_MEMCH);
  while(!dmac_channel_transfer_done(DMAC_MEMCH));
//  DMAC->DMAC_EBCIER = 0; //11-3-18
  i = DMAC->DMAC_EBCIER; //11-3-18
  DMAbuffer2DAC((uint32_t *)DACADD, buffer, ARBparms.ppp * NP * CHANS / 4);
  Bcount = 2;
  if(dmac_channel_fifo_empty(DMAC_MEMCH)) Bcount = -1;    // This will only be true when there is a pending interrupt, can't clear it!
                                                          // Bcount is always -1 due to semi-colon, 11-3-18 removed semi-colon
  return;
}

// This function starts the DMA continous transfer.
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
    if(i < NumLLI)
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
//  if((ARBparms.CompressEnable) && (ARBparms.Order > 2))
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

  i = DMAC->DMAC_EBCISR;

  Bcount = 0;
  if(!NoTrigger) dmac_channel_enable(DMAC_MEMCH);
}

// ******* End DMA routines *********

void Software_Reset()
{
  //============================================================================================
  //   führt ein Reset des Arduino DUE aus...
  //
  //   Parameter: keine
  //   Rueckgabe: keine
  //============================================================================================
  const int RSTC_KEY = 0xA5;
  RSTC->RSTC_CR = RSTC_CR_KEY(RSTC_KEY) | RSTC_CR_PROCRST | RSTC_CR_PERRST;
  while (true);
}

void WriteWFrange(float value)
{
  int i = ARBparms.GainDACm * value + ARBparms.GainDACb;
  if(i > 65565) i = 65565;
  if(i < 0) i = 0;
  AD5625(DACadr,DACrangeCH,i,3);
}

void WriteWFoffset(float value)
{
  int i = ARBparms.OffsetDACm * value + ARBparms.OffsetDACb;
  if(i > 65565) i = 65565;
  if(i < 0) i = 0;
  AD5625(DACadr,DACoffsetCH,i,3);
}

void WriteWFaux(float value)
{
  int i = ARBparms.AuxDACm * value + ARBparms.AuxDACb;
  if(i > 65565) i = 65565;
  if(i < 0) i = 0;
  AD5625(DACadr,DACauxCH,i,3);
}

void WriteBoardBias(int board, float value)
{
  int i = ARBparms.BiasCalM[board] * value + ARBparms.BiasCalB[board];
  if(i > 4095) i = 4095;
  if(i < 0) i = 0;
  analogWrite(DACaddBias[board],i);
  dacc_set_timing(DACC_INTERFACE, 0x01, 0, 0x10);
}

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

// AD5625 is a 4 channel DAC.
//
// This function outputs a value to the selected channel.
// adr = TWI address of device
// chan = channel number, 0,1,2, or 3
// val = binary value to output to the DAC
//
// Return the status of the TWI transaction, 0 if no errors.
int AD5625(int8_t adr, uint8_t chan, uint16_t val)
{
  AD5625(adr, chan, val, 0);
}

int AD5625(int8_t adr, uint8_t chan, uint16_t val,int8_t Cmd)
{
  int iStat;

  Wire1.beginTransmission(adr);
  Wire1.write((Cmd << 3) | chan);
  //    if(chan <= 3) val <<= 4;
  Wire1.write((val >> 8) & 0xFF);
  Wire1.write(val & 0xFF);
  {
    iStat = Wire1.endTransmission();
  }
  return (iStat);
}

// This function enables the internal voltage reference in the
// AD5625
int AD5625_EnableRef(int8_t adr)
{
  int iStat;

  Wire1.beginTransmission(adr);
  Wire1.write(0x38);
  Wire1.write(0);
  Wire1.write(1);
  iStat = Wire1.endTransmission();
  return (iStat);
}

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
  byte generator = 0x1D;
  byte crc = 0;

  for(int i=0; i<bsize; i++)
  {
    crc ^= buf[i];
    for(int j=0; j<8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = ((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// This function will send the FLASH saved setting to the serial port using the 
// active serial port. The data is converted to an ASCII hex block and sent using
// the protocol described above. The MIPS host app is designed to use this function.
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
  for(int i=0; i<sizeof(ARB_PARMS); i++)
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

void SerialtoFLASH(void)
{
  char sbuf[3],*Token,c;
  int  addr,board,numBytes,val,crc;
  byte *buf;
  uint32_t start;

  SendACK;
  start = millis();
  // Receive the number of bytes
  while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutS2F; }
  sscanf(Token,"%d",&numBytes); 
  GetToken(true); // Get the \n and toss
  buf = new byte[numBytes];
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
  return;
TimeoutS2F:
  serial->println("\nFLASH data receive from host timedout!");
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
void ProgramFLASH(char * Faddress,char *Fsize)
{
  String sToken;
  uint32_t FlashAddress;
  int    numBytes,fi,val,tcrc;
  char   c,buf[3],*Token;
  byte   fbuf[256],b,crc=0;
  uint32_t start;
  
  FlashAddress = strtol(Faddress, 0, 16);
  sToken = Fsize;
  numBytes = sToken.toInt();
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

// SMC commands

void SetSMCmode(char *val)
{
   int i;
   
   sscanf(val,"%x",&i);  
   serial->println(i,16);
   smc_set_mode(SMC,PARALLEL_CS_0,i);
}
void SetCycleTiming(int val)
{
    Parallel.setCycleTiming(val, val);  

}
void SetPulseTiming(int rw, int csrw)
{
    Parallel.setPulseTiming(rw, csrw, rw, csrw); 
}
void SetAddressSetup(int val)
{
    Parallel.setAddressSetupTiming(val, val, val, val);
}
