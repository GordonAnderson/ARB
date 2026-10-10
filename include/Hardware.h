//
// Hardware.h
//
// Pin assignments, AD5625 DAC channels and the DMA controller helpers.
//
#ifndef Hardware_h
#define Hardware_h

#include <Arduino.h>

// DMA variables and structures
#define DMAC_MEMCH 3

extern volatile int Bcount;

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

typedef struct
{
  uint32_t  SADDR;
  uint32_t  DADDR;
  uint32_t  CTRLA;
  uint32_t  CTRLB;
  uint32_t  DSCR;
} LLI;    // DMA list structure

// IO pins

// The two hardware trigger lines. Sync is on line 1 and compress on line 2 by default, the
// SSYNCLINE and SCOMPLINE commands can swap them.
#define ARBline1     A0      // Used to signal a sync operation or trigger
#define ARBline2     22      // Used to flag compression mode

extern int ARBsync;          // Pin currently used for sync, ARBline1 by default
extern int CompressPin;      // Pin currently used for compress, ARBline2 by default

#define ExtClockSel  24      // Output to the CPLD is select the external clock source, the MIPS controller
                             // or the external clock source using the ext clock adapter
#define ClockMode    23      // Clock mode to the PLD, high = external clock

#define PowerEnable  25      // Enable the power supply, logic high disables

// AD5625 4 channel DAC constants. Sets the range, offset and aux outputs and their reference.
#define DACadr      0x12

#define DACrangeCH   0
#define DACoffsetCH  3
#define DACauxCH     2
#define DACrefCH     1

void DMAbuffer2DAC(uint32_t *dst, uint32_t *src, uint32_t n, bool NoTrigger = false);


#endif
