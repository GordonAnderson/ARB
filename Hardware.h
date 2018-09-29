#ifndef Hardware_h
#define Hardware_h

// DMA variables and structures
#define DMAC_MEMCH 3

extern volatile int Bcount;

typedef struct
{
  uint32_t  SADDR;
  uint32_t  DADDR;
  uint32_t  CTRLA;
  uint32_t  CTRLB;
  uint32_t  DSCR;
} LLI;    // DMA list structure

// IO pins
#define ARBsync      A0      // Used to signal a sync operation or trigger
#define CompressPin  22      // Used to flag compression mode
#define ExtClockSel  24      // Output to the CPLD is select the external clock source, the MIPS controller
                             // or the external clock source using the ext clock adapter
#define ClockMode    23      // Clock mode to the PLD, high = external clock

#define PowerEnable  25      // Enable the power supply, logic high disables

// 4 channel DAC constants
#define DACadr      0x12

#define DACrangeCH   0
#define DACoffsetCH  3
#define DACauxCH     2
#define DACrefCH     1

void DMAbuffer2DAC(uint32_t *dst, uint32_t *src, uint32_t n, bool NoTrigger = false);


#endif




