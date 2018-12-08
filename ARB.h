#ifndef ARB_H_
#define ARB_H_

// This is the max digitizer frequency in the ARB mode, this sets the max frequency.
#define MAXARBRATE 1280000

#define ESC   27
#define ENQ   5

// TWI commands and constants
#define TWI_SET_FREQ        0x01      // Set frequency, Hz, 16 bit unsigned word
#define TWI_SET_WAVEFORM    0x02      // Set waveform type, 8 bit unsigned type
#define TWI_SET_REF         0x03      // Set reference defines p-p voltage, 16 bit unsigned, 0 to 4095
#define TWI_SET_OFFSET      0x04      // Set offset, 16 bit unsigned, 0 to 4095
#define TWI_SET_DIR         0x05      // Set waveform direction, 8 bit unsigned. 0 = forward, 1 = reverse
#define TWI_SET_VECTOR      0x06      // Set the waveform vector format:
                                      // 8 bit offset into buffer, 8 bit number of bytes (29 max)
                                      // 8 bit unsigned values. 0 to 255
#define TWI_SET_ENABLE      0x07      // Enables or diables the ARB outputs. 0 = diaable, 1 = enables
#define TWI_SET_RANGE       0x08      // Sets the output range, 0 to 50, float
#define TWI_SET_OFFSETV     0x09      // Sets the output offset, -50 to 50, float
#define TWI_SET_AUX         0x0A      // Sets the aux output, -50 to 50, float
#define TWI_SET_BUFFER_LEN  0x0B      // Define the buffer length in samples, word
#define TWI_SET_NUM_BUFFER  0x0C      // Number of times to play buffer on each trigger, byte
#define TWI_SET_SET_BUFFER  0x0D      // Set all values in buffer, float
#define TWI_SET_SET_CHANNEL 0x0E      // Set a channel number in buffer, byte, float
#define TWI_SET_SET_CHN_RNG 0x0F      // Set a channel value over range in buffer, byte, word, work, float
#define TWI_SET_MODE        0x10      // Set ARB mode, 0 = TWAVEmode, 1 = ARBmode
#define TWI_SET_DAC         0x11      // Set DAC channel, 8 bit DAC number, float DAC level, -100 to 100
#define TWI_SET_ARB_INDEX   0x12      // Set ARB buffer index, 16 bit word that points to DAC location in buffer
#define TWI_SET_ARB_VECTOR  0x13      // Set ARB vector in buffer, format:
                                      // 8 bit channel number, 8 bit number of bytes (29 max)
                                      // 8 bit unsiged values. 0 to 255. The index will automatically advance to allow long buffer fills.
#define TWI_SET_SYNC_ENA    0x14      // Set external sync enable (true or false) 
#define TWI_SET_COMP_ENA    0x15      // Set compression enable (true or false) 
#define TWI_SET_COMP_ORDER  0x16      // Set compression order, byte value, 0 to 255 
#define TWI_SET_COMP_EXT    0x17      // This flag will enable the hardware line to define normal/compression modes
                                      // true will enable, when the hardware line is high then we are in compress mode.
#define TWI_SET_EXT_CLOCK   0x18      // This command enables external (off board) clock. accepts true or false, true = external clock
#define TWI_SET_BRD_BIAS    0x19      // This command sets the board bias voltage, 1 byte board number, 1 float with value
#define TWI_SET_PWR         0x20      // This command turns on and off power supply. accepts true or false, true = on
#define TWI_SET_TST_ENABLE  0x21      // This command enables voltage testing and shutdown. accepts true or false, true = enable
#define TWI_SET_CRAMP       0x22      // This command sets the compression order ramp variable, 16 bit int, -200 to 200
#define TWI_SET_CRAMPORDER  0x23      // This command sets the compression order ramp step size, 16 bit int, 1 to 200

// The following commands are used for the pulse sequence generator. The update
// commands send the data that is queued and loaded when the load updates command is sent to the ARB
#define TWI_UPDATE_AUX      0x24      // Updates the aux output, -50 to 50, float
#define TWI_UPDATE_BRD_BIAS 0x25      // This command sets the board bias voltage, 1 byte board number, 1 float with value
#define TWI_LOAD_UPDATES    0x26      // Updated values are loaded into the hardware via this command

#define TWI_SERIAL             0x27   // This command enables the TWI port to process serial commands
#define TWI_SET_COMP_ORDER_EX  0x28   // Set compression order, word value, 0 to 65535 

// The following command support amplitude and frequence scanning
#define TWI_SWPSTARTFREQ    0x29      // Define the sweep start frequency in Hz
#define TWI_SWPSTOPFREQ     0x30      // Define the sweep stop frequency in Hz
#define TWI_SWPSTARTV       0x31      // Define the sweep start voltage, p-p
#define TWI_SWPSTOPV        0x32      // Define the sweep stop voltage, p-p
#define TWI_SWPTIME         0x33      // Define the sweep time in seconds
#define TWI_SWPGO           0x34      // Start a sweep

#define TWI_SET_PPP         0x35      // Sets the points per period byte
#define TWI_SAVE            0x36      // Save settings to flash

#define TWI_SET_SEXTSRC     0x37      // Select the external clock source

#define TWI_READ_REQ_FREQ   0x81      // Returns requested frequency
#define TWI_READ_ACT_FREQ   0x82      // Returns actual frequency
#define TWI_READ_STATUS     0x83      // Returns status byte (ARB system status)
#define TWI_READ_PPP        0x84      // Returns points per period byte
#define TWI_READ_VERSION    0x85      // Returns a float that contains the ARB version number
#define TWI_READ_SWEEP_STATUS 0x86    // Returns sweep system status

// Waveform types, used in TWI command
#define TWI_WAVEFORM_SIN    0x01
#define TWI_WAVEFORM_RAMP   0x02
#define TWI_WAVEFORM_TRI    0x03
#define TWI_WAVEFORM_PULSE  0x04
#define TWI_WAVEFORM_ARB    0x05

enum SweepState
{
  SS_IDLE,
  SS_START,
  SS_STOP,
  SS_SWEEPING
};

typedef struct
{
   int            OrginalFreq;
   int            StartFreq;
   int            StopFreq;
   float          OrginalVoltage;
   float          StartVoltage;
   float          StopVoltage;
   float          SweepTime;          // In seconds
   unsigned int   SweepStartTime;     // In millisec counts
   unsigned int   CurrentSweepTime;   // In millisec counts
   SweepState     State;  
} FreqSweep;

enum ARBmodes
{
  TWAVEmode,
  ARBmode
};

#define MINDMAFREQ  0

typedef struct
{
  int       rev = 2;
  ARBmodes  Mode = TWAVEmode; 
  // Signature
  int     signature = 0xA55AE99E;    // This is used to validate a restore from flash function 
  // General parameters
  int     TWIarbAdd = 0x40;
  float   VoltageRange = 25;
  float   VoltageOffset = 0;
  float   VoltageAux = 0;
  float   Bias[2] = {0,0};
  bool    Enabled = false;            // Enables or disables all outputs
  int     RequestedFreq = 10000;      // Requested output frequency
  int     ActualFreq = 0;             // Actual output frequency
  bool    SyncEnable = false;         
  // Twave mode parameters
  bool    Direction = true;           // Twave direction
  int     wft = TWI_WAVEFORM_SIN;     // Twave wafefirm type
  // ARB mode parameters
  int     Bufferlength = 1024;
  int     NumBuffers = 1;
  // Hardware calibration parameters
  // DAC values are sent in percent of full scale, -100 to 100 integers. The following gains
  // and offsets allow trimming each channel. DAC counts = 127 + (% * gain) + offset.
  float   DACgains[8] = {1.23,1.23,1.23,1.23,1.23,1.23,1.23,1.23};
  float   DACoffsets[8] = {124,124,124,124,124,124,124,124};
  // Rev 2.0 ARB used a 4 channel DAC (3 volt reference)  for gain, offset, and aux (guard) channel output.
  // The channels are: 0 = gain, 1 = ref for offset and aux channel (set to fixed value, 1.25 volts)
  // 2 = Aux output, 3 = offset
  // The following calibration parameters convert range (0 to 50V) into DAC counts
  float   GainDACm = 377.55;          // slope
  float   GainDACb = 0;               // offset
  // The following calibration parameters convert offset and aux channel into DAC counts
  float   OffsetDACm = -348.04;       // slope
  float   OffsetDACb = 29663.50;      // offset
  float   AuxDACm = 346.27;           // slope
  float   AuxDACb = 29689.65;         // offset
  // Ref for offset and aux channel, fixed at 1.25 volts
  float   DACrefVoltage = 3.0;        // On board reference voltage
  float   AuxOffRef = 1.39;           // Offset and Aux out refrence voltage
  int     RefDAC = 30500;             // (AuxOffRef/DACrefVoltage) * 65535
  // Board bias cal parameters
  float   BiasCalM[2] = {10,10};
  float   BiasCalB[2] = {2800,2800};
  // Compression parameters
  bool    CompressEnable = false;
  bool    CompressHardware = false;
  int     Order = 2;
  int     CompressRamp = 0;
  int     CrampOrder = 1;
  // Timing selection parameters
  bool    CPLD = true;  // True if using CPLD
  // Point period, user adjustable
  int     ppp = 32;
  // Timing option for external clock opearation 8/21/18
  bool    XPtiming = true;
  // Set to true to enable ISR processing of compresion signal
  bool    ISRcompress = false;
} ARB_PARMS;

#endif /* ARB_H_ */





