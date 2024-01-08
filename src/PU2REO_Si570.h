//******************************************************************************
//                        General Constants Definitions                       **
//******************************************************************************
#include <Wire.h>

//******************************************************************************
//**                         Si570 Definitions                                **
//******************************************************************************

#define ERROR_NO_ERROR                       0               // No Error detected
#define ERROR_NO_VALID_COMBO                -1               // No valid dividers found for the frequency passed

// these must be floating point number especially 2^28 so that
// there is enough memory to use them in the calculation
#define POW_2_8                        256.000               // 2^8
#define POW_2_16                     65536.000               // 2^16
#define POW_2_24                  16777216.000               // 2^24
#define POW_2_28                 268435456.000               // 2^28
#define FOUT_START_UP                   10.000               // MHz
#define FDCO_MAX                      5670.000               // MHz - From Si570 DataSheet. DO NOT CHANGE!!!
#define FDCO_MIN                      4850.000               // MHz - From Si570 DataSheet. DO NOT CHANGE!!!
#define DIVIDER_FACTOR                  32.000               // Divider factor being used
#define PPM                               3500               // +/- max ppm from center frequency

// adresses & directions
#define SI570_ADDRESS                   0x55
#define SI570_READ                      0x01
#define SI570_WRITE                     0x00

// adresses of the Si570 registers
#define SI570_REG_7                     0x07
#define SI570_REG_8                     0x08
#define SI570_REG_135                   0x87
#define SI570_REG_137                   0x89

// buffers size
#define SI570_MAX_CMD_SIZE              4
#define SI570_MAX_BUF_SIZE              8

//******************************************************************************
//**                               Si570                                      **
//******************************************************************************

//------------------------------------------------------------------------------
// structs
//------------------------------------------------------------------------------
typedef struct
{
    uint16_t  N1;                                  // N1 Output Divider
    uint16_t  HSDiv;                               // High Speed Divider
    float     RFreq;                               // High-resolution 38-bit fractional multiplier
    float     FXtal;                               // Internal fixed-frequency crystal
    float     CurrentFreq;                         // Si570 Actual operating frequency.
    float     CurrentRFreq;                        // Actual High-resolution 38-bit fractional multiplier
} Si570_st;

class PU2REO_Si570
{
//------------------------------------------------------------------------------
// Prototypes
//------------------------------------------------------------------------------
public:
            PU2REO_Si570(void);
  void      Init(void);
  void      Reset(void);
  float     Get_Frequency(void);
  float     Get_RFreq(void);
  uint16_t  Get_N1(void);
  int16_t   Get_HSdiv(void);
  void      Get_Registers(void);
  int16_t   Set_Frequency(float currentFrequency);
  int16_t   Set_Frequency_Small_Change(float currentFrequency);
  int16_t   Set_Frequency_Large_Change(float currentFrequency);
  uint8_t   SetBits(uint8_t original, uint8_t reset_mask, uint8_t new_val);
  void      Read(uint8_t init_reg, uint8_t *buffer, uint8_t size);
  void      Write(uint8_t *buffer, uint8_t size);

//------------------------------------------------------------------------------
// Global variables
//------------------------------------------------------------------------------

int32_t                     frac_bits;
uint8_t                     CmdBuffer[SI570_MAX_CMD_SIZE];
uint8_t                     DataBuffer[SI570_MAX_BUF_SIZE];
Si570_st                    Si570_Data;
};
