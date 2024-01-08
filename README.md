# PU2REO_Si570
Library for the Si570 10 MHz to 1.4 GHz I2C programmable XO/VCXO, in the Arduino environment

# Public Methods
```cpp
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
```

# What sets this library apart from the others?
In this library, the frequency of the internal crystal oscillator (Si570_Data.FXtal) is calculated based on the initial frequency output after an internal reset of the Si570. In some versions I have seen, you have to enter this variable manually and therefore, the accuracy of the output frequency decreases.
```cpp
void PU2REO_Si570::Reset(void)
{
    // writes command to the Si570
    CmdBuffer[0] = SI570_REG_135;  // Position
    CmdBuffer[1] = 0x01;           // Reset command
    this->Write(CmdBuffer, 2);     // Send command string

    // get registers and calculate current frequency
    this->Get_Registers();
    Si570_Data.FXtal = (FOUT_START_UP * Si570_Data.N1 * Si570_Data.HSDiv) / Si570_Data.RFreq; //MHz
    Si570_Data.CurrentFreq = FOUT_START_UP;
    Si570_Data.CurrentRFreq = Si570_Data.RFreq;
}
```
Another point is the possibility of reading/writing any of the Si570 internal registers at any time.

# A wave of hand to:
Gerrit Polder, PA3BYA, the one who made it first for mbed around 2010.
