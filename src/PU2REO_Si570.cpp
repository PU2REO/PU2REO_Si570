//******************************************************************************
//**
//** Project......: Arduino Si570 library.
//**                Copyright (c) 2015-2024, PU2REO
//**                Based on mbed SI570 Library, by Gerrit Polder, PA3BYA, 2010
//**
//** Platform.....: Arduino (Can afford anyone I2C compatible, with minor changes)
//**
//** Licence......: Permission is hereby granted, free of charge, to any person
//** obtaining a copy of this software and associated documentation files
//** (the "Software"), to deal in the Software without restriction,
//** including without limitation the rights to use, copy,
//** modify, merge, publish, distribute, sublicense, and/or sell
//** copies of the Software, and to permit persons to whom the Software is
//** furnished to do so, subject to the following conditions:
//**
//** The above copyright notice and this permission notice shall be included in
//** all copies or substantial portions of the Software.
//**
//** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//** OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//** THE SOFTWARE.
//**
//**
//** Programmer...: PU2REO
//**                I'd like to thank Gerrit Polder, PA3BYA, for his arduous work.
//**
//** Description..: Calculations for the Si570 chip and Si570 program algorithme.
//**                Calculate fast and precise code to program the Si570 device.
//**
//** History......: v1.0.0 - 07/01/2024: Initial version - based on my PICPLCv6 Board (18F4520) version
//******************************************************************************

#include <Math.h>
#include "PU2REO_Si570.h"

//******************************************************************************
//**                          Si570 Functions                                 **
//******************************************************************************

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570
// Description:  Creates a new instance for the device
// Paramaters:   None
// Output:       None
//------------------------------------------------------------------------------
PU2REO_Si570::PU2REO_Si570() 
{
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Init
// Description:  Initiates device Si570 on I2C line
// Paramaters:   None
// Output:       None
//------------------------------------------------------------------------------
void PU2REO_Si570::Init() 
{
    // start I2C comunnications
    Wire.begin();

    // reset to factory defaults
    this->Reset();
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Reset
// Description:  Reacall all factory values from Si570 NVM into RAM
// Paramaters:   None
// Output:       None
//------------------------------------------------------------------------------
void PU2REO_Si570::Reset(void)
{
    // writes command to the Si570
    CmdBuffer[0] = SI570_REG_135;  // Position
    CmdBuffer[1] = 0x01;           // Data
    this->Write(CmdBuffer, 2);     // Send command string

    // get registers and calculate current frequency
    this->Get_Registers();
    Si570_Data.FXtal = (FOUT_START_UP * Si570_Data.N1 * Si570_Data.HSDiv) / Si570_Data.RFreq; //MHz
    Si570_Data.CurrentFreq = FOUT_START_UP;
    Si570_Data.CurrentRFreq = Si570_Data.RFreq;
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Get_Registers
// Description:  Retrieve registers contents from Si570 into Si570 struct
// Paramaters:   None
// Output:       None
//------------------------------------------------------------------------------
void PU2REO_Si570::Get_Registers(void)
{
    // Read 6 positions, starting on REG7
    this->Read(SI570_REG_7, DataBuffer, 6);

     // HS_DIV conversion
    Si570_Data.HSDiv = ((DataBuffer[0] & 0xE0) >> 5) + 4; // get reg 7 bits 5, 6, 7
    // hsdiv's value could be verified here to ensure that it is one
    // of the valid HS_DIV values from the datasheet.
    // n1 conversion
    Si570_Data.N1 = (uint16_t)((( DataBuffer[0] & 0x1F ) << 2 ) + // get reg 7 bits 0 to 4
                          (( DataBuffer[1] & 0xC0 ) >> 6 ));  // add with reg 8 bits 7 and 8

    // Check N1 constraints
    if (Si570_Data.N1 == 0) 
    {
        Si570_Data.N1 = 1;
    } 
    else if (Si570_Data.N1 & 1 != 0) 
    {
        // add one to an odd number
        Si570_Data.N1 = Si570_Data.N1 + 1;
    }

    // fraction part of the frequency
    frac_bits = (( DataBuffer[2] & 0xF )   * POW_2_24);
    frac_bits = frac_bits + (DataBuffer[3] * POW_2_16);
    frac_bits = frac_bits + (DataBuffer[4] * POW_2_8);
    frac_bits = frac_bits + DataBuffer[5];

    // RFreq calculation
    Si570_Data.RFreq = (float)frac_bits;
    Si570_Data.RFreq = Si570_Data.RFreq / POW_2_28;
    Si570_Data.RFreq = Si570_Data.RFreq + ((( DataBuffer[1] & 0x3F ) << 4 ) + (( DataBuffer[2] & 0xF0 ) >> 4 ));
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Get_Frequency
// Description:  Retrieve actual output frequency
// Paramaters:   None
// Output:       Actual output frequency in [MHz]
//------------------------------------------------------------------------------
float PU2REO_Si570::Get_Frequency(void)
{
    // get actual frequency
    this->Get_Registers();
    return (Si570_Data.RFreq * Si570_Data.FXtal) / (Si570_Data.HSDiv * Si570_Data.N1);
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Get_RFreq
// Description:  Retrieve RFreq register value
// Paramaters:   None
// Output:       Actual RFreq Register
//------------------------------------------------------------------------------
float PU2REO_Si570::Get_RFreq(void)
{
    // get actual RFreq
    this->Get_Registers();
    return Si570_Data.RFreq;
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Get_N1
// Description:  Retrieve slow speed divider value
// Paramaters:   None
// Output:       Actual N1 Register
//------------------------------------------------------------------------------
uint16_t PU2REO_Si570::Get_N1(void)
{
    // get actual N1
    this->Get_Registers();
    return Si570_Data.N1;
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Get_HSDiv
// Description:  Retrieve High Speed divider value
// Paramaters:   None
// Output:       Actual HSDiv Register
//------------------------------------------------------------------------------
int16_t PU2REO_Si570::Get_HSdiv(void)
{
    // get actual HSDiv
    this->Get_Registers();
    return Si570_Data.HSDiv;
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Set_Frequency
// Description:  Sets new oscilating frequency for the Si570 unit, considering
//               small (<= 3500 [ppm]) and large changes (> 3500 [ppm])
// Paramaters:   Frequency [MHz]
// Output:       Non-zero, if an error occurs
//------------------------------------------------------------------------------
int16_t PU2REO_Si570::Set_Frequency(float frequency)
{
    int16_t err;
    
    // calculates difference between old and new frequency
    float diff = 1000000 * (abs(frequency - Si570_Data.CurrentFreq) / Si570_Data.CurrentFreq);

    // check if it is a small change (< 3500 ppm)
    if (diff < PPM)
    {
        // small change procedeure
        err = this->Set_Frequency_Small_Change(frequency);
    } 
    else 
    {
        // large change procedeure
        err = this->Set_Frequency_Large_Change(frequency);
    }
    return err;
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Set_Frequency_Small_Change
// Description:  Sets new oscilating frequency for the Si570 unit, considering
//               ONLY small (<= 3500 [ppm]) changes
// Paramaters:   Frequency [MHz]
// Output:       Non-zero, if an error occurs
//------------------------------------------------------------------------------
int16_t PU2REO_Si570::Set_Frequency_Small_Change(float frequency)
{
    uint8_t    reg135, counter, i;
    uint16_t   whole;
    uint8_t    reg[6];

    Si570_Data.RFreq = Si570_Data.CurrentRFreq * frequency / Si570_Data.CurrentFreq;

    // Read 1 position, starting on REG8
    this->Read(SI570_REG_8, DataBuffer, 1);
    reg[1] = DataBuffer[0];
    reg[2] = 0;

    // convert new RFREQ to the binary representation
    // separate the integer part
    whole = floor(Si570_Data.RFreq);
    
    // get the binary representation of the fractional part
    frac_bits = (uint32_t)floor((Si570_Data.RFreq - whole) * POW_2_28);
    
    // set reg 12 to 10 making frac_bits smaller by
    // shifting off the last 8 bits everytime
    for (counter=5; counter >=3; counter--) 
    {
        reg[counter] = frac_bits & 0xFF;
        frac_bits = frac_bits >> 8;
    }
    // set the last 4 bits of the fractional portion in reg 9
    reg[2] = this->SetBits(reg[2], 0xF0, (frac_bits & 0xF));
    
    // set the integer portion of RFREQ across reg 8 and 9
    reg[2] = this->SetBits(reg[2], 0x0F, (whole & 0xF) << 4);
    reg[1] = this->SetBits(reg[1], 0xC0, (whole >> 4) & 0x3F);

    // Load the new frequency
    // Read 1 positions, starting on REG135
    this->Read(SI570_REG_135, DataBuffer, 1);
    reg135 = DataBuffer[0];

    // set the Freeze M bit in that register
    DataBuffer[0] = SI570_REG_135;
    DataBuffer[1] = reg135 | 0x20;
    this->Write(DataBuffer, 2);

    // load the new values into the device at registers 8 to 12;
    DataBuffer[0] = SI570_REG_8;
    for (i=1; i<6; i++) 
    {
        DataBuffer[i] = reg[i];
    }
    this->Write(DataBuffer, 6);

    // Read 1 position, starting on REG135
    this->Read(SI570_REG_135, DataBuffer, 1);
    reg135 = DataBuffer[0];

    // clear the M bit in that register
    DataBuffer[0] = SI570_REG_135;
    DataBuffer[1] = reg135 & 0xDF;
    this->Write(DataBuffer, 2);

    return ERROR_NO_ERROR;
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Set_Frequency_Large_Change
// Description:  Sets new oscilating frequency for the Si570 unit, considering
//               ONLY large (> 3500 [ppm]) changes
// Paramaters:   Frequency [MHz]
// Output:       Non-zero, if an error occurs
//------------------------------------------------------------------------------
int16_t PU2REO_Si570::Set_Frequency_Large_Change(float frequency)
{
    const uint16_t HS_DIV[6] = {11, 9, 7, 6, 5, 4};
    uint8_t        counter, reg137, ValidCombo, i;
    uint16_t       divider_max, curr_div, whole;
    float          curr_n1, n1_tmp;
    uint8_t        buf[8];
    uint8_t        reg[6];

    // find dividers (get the max and min divider range for the HS_DIV and N1 combo)
    divider_max = floor(FDCO_MAX / frequency);          // floorf for SDCC
    curr_div    = ceil(FDCO_MIN / frequency);           // ceilf for SDCC
    ValidCombo  = 0;
    while ((curr_div <= divider_max) && (!ValidCombo))
    {
        //check all the HS_DIV values with the next curr_div
        for (counter=0; counter<6; counter++) 
        {
            // get the next possible n1 value
            Si570_Data.HSDiv = HS_DIV[counter];
            curr_n1 = (curr_div * 1.0) / (Si570_Data.HSDiv * 1.0);
            // determine if curr_n1 is an integer and an even number or one
            // then it will be a valid divider option for the new frequency
            n1_tmp = floor(curr_n1);
            n1_tmp = curr_n1 - n1_tmp;
            if (n1_tmp == 0.0) 
            {
                //then curr_n1 is an integer
                Si570_Data.N1 = (uint16_t) curr_n1;
                if (((Si570_Data.N1 == 1) || ((Si570_Data.N1 & 1) == 0)) && (Si570_Data.N1 <= 128))
                {
                    // then the calculated N1 is either 1 or an even number
                    ValidCombo = 1;
                }
            }
            if (ValidCombo == 1) break;
        }
        if (ValidCombo == 1) break;
        //increment curr_div to find the next divider
        //since the current one was not valid
        curr_div = curr_div + 1;
    }

    // if(ValidCombo == 0) at this point then there's an error
    // in the calculation. Check if the provided frequencies
    // are valid.
    if (ValidCombo == 0)
        return ERROR_NO_VALID_COMBO;

    // calculates new RFreq
    Si570_Data.RFreq = (frequency * (float)Si570_Data.N1 * (float)Si570_Data.HSDiv) / Si570_Data.FXtal;

    // new HS_DIV conversion
    Si570_Data.HSDiv = Si570_Data.HSDiv - 4;

    //reset this memory
    reg[0] = 0;
    //set the top 3 bits of reg 13
    reg[0] = (Si570_Data.HSDiv << 5);
    // convert new N1 to the binary representation
    if (Si570_Data.N1 == 1)
    {
       Si570_Data.N1 = 0;
    }
    else if ((Si570_Data.N1 & 1) == 0)
    {
       // if Si570_Data.N1 is even, subtract one
       Si570_Data.N1 = Si570_Data.N1 - 1;
    }
    
    // set reg 7 bits 0 to 4
    reg[0] = this->SetBits(reg[0], 0xE0, Si570_Data.N1 >> 2);
    
    // set reg 8 bits 6 and 7
    reg[1] = (Si570_Data.N1 & 3) << 6;

    // convert new RFREQ to the binary representation
    // separate the integer part
    whole = floor(Si570_Data.RFreq);
    
    // get the binary representation of the fractional part
    frac_bits = floor((Si570_Data.RFreq - (float)whole) * (float)POW_2_28);

    // set reg 12 to 10 making frac_bits smaller by
    // shifting off the last 8 bits everytime
    for (counter=5; counter >=3; counter--) 
    {
        reg[counter] = frac_bits & 0xFF;
        frac_bits = frac_bits >> 8;
    }
    
    // set the last 4 bits of the fractional portion in reg 9
    reg[2] = this->SetBits(reg[2], 0xF0, (frac_bits & 0xF));
    
    // set the integer portion of RFREQ across reg 8 and 9
    reg[2] = this->SetBits(reg[2], 0x0F, (whole & 0xF) << 4);
    reg[1] = this->SetBits(reg[1], 0xC0, (whole >> 4) & 0x3F);

    // Load the new frequency
    // get the current state of register 137
    // Read 1 position, starting on REG137
    this->Read(SI570_REG_137, buf, 1);
    reg137 = buf[0];

    // set the Freeze DCO bit in that register
    buf[0] = SI570_REG_137;
    buf[1] = reg137 | 0x10;
    this->Write(buf, 2);

    // load the new values into the device at registers 7 to 12;
    buf[0] = SI570_REG_7;
    for (i=1; i<7; i++)
    {
        buf[i] = reg[i-1];
    }
    this->Write(buf, 7);

    // Read 1 position, starting on REG137
    this->Read(SI570_REG_137, buf, 1);
    reg137 = buf[0];

    // clear the FZ_DCO bit in that register
    buf[0] = SI570_REG_137;
    buf[1] = reg137 & 0xEF;
    this->Write(buf, 2);

    // set the NewFreq bit, bit will clear itself once the device is ready
    buf[0] = SI570_REG_135;
    buf[1] = 0x40;
    this->Write(buf, 2);

    // adjusts new values
    Si570_Data.CurrentFreq = frequency;
    Si570_Data.CurrentRFreq = Si570_Data.RFreq;
    return ERROR_NO_ERROR;
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::SetBits
// Description:  Change bits on the Si570 unit registers
// Paramaters:   Original Byte, Byte mask, New Value
// Output:       Modified Byte
//------------------------------------------------------------------------------
uint8_t PU2REO_Si570::SetBits(uint8_t original, uint8_t reset_mask, uint8_t new_val)
{
    // set bits in a byte, accordingly to the mask
    return (( original & reset_mask ) | new_val );
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Read
// Description:  Reads values from Si570 registers into uProcessor memory buffer
// Paramaters:   Initial Adress, Memory Buffer, Amount of Bytes
// Output:       None (values are stored in the memory buffer, passes as reference)
//------------------------------------------------------------------------------
void PU2REO_Si570::Read(uint8_t init_reg, uint8_t *buffer, uint8_t size)
{
    uint8_t i;

    // initializes Si570 I2C communications
    Wire.beginTransmission(SI570_ADDRESS);
    Wire.write(init_reg);

    int error = Wire.endTransmission();
    if (error != 0) 
    {
      return;
    }

    int len = Wire.requestFrom(SI570_ADDRESS, size);
    for (int i = 0; i < len && Wire.available(); i++)
    {
      buffer[i] = Wire.read();
    }
}

//------------------------------------------------------------------------------
// Procedure:    PU2REO_Si570::Write
// Description:  Writes values from uProcessor memory buffer to Si570 registers,
//               starting on the address of the Si570 unit denoted by buffer[0]
// Paramaters:   Memory Buffer, Amount of Bytes
// Output:       None
//------------------------------------------------------------------------------
void PU2REO_Si570::Write(uint8_t *buffer, uint8_t size)
{
    // initializes Si570 I2C communications
    Wire.beginTransmission(SI570_ADDRESS);
    
    // write data
    Wire.write(buffer, size);  // buffer comes with register address and data from the caller
    
    // end communication and check for errors
    int error = Wire.endTransmission();
    if (error != 0) {
      return;
    }
    return;
}
