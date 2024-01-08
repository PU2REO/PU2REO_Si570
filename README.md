# PU2REO_Si570
Library for the Si570 10 MHz to 1.4 GHz I2C programmable XO/VCXO, in the Arduino environment

# What sets this version apart from the others?
This version we just calculate internal Crystal Oscillator from initial frequency after a reset:
```cs
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

