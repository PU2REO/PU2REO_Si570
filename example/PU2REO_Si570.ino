#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <RotaryEncoder.h> 
#include <ButtonV2.h>
#include <PU2REO_Si570.h>
// #include "debug.h"

///////////////////////////////
// Encoder Definitions
///////////////////////////////
#define ENC_KEY_PRESS_TIME                        500                   // minimum time to set button "Holding" state in [ms]

///////////////////////////////
// Digital Inputs
///////////////////////////////
#define DI_ENC_KEY                                A1                    // Digital Input - Encoder Key - Active Low
#define DI_ENC_PIN1                               A2                    // Digital Input - Encoder Pin 2
#define DI_ENC_PIN2                               A3                    // Digital Input - Encoder Pin 1

///////////////////////////////
// Timer definitions
///////////////////////////////
#define MAX_NR_TIMER_DOWN                          1
#define TIMERDOWN_BACKLIGHT                        0 
#define BACKLIGHT_DELAY                            1500 

///////////////////////////////
// Si570 definitions
///////////////////////////////
#define MIN_SI570_FREQ                             2.00     // [MHz]
#define MAX_SI570_FREQ                             1500.00  // [MHz]

///////////////////////////////
// Global definitions
///////////////////////////////
#define MIN_COEF_INDEX                             0
#define MAX_COEF_INDEX                             6

///////////////////////////////
// structures
///////////////////////////////
typedef struct
{
    uint16_t     Counter;                                               // timer down counter   
} TimerDown_typ;

typedef struct
{
    float        DispFreq;                                              // Frequency to be displayed
    int8_t       CoefIndex;                                             // CoefTable Index
} VFO_typ;

///////////////////////////////
// Global variables
///////////////////////////////
PU2REO_Si570                  Si570;                                                                // Si570 device creation
LiquidCrystal_I2C             lcd(0x27,20,4);                                                       // set the LCD address to 0x27 for a 16 chars and 2 line display
RotaryEncoder                 encoder(DI_ENC_PIN1, DI_ENC_PIN2, RotaryEncoder::LatchMode::FOUR3);   // define Encoder control variable - Library v1.5.0
ButtonV2                      EncKey;                                                               // define Improved Button control variable
VFO_typ                       VFO;                                                                  // define VFO control variable
volatile TimerDown_typ        TimerDown[MAX_NR_TIMER_DOWN];                                         // timer vector
const float                   CoefTable[] = { 0.0001, 0.001, 0.01, 0.1,  1,  10, 100};                      // kilo Hertz or Channels
char                          LCDstr[11];                                                           // temporary string buffer

///////////////////////////////
// function prototypes
///////////////////////////////
float                        CheckLimitsFloat64(float Variable, float Minimum, float Maximum);
void                         UpdateDisplay(void);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Timer 1 Interrupt - 10 [ms] - Manage Timers Vector
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER1_COMPA_vect)
{
    // Timers down
    for(uint8_t i=0; i<MAX_NR_TIMER_DOWN; i++)
    {
        // decrements only if greater than zero
        if(TimerDown[i].Counter != 0)
        {
            TimerDown[i].Counter--;
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT1_vect) 
{
    // check ncoder state when inputs have changed
    encoder.tick();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Function Setup
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{
    // stop interrupts
    cli();

    // set entire TCCR1A and TCCR1B registers to 0
    TCCR1A = 0;
    TCCR1B = 0;

    // initialize Timer 1 counter value to 0	
    TCNT1  = 0; 

    // set compare match register for 100 Hz increments
    OCR1A = 19999; // = 16000000 / (8 * 100) - 1 (must be <65536) ///////////////////// 10ms

    // turn on CTC mode
    TCCR1B |= (1 << WGM12);

    // Set CS12, CS11 and CS10 bits for 8 prescaler
    TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);

    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    // allow interrupts
    sei(); 

    // Interrupts for encoder pins A2 and A3
    PCICR  |= (1 << PCIE1);                               // This enables Pin Change Interrupt 1 that covers the Analog input pins or Port C.
    PCMSK1 |= (1 << PCINT10) | (1 << PCINT11);            // This enables the interrupt for pin 2 and 3 of Port C.

    // Encoder Key PinMode
    pinMode(DI_ENC_KEY,  INPUT_PULLUP);                   // Active Low
    
    // put your setup code here, to run once:
    // Serial.begin(115200);
    // while (!Serial); // Waiting for Serial Monitor
    // Serial.println("Serial Ok");

    // Initializes Si570
    Si570.Init();
    
    // Initializes LCD
    lcd.init();
    lcd.backlight();

    // initialize encoder buton status
    EncKey.SetStateAndTime(LOW, ENC_KEY_PRESS_TIME);

    // data initialization
    VFO.DispFreq = Si570.Get_Frequency();
    VFO.CoefIndex = 0;
    TimerDown[TIMERDOWN_BACKLIGHT].Counter = BACKLIGHT_DELAY;

    // First update on display
    UpdateDisplay();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// Funcion Loop
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() 
{
    bool       btmpFreqHasChanged      = false; 

    /////////////////////////////
    // manage encoder
    /////////////////////////////
    // manage button
    byte type = EncKey.CheckButton(DI_ENC_KEY); // current time and length of time to press the button as many times as you can ie. 1.5 seconds
    switch (type)
    {
        case WAITING:
          // ***************
          // ** NOT USED! **
          // ***************
          break;
      
        // button pressed once
        case PRESSED:
          // turn on backlight
          TimerDown[TIMERDOWN_BACKLIGHT].Counter = BACKLIGHT_DELAY;
          lcd.backlight();

          // controls clock coef index - increase and check limits
          VFO.CoefIndex++;
          if (VFO.CoefIndex > MAX_COEF_INDEX)
          {
              // reset CoefIndex
              VFO.CoefIndex = MIN_COEF_INDEX;
          }
          // change has ocurred: update display
          btmpFreqHasChanged = true;
          break;
          
        // button pressed twice
        case DOUBLE_PRESSED:
          // ***************
          // ** NOT USED! **
          // ***************
          break;
  
        // button pressed three times
        case MULTI_PRESSED:
          // ***************
          // ** NOT USED! **
          // ***************
          break;
        
        // button held pressed
        case HELD:
          break;
    }  
  
    // manage encoder position
    int newPosition = encoder.getPosition();                // variable does not retain its value between function calls
    if (newPosition != 0) 
    { 
      // turn on backlight
      TimerDown[TIMERDOWN_BACKLIGHT].Counter = BACKLIGHT_DELAY;
      lcd.backlight();

      // calculate new frequency     
      VFO.DispFreq = VFO.DispFreq + (float)newPosition * CoefTable[VFO.CoefIndex];
      VFO.DispFreq = CheckLimitsFloat64(VFO.DispFreq, MIN_SI570_FREQ, MAX_SI570_FREQ);

      // Change Frequency
      Si570.Set_Frequency(VFO.DispFreq);
      
      // new freq has been set
      btmpFreqHasChanged = true;
 
      // reset encoder position
      encoder.setPosition(0);
    } // if (newPosition != 0)
    /////////////////////////////
    // end manage encoder
    /////////////////////////////

    /////////////////////////////
    // Display Management
    /////////////////////////////
    if (btmpFreqHasChanged)
    {
        UpdateDisplay();
        btmpFreqHasChanged = false;          
    }
    if (TimerDown[TIMERDOWN_BACKLIGHT].Counter == 0)
    {
        lcd.noBacklight();
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check boundaries of a 64bits variable
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float CheckLimitsFloat64(float Variable, float Minimum, float Maximum)
{
    if (Variable < Minimum) return Minimum;
    if (Variable > Maximum) return Maximum;
    return Variable;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Check boundaries of a 64bits variable
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void UpdateDisplay(void)
{
    // display Frequency    
    lcd.setCursor(0,0);
    lcd.print("Freq: ");
    dtostrf(VFO.DispFreq, 10, 4, LCDstr);
    lcd.printstr(LCDstr);

    // display multiplication coefficient
    lcd.setCursor(0,1);
    lcd.print("Coef: x ");
    dtostrf(CoefTable[VFO.CoefIndex], 8, 4, LCDstr);
    lcd.print(LCDstr);

    // display N1 and HighSpeed divisors
    lcd.setCursor(0,2);
    sprintf(LCDstr, "N1: %3d     HS: %3d", Si570.Get_N1(), Si570.Get_HSdiv());
    lcd.print(LCDstr);
    
    // display RFreq
    lcd.setCursor(0,3);
    dtostrf(Si570.Get_RFreq(), 12, 6, LCDstr);
    lcd.print("RFreq: ");
    lcd.print(LCDstr);    
}