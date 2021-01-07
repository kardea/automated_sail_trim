// ------------------------- HEADER INFORMATION, DEFITIONS -------------------------

#include "msp430.h"         // header file for msp430 functionality

#define SERVO_OUTPUT BIT2   // pulse output to servo through pin P1.2
#define POSITION_INPUT BIT1 // position input from position sensor to pin P1.1

#define SMCLK_FREQ 1100000  // approximate frequency of msp430 SMCLK (clock)
#define SERVO_FREQ 50       // approximate frequency for Futaba FP-S148 servo motor

#define CENTRE_OFFSET 0     // single point of control over position offset, can change this value if boom isn't at centre with pulse = 1700
#define CENTRE 1700         // pulse length for boom at centre position
#define PORT_RUN_POSITION 1200 // pulse length for the port run sail position
#define STBD_RUN_POSITION 2200 // pulse length for the starboard run sail position

#define WIND_OFFSET 0       // single point of control over wind offset, can change this value to calibrate any offset in wind direction and sensor reading


// ------------------------- FUNCTION DECLARATIONS ----------------------------

// comments provided with function definitions
void disableWatchdog(void);
void initPWM(void);
void initADC(void);
void initClock(void);
void inIrons(int);
void setSailPort(int, int, int, float);
void portRun();
void setSailStbd(int, int, int, float);
void stbdRun();
void samplingAndConversionStart();
void waitOnBusyADC(void);
void runAndGybe(int);
int  calcAppWind(int);


// ------------------------- FUNCTIONS -----------------------------------------


// main method initializes functionality, takes positon sensor input, calculates sail position, sends corresponding pulse to servo
    disableWatchdog(); // disable watchdog timer
    initPWM();         // initialize PWM pulse funtionality
    initADC();         // initialize ADC sampling and conversion functionality
    initClock();       // initiliaze msp430 clock
  
    while(1){ 
        samplingAndConversionStart(); // start sampling and converting ADC input from position sensor
        waitOnBusyADC();              // wait if ADC busy sampling/converting
        
        int APPARENT_CENTRE = CENTRE + CENTRE_OFFSET; // define apparent centre position for boom using offset 
        int APPARENT_WIND = calcAppWind(ADC10MEM);    // define apparent wind position 

        // optimal angle between wind and sail varies depending on wind position relative to boat, but is consistent within each of the following ranges of wind position 
        
        // apparent wind between 0-0x47 or 3B8-0x3FF, corresponds to wind between 335-360, 0-25 degrees, boat can't sail with the wind this close
        if (APPARENT_WIND <= 0x47 || (APPARENT_WIND > 0x3B8 && APPARENT_WIND <= 0x3FF)) {
            inIrons(APPARENT_CENTRE); // position sail for being "in irons"
        }
        // apparent wind between 0x47 - 0x1B8, corresponds to wind between 205-335 degrees, boat is on a "port tack" and optimal sail position is determined by the PORT_MULTIPLIER
        if (APPARENT_WIND > 0x47 && APPARENT_WIND <= 0x1B8) {
            float PORT_MULTIPLIER = ((APPARENT_CENTRE - PORT_RUN_POSITION)/(0x1B8 - 0x47));
            setSailPort(APPARENT_CENTRE, APPARENT_WIND, 0x47, PORT_MULTIPLIER);
        }
        // apparent wind between 0x1B8-0x246, corresponds to wind between 155-205 degrees, boat is sailing "downwind" and will either be on a "run" or will "gybe"
        if (APPARENT_WIND > 0x1B8 && APPARENT_WIND <= 0x246) {
            runAndGybe(APPARENT_WIND);
        }
        // apparent wind between 0x246-0x3B8, corresponds to wind between 25-155 degrees, boat is on a "starboard (stbd) tack"
        if (APPARENT_WIND > 0x246 && APPARENT_WIND <= 0x3B8) {
            float STBD_MULTIPLIER = ((STBD_RUN_POSITION - APPARENT_CENTRE)/(0x3B8 - 0x246));
            setSailStbd(APPARENT_CENTRE, APPARENT_WIND, 0x3B8, STBD_MULTIPLIER); 
        }
    }
}

// calculate the apparent wind direction (ADC10MEM is value from position sensor)
int calcAppWind(int ADC10MEM){
    // if the position value + offset is within the 0x0-0x3FF ADC range (corresponds to 0-360 degrees), then set apparent wind = ADC10MEM+OFFSET
    if ((ADC10MEM + WIND_OFFSET) <= 0x3FF && (ADC10MEM + WIND_OFFSET) >= 0){
        return ADC10MEM + WIND_OFFSET;
    }
    // if the position value + offset is outside the ADC range, then take the modulus of the position + offset with the maximum ADC value
    else {
        return (ADC10MEM + WIND_OFFSET) % 0x3FF;
    }
}

// boat is on a downwind course, will either be on a port run, starboard run, or gybe
void runAndGybe(int APPARENT_WIND){
    // apparent wind is between 0x1B8-0x1E8, or 188-205 degrees, position sail in the port run position
    if (APPARENT_WIND > 0x1B8 && APPARENT_WIND <= 0x1E8){
        TA0CCR1 = PORT_RUN_POSITION;
    }
     // apparent wind is between 0x1E8-0x217, or 172-188 degrees, this is the range where the boat will "gybe", and the sail will go from one run position to the other
    if (APPARENT_WIND > 0x1E8 && APPARENT_WIND <= 0x217) {
        float GYBE_MULTIPLIER = ((STBD_RUN_POSITION-PORT_RUN_POSITION)/(0x217-0x1E8));
        TA0CCR1 = PORT_RUN_POSITION + GYBE_MULTIPLIER*APPARENT_WIND;
    }
    // apparent wind is between 0x217-0x246, or 155-172 degrees, position sail in the starboard run position
    if (APPARENT_WIND > 0x217 && APPARENT_WIND <= 0x246){
        TA0CCR1 = STBD_RUN_POSITION; 
    }
}

// boat is "in irons" where the wind is too close to centre to sail, position sail in centre of boat
void inIrons(int APPARENT_CENTRE){
    TA0CCR1 = APPARENT_CENTRE; 
}

// boat is on a port tack, sail is positioned according the apparent wind and the multiplier so it ranges from the centre position to the port run position
void setSailPort(int APPARENT_CENTRE, int APPARENT_WIND, int START_WIND, float PORT_MULTIPLIER){
    TA0CCR1 = APPARENT_CENTRE - PORT_MULTIPLIER*(APPARENT_WIND - START_WIND);
}

// boat is on a starboard tack, sail is positioned according the apparent wind and the multiplier so it ranges from the centre position to the starboard run position
void setSailStbd(int APPARENT_CENTRE, int APPARENT_WIND, int END_WIND, float STBD_MULTIPLIER){
    TA0CCR1 = APPARENT_CENTRE + STBD_MULTIPLIER*(END_WIND - APPARENT_WIND);
}

// sample ADC (analog to digital conversion, input from position sensor)
void samplingAndConversionStart(){
    ADC10CTL0 |= ENC + ADC10SC;
}

// initialize PWM pulse for servo - based on code from //https://forum.43oh.com/topic/3838-servo-control-with-msp430-g2553/
// functionality described in TI MSP430 data sheet
void initPWM() {
  int PWM_PERIOD = SMCLK_FREQ/SERVO_FREQ;   // define the PWM pulse period
  P1DIR |= SERVO_OUTPUT;                    // set P1 direction so SERVO_OUTPUT bit is output
  P1SEL |= SERVO_OUTPUT;                    // set P1 function for SERVO_OUTPUT bit
  TA0CCR0 = PWM_PERIOD - 1;                 // define the period for TA0CCR0 register, will be used to calculate duty cycle for PWM pulse
  TA0CCR1 = 0;                              // initialization - gets set in main, duty cycle for PWM pulse = TA0CCR0/TA0CCR1
  TA0CCTL1 = OUTMOD_7;                      // enables mode for CCR1 signal
}

// initialize clock for pulses - based on code from //https://forum.43oh.com/topic/3838-servo-control-with-msp430-g2553/
// functionality described in TI MSP430 data sheet
void initClock() {
    TA0CTL=TASSEL_2+MC_1;                     // starting timer by setting clock source SMCLK of timer and UP mode
}

// initializa ADC 
// functionality described in TI MSP430 data sheet
void initADC() {
    ADC10CTL0 = ADC10SHT_2 + ADC10ON;         // ADC10ON
    ADC10CTL1 = INCH_1;                       // set input A1
    ADC10AE0 |= POSITION_INPUT;               // PA.1 ADC option select
}

// wait on busy ADC (busy sampling/converting)
void waitOnBusyADC(){
    while (ADC10CTL1 &ADC10BUSY);
}

// disable watchdog timer
void disableWatchdog() {
    WDTCTL = WDTPW | WDTHOLD; 
}

