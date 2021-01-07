#include "msp430.h" 

#define SERVO_OUTPUT BIT2 
#define POSITION_INPUT BIT1 
#define SMCLK_FREQ 1100000 
#define SERVO_FREQ 50 
#define CENTRE_OFFSET 0 
#define CENTRE 1700 
#define PORT_RUN_POSITION 1200 
#define STBD_RUN_POSITION 2200 
#define WIND_OFFSET 0 

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


int main(void) {
    disableWatchdog(); 
    initPWM(); 
    initADC(); 
    initClock();
  
    while(1){ 
        samplingAndConversionStart();
        waitOnBusyADC(); 
       
        int APPARENT_CENTRE = CENTRE + CENTRE_OFFSET; 
        int APPARENT_WIND = calcAppWind(ADC10MEM); 
        
        if (APPARENT_WIND <= 0x47 || (APPARENT_WIND > 0x3B8 && APPARENT_WIND <= 0x3FF)) {
            inIrons(APPARENT_CENTRE); 
        }
        if (APPARENT_WIND > 0x47 && APPARENT_WIND <= 0x1B8) {
            float PORT_MULTIPLIER = ((APPARENT_CENTRE - PORT_RUN_POSITION)/(0x1B8 - 0x47));
            setSailPort(APPARENT_CENTRE, APPARENT_WIND, 0x47, PORT_MULTIPLIER); 
        }
        if (APPARENT_WIND > 0x1B8 && APPARENT_WIND <= 0x246) {
            runAndGybe(APPARENT_WIND); 
        }
        if (APPARENT_WIND > 0x246 && APPARENT_WIND <= 0x3B8) {
            float STBD_MULTIPLIER = ((STBD_RUN_POSITION - APPARENT_CENTRE)/(0x3B8 - 0x246));
            setSailStbd(APPARENT_CENTRE, APPARENT_WIND, 0x3B8, STBD_MULTIPLIER); 
        }
    }
}

int calcAppWind(int ADC10MEM){
    if ((ADC10MEM + WIND_OFFSET) <= 0x3FF && (ADC10MEM + WIND_OFFSET) >= 0){
        return ADC10MEM + WIND_OFFSET;
    }
    else {
        return (ADC10MEM + WIND_OFFSET) % 0x3FF; 
    }
}

void runAndGybe(int APPARENT_WIND){
    if (APPARENT_WIND > 0x1B8 && APPARENT_WIND <= 0x1E8){
        TA0CCR1 = PORT_RUN_POSITION;
    }
    if (APPARENT_WIND > 0x1E8 && APPARENT_WIND <= 0x217) {
        float GYBE_MULTIPLIER = ((STBD_RUN_POSITION-PORT_RUN_POSITION)/(0x217-0x1E8));
        TA0CCR1 = PORT_RUN_POSITION + GYBE_MULTIPLIER*APPARENT_WIND;
    }
    if (APPARENT_WIND > 0x217 && APPARENT_WIND <= 0x246){
        TA0CCR1 = STBD_RUN_POSITION; 
    }
}

void inIrons(int APPARENT_CENTRE) {
    TA0CCR1 = APPARENT_CENTRE; 
}

void setSailPort(int APPARENT_CENTRE, int APPARENT_WIND, int START_WIND, float PORT_MULTIPLIER){
    TA0CCR1 = APPARENT_CENTRE - PORT_MULTIPLIER*(APPARENT_WIND - START_WIND);
}

void setSailStbd(int APPARENT_CENTRE, int APPARENT_WIND, int END_WIND, float STBD_MULTIPLIER){
    TA0CCR1 = APPARENT_CENTRE + STBD_MULTIPLIER*(END_WIND - APPARENT_WIND);
}

void samplingAndConversionStart(){
    ADC10CTL0 |= ENC + ADC10SC;
}

// initialize pwm pulse for servo - based on code from //https://forum.43oh.com/topic/3838-servo-control-with-msp430-g2553/
void initPWM() {
  int PWM_PERIOD = SMCLK_FREQ/SERVO_FREQ; 
  P1DIR |= SERVO_OUTPUT; 
  P1SEL |= SERVO_OUTPUT;
  TA0CCR0 = PWM_PERIOD - 1; 
  TA0CCR1 = 0;
  TA0CCTL1 = OUTMOD_7; 
}

// initialize clock for pulses - based on code from //https://forum.43oh.com/topic/3838-servo-control-with-msp430-g2553/
void initClock() {
    TA0CTL=TASSEL_2+MC_1; 
}

void initADC() {
    ADC10CTL0 = ADC10SHT_2 + ADC10ON;         
    ADC10CTL1 = INCH_1;                      
    ADC10AE0 |= POSITION_INPUT;              
}

void waitOnBusyADC(){
    while (ADC10CTL1 &ADC10BUSY);
}

void disableWatchdog() {
    WDTCTL = WDTPW | WDTHOLD; 
}

