# automated_sail_trim
Code for an automated sail trimming system developed for a small model boat.

Hardware:
- TI MSP430 Microcontroller
  - data sheet: https://www.ti.com/lit/ds/slas735j/slas735j.pdf?ts=1609977325802&ref_url=https%253A%252F%252Fwww.ti.com%252Ftool%252FMSP-EXP430G2ET%253FkeyMatch%253DMSP-EXP430G2ET%2526tisearch%253DSearch-EN-everything
- Bournes Inc. 3382 Rotary Position Sensor
  - data sheet: https://www.bourns.com/docs/Product-Datasheets/3382.pdf
- Futaba FP-S148 Servo Motor
  - specifications: https://servodatabase.com/servo/futaba/s148

Connections:
- connection between rotary position sensor and microcontroller named POSITION_INPUT, defined as BIT1 (pin P1.1 on microcontroller)
- connection between servo motor and microcontroller named SERVO_OUTPUT, defined as BIT2 (pin P1.2 on microcontroller)

Code:
- Control code for the system was written in C. The rotary position sensor is used to determine the wind direction. Based on the wind direction, the optimal sail position is   calculated. A pulse length corresponding the optimal sail position is sent to the servo, which positions the sail accordingly. 
- The file auto_sail_trim_control_commented.c contains detailed comments explaining the calculations and sail position. The file auto_sail_trim.c has the same code but with minimal comments. 
