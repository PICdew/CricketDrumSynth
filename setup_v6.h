// Cricket synth
// defines and initialization
// final changes

#include<p33fj32MC302.h>

// logic control pins
#define SHIFT_DATA    PORTBbits.RB8
#define SHIFT_CLK     PORTBbits.RB4

// switches/buttons
#define START_STOP    PORTBbits.RB7
#define MANUAL_CTL    PORTAbits.RA1

#define WAVEFORM_2    PORTAbits.RA2
#define WAVEFORM_1    PORTAbits.RA3
#define OSC2_SEL      PORTAbits.RA4

// external I/O
#define AUDIO_OUT     9   // RB9

// LED output patterns
#define POD3_YEL      0b10000000
#define POD3_BLU      0b01000000
#define POD2_RED      0b00100000
#define POD2_GRN      0b00010000
#define POD1_YEL      0b00001000
#define POD1_RED      0b00000100
#define ALL_LEDS      0b11111100

#define OFF           0
#define FADESTEPS     96

#define CLOSE_TIME    5000
#define OPEN_TIME     200

#define TIMER_PERIOD	10

#define LEDSP_MAX     500         // max LED fade/chase speed
#define BYTE_MAX      255
#define KNOB_MAX      1023
#define VOL_MAX	  	  255
#define VMAX_SQ       VOL_MAX*VOL_MAX
#define PWM_MAX	  	  63 // working version
#define PITCH_DIV     32
#define ENV_LEN       500
#define SAMP_LEN      50
#define TIME_CONSTANT 240000
#define DECAY_SPD_MIN 156
#define ONEQKNOB      800
#define TWOQKNOB      950
#define FILTER_ORDER  2
#define LFOSCALE      20
#define INTMAX        32767
#define INTMIN        -32768
#define FILTSCALE     8
#define ENV_INC       2000
#define PUNCH         3000
#define UPDATE_FREQ   1

enum{FALSE, TRUE};
enum{CLOSED, OPEN};
enum{FWD, REV};
enum{AND,OR,XOR};
                            
// initialize ports, etc.
void initHardware()
{

  // master control stuff
  RCONbits.SWDTEN = 0;              // watchdog timer off
  
  CLKDIVbits.PLLPRE = 0;            // 7.37 MHz / 2 = 3.685 MHz
  PLLFBD = 41;                      // 3.685 MHz * 43 = 158.46 MHz
  CLKDIVbits.PLLPOST = 0;           // 158.46 MHz / 2 = 79.2275 Mhz
                                    // 79.2275 MHz / 2 = 39 MIPS  

  // ports
  TRISA = 0b11111;                // all A pins inputs
  TRISB = 0b1111110010001011;     // all B pins ins but B2, B4-6, B8-9 
  PORTB = 0x0000;                 // init port
  ODCB = 0x0000;                  // turn off open-drain outputs
  
  CNEN1 = 0x0000;                 // change notification off
  CNEN2 = 0x0000;
  CNPU1 = 0x0000;                 // change not. pullups off        
  CNPU2 = 0x0000;       
  CMCON = 0x0000;                 // comparator off
  PMCONbits.PMPEN = 0;            // parallel master port off
  PMCONbits.PTBEEN = 0;           // PMBE port disabled
  PMD1bits.I2C1MD = 1;            // I2C port disabled
  
  // oscillator
  OSCCONbits.COSC = 0b001;        // fast RC oscillator w/ PLL
  OSCCONbits.LPOSCEN = 0;         // disable ext oscillator
  CLKDIVbits.DOZE = 0b000;        // no clock reduction

  // ADC
  AD1CON1bits.ADON = 0;           // ADC off
  AD1PCFGL = 0b11110010;          // set A0, 2, 3 to analog
  AD1CON1bits.FORM = 0b00;        // unsigned integer output
  AD1CON1bits.SSRC = 0;           // manual control
  AD1CON3bits.ADCS = 0b00001011;  // TAD = 12 TCY
  AD1CON3bits.SAMC = 0b00010;     // sample time = 2 TAD
  AD1CON1bits.ASAM = 0;           // auto-sampling off
  AD1CON1bits.ADON = 1;           // ADC on
  
  // PWM generators
  PWM1CON1 = 0x00;                  // disable both
  PWM2CON1 = 0x00;

  // timer
  T1CON = 0;                    // init register
  T1CONbits.TCS = 0;            // use internal clock
  T1CONbits.TCKPS1 = 0;         // no prescaler
  T1CONbits.TCKPS0 = 0;    
  TMR1 = 0;                     // clear timer
  PR1 = TIMER_PERIOD;           // load timer with init value
  
  // interrupts
  INTCON2bits.ALTIVT = 0;       // use std vector table
  IEC0 = IEC1 = IEC2 = 0;       // disable all interrupts
  IEC3 = IEC4 = 0;
  
  IPC0 = 0b0111000000000110;    // timer 1 priority level 7
                                // ext interrupt priority level 6

  IFS0 = 0;                     // clear all flags

  // wait for clock switch
  while(OSCCONbits.COSC != 0b001);
  // wait for PLL lock
  while(OSCCONbits.LOCK != 1);
  
  IEC0bits.T1IE = 1;            // enable timer 1 interrupt
  
  T1CONbits.TON = 1;            // timer on
  
}
