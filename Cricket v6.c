// Cricket synth
// main and function code
// daniel ford, 2014
// final changes

#include "setup_v6.h"
#include "waves.h"

/******************* GLOBAL VARIABLES **********************/

enum{OSC1,OSC2};
enum{ATTACK,HOLD,DECAY,SUSTAIN,RELEASE,END};

volatile unsigned int close = 0;        // debouncing
volatile unsigned int open = 0;
volatile unsigned char trigger = FALSE;
volatile unsigned char pressState = OPEN;

signed char * osc1 = squ50;
volatile unsigned char osc1_pitch = 0;
volatile unsigned int osc1_phase = 0;
volatile unsigned int osc1_vol = VOL_MAX;
volatile unsigned int o1_vol_temp = 0;
volatile unsigned char o1_out_temp = 0;

signed char * osc2 = squ50;
volatile unsigned char osc2_pitch = 0;
volatile unsigned int osc2_phase = 0;
volatile unsigned int osc2_vol = VOL_MAX;
volatile unsigned int o2_vol_temp = 0;
volatile unsigned char o2_out_temp = 0;

signed char * LFO = tri;
volatile unsigned char LFO_speed = 0;
volatile unsigned int LFO_phase = 0;
volatile unsigned char LFO_depth = 0;

volatile unsigned int osc_ct = 0;
volatile unsigned long oscs_DC = 0L;
volatile unsigned int PWMmask = 0;

volatile unsigned int oscMix = 0;
volatile unsigned char FM_depth = 0;
volatile unsigned char AM_depth = 0;
volatile unsigned char AM_on = FALSE;
volatile unsigned char loRng = FALSE;
volatile unsigned int rngct = 0;
volatile unsigned int phase_temp = 0;

unsigned char * envA = attEnv;
unsigned char * envD = exp10Env;
unsigned char * envR = expEnv;
volatile unsigned long env_ct = 0L;
volatile unsigned char env_phase = 0;
volatile unsigned char envState = END;
volatile unsigned char envVol = 0;
volatile unsigned char update_ct = 0;

volatile unsigned int cutoff = 0;
volatile unsigned int res = 0;
volatile unsigned int attack = 0;
volatile unsigned int decay = 0;
volatile unsigned int release = 0;
volatile unsigned char sustain = 0;
volatile unsigned char scale = 1;
volatile unsigned char logic = AND;

// controls
// left/right is from viewer's perspective, looking at cricket from front
struct {

  volatile unsigned int L_blk_up;
  volatile unsigned int L_blk_down;
  volatile unsigned int L_white;
  volatile unsigned int L_silv;
  
  volatile unsigned int R_blk_up;
  volatile unsigned int R_blk_down;
  volatile unsigned int R_yel;
  volatile unsigned int R_silv;
  
  volatile unsigned int front_L;
  volatile unsigned int front_R;

  volatile unsigned char sw_black;
  volatile unsigned char sw_minitog;
  volatile unsigned char sw_toggleL;
  volatile unsigned char sw_toggleC;
  volatile unsigned char sw_toggleR;
  
  unsigned char numCont;
  
} ctrl;

/******************* PROTOTYPES **********************/

// setup and UI
void initState();
void pollInputs(); 

// display functions
void outputLEDs(unsigned char data);
void fadeLEDs(unsigned char LEDs, int time);
void chaseLEDs(int speed, unsigned int fadeSpeed);

/******************* INTERRUPTS **********************/

// vector for timer 1 interrupt
void __attribute__((__interrupt__)) _T1Interrupt(void);
// vector for external interrupt
void __attribute__((__interrupt__)) _INT0Interrupt(void); 
 
// timer 1 interrupt ISR 
void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{    

  
  // check whether button's pressed 
  if( START_STOP && pressState == OPEN ) close++;
  if( pressState == OPEN && close >= CLOSE_TIME ) trigger=TRUE;
  
  // trigger on button press
  if( trigger ){
    env_phase = 0; env_ct = 0;
    osc1_phase = osc2_phase = 0;
    envState = ATTACK;
    pressState = CLOSED; trigger = FALSE; close = 0;
  }

  // check whether button's released  
  if( !START_STOP && pressState == CLOSED ) open++;    
  if( pressState == CLOSED && open > OPEN_TIME ) {
    pressState = OPEN; open = 0; }
  
  
  // mix oscs and apply envelope to create output volume level
  if(update_ct >= UPDATE_FREQ){
    if(envState == ATTACK){
      envVol = attEnv[env_phase];
      if(env_ct >= attack){
        ++env_phase; env_ct = 0; }
      else
        ++env_ct;
      if(env_phase == (PHASE_SZ-1)){
        env_ct = 0; env_phase = 0;
        envState = SUSTAIN; }
      }
    
    // arranged this way for roughly equal compute time
    // as other stages so not to affect pitch
    if(envState == HOLD){
      envVol = attEnv[0xFF];
      
      // dummy operations
      if(0) env_ct = env_ct;
      else
        ++env_ct;
      if(env_ct >= attack){
        ++env_phase; }
      
      if(env_ct >= PUNCH){
        env_ct = 0; env_phase = 0;
        envState = RELEASE; }
    }
    
    if(envState == DECAY){
    // run through decay envelope until the specified
    // sustain level is reached
    }

    // arranged this way for roughly equal compute time
    // as other stages so not to affect pitch  
    if(envState == SUSTAIN){
      //envVol = sustain;

      // dummy operations
      if(0) env_ct = env_ct;
      else
        ++env_ct;
      if(env_ct >= attack){
        ++env_phase; }
        
      if(env_ct >= PUNCH*25){
        env_ct = 0; env_phase = 0;
        envState = RELEASE; }
    }
    
    if(envState == RELEASE){
      envVol = envR[env_phase];
      if(env_ct >= release){
        ++env_phase; env_ct = 0; }
      else
        ++env_ct;
      if(env_phase == (PHASE_SZ-1)){
        env_phase = 0; envState = END; }
    }
    
    if(envState == END){
      envVol = 0;
    }
  // done, reset update counter
  update_ct = 0;
  }  
  
  // premultiply and scale volumes to avoid overflow
  o1_vol_temp = (osc1_vol * envVol) >> 7; 
  o2_vol_temp = (osc2_vol * envVol) >> 7;
  
  // convert to unsigned for final PWM output
  o1_out_temp = osc1[osc1_phase]+0x80;
  o2_out_temp = osc2[osc2_phase]+0x80;
  
  // calculate final 6-bit output level
  if(!ctrl.sw_toggleR){
    oscs_DC = ( scale * ( (o1_out_temp * o1_vol_temp) + 
              (o2_out_temp * o2_vol_temp) ) ) >> 10;
  }
  // bitwise AND/OR/XOR of osc1 and osc2
  else{
    switch(logic){   
      case AND:
      oscs_DC = ( scale * ( (o1_out_temp * o1_vol_temp) & 
                (o2_out_temp * o2_vol_temp) ) ) >> 10;
      break;
      case OR:
      oscs_DC = ( scale * ( (o1_out_temp * o1_vol_temp) |
                (o2_out_temp * o2_vol_temp) ) ) >> 10;
      break;
      case XOR:
      oscs_DC = ( scale * ( (o1_out_temp * o1_vol_temp) ^ 
                (o2_out_temp * o2_vol_temp) ) ) >> 10;
      break;      
   }
  }
  
  // update osc PWM outputs
  if( osc_ct <= oscs_DC ){ 
    PWMmask = LATB; PWMmask |= (1<<AUDIO_OUT);
    PORTB = PWMmask;        
    }                                              

  if( osc_ct > oscs_DC ){
    PWMmask = LATB; PWMmask &= ~(1<<AUDIO_OUT);
    PORTB = PWMmask;
    }
  // common to both
  if( osc_ct == PWM_MAX ){
    osc_ct = 0;
    
    // update osc1 pitch w/ FM, LFO modulation
    osc1_phase += (osc1_pitch + (((osc2[osc2_phase]+0x80) * FM_depth) >> 4) + (((LFO[LFO_phase]+0x80)*LFO_depth)>>6) );
    while(osc1_phase > (PHASE_SZ-1) ) osc1_phase -= PHASE_SZ;
    
    // update osc2 pitch w/ range select, LFO modulation
    if(loRng){
        if(rngct>LFOSCALE){
          osc2_phase += (osc2_pitch + (((LFO[LFO_phase]+0x80)*LFO_depth)>>6));
          rngct=0;
          }
        else rngct++;
      }
    else osc2_phase += (osc2_pitch + (((LFO[LFO_phase]+0x80)*LFO_depth)>>6));
    while(osc2_phase > (PHASE_SZ-1) ) osc2_phase -= PHASE_SZ;

    // update LFO speed
    LFO_phase += LFO_speed;
    while(LFO_phase > (PHASE_SZ-1) ) LFO_phase -= PHASE_SZ;

    }  
  
  // increment timing variables
  ++osc_ct; ++update_ct;
	
  // clear timer 1 interrupt flag 
  IFS0bits.T1IF = 0;
  
}

// external interrupt ISR
void __attribute__((__interrupt__, no_auto_psv)) _INT0Interrupt(void)
{	
  // this interrupt not in use
  // clear external interrupt flag 
  IFS0bits.INT0IF = 0;
}

/******************* MAIN **********************/

int main(void)
{

  int speed = 0;

  // initialize everything
  initState();
  initHardware();
  
  while(1) {
    pollInputs(); 
    if(ctrl.sw_black){
      speed = ctrl.L_blk_up - ctrl.R_blk_up + ctrl.R_silv - ctrl.L_white;
      chaseLEDs(speed>>2, 0);
      }
    else if(!ctrl.sw_black){
      speed = ctrl.L_blk_down - ctrl.R_yel + ctrl.R_blk_down - ctrl.front_L;
      chaseLEDs(-(speed>>2),0);
      }
	}
  
  return 0;

}

/*************************** CONTROL FUNCTIONS *****************/

// initialize state variables
void initState(void)
{
  
  ctrl.L_blk_up = 0;
  ctrl.L_blk_down = 0;
  ctrl.L_white = 0;
  ctrl.L_silv = 0;
  
  ctrl.R_blk_up = 0;
  ctrl.R_blk_down = 0;
  ctrl.R_yel = 0;
  ctrl.R_silv = 0;
  
  ctrl.front_L = 0;
  ctrl.front_R = 0;
  
  ctrl.numCont = 9;  
  
  ctrl.sw_black = 0;
  ctrl.sw_minitog = 0;
  ctrl.sw_toggleL = 0;
  ctrl.sw_toggleC = 0;
  ctrl.sw_toggleR = 0;
  
  outputLEDs(0);
  
};

// get status of knobs, photocells, bump switches, personality encoder
void pollInputs()
{
  
  // set up variables to work around the fact that RB5 can't be set
  // directly for some reason... can't figure out why,
  // all peripherals are disabled and other port b pins can be set
  static unsigned char mask = 0;
  static unsigned char mux_a = 2;   // RB2
  static unsigned char mux_b = 5;   // RB5
  static unsigned char mux_c = 6;   // RB6
  static unsigned char position = 0;
  static unsigned char wave = 0;
  static unsigned long sampTime = 0;
  static unsigned char LFOwave = 0;
  
  // check switches, set osc waveforms
  wave = WAVEFORM_2<<1 | WAVEFORM_1;
  if(!OSC2_SEL){
    switch(wave){
      case 0: osc1 = saw; break;
      case 1: osc1 = tri; break;
      case 2: osc1 = noise; break;
      case 3: osc1 = squ50; break;
    }    
  }
  if(OSC2_SEL){
    switch(wave){
      case 0: osc2 = saw; break;
      case 1: osc2 = tri; break;
      case 2: osc2 = noise; break;
      case 3: osc2 = squ50; break;
    }    
  }

  // read front panel switches
  ctrl.sw_minitog = PORTBbits.RB10;
  ctrl.sw_toggleR = PORTBbits.RB11;
  ctrl.sw_toggleL = PORTBbits.RB12;
  ctrl.sw_toggleC = PORTBbits.RB13;
  ctrl.sw_black = PORTBbits.RB15;

  // set LFO waveform
  LFOwave = ctrl.sw_toggleL<<1 | ctrl.sw_toggleR;
  switch(LFOwave){
    case 0: LFO = saw; break;
    case 1: LFO = tri; break;
    case 2: LFO = noise; break;
    case 3: LFO = squ50; break;
  }
  
  // read all analog inputs 
  switch(position)
  {
  
    // left black top knob
    case 0:
    mask = LATB;                        // set switch to input 0
    mask &= ~(1<<mux_c);                // clears bit
    mask &= ~(1<<mux_b);                // WHY CAN'T I JUST SET RB5???
    mask &= ~(1<<mux_a);
    PORTB = mask;    
    sampTime = 0;
    AD1CON1bits.SAMP = 1;               // begin sampling
    while(++sampTime<SAMP_LEN);         // wait for sampling
    AD1CON1bits.SAMP = 0;               // begin conversion
    while(!AD1CON1bits.DONE);           // wait for conversion to complete
    ctrl.L_blk_up = ADC1BUF0; // save result
    break;
    
    // left black down knob
    case 1:
    mask = LATB;                        
    mask &= ~(1<<mux_c);                
    mask &= ~(1<<mux_b);                
    mask |= (1<<mux_a);                 // sets bit
    PORTB = mask;     
    sampTime = 0;
    AD1CON1bits.SAMP = 1;               // begin sampling
    while(++sampTime<SAMP_LEN);         // wait for sampling
    AD1CON1bits.SAMP = 0;               // begin conversion
    while(!AD1CON1bits.DONE);           // wait for conversion to complete
    ctrl.L_blk_down = ADC1BUF0;
    break;
    
    // left white knob
    case 2:
    mask = LATB;                        
    mask &= ~(1<<mux_c);                
    mask |= (1<<mux_b);                
    mask &= ~(1<<mux_a);
    PORTB = mask;      
    sampTime = 0;
    AD1CON1bits.SAMP = 1;               // begin sampling
    while(++sampTime<SAMP_LEN);         // wait for sampling
    AD1CON1bits.SAMP = 0;               // begin conversion
    while(!AD1CON1bits.DONE);           // wait for conversion to complete  
    ctrl.L_white = ADC1BUF0;
    break;
    
    // left silver knob      
    case 3:
    mask = LATB;                        
    mask &= ~(1<<mux_c);                
    mask |= (1<<mux_b);                
    mask |= (1<<mux_a);
    PORTB = mask;        
    sampTime = 0;
    AD1CON1bits.SAMP = 1;               // begin sampling
    while(++sampTime<SAMP_LEN);         // wait for sampling
    AD1CON1bits.SAMP = 0;               // begin conversion
    while(!AD1CON1bits.DONE);           // wait for conversion to complete  
    ctrl.L_silv = ADC1BUF0;
    break;
    
    // right black down knob     
    case 4:
    mask = LATB;                        
    mask |= (1<<mux_c);                
    mask &= ~(1<<mux_b);                
    mask &= ~(1<<mux_a);
    PORTB = mask;         
    sampTime = 0;
    AD1CON1bits.SAMP = 1;               // begin sampling
    while(++sampTime<SAMP_LEN);         // wait for sampling
    AD1CON1bits.SAMP = 0;               // begin conversion
    while(!AD1CON1bits.DONE);           // wait for conversion to complete  
    ctrl.R_blk_down = ADC1BUF0;
    break;
    
    // right silver knob       
    case 5:
    mask = LATB;                        
    mask |= (1<<mux_c);                
    mask &= ~(1<<mux_b);                
    mask |= (1<<mux_a);
    PORTB = mask;        
    sampTime = 0;
    AD1CON1bits.SAMP = 1;               // begin sampling
    while(++sampTime<SAMP_LEN);         // wait for sampling
    AD1CON1bits.SAMP = 0;               // begin conversion
    while(!AD1CON1bits.DONE);           // wait for conversion to complete  
    ctrl.R_silv = ADC1BUF0;
    break;
    
    // right yellow knob      
    case 6:
    mask = LATB;                        
    mask |= (1<<mux_c);                
    mask |= (1<<mux_b);                
    mask &= ~(1<<mux_a);
    PORTB = mask;         
    sampTime = 0;
    AD1CON1bits.SAMP = 1;               // begin sampling
    while(++sampTime<SAMP_LEN);         // wait for sampling
    AD1CON1bits.SAMP = 0;               // begin conversion
    while(!AD1CON1bits.DONE);           // wait for conversion  
    ctrl.R_yel = ADC1BUF0; 
    break;
    
    // right black up knob       
    case 7:
    mask = LATB;                        
    mask |= (1<<mux_c);                
    mask |= (1<<mux_b);                
    mask |= (1<<mux_a);
    PORTB = mask;         
    sampTime = 0;
    AD1CON1bits.SAMP = 1;               // begin sampling
    while(++sampTime<SAMP_LEN);         // wait for sampling
    AD1CON1bits.SAMP = 0;               // begin conversion
    while(!AD1CON1bits.DONE);           // wait for conversion  
    ctrl.R_blk_up = ADC1BUF0;
    break;

    // front left knob       
    case 8:  
    AD1CHS0bits.CH0SA = 0b00010;     // select input AN2 for sampling
    sampTime = 0;
    AD1CON1bits.SAMP = 1;            // begin sampling
    while(++sampTime<SAMP_LEN);      // wait for sampling
    AD1CON1bits.SAMP = 0;            // begin conversion
    while(!AD1CON1bits.DONE);        // wait for conversion
    ctrl.front_L = ADC1BUF0;
    break;    

    // front right knob       
    case 9:  
    AD1CHS0bits.CH0SA = 0b00011;     // select input AN3 for sampling
    sampTime = 0;
    AD1CON1bits.SAMP = 1;            // begin sampling
    while(++sampTime<SAMP_LEN);      // wait for sampling
    AD1CON1bits.SAMP = 0;            // begin conversion
    while(!AD1CON1bits.DONE);        // wait for conversion
    ctrl.front_R = ADC1BUF0;
    AD1CHS0bits.CH0SA = 0b00000;     // change back to AN0
    break; 
    
  }

  // reset ADC step control
  ++position;
  if( position > ctrl.numCont+1 ) position = 0;

  // knob assignments
  
  // pitches
  osc1_pitch = ctrl.L_blk_down>>3;
  if(osc1_pitch == 0) osc1_pitch = 1;
  
  osc2_pitch = ctrl.L_white>>3;   
  if(osc2_pitch == 0) osc2_pitch = 1;

  // osc2 -> osc1 AM/FM depth
  FM_depth = ctrl.R_yel>>2;
  
  // osc2 low/high pitch range
  if(!ctrl.sw_minitog)
    loRng = TRUE;
  else loRng = FALSE;
  
  // raw range: 356-1023 / 0x164-0x3FF
  // scaled: 0-1334 / 0x0-0x536
  attack = (ctrl.L_blk_up-0x164)*2;  
  if(attack == 0) attack = 1;
  
  // raw range: 172-1023 / 0xAC-0x3FF
  // scaled: 0-1277 / 0x0-0x4FD
  if(ctrl.R_blk_up >= 0xAC)
    release = (ctrl.R_blk_up-0xAC)*1.5;
  if(release == 0) release = 1;
  
  // volume
  oscMix = ctrl.R_silv>>2;
  osc2_vol = oscMix; osc1_vol = VOL_MAX - osc2_vol;
  
  scale = ctrl.L_silv>>3;
  if(scale == 0) scale = 1;
  
  // logic select
  if(ctrl.R_blk_down < ONEQKNOB)
    logic = AND;
  if( (ctrl.R_blk_down >= ONEQKNOB) && (ctrl.R_blk_down < TWOQKNOB) )
    logic = OR;
  if( ctrl.R_blk_down > TWOQKNOB )
    logic = XOR;  
  
  // LFO speed
  LFO_speed = ctrl.front_R>>5;
  LFO_depth = ctrl.front_L>>5;
  
}

/*************************** DISPLAY FUNCTIONS *****************/

// turn LEDs on/off through shift register
// sends out MSB -> LSB
void outputLEDs(unsigned char data)
{

  int i=0;
  static unsigned char dataTemp = 0;    
  
  // only update output if incoming data has changed
  if( data != dataTemp )
  {

    // save last data byte for comparison
    dataTemp = data;
  
    // loop through data, setting clock high after each write
    for(i=0;i<8;i++)
    {
      SHIFT_CLK = 0;
      SHIFT_DATA = data & 0b00000001;
      SHIFT_CLK = 1;
      data >>= 1;
    }
   
  }
  
}

// fade/blink LEDs
// smaller period and/or time blinks, larger values fade
void fadeLEDs(unsigned char LEDs, int time)
{

  static int counter = 0;
  static int period = FADESTEPS;
  static int duty = 0;
  static int step = 0;
  static int count = 0;
  
  // main PWM loop that sets LED brightness
  if( counter <= duty )
    outputLEDs(LEDs);
  if( counter > duty )
    outputLEDs(OFF);
  if( counter > period )
    counter = 0;
    
  ++counter;
  ++count;

  // outer loop that steps brightness up/down
  if( count >= time )
  {
    if( step <= period/2 )
      ++duty;
    if( step > period/2 )
      --duty;
    if( step > period )
      step = 0;

    if( duty < 1 ) duty = 1;
    if( duty > period ) duty = period;
      
    ++step;
    count = 0;
  }
  
}

// chase LEDs around the robot
void chaseLEDs(int speed, unsigned int fadeSpeed)
{

  static unsigned char currLED = POD2_RED;
  static int idxLED = 0;
  
  if( fadeSpeed == 0 )
      outputLEDs(currLED);
  else fadeLEDs(currLED, fadeSpeed);

  if( speed > 0 )
  {
    idxLED++;
    if( idxLED >= speed )
    {
      switch(currLED)
      {
          case POD2_RED:
          currLED = POD2_GRN;
          break;
          case POD2_GRN:
          currLED = POD1_RED;
          break;
          case POD1_RED:
          currLED = POD1_YEL;
          break;
          case POD1_YEL:
          currLED = POD3_BLU;
          break;
          case POD3_BLU:
          currLED = POD3_YEL;
          break;
          case POD3_YEL:
          currLED = POD2_RED;
          break;
      }
      idxLED = 0; 
    }
  }
  
  if( speed < 0 )
  {
    idxLED--;
    if( idxLED <= speed )
    {
      switch(currLED)
      {
        case POD2_RED:
        currLED = POD3_YEL;
        break;
        case POD2_GRN:
        currLED = POD2_RED;
        break;
        case POD1_RED:
        currLED = POD2_GRN;
        break;
        case POD1_YEL:
        currLED = POD1_RED;
        break;
        case POD3_BLU:
        currLED = POD1_YEL;
        break;
        case POD3_YEL:
        currLED = POD3_BLU;
        break;
      }
      idxLED = 0;  
    }
  } 

}


