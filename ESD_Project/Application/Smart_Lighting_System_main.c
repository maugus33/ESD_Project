/******************************************************************************
 *         MSP430G2-LaunchPad CapTouch BoosterPack User Experience
 * 
 * This application operates on the LaunchPad platform using the MSP430G2452
 * device and the CapTouch BoosterPack plugin board. The capacitive touch and 
 * proximity sensing are enabled by the pin oscillator feature new to the 
 * MSP430G2xx2 family devices. The User Experience application also utilizes 
 * the cap touch library to realize & measure the capacitive touch and proximity
 * sensors. The cap touch library also provides layers of abstractions to 
 * generate higher logical outputs such as logical touches, geometry (in this 
 * hardware, a four-button wheel), and even gestures. 
 * 
 * The User Experience application starts up and remains in 'sleep' mode, 
 * sampling the proximity sensor every ~8.3ms [VLO/100=12kHz/100=120Hz]. Upon 
 * registering a valid proximity event [hand/finger/object hovering ~3-5cm from
 * the BoosterPack], the application wakes up to operate in the 'active' mode.
 * 
 * In active mode, the application samples and registers individual finger touches 
 * on the 16-position wheel or the center button as well as simple gestures 
 * [Clockwise & Counter-clockwise] while the finger moves along and remains on 
 * the wheel.      
 * 
 * a 9600 baud UART link is also implemented using SW TimerA to provide 
 * application and cap touch data back to the PC via the UART-USB back channel.
 * The application sends UART data upon events such as wake up, sleep, touch,
 * or gesture.  
 *   
  ******************************************************************************/

/*----------------- Cap Touch Sensors port definition--------------------------
 * There are 6 cap touch sersors:
 * Wheel:      Up      P2.4
 *             Down    P2.2
 *             Left    P2.1
 *             Right   P2.3
 * Prox Sensor:        P2.0
 * Center button:      P2.5

*--------------------------- LCD Port Definition -------------------------------
* Pin 1       GND
* Pin 2       Vcc (+3.3V)
* Pin 3       Potentiometer
* Pin 4       RS - P1.4
* Pin 5       R/W - P1.5
* Pin 6       E - P1.7
* Pin 7       NC
* Pin 8       NC
* Pin 9       NC
* Pin 10      NC
* Pin 11      D4 - P1.0
* Pin 12      D5 - P1.1
* Pin 13      D6 - P1.2
* Pin 14      D7 - P1.3
* Pin 15      Vcc (+3.3V)
* Pin 16      GND

*--------------------------- Other Ports Definitions ---------------------------
* P1.6        Light LED
* P2.6        Input from Motion sensor
* P2.7        Display enable pin
*-----------------------------------------------------------------------------*/

/*-------------------------- HEADER FILE INCLUDES ----------------------------*/
#include <msp430.h>
#include <msp430g2452.h>
#include <stdio.h>
#include <string.h>
#include <intrinsics.h>
#include "CTS_Layer.h"
#include "uart.h"
#include "lcd.h"

/*------------------------ GLOBAL CONSTANTS DATA------------------------------*/
#define WAKE_UP_UART_CODE       0xBE
#define SLEEP_MODE_UART_CODE    0xDE
#define MIDDLE_BUTTON_CODE      0x80
#define INVALID_GESTURE         0xFD
#define GESTURE_START           0xFC
#define GESTURE_STOP            0xFB
#define COUNTER_CLOCKWISE       1
#define CLOCKWISE               2
#define GESTURE_POSITION_OFFSET 0x20
#define WHEEL_POSITION_OFFSET   0x30

#define WHEEL_TOUCH_DELAY       12        //Delay between re-sendings of touches
#define MAX_IDLE_TIME           200
#define PROXIMITY_THRESHOLD     60
//#define PROXIMITY_THRESHOLD     150
#define PRESET_LEVEL            60        //Preset brightness level = 60%
#define LIGHT                   BIT6
#define DISPLAY                 BIT7

/*------------------------------- GLOBAL DATA --------------------------------*/
unsigned int wheel_position=ILLEGAL_SLIDER_WHEEL_POSITION,
             last_wheel_position=ILLEGAL_SLIDER_WHEEL_POSITION;
unsigned int deltaCnts[1]; //Address where proximity sensor measurements are written
unsigned int prox_raw_Cnts;
int display_value = PRESET_LEVEL;  //Brightness level in percentage 0-100%
typedef enum {SLEEP,WAKING,IDLE,ACTIVE} system_state;
static volatile system_state state = SLEEP;

/*-------------------------- FUNCTIONS PROTOTYPES ----------------------------*/
void InitLaunchPadCore(void);
void SendData(unsigned char touch);
void myprint(char *msg);
void CapTouchIdleMode(void);
void MeasureCapBaseLine(void);
void setUpTimer1(void);
void setUpTimer2(void);
unsigned char GetGesture(unsigned char wheel_position);
void CapTouchActiveMode();
void fadeLight(int valuePWM);   //Function used to fade the LED


/******************************************************************************/
void main(void)
{
  WDTCTL = WDTPW | WDTHOLD;        // Stop watchdog timer

  InitLaunchPadCore();             //Initial settings for the system

  BCSCTL1 = CALBC1_1MHZ;           //Set calibrated range for DCO to 1MHz
  DCOCTL = CALDCO_1MHZ;            //Set DCO to 1MHz
  BCSCTL2 |= DIVS_3;               //Set SMCLK = DCO/8 = 125kHz

  /* Establish baseline for the proximity sensor */
  TI_CAPT_Init_Baseline(&proximity_sensor);
  TI_CAPT_Update_Baseline(&proximity_sensor,5);



  while (1)
  {
    //get current state
    system_state currentState = state;
    switch(currentState){
    case SLEEP:
         //put micro to sleep. wake up when motion sensor senses
         P1OUT &= ~LIGHT;               //Set the Light LED to low
         P2IE  |=  (BIT6);              //Enable interrupt for P2.6
         P2IES &= ~(BIT6);              // trigger interrupt on high to low
         P2IFG  &=  ~(BIT6);            //  P2.6    IFG clear
         _low_power_mode_3();
         break;
    case WAKING:
        //turn on lights and set to default setting
        P1OUT |= LIGHT; // lights on
        fadeLight(display_value);//set light to display value
        //set state to idle
        state = IDLE;
        break;
    case IDLE:
        //continually poll proximity sensor until activity then remeasure baseline
        CapTouchIdleMode();
        MeasureCapBaseLine();
        break;
        case ACTIVE:
        //read the gestures of the cap touch
        CapTouchActiveMode();
     }
  }
}

/****************************FUNCTIONS DEFINITIONS*****************************/

/*------------------------------------ myprint ---------------------------------
 * Print a string
 * ---------------------------------------------------------------------------*/
void myprint(char *msg) {
  TimerA_UART_init();
  TimerA_UART_print(msg);
  TimerA_UART_shutdown();
}

/*----------------------------------- SendData --------------------------------
 * Send the string printed in "myprint" via UART
 * ---------------------------------------------------------------------------*/
void SendData(unsigned char touch)
{
  char buf[32];
  memset(buf, 0, sizeof(buf));
  sprintf(buf, "%d", touch);
  myprint(buf);
  myprint("\r\n");
}

/* ----------------------------InitLaunchPadCore--------------------------------
* Initial Settings for the project. That includes port definition and
* initialization
*-----------------------------------------------------------------------------*/
void InitLaunchPadCore(void)
{
  BCSCTL1 |= DIVA_0;             //ACLK/(0:1, 1:2, 2:4, 3:8)
  BCSCTL3 |= LFXT1S_2;           //Select ACLK from VLO (no crystal, 12kHz)
  
  //Set the display pins as outputs low
  P1OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT7);
  P1OUT &= ~LIGHT;               //Set the Light LED to low
  //Set the display pins as outputs
  P1DIR |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT7);
  P1DIR |= LIGHT;                //Set P1.6 as output

  P2SEL = 0x00;                  //Configure P2 as I/Os, No XTAL
  P2OUT &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);    //Cap touch sensors
  P2OUT &= ~DISPLAY;             //Set display enable pin to low
  P2DIR |= (BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5);  //Cap touch sensors
  P2DIR |= DISPLAY;              //Set the display enable pin as output
  P2DIR &= ~BIT6;                //Set P2.6 as input for the motion sensor
  P2IE  |=  (BIT6);              //  Enable  interrupt   for P2.6
  P2IES &= ~(BIT6);              // trigger interrupt on high to low
  P2IFG  &=  (~BIT6);            //  P2.6    IFG clear
}

/* ----------------setUpTimers-----------------------------------------
 * Setup for timers 1 and 2
 * timer 1 - proximity sensor polling
 * timer 2 - motion sensor polling
 * ------------------------------------------------------------------------*/
void setUpTimer1(void){
  TACCR1 = 100; // number of cycles in timer
  TACTL = TASSEL_1 + MC_1; // use aclk for timer source and use up mode
  TACCTL1 |= CCIE; //enable interrupts on timer
}
void setUpTimer2(void){

  TACCR2 = 65535; // number of cycles in timer. ~5secs
  TACTL = TASSEL_1 + MC_1; // use aclk for timer source and use up mode
  TACCTL2 |= CCIE; //enable interrupts on timer
}

/* ----------------CapTouchIdleMode-----------------------------------------
 * Device stays in LPM0 'sleep' mode, only Proximity Sensor is used to detect
 * any movement triggering device wake up                                  
 * ------------------------------------------------------------------------*/ 
void CapTouchIdleMode(void)
{
//  Send status via UART: 'sleep' = [0xDE, 0xAD]
  myprint("captouchIdleMode- SLEEP_MODE_UART_CODE: ");
  SendData(SLEEP_MODE_UART_CODE);
  
  BCSCTL1 = CALBC1_1MHZ;                //Set calibrated range for DCO to 1MHz
  DCOCTL = CALDCO_1MHZ;                 //Set DCO to 1MHz
  BCSCTL2 |= DIVS_3;                    //Set SMCLK = DCO/8 = 125KHz
  
  P2OUT &= ~DISPLAY;                       //Set display enable pin to low
  deltaCnts[0] = 0;
  
  //set up second timer to see if it needs to go back to deeper sleep.
  setUpTimer2();

  /* Sleeping in LPM3 with ACLK/100 = 12Khz/100 = 120Hz wake up interval */
  /* Measure proximity sensor count upon wake up */
  /* Wake up if proximity deltaCnts > THRESHOLD */  
  do
  {//checks every 8.3 ms

    setUpTimer1();
    _low_power_mode_0();
    //__bis_SR_register(LPM3_bits+GIE);
    TACCTL1 &= ~CCIE;//disable interrupts on timer
    if(state == SLEEP){return;}

    TI_CAPT_Custom(&proximity_sensor,deltaCnts);
  } while (deltaCnts[0] <= PROXIMITY_THRESHOLD);
  TACCTL2 &= ~CCIE;                 //disable interrupts on timer 2 due to activity
  P2OUT |= DISPLAY;                 //Set display enable pin to high
  __delay_cycles(100);                //give time for lcd to power on
  state = ACTIVE;
  lcd_init();                       //LCD initial settings
  send_string("Welcome");
  send_command(SECONDLINE);         //move cusor to the second line
  send_string("Level: ");
  send_number(display_value);


}
 
/* ----------------MeasureCapBaseLine--------------------------------------
 * Re-measure the baseline capacitance of the wheel elements & the center  
 * button. To be called after each wake up event.                          
 * ------------------------------------------------------------------------*/
void MeasureCapBaseLine(void)
{
  BCSCTL1 = CALBC1_8MHZ;        //Set calibrated range for DCO to 8MHz
  DCOCTL = CALDCO_8MHZ;         //Set DCO to 8MHz
  BCSCTL2 |= DIVS_3;            //Set SMCLK = DCO/8 = 1MHz
  
  TI_CAPT_Init_Baseline(&wheel);
  TI_CAPT_Update_Baseline(&wheel,2);
  TI_CAPT_Init_Baseline(&middle_button);
  TI_CAPT_Update_Baseline(&middle_button,2);  
}

/* ----------------GetGesture----------------------------------------------
 * Determine immediate gesture based on current & previous wheel position
 * ------------------------------------------------------------------------*/
unsigned char GetGesture(unsigned char wheel_position)
{
 unsigned char gesture = INVALID_GESTURE, direction, ccw_check, cw_check; 
// ******************************************************************************
// gesturing
// determine if a direction/swipe is occuring
// the difference between the initial position and
// the current wheel position should not exceed 8
// 0-1-2-3-4-5-6-7-8-9-A-B-C-D-E-F-0...
//
// E-F-0-1-2:  cw, 4
// 2-1-0-F-E: ccw, 4
// A-B-C-D-E-F

  //if(initial_wheel_position == INVALID_WHEEL_POSITION)
  //{
    //gesture = 0;
    //initial_wheel_position = wheel_position;
  //}
  //else

  if(last_wheel_position != ILLEGAL_SLIDER_WHEEL_POSITION) 
  {
    if(last_wheel_position  > wheel_position)
    {
      // E-D-C-B-A:  ccw, 4
      // counter clockwise: 0 < (init_wheel_position - wheel_position) < 8
      //                    gesture = init_wheel_position - wheel_position
      //
      // E-F-0-1-2:  cw, 4
      // clockwise:        0 < (init_wheel_position+wheel_position)-16 <8
      //                    
      ccw_check = last_wheel_position  - wheel_position;
      if(ccw_check < 8)
      {
        gesture = ccw_check;
        direction = COUNTER_CLOCKWISE;
      }
      else
      {
        // E-F-0-1-2:  cw, 4
        // 16 - 14 + 2 = 4
        cw_check = 16 - last_wheel_position  + wheel_position ;
        if(cw_check < 8)
        {
            gesture = cw_check;
            direction = CLOCKWISE;
        }
      }
    }
    else 
    {
      // initial_wheel_position <= wheel_position
      //
      // 2-1-0-F-E: ccw, 4
      // counter clockwise: 
      //                    0 < (init_wheel_position+wheel_position)-16 <8
      //                    gesture = init_wheel_position - wheel_position
      //
      // 0-1-2-3-4:  cw, 4
      // clockwise:        0 < (wheel_position - init_wheel_position) < 8
      //    
      cw_check = wheel_position - last_wheel_position ;
      if(cw_check < 8)
      {
        gesture = cw_check;
        direction = CLOCKWISE;
      }
      else
      {
        // 2-1-0-F-E: ccw, 4
        // 16 + 2 - 14 = 4
        ccw_check = 16 + last_wheel_position  - wheel_position ;
        if(ccw_check < 8)
        {
          gesture = ccw_check;
          direction = COUNTER_CLOCKWISE;
        }
      }
    }
  } 

  if (gesture == INVALID_GESTURE)
  	return gesture;
  if (direction == COUNTER_CLOCKWISE)
    return (gesture + 16);
  else
    return gesture;
}

/* ----------------------- CapTouchActiveMode ----------------------------------
 * Determine immediate gesture based on current & previous wheel position
 * ---------------------------------------------------------------------------*/
void CapTouchActiveMode()
{
  unsigned char idleCounter, activeCounter;
  unsigned char gesture, gestureDetected; 
  unsigned char centerButtonTouched = 0;
  unsigned int wheelTouchCounter = WHEEL_TOUCH_DELAY - 1;
  unsigned int mag_gesture = 0;          //
  int direction_gesture;          //Direction of gesture. ccw --> -1; cw --> 1
  
  gesture = INVALID_GESTURE;      // Wipes out gesture history
  
  /* Send status via UART: 'wake up' = [0xBE, 0xEF] */  
  myprint("CapTouchActiveMode- WAKE_UP_UART_CODE: ");
  SendData(WAKE_UP_UART_CODE);

  idleCounter = 0;
  activeCounter = 0;
  gestureDetected = 0;
  
  while (idleCounter++ < MAX_IDLE_TIME)
  {  
    BCSCTL1 = CALBC1_8MHZ;       //Set calibrated range for DCO to 8MHz
    DCOCTL = CALDCO_8MHZ;        //Set DCO to 8MHz
    BCSCTL2 |= DIVS_3;           //Set SMCLK = DCO/8 = 1MHz
    TACCTL0 &= ~CCIE;   

    /*update LCD screen */
    send_command(FIRSTLINE);
    send_string("Change light");
    send_command(SECONDLINE);      //move cusor to the second line
    send_string("Level: ");
    send_number(display_value);

    wheel_position = ILLEGAL_SLIDER_WHEEL_POSITION;
    wheel_position = TI_CAPT_Wheel(&wheel);
    
    /* Process wheel touch/position/gesture  if a wheel touch is registered*/
    /* Wheel processing has higher priority than center button*/
    if(wheel_position != ILLEGAL_SLIDER_WHEEL_POSITION)
    {
      centerButtonTouched = 0;

      /* Adjust wheel position based: rotate CCW by 2 positions */
      if (wheel_position < 0x08)
      {
         wheel_position += 0x40 - 0x08;
      }
      else
      {
         wheel_position -= 0x08;
            /* Adjust wheel position based: rotate CCW by 2 positions */
      }
      
      wheel_position = wheel_position >>2;  // divide by four
               
      gesture = GetGesture(wheel_position);            
      
      /* Add hysteresis to reduce toggling between wheel positions if no gesture 
        has been TRULY detected. */
      if ((gestureDetected==0) && ((gesture<=1) || (gesture==0x11) || (gesture==0x10)))
      {
        if (last_wheel_position != ILLEGAL_SLIDER_WHEEL_POSITION)
            wheel_position = last_wheel_position;
        gesture = 0;
      }
      
      if ((gesture != 0) && (gesture != 16) && (gesture != INVALID_GESTURE))           
      { /* A gesture has been detected */ 
        if (gestureDetected == 0)
        { /* Starting of a new gesture sequence */
          gestureDetected = 1;
        } 
                 
        /* Transmit gesture & position via UART to PC */
        myprint("CapTouchActiveMode- gesture: ");
        SendData(gesture);

        //
        if(gesture >= 16)
        {
          mag_gesture = gesture - 16;
          direction_gesture = -1;
        }
        else
        {
          mag_gesture = gesture;
          direction_gesture = 1;
        }

        //
        display_value += (2*mag_gesture*direction_gesture);

        if (display_value > 100)
        {
          display_value = 100;
        }
        if(display_value < 0)
        {
          display_value = 0;
        }

        myprint("CapTouchActiveMode- display_value: ");
        SendData(display_value);
        fadeLight(display_value);
      }

      else
        if (gestureDetected==0)
        { /* If no gesture was detected, this is constituted as a touch/tap */
          if (++wheelTouchCounter >= WHEEL_TOUCH_DELAY)
          {
          	/* Transmit wheel position [twice] via UART to PC */
          	wheelTouchCounter = 0;
          }
        }
       	else
          wheelTouchCounter = WHEEL_TOUCH_DELAY - 1;      	
        
      idleCounter = 0;                      // Reset idle counter
      activeCounter++;
      last_wheel_position = wheel_position;
    } 
    else   
    { /* no wheel position was detected */
       
      if(TI_CAPT_Button(&middle_button))
      { /* Middle button was touched */   
        if (centerButtonTouched==0)
        {
          /* Transmit center button code [twice] via UART to PC */
          myprint("CapTouchActiveMode- MIDDLE_BUTTON_CODE: ");
          SendData(MIDDLE_BUTTON_CODE);
          fadeLight(display_value);

          centerButtonTouched = 1;
          
          if(display_value < 100)
          {
            display_value = display_value + (20 - (display_value % 20));
          }
          else if(display_value == 100)
          {
            display_value = 0;
          }

          myprint("CapTouchActiveMode- Display value: ");
          SendData(display_value);
          fadeLight(display_value);
        }

        idleCounter = 0;
      }
      else    
      { /* No touch was registered at all [Not wheel or center button */
        centerButtonTouched = 0;
        if ((gesture == INVALID_GESTURE) || (gestureDetected == 0))
        { /* No gesture was registered previously */
          if (last_wheel_position  != ILLEGAL_SLIDER_WHEEL_POSITION)
            {
              wheelTouchCounter = WHEEL_TOUCH_DELAY - 1;
            }
        }
      }
      
      // Reset all touch conditions, turn off LEDs, 
      last_wheel_position= ILLEGAL_SLIDER_WHEEL_POSITION;      
      gesture = INVALID_GESTURE;
      gestureDetected = 0;
    } 
    
  /* ------------------------------------------------------------------------
   * Option:
   * Add delay/sleep cycle here to reduce active duty cycle. This lowers power
   * consumption but sacrifices wheel responsiveness. Additional timing 
   * refinement must be taken into consideration when interfacing with PC
   * applications GUI to retain proper communication protocol.
   * -----------------------------------------------------------------------*/
  } // while has been idle for too long. to to idle mode
  state = IDLE;
}

//Function used to fade the LED
void fadeLight(int valuePWM)
{
  P1SEL |= BIT6;                //set P1.6 to be used as TA0.1
  CCR0 = 100 - 0;               // PWM Period
  CCTL1 = OUTMOD_7;             // CCR1 reset/set
  CCR1 = valuePWM;              // CCR1 PWM duty cycle
  TACTL = TASSEL_2 + MC_1;      // SMCLK, up mode
}

 /************************* INTERRUPTS ***************************************/


// Interrupt service routine for Timer_A
#pragma vector= TIMER0_A1_VECTOR
__interrupt void TA2_ISR (void){
	switch(__even_in_range(TAIV, TA0IV_TACCR2)){ //faster checks instead of just switch(TAIV)
	case TA0IV_TACCR1:
		//time to poll the proximity sensor.get out of low power mode
		break;
	case TA0IV_TACCR2:
		// no one touched the wheel in 5 seconds. check to see if anyone is in the room
		if((P1IN & BIT6) != BIT6){
			//no one is in the room, go to deep sleep.
			state = SLEEP;
			TACCTL2 &= ~CCIE;//disable interrupts on timer 2
		}
	break;
	}

	__low_power_mode_off_on_exit();
}

//  Port    2   interrupt   service routine
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
	if((P2IFG & BIT6) == BIT6){ //If the motion sensor tripped...
		//user has walked into room. wake up system and set state to waking
		__low_power_mode_off_on_exit();// restores active mode
		state = WAKING;

		P2IE  &=  ~(BIT6); //  Disable  interrupt   for P2.6 so it doesn't trip accidentally
		P2IFG  &=  (~BIT6);    //  P2.6    IFG clear
	}
}
