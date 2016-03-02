/*
 * main.c
 *Connections
P1.0 - D4 Pin11
P1.1 - D5 Pin12
P1.2 - D6 Pin13
P1.3 - D7 Pin14
P1.4 - RS Pin4
P1.5 - R/W Pin5
P1.7 - E Pin7
P1.6 - LED light Pin6
 *
 ******************************************************************************/

#include  "msp430.h"
#include "lcd.h"
//definitions
#define     LIGHT                  BIT6
#define     SETTINGVAL             6


//global variables
volatile unsigned int lightSettings[SETTINGVAL] = {0,20,40,60,80,100};
volatile int currentLightSetting = 0;
//function prototypes
void fadeLight(int valuePWM);//Function used to fade the LED
void incrementSetting();//function used to increment light setting by 1

//todo: MA create menu list for users to scroll through
void main(void)
{

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
	P1DIR |= (LIGHT);				// set bit light as output

	//get lcd screen working
	lcd_init();
	send_string("Welcome");
	send_command(SECONDLINE);//move cusor to the second line
	send_string("Home!");

	while(1)
	{
		__delay_cycles(20000);
		fadeLight(0);//0%
		incrementSetting();
		__delay_cycles(20000);
		fadeLight(lightSettings[currentLightSetting]);//20%
		incrementSetting();
		__delay_cycles(20000);
		fadeLight(lightSettings[currentLightSetting]);//40%
		incrementSetting();
		__delay_cycles(20000);
		fadeLight(lightSettings[currentLightSetting]);//60%
		incrementSetting();
		__delay_cycles(20000);
		fadeLight(lightSettings[currentLightSetting]);//80%
		incrementSetting();
		__delay_cycles(20000);
		fadeLight(lightSettings[currentLightSetting]);//100%
		incrementSetting();
		__delay_cycles(20000);
		fadeLight(100);//0%




	}
}

//Function used to fade the LED
void fadeLight(int valuePWM)
{
	//P1SEL |= (BIT0 | BIT6);                    	// P1.0 and P1.6 TA1/2 options
	P1SEL |= BIT6;								//select bit6
	CCR0 = 100 - 0;                            	// PWM Period
	CCTL1 = OUTMOD_7;                           // CCR1 reset/set
	CCR1 = valuePWM;                            // CCR1 PWM duty cycle
	TACTL = TASSEL_2 + MC_1; // SMCLK, up mode
}

//function used to increment light setting by 1
void incrementSetting()
{
	currentLightSetting = (currentLightSetting + 1) % 6;
}

