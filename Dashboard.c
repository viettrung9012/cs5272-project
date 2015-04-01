/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    DASHBOARD_TEMPLATE.C
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code has been modified from the example Blinky project of the
				RealView Run-Time Library.
 *      Copyright (c) 2004-2011 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include <RTL.h>
#include <91x_lib.H>
#include "LCD.h"
#include <stdlib.h>

extern __irq void ADC_IRQ_Handler (void); /* ADC  interrupt routine           */
extern unsigned char AD_in_progress;      /* AD conversion in progress flag  */
extern short Potentiometer;
extern short SlideSensor;


unsigned char A0,A1; //A0 - Button 3.5, A1 - Button 3.6
unsigned char B0 = 0,B1 = 0,B2 = 0,B3 = 0,B4 = 0,B5 = 0,B6 = 0,B7 = 0; //B0-B7 represent LED's 0 through 7

unsigned short headlight_brightness,internal_brightness; // 0-5
bool isAlarmOn = 1;
bool isDoorOpen = 0;
bool isEngineOn = 0;
bool doorStateChanged = 0;
bool engineStateChanged = 0;

unsigned char wasAOpressed = 0;
unsigned char wasA1pressed = 0;

unsigned int i, counter;
unsigned char SENSOR = 0;

//Function to read input
void read_buttons()
{
	//BUTTON_3_5:
	A0 = !(GPIO3->DR[0x080]>>5); // Returns 1 when pressed and 0 when released
	//BUTTON_3_6:
	A1 = !(GPIO3->DR[0x100]>>6); // Returns 1 when pressed and 0 when released
	
	engineStateChanged = doorStateChanged = 0;
	
	if(wasAOpressed == 1 && A0 == 0){
		doorStateChanged = 1;
		isDoorOpen = !isDoorOpen;
	}
	if(wasA1pressed == 1 && A1 == 0){
		engineStateChanged = 1;		
		isEngineOn = !isEngineOn;
	}
	
	wasAOpressed = A0;
	wasA1pressed = A1;
}

void start_ADC ( )
{
	if (!AD_in_progress)  
	{             						    /* If conversion not in progress      */
        AD_in_progress = 1;                 /* Flag that AD conversion is started */
        ADC->CR |= 0x0423;                  /* Set STR bit (Start Conversion)     */
    }
	//Now the interrupt will be called when conversion ends
}

void read_input()
{
	//BUTTON_3_5:
	A0 = !(GPIO3->DR[0x080]>>5); // Returns 1 when pressed and 0 when released
	//BUTTON_3_6:
	A1 = !(GPIO3->DR[0x100]>>6); // Returns 1 when pressed and 0 when released
}

//Function to write to led
void write_led()
{
  unsigned char mask = 0;

  mask  = (B0<<0);
  mask |= (B1<<1);
  mask |= (B2<<2);
  mask |= (B3<<3);
  mask |= (B4<<4);
  mask |= (B5<<5);
  mask |= (B6<<6);
  mask |= (B7<<7);

  GPIO7->DR[0x3FC] = mask;
}


//Function to print an unsigned char value on the LCD Display
void print_uns_char (unsigned char value)
{	 
	int flag = 0,i;
	char c[4];
	do {
	   int rem = value%10;
	   value = value/10;
	   c[flag] = rem+48;
	   flag++;
	}while(value>0);
	for(i=flag-1;i>=0;i--)
		LCD_putc(c[i]);
}

OS_TID Car_controller_id; // Declare variable t_Lighting to store the task id
OS_TID PWM_controller_id; // Declare variable t_Lighting to store the task id
OS_TID Alarm_controller_id; // Declare variable t_Lighting to store the task id
OS_TID LCD_controller_id; // Declare variable t_Lighting to store the task id
enum SENSOR_States { TAP_OFF, TAP_ON} SENSOR_State;

void print_sensors(){
	int x = SlideSensor;
	int y = Potentiometer;
	char ss[5];
	char pm[5];
	sprintf(ss,"%d",SlideSensor);
	sprintf(pm,"%d",Potentiometer);
	LCD_on(); // Turn on LCD
	LCD_cls();
	LCD_puts("Speed: ");
	LCD_puts(ss);
	LCD_gotoxy(1,2);  // switch to the second line
	LCD_puts("Light: ");
	LCD_puts(pm);
	LCD_cur_off ();
}
enum CAR_States { CLOSE_OFF, OPEN_ON, OPEN_OFF, CLOSE_ON} CAR_State;

int TickFct_Sensor(int state) {
	switch(state) { // Transitions
      	case -1:
			state = CLOSE_OFF;				 
     		break;
	  	case CLOSE_OFF: // Door close, light off
	  	{
			if (doorStateChanged && (Potentiometer * 5 / 1023) <= 2 ) { // Open door
				if (Potentiometer * 5 / 1023 <= 2) {// Outside is dark
					B4 = B5 = B6 = 1; // LightOn
          			state = OPEN_ON; // Door open, light on
					counter = 0; // Start counter
				} else { // Outside is dark
					state = OPEN_OFF;
				}
        	}
			else // Nothing happens
				state = CLOSE_OFF;
			break;
		}
      	case OPEN_ON: // Door open, light on
		{
			if (counter <= 100){ // <= 20s
				if (doorStateChanged){ // Close door
					state = CLOSE_ON;
					counter = 0;
				} else {
					counter++;
				}
			} else { // > 20s
				B4 = B5 = B6 = 0;
				state = OPEN_OFF;
			}
			break;
		}
		case OPEN_OFF: // Door open, light off
		{
			if (doorStateChanged){ // Close door
				B4 = B5 = B6 = 1;
				state = CLOSE_ON
				counter = 0;
			}
			break;
		}
		case CLOSE_ON: // Door close, light on
		{
			if (counter <= 50){ // <= 10s
				counter++;
			} else {
				// need a dimOff method
				B4 = B5 = B6 = 0;
				state = CLOSE_OFF;
			}
			break;
		}
      default:
         state = -1; break;
      } // Transitions
   	CAR_State = state;
   	return state;
	//print_slider();
	//return state;
}
static void update_brightness(){
	headlight_brightness = (1023 - Potentiometer) * 5 / 1023;
	internal_brightness = SlideSensor * 6 / 550;
}
static void update_led(int pulse_state){
	B0 = B1 = B2 = (headlight_brightness > pulse_state)? 1:0;
	B3 = B4 = B5 = (internal_brightness > pulse_state)? 1:0;	
}
__task void TASK_Alarm(void) {
	bool b = 0;
  os_itv_set(100);
  while(1){
		b = !b;
		if(isAlarmOn){
			B6 = (b)?1:0;
			B7 = (b)?0:1;
		}else{
			B6 = B7 = 0;
		}
		os_itv_wait();
	}
}
__task void TASK_LCD(void) {
	const unsigned int taskperiod = 100; // Copy task period value in milliseconds here
  unsigned int LCDcounter = 0;
	bool isLCDused = 0;
  os_itv_set(taskperiod);
	while(1){
		if(doorStateChanged || engineStateChanged){
			LCDcounter = 0;
			isLCDused = 1;
		}else if(isLCDused){
			if(LCDcounter > 10){
				isLCDused = 0;
			}else{
				LCDcounter++;
			}
		}
		if(isLCDused){
			LCD_on(); // Turn on LCD
			LCD_cls();
			LCD_puts("Engine is ");
			LCD_puts((isEngineOn)?"on":"off");
			LCD_gotoxy(1,2);  // switch to the second line
			LCD_puts("Door is ");
			LCD_puts((isDoorOpen)?"opened":"closed");
			LCD_cur_off ();
		}else{
			print_sensors();
		}
		os_itv_wait();
   } // while (1)
}


__task void TASK_PWM(void) {
  int pulse_state = 0;
  int pulse_states = 6;
	const unsigned int taskperiod = 2; // Copy task period value in milliseconds here
  os_itv_set(taskperiod);
	while(1){
		update_led(pulse_state);
		write_led();
		if( ++pulse_state >= pulse_states ) pulse_state = 0;
		os_itv_wait();
	}
}
__task void TASK_SENSOR(void) {
  int state = -1;
	
	const unsigned int taskperiod = 200; // Copy task period value in milliseconds here
  os_itv_set(taskperiod);
	
	while(1){
		read_input();
		read_buttons();
		update_brightness();
		state = TickFct_Sensor(state);
		os_itv_wait();
   } // while (1)
}

/*----------------------------------------------------------------------------
 *        Task 1 'ADC_Con': ADC Conversion
 *---------------------------------------------------------------------------*/
__task void ADC_Con(void){
  // timing
	const unsigned int period = 100;
	os_itv_set(period);	
		for(;;){ 
		os_itv_wait();
		/* Do actions below */
		start_ADC();
		}
}	 // End ADC_Con(void)





/*----------------------------------------------------------------------------
 *        Task 0 'init': Initialize
 *---------------------------------------------------------------------------*/
__task void init (void) {

  unsigned int n = 0;
  		  
	/* Set up Potentiometer and Light sensor*/                                                              
  
  SCU->GPIOIN[4]  |= 0x07;                /* P4.0, P4.1 and P4.2 input  - mode 0             */
  SCU->GPIOOUT[4] &= 0xFFC0;              /* P4.0 output - mode 0             */

  /* To look up search for GPIO4_DIR in the peripherals manual 				  */
  GPIO4->DDR      &= 0xF8;                /* P4.0 and P4.1 direction - input  */
  SCU->GPIOANA    |= 0x0007;              /* P4.0 analog mode ON              */
	
	/* To understand the ADC configuration register consult the manual 				  */
	ADC->CR         |= 0x0042;              /* Set POR bit                      */
  for (n = 0; n < 100000; n ++);          /* Wait > 1 ms  (at 96 MHz)         */

  ADC->CR         &= 0xFFB7;              /* Clear STB bit                    */
  for (n = 0; n < 1500; n ++);            /* Wait > 15 us (at 96 MHz)         */

  ADC->CR         |= 0x0423;              /* Enable end of conversion interupt*/
  ADC->CCR         = 0x003F;              /* AD Conversion for Channels 0,1,2, No WDG on Ch 0    */

  /* Configure and enable IRQ for A/D Converter (ADC)                         */
  VIC0->VAiR[15]  = (unsigned int)ADC_IRQ_Handler; /* Setup ADC IRQ Hndl addr */
  VIC0->VCiR[15] |= 0x20;                 /* Enable the vector interrupt      */
  VIC0->VCiR[15] |= 15;                   /* Specify the interrupt number     */
  VIC0->INTER    |= (1<<15);              /* Enable ADC interrupt             */

  /* Configuring LED                     */
  SCU->GPIOOUT[7]  = 0x5555;
  GPIO7->DDR       = 0xFF;
  GPIO7->DR[0x3FC] = 0x00;

  /* LCD Setup                           */
  GPIO8->DDR       = 0xFF;
  GPIO9->DDR       = 0x07;

  /* Port 3 setup for button 3.5 and 3.6 */
  SCU->GPIOIN[3]  |= 0x60;
  SCU->GPIOOUT[3] &= 0xC3FF;
  GPIO3->DDR      &= 0x9F;

  LCD_init(); //Initialize LCD
  LCD_cur_off(); //Remove LCD cursor
  LCD_cls(); //Clearing LCD screen

 	counter=0;
	
	os_tsk_create(ADC_Con,0);	
  Car_controller_id = os_tsk_create(TASK_SENSOR,0);
  PWM_controller_id = os_tsk_create(TASK_PWM,1);
  Alarm_controller_id = os_tsk_create(TASK_Alarm,1);
  LCD_controller_id = os_tsk_create(TASK_LCD,1);
  
  os_tsk_delete_self ();
}


/*----------------------------------------------------------------------------
 *        Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {

  os_sys_init (init);                    /* Initialize RTX and start init    */
  return 0;
}

/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/