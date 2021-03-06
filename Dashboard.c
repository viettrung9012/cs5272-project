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
short potentiometer;
short slidesensor;
short forcesensor;
extern short SlideSensor;

unsigned char A0,A1; //A0 - Button 3.5, A1 - Button 3.6
unsigned char B0 = 0,B1 = 0,B2 = 0,B3 = 0,B4 = 0,B5 = 0,B6 = 0,B7 = 0; //B0-B7 represent LED's 0 through 7

extern OS_MBX p_box;
extern OS_MBX s_box;
extern OS_MBX f_box;

float BRAKE_MULTIPLIER = 0.07f;
float FRICTION_MULTIPLIER = 0.09f;
float MIN_FRICTION = 1.0f;
float ACC_MULTIPLIER = 0.03f;
float speed = 0;
int MAX_SPEED = 200;


unsigned short DARKNESS_THRESHOLD = 2; // if headlight brightness is equal or below this, it is dark.
unsigned short MAX_BRIGHTNESS = 5;
unsigned short MAX_POTENTIOMETER = 1023;

unsigned short ALARM_DELAY = 100; // 100
unsigned short PWM_DELAY = 2; // 2 : (MAX_BRIGHTNESS+1) * PWM_DELAY = PWM_CYCLE_LENGTH
unsigned short SENSOR_DELAY = 200; // 200
unsigned short LCD_STATE_DELAY = 100; // 100
unsigned short ACD_DELAY = 100; // 100

unsigned short LCD_STATE_DURATION_CYCLES = 5; // 5 (1.0s) duration of LCD state
unsigned short DIM_DURATION_CYCLES = 2; // 2 (0.4s) SENSOR_DELAY*DIM_DURATION_CYCLES = duration of each brightness state when dimming
unsigned short CAR_OPEN_ON_LIGHT_CYCLES = 100; // 100 (20s) SENSOR_DELAY*CAR_OPEN_ON_LIGHT_CYCLES = duration until lights turn off
unsigned short CAR_CLOSE_ON_CYCLES = 50; // 50 (10s) SENSOR_DELAY*CAR_CLOSE_ON_CYCLES = duration until lights dim off

unsigned short LCD_STATE_PRIORITY = 4;
unsigned short ALARM_PRIORITY = 3;
unsigned short PWM_PRIORITY = 2;
unsigned short SENSOR_PRIORITY = 1;
unsigned short ACD_PRIORITY = 0;

unsigned short headlight_brightness,internal_brightness; // 0 to MAX_BRIGHTNESS
bool isAlarmOn = FALSE;
bool isDoorOpen = FALSE;
bool isEngineOn = FALSE;
bool doorStateChanged = FALSE;
bool engineStateChanged = FALSE;

unsigned char wasAOpressed = 0;
unsigned char wasA1pressed = 0;

bool isInternalLightDimming = FALSE;
unsigned int internalLightDimCounter = 0;

unsigned int i, counter;
unsigned char SENSOR = 0;

OS_MUT LCD_Mutex;

void mbx_read(void) {
	void *p_msg, *s_msg, *f_msg;
	
	os_mbx_wait(&p_box, &p_msg, 0xffff);
	os_mbx_wait(&s_box, &s_msg, 0xffff);
	os_mbx_wait(&f_box, &f_msg, 0xffff);
	
	potentiometer = *(short *) p_msg;
	slidesensor = *(short *) s_msg;
	forcesensor = *(short *) f_msg;
	
	free(p_msg);
	free(s_msg);
	free(f_msg);
	
}

void write_LCD(){
	char ss[5];
	char pm[5];
	
	mbx_read();
	os_mut_wait(&LCD_Mutex, 0xFFFF);
	sprintf(ss,"%d",(int)(speed));//speed
	sprintf(pm,"%d",potentiometer);//potentiometer
	LCD_on(); // Turn on LCD
	LCD_cls();
	LCD_puts("Speed: ");
	LCD_puts(ss);
	LCD_gotoxy(1,2);  // switch to the second line
	LCD_puts("Light: ");
	LCD_puts(pm);
	LCD_cur_off ();
	
	os_mut_release(&LCD_Mutex);
}
//Function to read input
void read_buttons()
{
	//BUTTON_3_5:
	A0 = !(GPIO3->DR[0x080]>>5); // Returns 1 when pressed and 0 when released
	//BUTTON_3_6:
	A1 = !(GPIO3->DR[0x100]>>6); // Returns 1 when pressed and 0 when released
	
	engineStateChanged = doorStateChanged = FALSE;
	
	if(wasAOpressed && A0 == 0){
		doorStateChanged = TRUE;
		isDoorOpen = !isDoorOpen;
	}
	if(wasA1pressed == TRUE && A1 == 0){
		engineStateChanged = TRUE;		
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
OS_TID LCD_state_controller_id; // Declare variable t_Lighting to store the task id

void internalLightOn(){
	internal_brightness = MAX_BRIGHTNESS;
	isInternalLightDimming = FALSE;
}
void internalLightOff(){
	internal_brightness = 0;
	isInternalLightDimming = FALSE;
}
void internalLightDim(){	
	internalLightDimCounter = FALSE;
	isInternalLightDimming = TRUE;
}

enum CAR_States { CLOSE_OFF, OPEN_ON, OPEN_OFF, CLOSE_ON} CAR_State;
int TickFct_Sensor(int state) {
	switch(state) { // Transitions
		case -1:{
			state = CLOSE_OFF;				 
			break;
		}
		case CLOSE_OFF:{ // Door close, light off
			if (doorStateChanged && isDoorOpen) { // Open door
				if (potentiometer * MAX_BRIGHTNESS / MAX_POTENTIOMETER <= DARKNESS_THRESHOLD) {// Outside is dark
					internalLightOn();
								state = OPEN_ON; // Door open, light on
					counter = 0; // Start counter
				} else { // Outside is dark
					state = OPEN_OFF;
				}
			}
			else{ // Nothing happens
				state = CLOSE_OFF;
			}
			break;
		}
		case OPEN_ON:{ // Door open, light on
			if (engineStateChanged && isEngineOn){ // Engine starts
				internalLightOff();
				state = OPEN_OFF;
			} else {
				if (counter <= CAR_OPEN_ON_LIGHT_CYCLES){ // <= 20s
					if (doorStateChanged && !isDoorOpen){ // Close door
						state = CLOSE_ON;
						counter = 0;
					} else {
						counter++;
					}
				} else { // > 20s
					internalLightOff();
					state = OPEN_OFF;
				}
			}
			break;
		}
		case OPEN_OFF:{ // Door open, light off
			if (doorStateChanged && !isDoorOpen){ // Close door
				internalLightOn();
				state = CLOSE_ON;
				counter = 0;
			}
			break;
		}
		case CLOSE_ON:{ // Door close, light on
			if (engineStateChanged && isEngineOn) { // Engine starts
				internalLightOff();
				state = CLOSE_OFF;
			} else {
				if (counter <= CAR_CLOSE_ON_CYCLES){ // <= 10s
					counter++;
				} else {
					internalLightDim();
					state = CLOSE_OFF;
				}
			}
			break;
		}
		default:{
			state = -1;
			break;
		}
	} // Transitions
	CAR_State = state;
	return state;
}

enum ENGINE_States { CLOSE_OFF_ENGINE, OPEN_ON_ENGINE, OPEN_OFF_ENGINE, CLOSE_ON_ENGINE} ENGINE_State;
int EngineTickFct_Sensor(int state) {
	switch(state) { // Transitions
		case -1:{
			state = CLOSE_OFF_ENGINE;				 
			break;
		}
		case CLOSE_OFF_ENGINE:{ // Door close, engine off
			if (doorStateChanged && isDoorOpen) { // Open door
				state = OPEN_OFF_ENGINE;
			} else if (engineStateChanged && isEngineOn){ // Engine starts
				state = CLOSE_ON_ENGINE;
			}
			break;
		}
		case OPEN_ON_ENGINE:{ // Door open, engine on
			if (engineStateChanged && !isEngineOn){ // Engine stops
				state = OPEN_OFF_ENGINE;
				isAlarmOn = FALSE;
			} else if (doorStateChanged && !isDoorOpen){ // Close door
				state = CLOSE_ON_ENGINE;
				isAlarmOn = FALSE;
			}
			break;
		}
		case OPEN_OFF_ENGINE:{ // Door open, engine off
			if (doorStateChanged && !isDoorOpen){ // Close door
				state = CLOSE_OFF_ENGINE;
			} else if (engineStateChanged && isEngineOn){ // Engine starts
				state = OPEN_ON_ENGINE;
				isAlarmOn = TRUE;
			}
			break;
		}
		case CLOSE_ON_ENGINE:{ // Door close, engine on	
			if (engineStateChanged && !isEngineOn) { // Engine stops
				state = CLOSE_OFF_ENGINE;
			} else if (doorStateChanged && isDoorOpen){ // Open door
				state = OPEN_ON_ENGINE;
				isAlarmOn = TRUE;
			}
			break;
		}
		default:{
			state = -1;
			break;
		}
	} // Transitions
	ENGINE_State = state;
	return state;
}
static void update_speed(){
	double acc;
	double friction = speed * FRICTION_MULTIPLIER;
	
	if(friction<MIN_FRICTION) friction = MIN_FRICTION;
	acc = ((isEngineOn)? slidesensor*ACC_MULTIPLIER : 0) - forcesensor * BRAKE_MULTIPLIER - friction;
	speed += (int)acc;
	if(speed>MAX_SPEED) speed = MAX_SPEED;
	if(speed<0) speed = 0;
}
static void update_brightness(){
	headlight_brightness = (MAX_POTENTIOMETER - potentiometer) * 5 / MAX_POTENTIOMETER;
	if(isInternalLightDimming && ++internalLightDimCounter % DIM_DURATION_CYCLES == 0){ //Every 0.4 seconds
		internal_brightness --;
		if(internal_brightness == 0) isInternalLightDimming = FALSE; 
	}
}
static void update_led(int pulse_state){
	B0 = B1 = B2 = (headlight_brightness > pulse_state)? 1:0;
	B3 = B4 = B5 = (internal_brightness > pulse_state)? 1:0;	
}

__task void TASK_LCD_STATE(void) {
  int lcd_task_state_counter = -1;
	bool isHoldingMutex = FALSE;
	bool isWaiting = FALSE;
  os_itv_set(LCD_STATE_DELAY);
  while(1){
		if(lcd_task_state_counter > -1) lcd_task_state_counter++;
		if(!isWaiting && (doorStateChanged || engineStateChanged)){
			if(!isHoldingMutex){
				isWaiting = TRUE;
				os_mut_wait(&LCD_Mutex, 0xFFFF);
				isWaiting = FALSE;
				isHoldingMutex = TRUE;
			}
			LCD_on(); // Turn on LCD
			LCD_cls();
			
			LCD_puts("Engine is ");
			LCD_puts((isEngineOn)?"on":"off");
			LCD_gotoxy(1,2);  // switch to the second line
			LCD_puts("Door is ");
			LCD_puts((isDoorOpen)?"opened":"closed");
			LCD_cur_off ();
			lcd_task_state_counter = 0;
		}
		if(lcd_task_state_counter >= LCD_STATE_DURATION_CYCLES){
			lcd_task_state_counter = -1;
			os_mut_release(&LCD_Mutex);
			isHoldingMutex = FALSE;
		}
		os_itv_wait();
	}
}

__task void TASK_Alarm(void) {
	bool b = FALSE;
  os_itv_set(ALARM_DELAY);
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

__task void TASK_PWM(void) {
  int pulse_state = 0;
  int pulse_states = MAX_BRIGHTNESS + 1;
	const unsigned int taskperiod = PWM_DELAY; // Copy task period value in milliseconds here
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
  int en_state = -1;
	
	const unsigned int taskperiod = SENSOR_DELAY; // Copy task period value in milliseconds here
  os_itv_set(taskperiod);
	
	while(1){
		read_buttons();
		update_speed();
		update_brightness();
		state = TickFct_Sensor(state);
		en_state = EngineTickFct_Sensor(en_state);
		os_itv_wait();
   } // while (1)
}

/*----------------------------------------------------------------------------
 *        Task 1 'ADC_Con': ADC Conversion
 *---------------------------------------------------------------------------*/
__task void ADC_Con(void){
  // timing
	const unsigned int period = ACD_DELAY;
	os_itv_set(period);	
		for(;;){ 
		os_itv_wait();
		/* Do actions below */
			start_ADC();
			//mbx_read();
			write_LCD();
		}
}	 // End ADC_Con(void)


/*----------------------------------------------------------------------------
 *        Task 0 'init': Initialize
 *---------------------------------------------------------------------------*/
__task void init (void) {

  unsigned int n = 0;
  		  
	/* Set up potentiometer and Light sensor*/                                                              
  
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
	
	os_mut_init (&LCD_Mutex);

 	counter=0;
	
	os_tsk_create(ADC_Con,ACD_PRIORITY);	
  Car_controller_id = os_tsk_create(TASK_SENSOR,SENSOR_PRIORITY);
  PWM_controller_id = os_tsk_create(TASK_PWM,PWM_PRIORITY);
  Alarm_controller_id = os_tsk_create(TASK_Alarm,ALARM_PRIORITY);
  LCD_state_controller_id = os_tsk_create(TASK_LCD_STATE,LCD_STATE_PRIORITY);
  
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
