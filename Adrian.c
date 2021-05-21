 ///*
 //* Project Title: Pendulum Mimicking System
 //* Name: Adrian Wong Chee Seng
 //* Student ID: 28711327
 //* Name: Lim Wan Chang
 //* Student ID: 28667530
 //*/ 
//
///* Header Files */
//#include <asf.h>
//#include "stdio_serial.h"
//#include "conf_board.h"
//#include "conf_clock.h"
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#include "FastPID.h"
//#include <stdio.h>
//#include <assert.h>
//#include <string.h>
//#include <math.h>
//
///* Arduino Pin Initialization */
//#define MY_DIR					PIO_PC26_IDX //ARDUINO DUE DIGITAL PIN 4
//#define MY_INTERRUPT_ENCODER_B	PIO_PC28_IDX //ARDUINO DUE DIGITAL PIN 3
//#define MY_INTERRUPT_ENCODER_A	PIO_PB25_IDX //ARDUINO DUE DIGITAL PIN 2
//#define MY_SENSOR_INTERRUPT		PIO_PC24_IDX //ARDUINO DUE DIGITAL PIN 6
//#define MAX_MOTOR_PULSE			196			 //Maximum motor pulse 98*2 as resolution doubles for both edges
//
///* Queue and Semaphore handler */
//xSemaphoreHandle xBinarySemaphore,xCountingSemaphore;
//xQueueHandle xQueue;
//
///** PWM frequency in Hz */
//#define PWM_FREQUENCY      1000
///** Period value of PWM output waveform */
//#define PERIOD_VALUE       100
///** Initial duty cycle value */
//#define INIT_DUTY_VALUE    0
//
///** PWM channel instance for motor */
//pwm_channel_t g_pwm_channel_led0;
//
///* Initialized PID parameters */
//float Kp=5, Ki=0.42, Kd=3.4, Hz=10;
//int output_bits = 16;
//bool output_signed = true;
//
///* Initialized variable */
//uint32_t g_SetpointPulse=MAX_MOTOR_PULSE/2; //Midpoint of maximum pulses
//static int pulse=MAX_MOTOR_PULSE/2; //Initial setpoint
//int direction;
//float L=0.3; //Length of pendulum
//float g=9.8; //Gravitational acceleration
//float T=1.1; //Period of pendulum
//float dTheta=10.00; //Angle for measuring maximum angular velocity
//float omega=sqrt(9.8/0.3); //Angular frequency
//int setpoint[18]={0}; //Array for storing setpoints
//int index_set=0;
//int stop=1; //Stop flag
//
///* Function prototypes */
//static void configure_console(void);
//static void vSpeedTask( void *pvParameters );
//static void vPhotoHandlerTask( void *pvParameters );
//static void vSetPointTask( void *pvParameters );
//void pin_encoder_A_handler(const uint32_t id, const uint32_t index);
//void pin_edge_handler(const uint32_t id, const uint32_t index);
//
//int main(void)
//{
	//int i = 0;
	//uint32_t val;
//
	///* Initialize the SAM system. */
	//sysclk_init();
	//board_init();
//
	///* Initialize IO ports */
	//ioport_set_pin_dir(MY_INTERRUPT_ENCODER_A, IOPORT_DIR_INPUT);
	//ioport_set_pin_dir(MY_INTERRUPT_ENCODER_B, IOPORT_DIR_INPUT);
	//ioport_set_pin_dir(MY_SENSOR_INTERRUPT, IOPORT_DIR_INPUT);
	//ioport_set_pin_dir(MY_DIR, IOPORT_DIR_OUTPUT);
//
	//configure_console();
	//
	//// Initialize the Fast PID controller based on the given parameters.
	//InitializeFastPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
//
	///* PWM CONFIG */
//
	///* Enable PWM peripheral clock */
	//pmc_enable_periph_clk(ID_PWM);
//
	///* Disable PWM channels for LEDs */
	//pwm_channel_disable(PWM, PWM_CHANNEL_5);
//
	///* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	//pwm_clock_t clock_setting = {
		//.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		//.ul_clkb = 0,
		//.ul_mck = sysclk_get_cpu_hz()
	//};
	//pwm_init(PWM, &clock_setting);
//
	///* Initialize PWM channel for PWM_CHANNEL_5 */
	///* Period is left-aligned */
	//g_pwm_channel_led0.alignment = PWM_ALIGN_LEFT;
	///* Output waveform starts at a low level */
	//g_pwm_channel_led0.polarity = PWM_LOW;
	///* Use PWM clock A as source clock */
	//g_pwm_channel_led0.ul_prescaler = PWM_CMR_CPRE_CLKA;
	///* Period value of output waveform */
	//g_pwm_channel_led0.ul_period = PERIOD_VALUE;
	///* Duty cycle value of output waveform */
	//g_pwm_channel_led0.ul_duty = INIT_DUTY_VALUE;
	//g_pwm_channel_led0.channel = PWM_CHANNEL_5; //Arduino due Pin 8
//
	//pwm_channel_init(PWM, &g_pwm_channel_led0);
	//
	///* Enable PWM channels for LEDs */
	//pwm_channel_enable(PWM, PWM_CHANNEL_5);
//
	///* Encoder A Interrupt */
	//pmc_enable_periph_clk(ID_PIOB);
	//pio_set_input(PIOB, PIO_PB25, PIO_PULLUP);
	//pio_handler_set(PIOB, ID_PIOB, PIO_PB25, PIO_IT_EDGE, pin_encoder_A_handler);
	//pio_enable_interrupt(PIOB, PIO_PB25);
	//NVIC_SetPriority(PIOB_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	//NVIC_EnableIRQ(PIOB_IRQn);
	//
	///* Sensor Interrupt */
	//pmc_enable_periph_clk(ID_PIOC);
	//pio_set_input(PIOC, PIO_PC24, PIO_PULLUP);
	//pio_handler_set(PIOC, ID_PIOC, PIO_PC24, PIO_IT_RISE_EDGE, pin_edge_handler);
	//pio_enable_interrupt(PIOC, PIO_PC24);
	//NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
	//NVIC_EnableIRQ(PIOC_IRQn);
	//
	///* Create semaphores and queue */
	//vSemaphoreCreateBinary(xBinarySemaphore);
    //xCountingSemaphore = xSemaphoreCreateCounting( 100, 0 );
    //xQueue=xQueueCreate(100,sizeof(uint32_t));
	//
	///* Create tasks */
	//if(xBinarySemaphore!=NULL)
	//{
		//xTaskCreate( vPhotoHandlerTask, "PhotoelectricSensor", 960, NULL, 3, NULL );
		//xTaskCreate( vSetPointTask, "SetPoint", 960, NULL, 2, NULL );
		//xTaskCreate( vSpeedTask, "SpeedTask", 960, NULL, 1, NULL );
		//vTaskStartScheduler(); //Start task scheduler
	//}
//
	//for( ;; );
	//return 0;
//}
//
//void pin_encoder_A_handler(const uint32_t id, const uint32_t index)
//{
	//static unsigned int encoder_A=0;
	//static unsigned int encoder_B=0;
	//
	//if(((id == ID_PIOB) && (index == PIO_PB25))){
		//encoder_A=ioport_get_pin_level(MY_INTERRUPT_ENCODER_A);
		//encoder_B=ioport_get_pin_level(MY_INTERRUPT_ENCODER_B);
	//}
	//
	///* Keeping track of motor pulse using motor encoders*/
	//if((encoder_A==1&&encoder_B==0)||(encoder_A==0&&encoder_B==1))
	//{
		//pulse=(pulse+1)%MAX_MOTOR_PULSE;
	//}
	//else
	//{
		//pulse=pulse-1;
		//if(pulse<0){
			//pulse=MAX_MOTOR_PULSE+pulse;
		//}
	//}
//}
//
//void pin_edge_handler(const uint32_t id, const uint32_t index)
//{
	//portBASE_TYPE xHighPriorityTaskWoken=pdFALSE;
	//if ((id == ID_PIOC) && (index == PIO_PC24))
	//{
		//xSemaphoreGiveFromISR(xBinarySemaphore, &xHighPriorityTaskWoken);
	//}
	//portEND_SWITCHING_ISR( xHighPriorityTaskWoken );
//}
//
//static void vPhotoHandlerTask( void *pvParameters )
//{
	//portBASE_TYPE xStatus;
	//unsigned int pinVal = 0;
	//int counter=0;
	//int flag=0;
	//int direction_array[2];
	//int count=0;
	//int first_time;
	//int duration;
	//xSemaphoreTake(xBinarySemaphore,0);
	//
	//for( ;; ){
		//xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		//
		//pinVal = ioport_get_pin_level(MY_SENSOR_INTERRUPT);
		//
		///* Record the first photoelectric triggered time*/
		//if(count==0)
		//{
			//first_time=xTaskGetTickCount();
		//}
		//// Record the accumulated counts for each side of the cardboard
		//while(pinVal)
		//{
			//if(counter<500000)
			//{
				//counter++;
			//}
			//else
			//{
				//stop=1;
			//}
			//pinVal = ioport_get_pin_level(MY_SENSOR_INTERRUPT);
		//}
		//direction_array[count]=counter;
		//counter=0;
		//count++;
		//if(count==2)
		//{
			///* Calculate the time taken for the cardboard to cut through */
			//duration=xTaskGetTickCount()-first_time;
			//if(direction_array[0]<direction_array[1])
			//{
				//direction=0;//counterclockwise
			//}
			//else
			//{
				//direction=1;//clockwise
			//}
			//if(xQueue!=NULL && xBinarySemaphore!=NULL)
			//{
				//xStatus=xQueueSendToBack(xQueue, &duration, portMAX_DELAY);
				//if(xStatus==pdPASS)
				//{
					//xSemaphoreGive(xCountingSemaphore);
				//}
				//else
				//{
					//printf("Failed to send to queue\n");
				//}
			//}
			//count=0;
		//}
	//}
//}
//
//static void vSetPointTask( void *pvParameters )
//{
	//portBASE_TYPE xStatus;
	//uint32_t received;
	//float w_max;
	//double angle_max;
	//int setpoint_max;
	//int i=1;
	//float t;
	//float step_size=0.2;
	//xSemaphoreTake(xCountingSemaphore,0);
	///* As per most tasks, this task is implemented within an infinite loop. */
	//for( ;; )
	//{
		//xSemaphoreTake(xCountingSemaphore,portMAX_DELAY);
		//xStatus=xQueueReceive(xQueue,&received,100/portTICK_RATE_MS);
		//if(xStatus==pdPASS)
		//{
			///* Max setpoint (amplitude of the sinusoidal wave) calculation */
			//w_max=(dTheta/180*M_PI)/(((float)received)/1000); //Maximum angular velocity at lowest point
			//angle_max=acos((double)(1-w_max*w_max*L/(2*g)))*1.4/M_PI*180; //Maximum angle
			//setpoint_max=(int)(angle_max/360*MAX_MOTOR_PULSE); //Scaled maximum angle to maximum setpoint
			//
			///* Adjusting PID parameter for different angles */
			//if(angle_max<15){
				//Kp=8, Ki=0.5, Kd=4; //Small Amplitude
			//}
			//else if(angle_max<25){
				//Kp=5, Ki=0.42, Kd=3.4; //Moderate Amplitude
			//} 
			//else if(angle_max<50){
				//Kp=3, Ki=0.385, Kd=3.7; //Large Amplitude
			//}
			//
			///* Reset the setpoint array before putting in new values */
			//memset(setpoint,0,sizeof(setpoint));
			//
			////Filling the setpoint array with sinusoidal value offsets scaled between the maximum setpoint
			//while(i<6)
			//{
				//t=i*step_size*T/4;
				//setpoint[i-1]=setpoint_max*sin((double)(omega*t));
				//setpoint[9-i]=(setpoint_max*sin((double)(omega*t)))*0.3;
				//setpoint[8+i]=setpoint_max*sin((double)(omega*t));
				//setpoint[18-i]=setpoint_max*sin((double)(omega*t));
				//i++;
			//}
			//i=1;
			////Adjusting the setpoints using the offset calculated for different directions
			//if(direction==0)
			//{
				//int i=0;
				//for(i=0;i<9;i++)
				//{
					//setpoint[i]+=MAX_MOTOR_PULSE/2;
					//setpoint[9+i]=MAX_MOTOR_PULSE/2-setpoint[9+i];
				//}
			//}
			//else
			//{
				//int i=0;
				//for(i=0;i<9;i++)
				//{
					//setpoint[9+i]+=MAX_MOTOR_PULSE/2;
					//setpoint[i]=MAX_MOTOR_PULSE/2-setpoint[i];
				//}
			//}
			//stop=0;
			//index_set=0;
		//}
		//else
		//{
			//printf("Failed to read from queue\n");
		//}
//
	//}
//}
//
//static void vSpeedTask( void *pvParameters )
//{
	///* This tasks sets the direction of the mimicking system depending on the motion of the pendulum */
	///* It uses the calculated set points array to determine the swinging amplitude */
	//int16_t outputPID=0;
	//int16_t output=0;
	//for(;;){
		//while(index_set<18)
		//{
			///* Stop variable will be 1 initially */
			///* Stop variable will be 1 if the photoelectric sensor is triggered for a long time */
			//if(stop==1)
			//{
				//g_SetpointPulse=MAX_MOTOR_PULSE/2;
			//}
			//else
			//{
				//g_SetpointPulse=setpoint[index_set];
			//}
			//
			///* PID calculation */
			//outputPID = step(g_SetpointPulse,pulse);
			//output = abs(outputPID);
			//output = (int)((float)output*PERIOD_VALUE/550);
			//
			///* Determining direction */
			//if(pulse<g_SetpointPulse)
			//{
				////pwm_channel_update_duty(PWM,&g_pwm_channel_led0,0);
				//ioport_set_pin_level(MY_DIR,0);
				//pwm_channel_update_duty(PWM,&g_pwm_channel_led0,output);
			//}
			//else if(pulse>g_SetpointPulse)
			//{
				////pwm_channel_update_duty(PWM,&g_pwm_channel_led0,0);
				//ioport_set_pin_level(MY_DIR,1);
				//pwm_channel_update_duty(PWM,&g_pwm_channel_led0,output);
			//}
			//else
			//{
				//pwm_channel_update_duty(PWM,&g_pwm_channel_led0,0);
				//index_set++;
				////Reusing the same setpoints from previous swing if pendulum is not detected
				//if(index_set==17)
				//{
					//index_set=0;
				//}
			//}
			//vTaskDelay(1); //Delay the task to prevent the setpoints and PID to be updated too quickly
		//}
	//}
//}
//
///* UART console configuration */
//static void configure_console(void)
//{
	//const usart_serial_options_t uart_serial_options = {
		//.baudrate = CONF_UART_BAUDRATE,
		//.paritytype = CONF_UART_PARITY
	//};
//
	///* Configure console UART. */
	//sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	//stdio_serial_init(CONSOLE_UART, &uart_serial_options);
//}
//
//void vApplicationIdleHook( void )
//{
//}
//void vApplicationMallocFailedHook( void )
//{
	//for( ;; );
//}
//void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
//{
	//for( ;; );
//}
//void vApplicationTickHook( void )
//{
//}