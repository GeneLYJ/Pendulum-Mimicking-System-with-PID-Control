///*
 //* ECE 4075 Assignment
 //*
 //* Written By 	Lau Wei Jian, 28939794
 //*				Seah Ming-yi, 28663217
 //*/ 
//
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
///** Define constants used */
//#define MY_ENCODER_A		PIO_PB25_IDX //ARDUINO DUE DIGITAL PIN 2
//#define MY_ENCODER_B		PIO_PC28_IDX //ARDUINO DUE DIGITAL PIN 3
//#define MY_DIR				PIO_PC26_IDX //ARDUINO DUE DIGITAL PIN 4 (LOW - C, HIGH - CC)
//#define MY_PHOTOELECTRIC	PIO_PC25_IDX //ARDUINO DUE DIGITAL PIN 5
//
//#define MAX_MOTOR_RPM 500
//#define PULSE_PER_REV 196
//#define MAX_COUNT_VALUES 500000
//
//#define CLOCKWISE 0
//#define C_CLOCKWISE 1
//
///** Constants for pendulum equation*/ 
//#define G 9.8
//#define L 0.35
//#define T 1.1
//
///** Array to store duration values. */	
//static int16_t duration_values[2] = {0};
//
///**
 //* \brief Configure UART console.
 //*/
//
 ///** PWM frequency in Hz */
 //#define PWM_FREQUENCY      1000
 ///** Period value of PWM output waveform */
 //#define PERIOD_VALUE       100
 ///** Initial duty cycle value */
 //#define INIT_DUTY_VALUE    0
//
 ///** PWM channel instance for motor. */
 //pwm_channel_t g_pwm_channel_motor;
 //
 ///** Declare variables of xSemaphoreHandle. */
 //xSemaphoreHandle xBinarySemaphore;
 //
 ///** Declare variable of xQueueHandle. */
 //xQueueHandle xQueue;
//
///** Declare global variables. */
//float Kp=4, Ki=0.5, Kd=3, Hz=10;
//int output_bits = 16;
//bool output_signed = true;
//int16_t g_SetpointPosition = 98;
//int16_t encoder_count = 98;
//int16_t counter_flag = 0, direction_flag = 0;
//int setpoint_array[18] = {0};
//int setpoint_index = 0;
//float W = sqrt(G/L);
//int stop_flag = 1;
//
//// Function prototypes
//void PWM_init(void);
//static void configure_console(void);
//static void encoder_handler(const uint32_t id, const uint32_t index);
//static void photoelectric_handler(const uint32_t id, const uint32_t index);
//static void vPositionTask( void *pvParameters );
//static void vSpeedTask( void *pvParameters );
//static void vCounterTask( void *pvParameters );
//void set_waypoint(int total_duration);
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
	///* Configure pin modes for encoders A and B. */
	//ioport_set_pin_dir(MY_ENCODER_A, IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(MY_ENCODER_A, IOPORT_MODE_PULLUP);
	//ioport_set_pin_dir(MY_ENCODER_B, IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(MY_ENCODER_B, IOPORT_MODE_PULLUP);
	//ioport_set_pin_dir(MY_DIR, IOPORT_DIR_OUTPUT);
	//
	///* Configure console. */
	//configure_console();
	//
	///* Initialize PWM channel. */
	//PWM_init();
		//
	//// Initialize the Fast PID controller based on the given parameters.
	//InitializeFastPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
//
	///* ENCODER A INTERRUPT */
	//pmc_enable_periph_clk(ID_PIOB);
	//pio_set_input(PIOB, PIO_PB25, PIO_PULLUP);
	//pio_handler_set(PIOB, ID_PIOB, PIO_PB25, PIO_IT_EDGE, encoder_handler);
	//pio_enable_interrupt(PIOB, PIO_PB25);
	//NVIC_SetPriority(PIOB_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	//NVIC_EnableIRQ(PIOB_IRQn);
	//
	///* PHOTO ELECTRIC INTERRUPT */
	//pmc_enable_periph_clk(ID_PIOC);
	//pio_set_input(PIOC, PIO_PC25, PIO_PULLUP);
	//pio_handler_set(PIOC, ID_PIOC, PIO_PC25, PIO_IT_EDGE, photoelectric_handler);
	//pio_enable_interrupt(PIOC, PIO_PC25);
	//NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
	//NVIC_EnableIRQ(PIOC_IRQn);
//
	///* Create queue that can store 100 data of type long. */
	//xQueue = xQueueCreate(100, sizeof(long));
	//
	///* Create semaphores. */
	//vSemaphoreCreateBinary(xBinarySemaphore);
//
	///* Create tasks and schedule them if semaphores and queue have been created successfully. */
	//if (xBinarySemaphore!= NULL && xQueue != NULL)
	//{
		//xTaskCreate( vPositionTask, "Position", 960, NULL, 1, NULL );
		//xTaskCreate( vSpeedTask, "Speed", 960, NULL, 2, NULL );
		//xTaskCreate( vCounterTask, "Counter", 960, NULL, 3, NULL );
		//
		//vTaskStartScheduler();
	//}
//
	//for( ;; );
	//return 0;
//}
//
///* Function to initialize PWM configurations. */
//void PWM_init(void)
//{
	///* PWM CONFIG */
//
	///* Enable PWM peripheral clock */
	//pmc_enable_periph_clk(ID_PWM);
//
	///* Disable PWM channels for LEDs */
	//pwm_channel_disable(PWM, PWM_CHANNEL_6);
//
	///* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	//pwm_clock_t clock_setting = {
		//.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		//.ul_clkb = 0,
		//.ul_mck = sysclk_get_cpu_hz()
	//};
	//pwm_init(PWM, &clock_setting);
//
	///* Initialize PWM channel for PWM_CHANNEL_6 */
	///* Period is left-aligned */
	//g_pwm_channel_motor.alignment = PWM_ALIGN_LEFT;
	///* Output waveform starts at a low level */
	//g_pwm_channel_motor.polarity = PWM_LOW;
	///* Use PWM clock A as source clock */
	//g_pwm_channel_motor.ul_prescaler = PWM_CMR_CPRE_CLKA;
	///* Period value of output waveform */
	//g_pwm_channel_motor.ul_period = PERIOD_VALUE;
	///* Duty cycle value of output waveform */
	//g_pwm_channel_motor.ul_duty = INIT_DUTY_VALUE;
	//g_pwm_channel_motor.channel = PWM_CHANNEL_6;
//
	//pwm_channel_init(PWM, &g_pwm_channel_motor);
	//
	///* Enable PWM channels for LEDs */
	//pwm_channel_enable(PWM, PWM_CHANNEL_6);
//}
//
///* Task to update the set point position. */
//static void vPositionTask( void *pvParameters )
//{
	//int16_t outputPID = 0, output = 0;
	//for(;;)
	//{
		///* If the pendulum is not stopped, the array storing set points from the modeled 
		//function is looped through and passed to the PID controller. */
		//while (setpoint_index < 18)
		//{
			///* If stop flag is set, meaning the pendulum is at its neutral position, set the PID setpoint
			//neutral position. */
			//if (stop_flag)
			//{
				//g_SetpointPosition = 98;
			//} else
			//{
				//g_SetpointPosition = setpoint_array[setpoint_index];
			//}
			//
			///* The set point and current encoder count is passed to the PID controller and 
			//the PID output is used to update the PWM duty cycle and direction of spin. */
			//outputPID = step(g_SetpointPosition,encoder_count);
			//g_pwm_channel_motor.channel = PWM_CHANNEL_6;
			//output = abs(outputPID);
			//output = (int)((float)output*PERIOD_VALUE/MAX_MOTOR_RPM);
//
			//if (outputPID > 0)
			//{
				//ioport_set_pin_level(MY_DIR, LOW);
				//pwm_channel_update_duty(PWM, &g_pwm_channel_motor, output);
			//}
			//else if (outputPID < 0)
			//{
				//ioport_set_pin_level(MY_DIR, HIGH);
				//pwm_channel_update_duty(PWM, &g_pwm_channel_motor, output);
			//}
			//else
			//{
				//pwm_channel_update_duty(PWM, &g_pwm_channel_motor, 0);
			//}
			//
			///* Updates the PID setpoint when the motor has reached the current setpoint */
			//if(abs(encoder_count - g_SetpointPosition) == 0)
			//{
				//setpoint_index++;
				//
				//if(setpoint_index == 17)
				//{
					//setpoint_index = 0;
				//}
			//}
			//
			///* Task is delayed for 1 tick to allow other tasks to run and so that the Integral error
			//doesn't accumulate too quickly. */
			//vTaskDelay(1);
			//
		//}
		//
	//}	
//}
//
///* Task to calculate maximum angular speed of pendulum. */
//static void vSpeedTask( void *pvParameters )
//{
	//long lReceivedValue;
	//long dutyCycle = 0;
	//uint32_t rpm = 0;
	//int angle = 0;
	//int setpoint_array[5] = {0};
	//portBASE_TYPE xStatus;
	//
	///* As per most tasks, this task is implemented within an infinite loop. */
	//for( ;; )
	//{
		///* Based on the total duration received from the queue, the task fills the 
		//set point array based on values obtained from the modeled sine function of 
		//the pendulum's motion. */
		//xStatus = xQueueReceive(xQueue, &lReceivedValue, portMAX_DELAY);
		//if(xStatus==pdPASS)
		//{
			///* Resets the index and setpoint array to 0 whenever the algorithm generates a new model
			//for the system to follow */
			//setpoint_index = 0;
			//memset(setpoint_array,0,sizeof(setpoint_array));
			//set_waypoint(lReceivedValue);
			//stop_flag = 0;
		//}
		//else
		//{
			//printf("Failed to read from queue\n");
		//}
		//
	//}
//}
//
///* Task to obtain duration of the pendulum swinging across the photoelectric sensor. */
//static void vCounterTask( void *pvParameters )
//{
	//long lReceivedValue;
	//uint32_t pinVal = 0;
	//static int counter = 0;
	//int max_count = 0;
	//int prev_tick = 0;
	//int curr_tick = 0;
	//int duration = 0;
	//int total_duration = 0;
	//portBASE_TYPE xStatus;
	//xSemaphoreTake(xBinarySemaphore,0);
	//
	///* As per most tasks, this task is implemented within an infinite loop. */
	//for( ;; )
	//{
		//xSemaphoreTake(xBinarySemaphore,portMAX_DELAY);
		//{
			///* The duration of each segment of the pendulum swinging across the photoelectric sensor
			//is recorded using xTaskGetTickCount() and stored in a 2-variable array. A counter is 
			//incremented while the photoelectric sensor detects the pendulum. If the counter exceeds 
			//MAX_COUNT_VALUES, the pendulum is determined to have stopped. */ 
			//pinVal = ioport_get_pin_level(MY_PHOTOELECTRIC);
			//prev_tick = xTaskGetTickCount();
			//while(pinVal)
			//{
				//if(counter<MAX_COUNT_VALUES)
				//{
					//counter++;
				//}
				//else
				//{
					//stop_flag = 1;
					//break;
				//}
				//pinVal = ioport_get_pin_level(MY_PHOTOELECTRIC);
			//}	
			//curr_tick = xTaskGetTickCount();
			//duration = curr_tick - prev_tick;
			//
			///* To determine the direction of the pendulum's swing, the length of the durations stored 
			//in the array is compared.*/
			//if (!counter_flag)
			//{
				//duration_values[0] = duration;
				//counter_flag = 1;
			//}
			//else if (counter_flag)
			//{
				//duration_values[1] = duration;
				//
				//if(duration_values[0] > duration_values[1])
				//{
					//direction_flag = CLOCKWISE;
					//total_duration = duration_values[0] + 2*duration_values[1];				
				//}
				//else
				//{
					//direction_flag = C_CLOCKWISE;
					//total_duration = duration_values[1] + 2*duration_values[0];
				//}
				//counter_flag = 0;
				///* Once the total duration is computed, write it to the queue for vSpeedTask to read */
				//if(xQueue != NULL)
				//{
					//xStatus = xQueueSendToBack(xQueue,&total_duration,portMAX_DELAY);
				//}
				//duration_values[0] = 0;
				//duration_values[1] = 0;
			//}
			//counter=0;
		//}
	//}
//}
//
///* Function to calculate way points that the system should follow. */
//void set_waypoint(int total_duration)
//{
	//float ang_speed = 0;
	//double angle = 0;
	//int max_setpoint = 0;
	//float separation = 0.2;
	//float t;
	//int temp = 0;
	//
	///* Calculate the pendulum's maximum angular speed and the maximum way
	//point the system should reached. */
	//ang_speed = 10/(0.001*total_duration)/180*M_PI;
	//angle = acos((double)(1-(ang_speed*ang_speed*L/(2*G))))/M_PI*180;
	//max_setpoint =  (int)((angle * 0.7) / 360 * PULSE_PER_REV);
	//
	///* The maximum way point is stored in an array accordingly. */
	//setpoint_array[4] = 98 + max_setpoint;
	//setpoint_array[13] = 98 - max_setpoint;
	//
	///* The array is filled up with remaining way points obtained from a 
	//sine function used to model the pendulum's theoretical motion. */
	//for (int i = 1; i < 5; i++)
	//{
		//t = 0.25*(5-i)*separation*T;
		//setpoint_array[4 + i] = max_setpoint*sin((double)(W*t)) + 98;
		//setpoint_array[4 - i] = max_setpoint*sin((double)(W*t)) + 98;
		//setpoint_array[13 + i] = max_setpoint*sin((double)(W*t + M_PI)) + 98;
		//setpoint_array[13 - i] = max_setpoint*sin((double)(W*t + M_PI)) + 98;
	//} 
//
	///* If the pendulum is swinging in the counter-clockwise direction, reorder the elements in the 
	//setpoint array to represent a phase shift of pi */
	//if (!direction_flag)
	//{
		//for (int i = 0; i<9; i++)
		//{
			//temp = setpoint_array[i];
			//setpoint_array[i] = setpoint_array[i+9];
			//setpoint_array[i+9] = temp;
			//
		//}
	//}
//}
//
///* ISR for encoder A. */
//static void encoder_handler(const uint32_t id, const uint32_t index)
//{
	//static uint32_t encoderAVal = 0, encoderBVal = 0;
	//portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	//
	///* If an edge is detected, the encoder count will increases or decreases according
	//to the direction of spin. */
	//if (id == ID_PIOB && index == PIO_PB25)
	//{
		//encoderAVal = ioport_get_pin_level(MY_ENCODER_A);
		//encoderBVal = ioport_get_pin_level(MY_ENCODER_B);
		//
		//if ((encoderAVal == 0 && encoderBVal == 1) || (encoderAVal == 1 && encoderBVal == 0))
		//{
			//encoder_count ++;
		//}
		//else
		//{
			//encoder_count --;
		//}
		//
		//if (encoder_count > PULSE_PER_REV)
		//{
			//encoder_count -= PULSE_PER_REV;
		//}
		//else if (encoder_count < 0)
		//{
			//encoder_count = PULSE_PER_REV + encoder_count;
		//}
	//}
//}
//
///* ISR for photoelectric sensor. */
//static void photoelectric_handler(const uint32_t id, const uint32_t index)
//{
	//portBASE_TYPE xHighPriorityTaskWoken=pdFALSE;
	//portBASE_TYPE xStatus;
	//int pinVal = 0;
	//static int counter = 0;
	//int max_count = 0;
	//static bool previousState = LOW;
	//bool currentState = LOW;
	//
	//if ((id == ID_PIOC) && (index == PIO_PC25))
	//{
		///* If there is a change in pin level, a semaphore will be given to allow
		//vCounterTask to run. */
		//currentState = ioport_get_pin_level(MY_PHOTOELECTRIC);
		//if(currentState != previousState && currentState == HIGH)
		//{
			//xSemaphoreGiveFromISR(xBinarySemaphore,&xHighPriorityTaskWoken);
		//}
		//
		//previousState = currentState;
	//}
	//portEND_SWITCHING_ISR( xHighPriorityTaskWoken );
//}
//
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