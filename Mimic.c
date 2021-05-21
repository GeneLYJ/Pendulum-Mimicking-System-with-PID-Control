//// Name: Ng Lik Sern, Tay Quan Lin
//// ID: 28252101, 26721384
//// authcate: lngg0002, qltay1
//#include <asf.h>
//#include "stdio_serial.h"
//#include "conf_board.h"
//#include "conf_clock.h"
///* FreeRTOS includes. */
//#include "FreeRTOS.h"
//#include "task.h"
//#include "queue.h"
//#define SENSOR_INTERRUPT PIO_PC24_IDX // ARDUINO DUE DIGITAL PIN 6
//#define MY_OUTPUT_1 PIO_PC23_IDX // ARDUINO DUE DIGITAL PIN 7
//#define DIRCONTROL PIO_PC26_IDX // ARDUINO DUE DIGITAL PIN 4
//#define ENCODERA	PIO_PB25_IDX //ARDUINO DUE DIGITAL PIN 2
//#define ENCODERB	PIO_PC28_IDX //ARDUINO DUE DIGITAL PIN 3
//
 ///** PWM frequency in Hz */
 //#define PWM_FREQUENCY      1000
 ///** Period value of PWM output waveform */
 //#define PERIOD_VALUE       200
 ///** Initial duty cycle value */
 //#define INIT_DUTY_VALUE    0
//
 ///** PWM channel instance for motor */
 //pwm_channel_t g_pwm_channel_led0;
///* The task function. */
//void vMimicTask( void *pvParameters );
//void pin_edge_handler(const uint32_t id, const uint32_t index);
//void encoder_handler(const uint32_t id, const uint32_t index);
//void vInitSpeedTask( void *pvParameters );
//
//xQueueHandle xQueue;
//xSemaphoreHandle xBinarySemaphore;
//
//// Global variables
//int32_t g_time1 = 0;
//int32_t g_time2 = 0;
//int32_t g_period = 0; // g_period actually half of a full oscillation period
//int32_t g_pulses = 0;
//int16_t g_dir = 0;
//bool first = HIGH;
//
//// Lookup tables
//int32_t speedShort[40] = {5,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,32,33,34,35,36,39,40,42,44,47,48,52,54,59,61};
//int32_t speedLong[40] = {11,15,17,19,21,23,25,26,28,30,32,34,36,38,40,42,43,45,47,50,51,53,55,57,59,62,65,66,68,72,75,79,83,88,92,98,103,112,117,130};
//int32_t lookUpSpeed[41] = {17,15,13,12,11,11,11,11,10,10,10,10,10,9,9,9,9,9,9,8,8,8,8,8,8,7,7,7,7,7,7,7,7,7,6,6,6,6,6,5,0};
//
//static void configure_console(void)
//{
	//const usart_serial_options_t uart_serial_options = {
		//.baudrate = CONF_UART_BAUDRATE,
		//#ifdef CONF_UART_CHAR_LENGTH
		//.charlength = CONF_UART_CHAR_LENGTH,
		//#endif
		//.paritytype = CONF_UART_PARITY,
		//#ifdef CONF_UART_STOP_BITS
		//.stopbits = CONF_UART_STOP_BITS,
		//#endif
	//};
	///* Configure console UART. */
	//sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	//stdio_serial_init(CONSOLE_UART, &uart_serial_options);
//}
//
//int main( void )
//{
	///* This function initializes the MCU clock */
	//sysclk_init();
	///* Board initialization */
	//
	//board_init();
//
	//xQueue = xQueueCreate(5, sizeof(int16_t));
	//vSemaphoreCreateBinary( xBinarySemaphore );
//
	//ioport_init();
//
	//ioport_set_pin_dir(SENSOR_INTERRUPT, IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(SENSOR_INTERRUPT, IOPORT_MODE_PULLUP);
	//ioport_set_pin_dir(MY_OUTPUT_1, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_dir(DIRCONTROL, IOPORT_DIR_OUTPUT);
	//ioport_set_pin_mode(DIRCONTROL, IOPORT_MODE_PULLUP);
	//ioport_set_pin_dir(ENCODERA,IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(ENCODERA, IOPORT_MODE_PULLUP);
	//ioport_set_pin_dir(ENCODERB,IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(ENCODERB, IOPORT_MODE_PULLUP);
	///* Initialize the serial I/O(console ) */
	//configure_console();
	//pmc_enable_periph_clk(ID_PIOC);
	//pio_set_input(PIOC, PIO_PC24, PIO_PULLUP);
	//pio_handler_set(PIOC, ID_PIOC, PIO_PC24, PIO_IT_EDGE, pin_edge_handler);
	//pio_enable_interrupt(PIOC, PIO_PC24);
	//NVIC_SetPriority(PIOC_IRQn,(11 << (8 - configPRIO_BITS)));
	//NVIC_EnableIRQ(PIOC_IRQn);
	//
	//pmc_enable_periph_clk(ID_PIOB);
	//pio_set_input(PIOB, PIO_PB25, PIO_PULLUP);
	//pio_handler_set(PIOB, ID_PIOB, PIO_PB25, PIO_IT_EDGE, encoder_handler);
	//pio_enable_interrupt(PIOB, PIO_PB25);
	//NVIC_SetPriority(PIOB_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
	//NVIC_EnableIRQ(PIOB_IRQn);
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
	//g_pwm_channel_led0.alignment = PWM_ALIGN_LEFT;
	///* Output waveform starts at a low level */
	//g_pwm_channel_led0.polarity = PWM_LOW;
	///* Use PWM clock A as source clock */
	//g_pwm_channel_led0.ul_prescaler = PWM_CMR_CPRE_CLKA;
	///* Period value of output waveform */
	//g_pwm_channel_led0.ul_period = PERIOD_VALUE;
	///* Duty cycle value of output waveform */
	//g_pwm_channel_led0.ul_duty = INIT_DUTY_VALUE;
	//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
//
	//pwm_channel_init(PWM, &g_pwm_channel_led0);
	//
	///* Enable PWM channels for LEDs */
	//pwm_channel_enable(PWM, PWM_CHANNEL_6);
//
	//if(xQueue != NULL && xBinarySemaphore != NULL)
	//{
	//xTaskCreate( vMimicTask, "Task 1", 1024, NULL, 2, NULL );
	//xTaskCreate( vInitSpeedTask, "Task 2", 1024, NULL, 3, NULL );
	//vTaskStartScheduler();
	//}
	//else
	//{
		//printf("Could not create queue or semaphore");
	//}
	//for( ;; );
//}
//
//void pin_edge_handler(const uint32_t id, const uint32_t index)
//{
	//unsigned int pinVal = 0;
	//static int16_t counter = 0;
	//static int32_t prevTime = 0;
	//static int32_t periodStart = 0;
	//portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//
	//if((id != ID_PIOC) || (index != PIO_PC24)){
		//return;
	//}
//
	//pinVal = ioport_get_pin_level(SENSOR_INTERRUPT);
//
	//// Check current counter to identify which part of pendulum triggered sensor 
	//if(pinVal&&counter==0) // If first part
	//{
		//prevTime = xTaskGetTickCountFromISR();
		//if(g_time1>=g_time2) // Predict direction from previous data 
		//{
			//g_dir = 1;
			//printf("Left\n");
		//}
		//else
		//{
			//g_dir = 2;
			//printf("Right\n");
		//}
		//counter++;
	//}
	//else if(!pinVal&&counter==1) // If second part
	//{
		//g_time1 = xTaskGetTickCountFromISR()-prevTime;
		//prevTime = xTaskGetTickCountFromISR();
		//g_period = xTaskGetTickCountFromISR()-periodStart;
		//periodStart = xTaskGetTickCountFromISR();
		//if(g_period>=500&&g_period<=800) // Checking for uninterrupted oscillation
		//{
			////printf("Semaphore sent\n");
			//xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
		//}
		//else
		//{
			//first = HIGH; // Reset control variable to offset speed when starting oscillation
		//}
		//counter++;
	//}
	//else if(pinVal&&counter==2) // If third part
	//{
		//if(xTaskGetTickCountFromISR()-prevTime>300) // Checking to ensure counter matches pendulum
		//{
			//counter = 1;
		//}
		//else
		//{
			//counter++;
		//}
		//prevTime = xTaskGetTickCountFromISR();
	//}
	//else if(!pinVal&&counter==3)
	//{
		//g_time2 = xTaskGetTickCountFromISR()-prevTime;
		//printf("Time1: %d\tTime2: %d\t\n",g_time1,g_time2);
		////printf("Period: %d\n",g_period);
		//counter = 0;
	//}
	//else
	//{
		//return;
	//}
	//portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	//
//}
//
//void encoder_handler(const uint32_t id, const uint32_t index)
//{
	//static int16_t state = 0;
	//static int32_t riseTime = 0;
	//bool encoderA = LOW;
	//bool encoderB = LOW;
	//portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//
	//if((id != ID_PIOB) || (index != PIO_PB25)){
		//return;
	//}
//
	//encoderA = ioport_get_pin_level(ENCODERA);
	//encoderB = ioport_get_pin_level(ENCODERB);
//
	//if(state==0&&encoderA)
	//{
		//state = 1;
		//riseTime = xTaskGetTickCountFromISR();
	//}
	//else if(state==1&&!encoderA&&(xTaskGetTickCountFromISR()-riseTime)>3)
	//{
		//if(encoderB)
		//{
			//g_pulses++;
		//}
		//else if(!encoderB)
		//{
			//g_pulses--;
		//}
		//state = 0;
//
		//if(g_pulses==96 || g_pulses==-96)
		//{
			//g_pulses=0;
			////printf("Full revolution\n");
		//}
//
		////printf("Count: %d\n",g_pulses);
	//}
	//else
	//{
		//return;
	//}
//
//}
//
//void vInitSpeedTask( void *pvParameters )
//{
	//portBASE_TYPE xStatus;
	//int16_t index = 0;
	//printf("task1 created\n");
	//xSemaphoreTake( xBinarySemaphore, 0);
//
	//for( ;; ){
		//xSemaphoreTake( xBinarySemaphore, portMAX_DELAY); // Triggered by middle pf pendulum passing by sensor
		//if(g_dir==1) // If pendulum heading clockwise
		//{
			//for(uint16_t i =0;i<40;i++) // Verify index of initial speed of motor using lookup table and pendulum passing duration
			//{
				//if(g_time1<=speedShort[i])
				//{
					//index = i;
					//break;
				//}
				//else if(g_time1>speedShort[39])
				//{
					//index = 40;
					//break;
				//}
			//}
		//}
		//else if(g_dir==2) // If pendulum heading counter-clockwise
		//{
			//for(uint16_t i =0;i<40;i++) // Verify index of initial speed of motor using lookup table and pendulum passing duration
			//{
				//if(g_time1<=speedLong[i])
				//{
					//index = i;
					//break;
				//}
				//else if(g_time1>speedLong[39])
				//{
					//index = 40;
					//break;
				//}
			//}
		//}
		//xStatus = xQueueSendToBack(xQueue,&index,portMAX_DELAY);
	//}
//}
//
//void vMimicTask( void *pvParameters )
//{
	////portBASE_TYPE xStatus;
	//int32_t startTime = 0;
	//int32_t duration = 0;
	//int16_t initSpeed = 0;
	//int16_t oriSpeed = 0;
	//int16_t halfSpeed = 0;
	//int16_t index = 0;
	//int16_t PWMduty = 0;
	//static int32_t currentPeriod = 0;
	//printf("task2 created\n");
	//
	//for( ;; ){
		//xQueueReceive(xQueue,&index,portMAX_DELAY); // Triggered when index received from other Task
		//initSpeed = lookUpSpeed[index]; // Acquire initial speed from lookup table
//
		//if(first) // If beginning new oscillation
		//{
			//oriSpeed = initSpeed;
			//initSpeed += initSpeed/2; // Offset speed to increase motor momentum
		//}
//
		//halfSpeed = initSpeed; // Initial speed from lookup table taken as half of max speed
		//initSpeed += halfSpeed/2; // Difference between min and max speeds for rising cycle
		//currentPeriod = g_period - 5; // Offset in period to increase synchronization
//
		//if(g_dir==1) // If clockwise
		//{
			//duration = 0;
			//startTime = xTaskGetTickCount();
			//ioport_set_pin_level(DIRCONTROL,HIGH);
			//while(duration<(currentPeriod/2)) // For half of period (Rising)
			//{
				//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
				//duration = xTaskGetTickCount()-startTime;
				//PWMduty = halfSpeed/2 + initSpeed - duration/((currentPeriod/2)/(initSpeed+1)); // Speed decreases with time while rising
				//pwm_channel_update_duty(PWM, &g_pwm_channel_led0, PWMduty);
			//}
//
			//initSpeed = halfSpeed; // Difference between min and max speeds for falling cycle
//
			//if(first) // If new oscillation
			//{
				//initSpeed = oriSpeed;
				//halfSpeed = initSpeed;
			//}
//
			//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
			////pwm_channel_update_duty(PWM, &g_pwm_channel_led0, 0);
			//duration = 0;
			//startTime = xTaskGetTickCount();
			//ioport_set_pin_level(DIRCONTROL,LOW);
//
			//while(duration<(currentPeriod/2)) // For half of period (Falling)
			//{
				//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
				//duration = xTaskGetTickCount()-startTime;
				//PWMduty = halfSpeed + duration/((currentPeriod/2)/(initSpeed+1)); // Speed increases with time
				//pwm_channel_update_duty(PWM, &g_pwm_channel_led0, PWMduty);
			//}
//
			//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
			//pwm_channel_update_duty(PWM, &g_pwm_channel_led0, 0); // Stop motor
//
		//}
		//else if(g_dir==2) // If counter-clockwise
		//{
			//duration = 0;
			//startTime = xTaskGetTickCount();
			//ioport_set_pin_level(DIRCONTROL,LOW);
//
			//while(duration<(currentPeriod/2)) // For half of period (Rising)
			//{
				//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
				//duration = xTaskGetTickCount()-startTime;
				//PWMduty = halfSpeed/2 + initSpeed - duration/((currentPeriod/2)/(initSpeed+1)); // Speed decreases with time
				//pwm_channel_update_duty(PWM, &g_pwm_channel_led0, PWMduty);
			//}
//
			//initSpeed = halfSpeed; // Difference between min and max speeds for falling cycle
//
			//if(first) // If new oscillation
			//{
				//initSpeed = oriSpeed;
				//halfSpeed = initSpeed;
			//}
//
			//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
			////pwm_channel_update_duty(PWM, &g_pwm_channel_led0, 0);
			//duration = 0;
			//startTime = xTaskGetTickCount();
			//ioport_set_pin_level(DIRCONTROL,HIGH);
//
			//while(duration<(currentPeriod/2)) // For half of period (Falling)
			//{
				//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
				//duration = xTaskGetTickCount()-startTime;
				//PWMduty = halfSpeed + duration/((currentPeriod/2)/(initSpeed+1)); // Speed increases with time
				//PWMduty += 1; // Offset to correct deviation
				//pwm_channel_update_duty(PWM, &g_pwm_channel_led0, PWMduty);
			//}
//
			//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
			//pwm_channel_update_duty(PWM, &g_pwm_channel_led0, 0); // Stop motor
		//}
//
		//if(first)
		//{
		//first = LOW;
		//}
	//}
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