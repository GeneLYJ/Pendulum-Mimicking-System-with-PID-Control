/* Pendulum Mimicking System Assignment
 * ID: 28696492
 *
 * Created: 5/15/2021 00:44:12 AM
 * Author: Low You Jun
 */ 

#include <asf.h>
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "FastPID.h"
#include <stdio.h>
#include <assert.h>
#include <string.h>
#include <math.h>
#include "stdlib.h"


/* Pin Define */
#define PHOTOSENSOR	PIO_PC24_IDX  // Pin 6
#define MOTOR_DIR_PIN PIO_PC26_IDX //ARDUINO DUE DIGITAL PIN 4
#define ENCA PIO_PC28_IDX // Pin 3
#define ENCB PIO_PB25_IDX // Pin 2
#define PWM6 PIO_PC23_IDX // Pin 7

#define CW 1	// Clockwise ++
#define CCW 0	// --

#define HALFROTATION 96
#define ORIGIN 0
#define STOP 800

/** PWM frequency in Hz */
#define PWM_FREQUENCY      1000
/** Period value of PWM output waveform */
#define PERIOD_VALUE       100
/** Initial duty cycle value */
#define INIT_DUTY_VALUE    0


/* PID configuration */
float Kp= 0.2, Ki= 0.01 , Kd=0.0, Hz=10;
int output_bits = 16;
bool output_signed = true;
 
/** PWM channel instance for motor. */
pwm_channel_t g_pwm_channel_led0;
 
/** Declare variables of xSemaphoreHandle. */
xSemaphoreHandle xBinarySemaphore;
 
/** Declare variable of xQueueHandle. */
xQueueHandle xQueue;


// Encoder counts
int g_pulses = 45;
int flag = 2;
int motorPeriod;
float ratio = 0;


//void ADC_Handler(void);
static void configure_console(void);
//static uint32_t adc_read_buffer(Adc * pADC, int16_t * pwBuffer, uint32_t dwSize);
static void vHandlerTask( void *pvParameters );
static void vPosPend( void *pvParameters );
static void motor( void *pvParameters);

//ISR
static void photoelectric_handler(const uint32_t id, const uint32_t index);
void encoder_handler(const uint32_t id, const uint32_t index);


int main(void){
	
	int i = 0;
	uint32_t val;
	BaseType_t res;

	/* Initialize the SAM system. */
	sysclk_init();
	board_init();
	
	/* Configure console. */
	configure_console();
	
	/* PID */
	// Initialize the Fast PID controller based on the given parameters.
	InitializeFastPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
	
	/* IOPORT */
	ioport_set_pin_dir(PHOTOSENSOR, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(PHOTOSENSOR, IOPORT_MODE_PULLUP);
	
	ioport_set_pin_dir(ENCA, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(ENCA, IOPORT_MODE_PULLUP);
	ioport_set_pin_dir(ENCB, IOPORT_DIR_INPUT);
	ioport_set_pin_mode(ENCB, IOPORT_MODE_PULLUP);
	
	ioport_set_pin_dir(MOTOR_DIR_PIN, IOPORT_DIR_OUTPUT);
	
	
	/* PWM CONFIG */
	pmc_enable_periph_clk(ID_PWM);
	pwm_channel_disable(PWM, PWM_CHANNEL_6);
	
	pwm_clock_t clock_setting = {
		.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);
	
	g_pwm_channel_led0.alignment = PWM_ALIGN_LEFT;
	g_pwm_channel_led0.polarity = PWM_LOW;
	g_pwm_channel_led0.ul_prescaler = PWM_CMR_CPRE_CLKA;
	g_pwm_channel_led0.ul_period = PERIOD_VALUE;
	g_pwm_channel_led0.ul_duty = INIT_DUTY_VALUE;
	g_pwm_channel_led0.channel = PWM_CHANNEL_6;
	
	pwm_channel_init(PWM, &g_pwm_channel_led0);
	pwm_channel_enable(PWM, PWM_CHANNEL_6);
	//ioport_set_pin_level(PWM6, LOW);
	ioport_set_pin_level(MOTOR_DIR_PIN, LOW);
	
	/* PHOTO ELECTRIC INTERRUPT */
	pmc_enable_periph_clk(ID_PIOC);
	pio_set_input(PIOC, PIO_PC24, PIO_PULLUP);
	pio_handler_set(PIOC, ID_PIOC, PIO_PC24, PIO_IT_EDGE, photoelectric_handler);
	
	/* Encoder Interrupt */
	pio_set_input(PIOC, PIO_PC28, PIO_PULLUP); // Digital Pin 3
	pio_handler_set_pin(ENCA, PIO_IT_EDGE, encoder_handler);

	/* Create queue that can store 60 data of type long. */
	long array[2] = {0};
	xQueue = xQueueCreate(60, sizeof(array));
	
	/* Create semaphores. */
	vSemaphoreCreateBinary(xBinarySemaphore);
	
	
	/* Create tasks and schedule them if semaphores and queue have been created successfully. */
	if (xBinarySemaphore!= NULL && xQueue != NULL)
	{
	
		/* Task Create */
		
		xTaskCreate( motor, "motor", 512, NULL, 1, NULL);
		xTaskCreate( vHandlerTask, "Handler", 1024, NULL, 2, NULL );
		res = xTaskCreate( vPosPend, "PosPend", 1024, NULL, 3, NULL);
	
		if(res == pdPASS )
		{
			// PHOTO ISR Enabled
			pio_enable_interrupt(PIOC, PIO_PC24);
			NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2); //0xC0
			
			// ENC ISR Enabled
			pio_enable_interrupt(PIOC, PIO_PC28);
			NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1); //0xB0
			
			// Enable PIO C
			NVIC_ClearPendingIRQ(PIOC_IRQn);
			NVIC_EnableIRQ(PIOC_IRQn);
			
			// Start Scheduler
			vTaskStartScheduler();
		}
		
	}
	
	for( ;; );
	return 0;
}



// 1.
static void motor( void *pvParameters)
{
	printf("Task 1\n");
	ioport_set_pin_level(MOTOR_DIR_PIN, CCW);
	
	int scaled_PWM = 0;
	int dir;
	
	long T1 = 0;
	long currentT = 0;
	float t = 0;
	

	float displacement;
	float prevDis;
	int16_t PIDerr = 0;
	

	
	
	for(;;){
		if (flag == 1){
			flag = 0;
			T1 = xTaskGetTickCount();
			prevDis = 0;
			
			/* if gap is large 
			printf("%d\n", g_pulses);
			PIDerr = step(ORIGIN, g_pulses);
			//PIDerr = -PIDerr;
			dir =  ioport_get_pin_level(MOTOR_DIR_PIN);
			if (((PIDerr > 0) && (dir == CCW)) || ((PIDerr < 0) && (dir == CCW))){
				PIDerr = -PIDerr;
			}*/
			PIDerr = 0;
				
			
		}else if(flag == 0){
			currentT = xTaskGetTickCount();
		
			
			t = (float) (currentT - T1)/motorPeriod;
						
			displacement = cos(M_PI*t);
			if ((displacement < 0.0) && (prevDis > 0.0)) {
				dir =  ioport_get_pin_level(MOTOR_DIR_PIN);
				if (dir == CCW){
					dir = CW;
				}else{
					dir = CCW;
				}
				
				ioport_set_pin_level(MOTOR_DIR_PIN, dir);
			} 
			scaled_PWM = (int) (ratio+PIDerr)*fabs(displacement);
			
			if (t > 1.0){
				t = 1.0;
				flag = 2;
				scaled_PWM = 0;
				//printf("count: %d\n", motorPeriod);
			}
			prevDis = displacement;
			//printf("%d\n", motorPeriod);
			dir = ioport_get_pin_level(MOTOR_DIR_PIN);
			g_pwm_channel_led0.channel = PWM_CHANNEL_6;
			pwm_channel_update_duty(PWM, &g_pwm_channel_led0, scaled_PWM);
		}
		
		
		
		
	}	
}

// 2.
static void vHandlerTask( void *pvParameters )
{
	printf("Task 2\n");
	uint32_t pinVal = 0;
	portBASE_TYPE xStatus;
	long lReceivedValue[2] = {0};
	int spd;
	int g_angle = 0;
	int scaled = 0;
	static int prevAngle = 0;
	int start = 0;
	
	for( ;; ){
		xStatus = xQueueReceive(xQueue, &lReceivedValue, portMAX_DELAY);
		
		if(xStatus==pdPASS){
			motorPeriod = (int)lReceivedValue[1];
			spd = (int)lReceivedValue[0];
			start = 1;
			//printf("%d, %d\n", (int)motorPeriod, (int)spd);
			if(motorPeriod > STOP){
				start = 0;
			}
			}else{

			printf("Failed to read from queue\n");
		}
		
		if(start == 1){
			
			if (spd < 0){
				//printf("CCW\n");
				ioport_set_pin_level(MOTOR_DIR_PIN, CCW);
				}else{
				//printf("CW\n");
				ioport_set_pin_level(MOTOR_DIR_PIN, CW);
			}
			
			switch (abs(spd))
			{
				case 17 ... 24:
				motorPeriod = 630;
				ratio = 17;
				break;
				case 25 ... 27:
				motorPeriod = 610;
				ratio = 15;
				break;
				case 28 ... 29:
				motorPeriod = 590;
				ratio = 15;
				break;
				case 30 ... 34:
				motorPeriod = 570;
				ratio = 12;
				break;
				case 35 ... 38:
				motorPeriod = 555;
				ratio = 11;
				break;
				case 39 ... 42:
				motorPeriod = 530;
				ratio = 11;
				break;
				case 43 ... 45:
				motorPeriod = 500;
				ratio = 10;
				break;
				case 46 ... 50:
				motorPeriod = 480;
				ratio = 10;
				break;
				case 51 ... 55:
				motorPeriod = 460;
				ratio = 9;
				break;
				case 56 ... 60:
				motorPeriod = 450;
				ratio = 8;
				break;
				case 61 ... 72:
				motorPeriod = 430;
				ratio = 8;
				break;
				case 73 ... 100:
				motorPeriod = 400;
				ratio = 7;
				break;
				case 101 ... 185:
				motorPeriod = 380;
				ratio = 6;
				break;
				default:
				ratio = 0;
				break;
			}

			flag = 1;
			
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

// 3.
static void vPosPend( void *pvParameters)
{
	printf("Task 3\n");
	int duration[2];
	int pinVal;
	portBASE_TYPE xStatus;
	
	long timeFrame[2] = {0};
	long period;
	int spd;
	int j = -1;
	int direction;
	long stop;
	long lSend[2] = {0};
	
	static long prevTick = 0;
	long currentTick = 0;
	static bool previousState = LOW;
	bool currentState = LOW;
	static int prevPeriod = 0;

	//printf("vPosPend\n");
	xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
	for ( ;; ){
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		currentState = ioport_get_pin_level(PHOTOSENSOR);
		currentTick = xTaskGetTickCount();
		//printf("%d\n",j);
		stop = currentTick - prevTick;
		if (stop > 700){
			j = -1;
			previousState = LOW;
		}
		
		if((currentState != previousState) && (currentState == 1)){
			//printf("up: %d\n", currentTick);
			prevTick = currentTick;
			j++;
			timeFrame[j] = currentTick;
			
		}else if((currentState != previousState) && (currentState == 0)){
			//printf("down: %d\n", currentTick);
			duration[j] = (int) (currentTick - prevTick);
			//printf("d: %d\n", duration[j]);
			if (j == 1){
				timeFrame[j] = currentTick;
				
				if (duration[0] > duration[1]){
					//printf("CCW\n");
					//ioport_set_pin_level(MOTOR_DIR_PIN, CCW);
					spd = (int)(-(timeFrame[1] - timeFrame[0]));
					
				}else{
					//printf("CW\n");
					//ioport_set_pin_level(MOTOR_DIR_PIN, CW);
					spd = (int)((timeFrame[1] - timeFrame[0]));
				}
				
				
				period = ((long)(currentTick - prevPeriod));
				//printf("s: %d\n", spd);
				//printf("p: %d\n", period);
				
				prevPeriod = currentTick;
				memset(timeFrame,0,sizeof(timeFrame));
				memset(duration,0,sizeof(duration));
				j = -1;		
				
				lSend[0] = (long)spd;
				lSend[1] = period;
				if(xQueue != NULL)
				{
					xStatus = xQueueSendToBack(xQueue,&lSend,portMAX_DELAY);
				}
				period = 0;
			}
		}
		previousState = currentState;
	}
}


//////////////////////////////////////////
/* ISR for encoder */
void encoder_handler(const uint32_t id, const uint32_t index)
{
	static int counter = 0;

	uint32_t ENCA_Val = 0;
	uint32_t ENCB_Val = 0;
	
	static int prevA = 0;
	static int prevB = 0;
	
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if((id == ID_PIOC) && (index == PIO_PC28)){
		ENCA_Val = ioport_get_pin_level(ENCA); // Read Encoder A
		ENCB_Val = ioport_get_pin_level(ENCB); //Read Encoder B
		
		
		
		/* Pulse Count */
		if (ENCA_Val ^ ENCB_Val){
			g_pulses++;	// CW
		} else {
			if (ENCA_Val != prevA){
				g_pulses--;	// CCW
			}
		}
		

		prevA = ENCA_Val;
		prevB = ENCB_Val;
		if (g_pulses > 2*HALFROTATION) {
			g_pulses -= 2*HALFROTATION;
			} else if (g_pulses < -2*HALFROTATION) {
			g_pulses += 2*HALFROTATION;
		}
		
		//printf("A %d B %d p %d \n", ENCA_Val, ENCB_Val, g_pulses);
		
	}
	
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

/* ISR for photoelectric sensor. */
static void photoelectric_handler(const uint32_t id, const uint32_t index)
{
	portBASE_TYPE xHighPriorityTaskWoken=pdFALSE;
	portBASE_TYPE xStatus;
	int pinVal = 0;
	static int counter = 0;
	int max_count = 0;
	static bool previousState = LOW;
	static int prevTick = 0;
	int currentTick = 0;
	bool currentState = LOW;
	
	if((id != ID_PIOC) && (index != PIO_PC24)){
		return;
	}
	
	//currentTick = xTaskGetTickCountFromISR();
	
	//printf("photo\n");
	xSemaphoreGiveFromISR(xBinarySemaphore,&xHighPriorityTaskWoken);
	
	portEND_SWITCHING_ISR( xHighPriorityTaskWoken );
}
/**************************************************/


/**************************************************/
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.paritytype = CONF_UART_PARITY
	};

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONSOLE_UART, &uart_serial_options);
}
 
/**************************************************/
void vApplicationIdleHook( void )
{
}
void vApplicationMallocFailedHook( void )
{
	for( ;; );
}
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	for( ;; );
}
void vApplicationTickHook( void )
{
}