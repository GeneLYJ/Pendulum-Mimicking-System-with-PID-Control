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
//#include "FastPID.h"
//
//#define BUFFER_SIZE		100
//
//#define ENCA	PIO_PC28_IDX //ARDUINO DUE DIGITAL PIN 3
//#define ENCB	PIO_PB25_IDX //ARDUINO DUE DIGITAL PIN 2
//#define MOTOR_DIR_PIN PIO_PC26_IDX //ARDUINO DUE DIGITAL PIN 4
//
//#define MAX_PULSE 98*2
//
//#define ADC_BITS 10
//static int16_t gs_s_adc_values[BUFFER_SIZE] = { 0 };
//
//#define PWM_FREQUENCY      1000
//#define PERIOD_VALUE       100
//#define INIT_DUTY_VALUE    10
//
//pwm_channel_t g_pwm_channel_led0;
//
//float Kp=1, Ki=0, Kd=0, Hz=10;
//int output_bits = 16;
//bool output_signed = true;
//uint32_t g_SetpointPosition = 0;
//xSemaphoreHandle xSemaphore;
//int32_t g_pulses = 0;
//
//// Function prototypes
//void ADC_Handler(void);
//static void configure_console(void);
//static uint32_t adc_read_buffer(Adc * pADC, int16_t * pwBuffer, uint32_t dwSize);
//static void vHandlerTask( void *pvParameters );
//void encoder_handler(const uint32_t id, const uint32_t index);
//
//int main(void)
//{
	//int i = 0;
	//uint32_t val;
	//memset(gs_s_adc_values, 0x0, BUFFER_SIZE * sizeof(int16_t));
//
	///* Initialize the SAM system. */
	//sysclk_init();
	//board_init();
//
	//ioport_set_pin_dir(ENCA, IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(ENCA, IOPORT_MODE_PULLUP);
	//ioport_set_pin_dir(ENCB, IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(ENCB, IOPORT_MODE_PULLUP);
	//ioport_set_pin_dir(MOTOR_DIR_PIN, IOPORT_DIR_OUTPUT);
//
	//configure_console();
	//
	//vSemaphoreCreateBinary(xSemaphore);
	//
	//// TO DO: Initialize the Fast PID controller based on the given parameters.
	//InitializeFastPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
//
	///* ADC INTERRUPT */
	//pmc_enable_periph_clk(ID_ADC); 
	//adc_init(ADC, sysclk_get_main_hz(), ADC_FREQ_MAX, ADC_STARTUP_FAST);
	//adc_set_resolution(ADC, ADC_MR_LOWRES_BITS_10);
	//adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
//
	///* PWM CONFIG */
	//pmc_enable_periph_clk(ID_PWM);
	//pwm_channel_disable(PWM, PWM_CHANNEL_6);
	//pwm_clock_t clock_setting = {
		//.ul_clka = PWM_FREQUENCY * PERIOD_VALUE,
		//.ul_clkb = 0,
		//.ul_mck = sysclk_get_cpu_hz()
	//};
	//pwm_init(PWM, &clock_setting);
//
	//g_pwm_channel_led0.alignment = PWM_ALIGN_LEFT;
	//g_pwm_channel_led0.polarity = PWM_LOW;
	//g_pwm_channel_led0.ul_prescaler = PWM_CMR_CPRE_CLKA;
	//g_pwm_channel_led0.ul_period = PERIOD_VALUE;
	//g_pwm_channel_led0.ul_duty = INIT_DUTY_VALUE;
	//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
//
	//pwm_channel_init(PWM, &g_pwm_channel_led0);
	//pwm_channel_enable(PWM, PWM_CHANNEL_6);
	//ioport_set_pin_level(MOTOR_DIR_PIN, 0);
//
	///* ENCODER INTERRUPT */
	//pmc_enable_periph_clk(ID_PIOC);
	//pio_set_input(PIOC, PIO_PC28, PIO_PULLUP); // Digital Pin 3
	//pio_handler_set_pin(ENCA, PIO_IT_EDGE, encoder_handler);
//
	//BaseType_t xRes = 0;
	//xRes = xTaskCreate( vHandlerTask, "Handler", 512, NULL, 1, NULL );
	//if(xRes == pdTRUE){
		//
		//pio_enable_interrupt(PIOC, PIO_PC28);
		//NVIC_SetPriority(PIOC_IRQn, 11);
		//NVIC_ClearPendingIRQ(PIOC_IRQn);
		//NVIC_EnableIRQ(PIOC_IRQn);
		//
		//adc_enable_interrupt(ADC, ADC_ISR_RXBUFF);
		//adc_configure_trigger(ADC, ADC_TRIG_SW, 1);
		//adc_enable_channel(ADC, ADC_CHANNEL_1);
		//adc_start(ADC);
		//adc_read_buffer(ADC, gs_s_adc_values, BUFFER_SIZE);
		//NVIC_SetPriority(ADC_IRQn, 11);
		//NVIC_ClearPendingIRQ(ADC_IRQn);
		//NVIC_EnableIRQ(ADC_IRQn);
			//
		//vTaskStartScheduler();
	//}
//
	//for( ;; );
	//return 0;
//}
//
//static void vHandlerTask( void *pvParameters )
//{
	//uint32_t prevMillis = xTaskGetTickCount();
	//uint32_t currentMillis = 0;
	//uint32_t rpm;
	//uint32_t dutyCycle;
	//int16_t output = 0;
	//xSemaphoreTake(xSemaphore, 0);
	//
	//for(;;)
	//{	
		//xSemaphoreTake(xSemaphore, 100);
		//output = step(g_SetpointPosition, g_pulses);
		//if (output > 2)
		//{
			//ioport_set_pin_level(MOTOR_DIR_PIN, HIGH); // Set motor direction to counterclockwise
			//pwm_channel_update_duty(PWM, &g_pwm_channel_led0, INIT_DUTY_VALUE);
		//}
		//else if (output < -2)
		//{
			//ioport_set_pin_level(MOTOR_DIR_PIN, LOW);  // Set motor direction to clockwise
			//pwm_channel_update_duty(PWM, &g_pwm_channel_led0, INIT_DUTY_VALUE);
		//}
		//else
		//{
			//pwm_channel_update_duty(PWM, &g_pwm_channel_led0, 0);
		//}
		//printf("%d %d %d\n", g_SetpointPosition, g_pulses, output);
		//
		////fflush(stdout);
	//}
//}
//
//void encoder_handler(const uint32_t id, const uint32_t index)
//{
	//static int counter = 0;
	//uint32_t ENCA_Val = 0;
	//uint32_t ENCB_Val = 0;
	//uint32_t dir = 1; // 1: Clockwise
	//
	//portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	//if((id != ID_PIOC) || (index != PIO_PC28)){
		//return;
	//}
//
	//ENCA_Val = ioport_get_pin_level(ENCA); // Read Encoder A
	//ENCB_Val = ioport_get_pin_level(ENCB); //Read Encoder B
//
	//// This is a simple approach.
	//if (xSemaphore != NULL)
	//{
		//if ((ENCA_Val == 1 && ENCB_Val == 0) || (ENCA_Val == 0 && ENCB_Val == 1))
		//{
			//g_pulses++;
		//}			
		//else
		//{
			//g_pulses--;
		//}
		//
		//if (g_pulses > MAX_PULSE/2 + 1) g_pulses -= MAX_PULSE;
		//else if (g_pulses < -MAX_PULSE/2 - 1) g_pulses += MAX_PULSE;
		//xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
		//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		////printf("A %d B %d p %d \n", ENCA_Val, ENCB_Val, pulses);
	//}
//}
//
//void ADC_Handler(void)
//{
	//static uint32_t prevResult = 0;
	//uint32_t result = 0;
	//int i = 0;
	//uint32_t ADC_MAX_VAL = (1 << ADC_BITS) - 1;
	////Check the ADC conversion status
	//
	//if ((adc_get_status(ADC) & ADC_ISR_RXBUFF) == ADC_ISR_RXBUFF)
	//{
		//// Part 1(d)(i) - To be completed
		//for(i = 0; i < BUFFER_SIZE; i++){
			//result +=  gs_s_adc_values[i];
		//}
		//result /= BUFFER_SIZE;
		//g_SetpointPosition = (ADC_MAX_VAL-result)*MAX_PULSE/ADC_MAX_VAL - MAX_PULSE/2;
		//memset(gs_s_adc_values, 0x0, BUFFER_SIZE * sizeof(int16_t));
		///* Start new pdc transfer. */
		//adc_read_buffer(ADC, gs_s_adc_values, BUFFER_SIZE);
	//}
//}
//
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
///**
 //* \brief Read converted data through PDC channel.
 //*
 //* \param pADC The pointer of adc peripheral.
 //* \param pwBuffer The destination buffer.
 //* \param dwSize The size of the buffer.
 //*/
//static uint32_t adc_read_buffer(Adc * pADC, int16_t * pwBuffer, uint32_t dwSize)
//{
	///* Check if the first PDC bank is free. */
	//if ((pADC->ADC_RCR == 0) && (pADC->ADC_RNCR == 0)){
		//pADC->ADC_RPR = (uint32_t) pwBuffer; // Receive Pointer Register
		//pADC->ADC_RCR = dwSize;
		//pADC->ADC_PTCR = ADC_PTCR_RXTEN;
		//return 1;
	//} else {	/* Check if the second PDC bank is free. */
		//if (pADC->ADC_RNCR == 0) {
			//pADC->ADC_RNPR = (uint32_t) pwBuffer; // Receive Pointer Next Register
			//pADC->ADC_RNCR = dwSize;
			//pADC->ADC_PTCR = ADC_PTCR_RXTEN;
			//return 1;
		//} else {
			//return 0;
		//}
	//}
//}
 //
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
