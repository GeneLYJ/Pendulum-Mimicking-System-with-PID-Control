///*
 //* Speed.c
 //*
 //* Created: 5/12/2021 10:14:34 PM
 //*  Author: Lenovo
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
///* Pin Define */
//#define PHOTOSENSOR	PIO_PC24_IDX  // Pin 6
//#define MOTOR_DIR_PIN PIO_PC26_IDX //ARDUINO DUE DIGITAL PIN 4
//#define ENCA PIO_PC28_IDX // Pin 3
//#define ENCB PIO_PB25_IDX // Pin 2
//#define PWM6 PIO_PC23_IDX // Pin 7
//
//#define MAX_MOTOR_RPM 400
//#define CORNER 45
//#define ONEROTATION 90*2
//#define MAX_COUNT_VALUES 500000
//#define MAX_PERIOD 680.0
//#define MIN_PERIOD 594.0
//
//#define BUFFER		13
//
//
//#define CW 1	// Clockwise ++
//#define CCW 0	// --
//
///** PWM frequency in Hz */
//#define PWM_FREQUENCY      1000
///** Period value of PWM output waveform */
//#define PERIOD_VALUE       100
///** Initial duty cycle value */
//#define INIT_DUTY_VALUE    0
//
///* Pendulum Parameters */
//#define G  9.8
//#define L  0.35
////#define I  0.000218 // Moment of Inertia 1/12 *(M) * (a^2 + b^2) - M = 0.1kg, a = 0.15m, b = 0.06m
//
///* PID configuration */
//float Kp= 7, Ki= 0.01 , Kd=1.5, Hz=10;
//int output_bits = 16;
//bool output_signed = true;
//int32_t g_SetpointPosition = 0;
 //
///** PWM channel instance for motor. */
//pwm_channel_t g_pwm_channel_led0;
 //
///** Declare variables of xSemaphoreHandle. */
//xSemaphoreHandle xBinarySemaphore;
 //
///** Declare variable of xQueueHandle. */
//xQueueHandle xQueue;
//
///** Array to store duration values. */
//static int16_t duration_values[2] = {0};
//
//// Encoder counts
//int g_pulses = 0;
//int16_t g_set_arr[BUFFER] = {0};
//int flag = 0;
//int idx = 0;
//
//// vHandler - angle for motor & handler
////int g_angle;
//
//
//// Function prototypes
//float initialAngle(float period);
//void harmonic(float angle, float T);
//
////void ADC_Handler(void);
//static void configure_console(void);
////static uint32_t adc_read_buffer(Adc * pADC, int16_t * pwBuffer, uint32_t dwSize);
//static void vHandlerTask( void *pvParameters );
//static void vPosPend( void *pvParameters );
//static void motor( void *pvParameters);
//
////ISR
//static void photoelectric_handler(const uint32_t id, const uint32_t index);
//void encoder_handler(const uint32_t id, const uint32_t index);
//
//
//int main(void){
	//
	//int i = 0;
	//uint32_t val;
	//BaseType_t res;
//
	///* Initialize the SAM system. */
	//sysclk_init();
	//board_init();
	//
	///* Configure console. */
	//configure_console();
	//
	///* PID */
	//// Initialize the Fast PID controller based on the given parameters.
	//InitializeFastPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
	//
	///* IOPORT */
	//ioport_set_pin_dir(PHOTOSENSOR, IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(PHOTOSENSOR, IOPORT_MODE_PULLUP);
	//
	//ioport_set_pin_dir(ENCA, IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(ENCA, IOPORT_MODE_PULLUP);
	//ioport_set_pin_dir(ENCB, IOPORT_DIR_INPUT);
	//ioport_set_pin_mode(ENCB, IOPORT_MODE_PULLUP);
	//
	//ioport_set_pin_dir(MOTOR_DIR_PIN, IOPORT_DIR_OUTPUT);
	//
	//
//
	//
	///* Initialize PWM channel. */
	//
	//
	///* PWM CONFIG */
	//pmc_enable_periph_clk(ID_PWM);
	//pwm_channel_disable(PWM, PWM_CHANNEL_6);
	//
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
	////ioport_set_pin_level(PWM6, LOW);
	//ioport_set_pin_level(MOTOR_DIR_PIN, LOW);
	//
	///* PHOTO ELECTRIC INTERRUPT */
	//pmc_enable_periph_clk(ID_PIOC);
	//pio_set_input(PIOC, PIO_PC24, PIO_PULLUP);
	//pio_handler_set(PIOC, ID_PIOC, PIO_PC24, PIO_IT_EDGE, photoelectric_handler);
	//
	///* Encoder Interrupt */
	//pio_set_input(PIOC, PIO_PC28, PIO_PULLUP); // Digital Pin 3
	//pio_handler_set_pin(ENCA, PIO_IT_EDGE, encoder_handler);
//
	///* Create queue that can store 100 data of type long. */
	//long array[2] = {0};
	//xQueue = xQueueCreate(60, sizeof(array));
	//
	///* Create semaphores. */
	//vSemaphoreCreateBinary(xBinarySemaphore);
	//
	//
	///* Create tasks and schedule them if semaphores and queue have been created successfully. */
	//if (xBinarySemaphore!= NULL && xQueue != NULL)
	//{
	//
		///* Task Create */
		//
		//xTaskCreate( motor, "motor", 512, NULL, 1, NULL);
		//res = xTaskCreate( vHandlerTask, "Handler", 1024, NULL, 2, NULL );
		//xTaskCreate( vPosPend, "PosPend", 1024, NULL, 3, NULL);
	//
		//if(res == pdPASS )
		//{
			//// PHOTO ISR Enabled
			//pio_enable_interrupt(PIOC, PIO_PC24);
			//NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
			//
			//// ENC ISR Enabled
			//pio_enable_interrupt(PIOC, PIO_PC28);
			//NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
			//
			//// Enable PIO C
			//NVIC_ClearPendingIRQ(PIOC_IRQn);
			//NVIC_EnableIRQ(PIOC_IRQn);
			//
			//// Start Scheduler
			//vTaskStartScheduler();
		//}
		//
	//}
	//
	//for( ;; );
	//return 0;
//}
//// 2.
//static void vHandlerTask( void *pvParameters )
//{
	//printf("Task 2\n");
	//uint32_t pinVal = 0;
	//portBASE_TYPE xStatus;
	//long lReceivedValue[2] = {0};
	//int spd;
	//int g_angle = 0;
	//int scaled = 0;
	//static int prevAngle = 0;
	//
	//float period;
	//float rad;
//
	//for( ;; ){
		//xStatus = xQueueReceive(xQueue, &lReceivedValue, portMAX_DELAY);
		//if(xStatus==pdPASS){
			//period = (float)lReceivedValue[1];
			//spd = (int)lReceivedValue[0];
			//}else{
			////period = 655.0;
			//printf("Failed to read from queue\n");
		//}
		////printf("%d\n", (int)period);
		//rad = initialAngle(period);
		////printf("%d\n", (int)rad);
		//g_angle = (int) (rad*180.0/M_PI);
		//if (abs(spd) > 100){
			//if (g_angle > prevAngle){
				//g_angle = prevAngle;
			//}
		//}
		//if (spd < 0){
			//g_angle = -g_angle;
		//}
		////printf("%d\n", spd);
		////printf("%d\n",g_angle);	
		//prevAngle = g_angle;
		////g_SetpointPosition = (g_angle/90)*CORNER;
		//g_SetpointPosition = g_angle;
		//scaled = (int)((g_SetpointPosition/90.0)*CORNER);
		////printf("S: %d\n",g_SetpointPosition);
		//flag = 0;
		//if (idx > 13) {
			//idx = 0;
			//flag = 0;
		//}
		//harmonic(scaled, period);
		//idx = 0;
		//
		//flag = 1;
		///*for (int i=0; i <= BUFFER; i++){
			//printf("%d\n", g_set_arr[i]);
		//}*/
		////printf("vHand: %d\n", idx);
		//vTaskDelay(pdMS_TO_TICKS(20));
	//}
//}
//// 1.
//static void motor( void *pvParameters)
//{
	//printf("Task 1\n");
	//int duty_c = 0;
	////portBASE_TYPE xStatus;
	//ioport_set_pin_level(MOTOR_DIR_PIN, CCW);
	//
	////g_SetpointPosition = 31;
	////int32_t hold = 0;
	//
	//int16_t PIDerr = 0;
	//int16_t output = 0;
	////int scaled = 0;
	//int scaled_PWM = 0;
	//int dir;
	////int i = 0;
	////int flag = 1;
	//
	////g_pwm_channel_led0.channel = PWM_CHANNEL_6;
	////pwm_channel_update_duty(PWM, &g_pwm_channel_led0, duty_c);
	//for(;;){
		//if (flag == 1){
			////printf("%d, %d\n", scaled, g_SetpointPosition);
			////PIDerr = step(scaled, g_pulses);
			//PIDerr = step(g_set_arr[idx], g_pulses);
			////printf("%d\n",PIDerr);
			//
			////printf("%d, %d\n", i, PIDerr);
			//output = abs(PIDerr);
			////scaled_PWM = (int)((float) output*PERIOD_VALUE/MAX_MOTOR_RPM);
			//scaled_PWM = (int) output;
		//
			//if (PIDerr < 0)
			//{
				//duty_c = scaled_PWM;
				//ioport_set_pin_level(MOTOR_DIR_PIN, CCW);
				////flag = 1;
			//}else if(PIDerr > 0){
				//duty_c = scaled_PWM;
				//ioport_set_pin_level(MOTOR_DIR_PIN, CW);
				////flag = 1;
			//}else
			//{
				///*if (flag == 1){
				//
					//duty_c = 0;
					//scaled = 0;
					//g_SetpointPosition = 0;
					//hold = g_SetpointPosition;
					//g_SetpointPosition = -hold;
					//flag = 0;
					//vTaskDelay(40);
				//}*/
				//idx++;
				//if (idx > 13) {
					//idx = 0;
					//flag = 0;
					////printf("rst\n");
				//}
				////printf(("%d\n", idx));
				////printf("%d, %d\n", idx, PIDerr);
				//
			//}
			//if (duty_c > 25){
				//duty_c = 25;
			//}
			//if (idx > 13) {
				//idx = 0;
				//flag = 0;
				////printf("rst\n");
			//}
			//
			//dir = ioport_get_pin_level(MOTOR_DIR_PIN);
			//g_pwm_channel_led0.channel = PWM_CHANNEL_6;
			//pwm_channel_update_duty(PWM, &g_pwm_channel_led0, scaled_PWM);
			//printf("%d, %d\n", scaled_PWM, idx);
			////printf("d: %d, a: %d, e: %d, %d, %d\n", duty_c, g_set_arr[idx], output, g_pulses, idx);
			//vTaskDelay(1);
		//}
		//
		//vTaskDelay(1);
	//}	
//}
//// 3.
//static void vPosPend( void *pvParameters)
//{
	//printf("Task 3\n");
	//int duration[2];
	//int pinVal;
	//portBASE_TYPE xStatus;
	//
	//long timeFrame[2] = {0};
	//long period;
	//int spd;
	//int j = -1;
	//int direction;
	//long stop;
	//long lSend[2] = {0};
	//
	//static long prevTick = 0;
	//long currentTick = 0;
	//static bool previousState = LOW;
	//bool currentState = LOW;
	//static int prevPeriod = 0;
//
	////printf("vPosPend\n");
	//xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
	//for ( ;; ){
		//xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
		//currentState = ioport_get_pin_level(PHOTOSENSOR);
		//currentTick = xTaskGetTickCount();
		////printf("%d\n",j);
		//stop = currentTick - prevTick;
		//if (stop > 500){
			//j = -1;
			//previousState = LOW;
		//}
		//
		//if((currentState != previousState) && (currentState == 1)){
			////printf("up: %d\n", currentTick);
			//prevTick = currentTick;
			//j++;
			//timeFrame[j] = currentTick;
			//
		//}else if((currentState != previousState) && (currentState == 0)){
			////printf("down: %d\n", currentTick);
			//duration[j] = (int) (currentTick - prevTick);
			////printf("d: %d\n", duration[j]);
			//if (j == 1){
				//timeFrame[j] = currentTick;
				//
				//if (duration[0] > duration[1]){
					//printf("CCW\n");
					////ioport_set_pin_level(MOTOR_DIR_PIN, CCW);
					//spd = (int)(-(timeFrame[1] - timeFrame[0]));
					//
				//}else{
					//printf("CW\n");
					////ioport_set_pin_level(MOTOR_DIR_PIN, CW);
					//spd = (int)((timeFrame[1] - timeFrame[0]));
				//}
				//
				//
				//period = ((long)(currentTick - prevPeriod));
				////printf("s: %d\n", spd);
				//printf("p: %d\n", period);
				//
				//prevPeriod = currentTick;
				//memset(timeFrame,0,sizeof(timeFrame));
				//memset(duration,0,sizeof(duration));
				//j = -1;		
				//
				//lSend[0] = (long)spd;
				//lSend[1] = period;
				//if(xQueue != NULL)
				//{
					//xStatus = xQueueSendToBack(xQueue,&lSend,portMAX_DELAY);
				//}
				//period = 0;
			//}
		//}
		//previousState = currentState;
	//}
//}
//
//
////////////////////////////////////////////
///* ISR for encoder */
//void encoder_handler(const uint32_t id, const uint32_t index)
//{
	//static int counter = 0;
//
	//uint32_t ENCA_Val = 0;
	//uint32_t ENCB_Val = 0;
	//
	//static int prevA = 0;
	//static int prevB = 0;
	//
	//portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	//if((id == ID_PIOC) && (index == PIO_PC28)){
		//ENCA_Val = ioport_get_pin_level(ENCA); // Read Encoder A
		//ENCB_Val = ioport_get_pin_level(ENCB); //Read Encoder B
		//
		//
		//
		///* Pulse Count */
		//if (ENCA_Val ^ ENCB_Val){
			//g_pulses++;	// CW
		//} else {
			//if (ENCA_Val != prevA){
				//g_pulses--;	// CCW
			//}
		//}
		//
//
		//prevA = ENCA_Val;
		//prevB = ENCB_Val;
		//// Make sure pulses ranges from [0, 98]
		//if (g_pulses > ONEROTATION) {
			//g_pulses -= ONEROTATION;
			//} else if (g_pulses < -ONEROTATION) {
			//g_pulses += ONEROTATION;
		//}
		//
		////printf("A %d B %d p %d \n", ENCA_Val, ENCB_Val, g_pulses);
		//
	//}
	//
	//portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
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
	//static int prevTick = 0;
	//int currentTick = 0;
	//bool currentState = LOW;
	//
	//if((id != ID_PIOC) && (index != PIO_PC24)){
		//return;
	//}
	//
	//currentTick = xTaskGetTickCountFromISR();
	//
	////printf("photo\n");
	//xSemaphoreGiveFromISR(xBinarySemaphore,&xHighPriorityTaskWoken);
	//
	//portEND_SWITCHING_ISR( xHighPriorityTaskWoken );
//}
///**************************************************/
//void harmonic(float angle, float T) {
	//
	//if (T > MAX_PERIOD) {
		//T = MAX_PERIOD-70;
		//}else if (T < MIN_PERIOD) {
		//T = MIN_PERIOD;
	//}
	//
	//const int midpointIndex = BUFFER/2;
	//const float durationSpacing = (T/1000) / (BUFFER - 1);
	//float t;
	//
	//
	//// Fill up easiest parts of the buffer
	//g_set_arr[0] = 0;
	//g_set_arr[BUFFER- 1] = 0;
//
	//// Fill in the midpoint (i.e. its highest point)
	//g_set_arr[midpointIndex] = angle;
	//
//
	//// Fill up all other values in between
	//for (int i = 1; i < BUFFER - 1; i++) {
		//if (i != midpointIndex) {
			//t = durationSpacing * i;
			//g_set_arr[i] = (int16_t) round(sin(sqrt(G/L) * t) * angle);
		//}
		////printf("%d\n", g_set_arr[i]);
	//}
//}
//
//float initialAngle(float period) {
    ///* T = 2*pi*sqrt(L/g)*( 1 + angle^2/16 + 11*angle^2/3072)
	//polynomial solution for elliptc integral
	//*/
    //// d = c + bx + ax^2 -> a = 1
	//float c;
    //float b = 17.4545;           
    //float b_squared = 304.66;   
	//float T;
    ///* NOTE: This solution is generally valid only
    //for t > 1.2, however, by observation, we see that
    //the period of oscillation is almost always > 1.2
    //with small exceptions coming in at a minimum of
    //t=1.6. The oscillation generally ends at a period
    //of 1.21 seconds.*/
	//T = 2.0*period/1000.0;
    //// Set minimum value in case of small exceptions
    //if (period > MAX_PERIOD) {
        //T = 1.36;
    //}else if (period < MIN_PERIOD) {
		//T = 1.188;
	//}
	//
    //// float c = 3072.0 / 11 * (1 - T / (2 * M_PI) * T_0);
    //c = 279.273 - 235.19*T;       
//
    //// Find the quadratic solution. Only the positive solution x = angle^2
    //return sqrt((-b + sqrt(b_squared - 4.0 * c)) / 2.0);
//}
//
///**************************************************/
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
///**************************************************/
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
//
//// Photo ISR
///* If there is a change in pin level, a semaphore will be given to allow
	//vCounterTask to run. 
	//currentState = ioport_get_pin_level(PHOTOSENSOR);
	//if(currentState != previousState && currentState == HIGH)
	//{
		//currentTick = xTaskGetTickCountFromISR();
		//
		//printf("up: %d \n", currentTick);
	//}
	//else if(currentState != previousState && currentState == LOW)
	//{
		//currentTick = xTaskGetTickCountFromISR();
		//
		//printf("down: %d \n", currentTick);
	//}
	//previousState = currentState;*/
//
//// ENC ISR
		///*
		//if ((ENCA_Val == 0 && ENCB_Val == 1) || (ENCA_Val == 1 && ENCB_Val == 0)){
			////if ((ENCA_Val != prevA) && (ENCB_Val != prevB)){
				//g_pulses ++;	
			////}
		//}
		//else{
			////if ((ENCB_Val != prevA) && (ENCB_Val != prevB)){
				//g_pulses --;	
			////}
		//}*/