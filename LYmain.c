///**
 //* Name: Pong Loong Yeat
 //* ID: 28701879
 //* Email: lpon0001@student.monash.edu
 //* Assignment
 //*/
//
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
//// Project settings
//// #define PRINT_HALFPERIOD
//// #define PRINT_RELEASEANGLE
//// #define PRINT_FULLPERIOD
//// #define PRINT_PID_VALUES
//
///* Pin and motor settings*/
//#define MY_PHOTOELECTRIC    PIO_PC25_IDX    // ARDUINO DUE DIGITAL PIN 5
//#define ENCA                PIO_PC28_IDX    // ARDUINO DUE DIGITAL PIN 3
//#define ENCB                PIO_PB25_IDX    // ARDUINO DUE DIGITAL PIN 2
//#define MOTOR_DIR_PIN       PIO_PC26_IDX    // ARDUINO DUE DIGITAL PIN 4
//
//#define MAX_MOTOR_RPM       500
//
//// From: https://my.cytron.io/p-12v-430rpm-1kgfcm-32mm-planetary-dc-geared-motor-with-encoder
//#define PULSES_PER_ROTATION     98
//#define PULSES_PER_ROTATION_2   49
//
///* PWM settings */
//#define PWM_FREQUENCY               1000    // PWM frequency in Hz
//#define PERIOD_VALUE                100     // Period value of PWM output waveform
//#define INIT_DUTY_VALUE             0       // Initial duty cycle value
//
///* Pendulum setup */
//#define CONSTANT_G                  9.81    // Gravity
//#define CONSTANT_PENDULUM_LENGTH    0.35    // Does this really need a comment? ...
//#define CONSTANT_OMEGA              5.2942  // sqrt(CONSTANT_G / CONSTANT_PENDULUM_LENGTH)
//#define MIN_PERIOD                  1.2     // Smallest possible period
//
///* PWM channel instance for motor
//I'm using pin 8 so this corresponds
//to PWM_CHANNEL_5. */
//pwm_channel_t g_pwm_channel_pin8;
//
///* Enum for pendulum direction to motor
//direction. If the pendulum is swinging
//to the left, we want the motor to turn
//in a clockwise direction (hence the 1). */
//enum direction {
    //left = 1,
    //right = 0,
    //unknown = -1
//};
//
//// Error codes for debugging
//enum err {
    //NO_ERR = 0,
    //ERR_PREV_DIR_IS_SAME = -1,
    //ERR_DURATION_STORE_NOT_FULL = -2,
    //ERR_FAILED_TO_GIVE_SEMAPHORE = -3
//};
//
//// Warning codes for debugging
//enum warn {
    //NO_WARN = 0,
    //WARN_MIN_HALFPERIOD_EXCEEDED = -1,
    //WARN_UNKNOWN_DIR = -2,
    //WARN_DURATION_ZERO = -3
//};
//
///* PID control */
//float Kp=4, Ki=2.5, Kd=4.5, Hz=10;
//int output_bits = 16;
//bool output_signed = true;
//
//// Globals/additional stuff
//#define MAXIMUM_HALFPERIOD_TICK 1.0 * configTICK_RATE_HZ        // The smallest allowable half-period duration
//#define MINIMUM_HALFPERIOD_TICK 0.4 * configTICK_RATE_HZ        // The largest allowable half-period duration
//#define BUFFER_SIZE 21                                          // NOTE: This must be an odd number
//int16_t g_setpointPosition[BUFFER_SIZE] = {0};
//int16_t g_pulses = 0;                   // Number of pulses in encoder (ranges from [0, 98])
//uint32_t g_duration = 0;                // Swing duration in ticks
//uint32_t g_duration_store[2] = {0, 0};  // A "store" containing the swing duration in ticks
//uint32_t g_refTick = 0;                 // Tick count at which oscillation begins
//float g_releaseAngle = 0;               // The release angle
//float g_period_sec = 0;                 // Duration in seconds
//enum direction g_dir = unknown;         // Direction of the swing
//bool g_stop = true;                     // Stop flag
//bool g_ERR_FAILED_TO_GIVE_SEMAPHORE = false;        // Asserted only AFTER first semaphore error is detected
//
///* Declare a variable of type xSemaphoreHandle. This is used to reference the
//semaphore that is used to synchronize a task with an interrupt. */
//xSemaphoreHandle xBinarySemaphore;
//
///* Function prototypes */
//static void configure_console(void);
//void vApplicationMallocFailedHook( void );
//void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName );
//void vApplicationIdleHook( void );
//void vApplicationTickHook( void );
//
///* Tasks to be created */
//static void vUpdaterTask( void *pvParameters );
//static void vHandlerTask( void *pvParameters );
//void photoelectric_handler(const uint32_t id, const uint32_t index);
//void encoder_handler(const uint32_t id, const uint32_t index);
//
///* Utility functions */
//float computeReleaseAngle(float period);
//void fillPosValues(float releaseAngle, float period);
//void error(enum err errorCode);
//void warning(enum warn warningCode);
//void trap(enum err errorCode);
//
//
///**
 //*  Configure UART console.
 //*/
 //// [main_console_configure]
//static void configure_console(void) {
    //const usart_serial_options_t printf = {
        //.baudrate = CONF_UART_BAUDRATE,
        //#ifdef CONF_UART_CHAR_LENGTH
        //.charlength = CONF_UART_CHAR_LENGTH,
        //#endif
        //.paritytype = CONF_UART_PARITY,
        //#ifdef CONF_UART_STOP_BITS
        //.stopbits = CONF_UART_STOP_BITS,
        //#endif
    //};
//
    ///* Configure console UART. */
    //sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
    //stdio_serial_init(CONSOLE_UART, &printf);
//}
//
//int main(void) {
    ///* Initialize the SAM system. */
    //sysclk_init();
    //board_init();
//
    ///* Configure IO ports */
    //ioport_set_pin_dir(MY_PHOTOELECTRIC, IOPORT_DIR_INPUT);
    //ioport_set_pin_mode(MY_PHOTOELECTRIC, IOPORT_MODE_PULLUP);
    //ioport_set_pin_dir(ENCA, IOPORT_DIR_INPUT);
    //ioport_set_pin_mode(ENCA, IOPORT_MODE_PULLUP);
    //ioport_set_pin_dir(ENCB, IOPORT_DIR_INPUT);
    //ioport_set_pin_mode(ENCB, IOPORT_MODE_PULLUP);
    //ioport_set_pin_dir(MOTOR_DIR_PIN, IOPORT_DIR_OUTPUT);
//
    ///* UART */
    //configure_console();
//
    //// Initialize the Fast PID controller based on the given parameters.
    //InitializeFastPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
//
    ///* PWM CONFIG */
    //pmc_enable_periph_clk(ID_PWM);
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
    ///* Configure PWM */
    //g_pwm_channel_pin8.alignment = PWM_ALIGN_LEFT;
    //g_pwm_channel_pin8.polarity = PWM_LOW;
    //g_pwm_channel_pin8.ul_prescaler = PWM_CMR_CPRE_CLKA;
    //g_pwm_channel_pin8.ul_period = PERIOD_VALUE;
    //g_pwm_channel_pin8.ul_duty = INIT_DUTY_VALUE;
    //g_pwm_channel_pin8.channel = PWM_CHANNEL_5;
//
    //pwm_channel_init(PWM, &g_pwm_channel_pin8);
    //pwm_channel_enable(PWM, PWM_CHANNEL_5);
//
    ///* Dont leave pin value floating */
    //ioport_set_pin_level(MOTOR_DIR_PIN, 0);
//
    ///* PHOTO ELECTRIC INTERRUPT */
    //pmc_enable_periph_clk(ID_PIOC);
    //pio_set_input(PIOC, PIO_PC25, PIO_PULLUP);
    //pio_handler_set(PIOC, ID_PIOC, PIO_PC25, PIO_IT_EDGE, photoelectric_handler);
//
    ///* ENCODER INTERRUPT */
    //pio_set_input(PIOC, PIO_PC28, PIO_PULLUP); // Digital Pin 3
    //pio_handler_set_pin(ENCA, PIO_IT_EDGE, encoder_handler);
//
    ///* Create our semaphore and task */
    //vSemaphoreCreateBinary(xBinarySemaphore);
//
    //BaseType_t xResUpdater = xTaskCreate( vUpdaterTask, "Updater", 1024, NULL, 2, NULL );
    //BaseType_t xResHandler = xTaskCreate( vHandlerTask, "Handler", 512, NULL, 1, NULL );
//
    //if (xResUpdater == pdPASS &&
        //xResHandler == pdPASS &&
        //xBinarySemaphore != NULL) {
        //pio_enable_interrupt(PIOC, PIO_PC25);
        //NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 2);
//
        //pio_enable_interrupt(PIOC, PIO_PC28);
        //NVIC_SetPriority(PIOC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
//
        //NVIC_ClearPendingIRQ(PIOC_IRQn);
        //NVIC_EnableIRQ(PIOC_IRQn);
//
        //vTaskStartScheduler();
    //}
//
    //for( ;; );
    //return 0;
//}
//
///**
 //* The "Updater" task is responsible for updating
 //* any global values that do not need to be updated
 //* in the photoelectric ISR. This task should not run
 //* often and should only run once per pendulum half-swing.
 //*/
//static void vUpdaterTask( void *pvParameters ) {
    ///* Take the semaphore once to start with so the semaphore is empty before the
    //infinite loop is entered. The semaphore was created before the scheduler
    //was started so before this task ran for the first time.*/
    //xSemaphoreTake(xBinarySemaphore, 0);
//
    ///* As per most tasks, this task is implemented within an infinite loop. */
    //for(;;) {
        ///* Use the semaphore to wait for the event. The task blocks
        //indefinitely meaning this function call will only return once the
        //semaphore has been successfully obtained - so there is no need to check
        //the returned value. */
        //xSemaphoreTake(xBinarySemaphore, portMAX_DELAY);
//
        ///* By now, $g_duration has already been updated
        //globally so we can now make use of it. */
//
        ///* Accept only a certain range of values
        //in case we get outliers. */
        //if (g_duration > MINIMUM_HALFPERIOD_TICK && g_duration < MAXIMUM_HALFPERIOD_TICK) {
            //// Update duration
            //g_duration_store[0] = g_duration_store[1];      // Push previous value back
            //g_duration_store[1] = g_duration;
//
            ///* By observation, we notice that when the
            //pendulum swings to the left (assuming the
            //motor is pointed towards you), the tick/duration
            //is smaller than when it swings back. */
//
            //// Don't update direction until store is fully filled
            //if (g_duration_store[0] != 0) {
                //if (g_duration_store[1] < g_duration_store[0]) {
                    //// Pendulum swinging left
                    //g_dir = left;
//#ifdef PRINT_HALFPERIOD
                    //printf("Pendulum swinging left took %lu ticks\n", g_duration_store[1]);
//#endif
                //} else {
                    //g_dir = right;
//#ifdef PRINT_HALFPERIOD
                    //printf("Pendulum swinging right took %lu ticks\n", g_duration_store[1]);
//#endif
                //}
                //// Update release angle for current swing cycle
                //g_period_sec = ((float) g_duration_store[0] + g_duration_store[1]) / configTICK_RATE_HZ;
                //g_releaseAngle = computeReleaseAngle(g_period_sec);
//
//#ifdef PRINT_RELEASEANGLE
                //printf("Release angle for this cycle: %d\n", (int) round(g_releaseAngle * 180 / M_PI));
//#endif
//
                //// Fill position buffer
                //fillPosValues(g_releaseAngle, g_period_sec);
//
                //// Make sure we deassert stop flag
                //g_stop = false;
//
            //}
        //} else {
            //// Pendulum probably stopped
            //warning(WARN_MIN_HALFPERIOD_EXCEEDED);
            //g_stop = true;
//
            //// Reset release angle
            //g_releaseAngle = 0;
        //}
//
//#ifdef PRINT_FULLPERIOD
        //printf("Pendulum swing took %lu ticks\n", g_duration_store[0] + g_duration_store[1]);
//#endif
//
        //// Update tick count for new cycle
        //g_refTick = xTaskGetTickCount();
//
        ///* This will be blocked after updating values
        //and allows the "Handler" task to run and
        //update the motor position. The only time
        //this is unblocked is if there is a fresh
        //swing. */
    //}
//}
//
///**
 //* The "Handler" task is responsible for updating
 //* positional values to send to the motor. This
 //* task should run often and should only be blocked
 //* when the "Updater" task is running.
 //*/
//static void vHandlerTask( void *pvParameters ) {
    //int i = 0;
    //uint32_t dutyCycle = 0;
    //int16_t output = 0;
//
    ///* As per most tasks, this task is implemented within an infinite loop. */
    //for(;;) {
        //if (!g_stop) {
            //// Get PID step value
            //output = step(g_setpointPosition[i], g_pulses);
//
            //// Update PWM duty cycle
            //dutyCycle = (int) (abs(output) * PERIOD_VALUE / MAX_MOTOR_RPM);
//
            //if (output > 0) {
                //ioport_set_pin_level(MOTOR_DIR_PIN, left);
            //} else if (output < 0){
                //ioport_set_pin_level(MOTOR_DIR_PIN, right);
            //} else {
                //// Setpoint reached
                //dutyCycle = 0;
                //i++;
//
                //// Reset iteration counter if we exceed the buffer size
                //if (i > BUFFER_SIZE) {
                    //i = 0;
                //}
            //}
//
//#ifdef PRINT_PID_VALUES
            //printf(
                //"sp: %d, fb: %d, err: %d, dc: %lu, stop: %d\n",
                //g_setpointPosition[i], g_pulses, output, dutyCycle, g_stop
            //);
//#endif
        //} else {
            //dutyCycle = 0;
        //}
//
        //// Update PWM duty cycle
        //pwm_channel_update_duty(PWM, &g_pwm_channel_pin8, dutyCycle);
//
        //// Give it some time to reach
        //vTaskDelay(pdMS_TO_TICKS(10));
    //}
//}
//
///**
 //* Photoelectric sensor ISR. This ISR is triggered on the sensor's
 //* rising edge, which updates a global variable $g_duration
 //* (in ticks).
 //*/
//void photoelectric_handler(const uint32_t id, const uint32_t index) {
    //static bool previousState = LOW;
    //bool currentState = LOW;
//
    //static uint32_t prevTick = 0;
    //uint32_t currentTick = 0;
//
    //portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//
    //if ((id == ID_PIOC) && (index == PIO_PC25)) {
        //// Get initial tick count
        //if (prevTick == 0) {
            //prevTick = xTaskGetTickCountFromISR();
        //}
//
        //currentState = ioport_get_pin_level(MY_PHOTOELECTRIC); // Read IR sensor state
//
        //if (currentState != previousState && currentState == HIGH) {
            //currentTick = xTaskGetTickCountFromISR();
//
            //// Update half period duration
            //g_duration = currentTick - prevTick;
//
            //// Give up semaphore when triggered
            //BaseType_t xRes = xSemaphoreGiveFromISR(xBinarySemaphore, xHigherPriorityTaskWoken);
//
            //if (xRes != pdTRUE) {
                ///* Trap only if error is detected for the second time
                //since this error will occur on initial run. */
                //if (g_ERR_FAILED_TO_GIVE_SEMAPHORE) {
                    //trap(ERR_FAILED_TO_GIVE_SEMAPHORE);
                //}
                //g_ERR_FAILED_TO_GIVE_SEMAPHORE = true;
            //}
//
            //// Update prevTick for next iter
            //prevTick = currentTick;
        //}
//
        //// Update state for next round
        //previousState = currentState;
    //}
    //portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//}
//
///**
 //* Rotary encoder ISR. This ISR is triggered on the sensor's rising
 //* edge, which updates a global variable $g_pulses which represents
 //* the number of pulses in the encoder.
 //*/
//void encoder_handler(const uint32_t id, const uint32_t index) {
    //bool ENCA_Val;
    //bool ENCB_Val;
//
    //static int prevA = 0;
//
    //portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
//
    //// Only update if there is a semaphore, in case this is a false trigger
    //if ((id == ID_PIOC) && (index == PIO_PC28)) {
        //ENCA_Val = ioport_get_pin_level(ENCA); // Read Encoder A
        //ENCB_Val = ioport_get_pin_level(ENCB); // Read Encoder B
//
        ///* Iterate $pulses if ENCA_Val and ENCB_Val have different
        //logical values (which represents phase 2 and 4). */
        //if (ENCA_Val ^ ENCB_Val) {
            //g_pulses++;
        //} else {
            //if (ENCA_Val != prevA){
                //g_pulses--;
            //}
        //}
//
        //prevA = ENCA_Val;
//
        //// Make sure pulses ranges from [0, 98]
        //if (g_pulses > PULSES_PER_ROTATION) {
            //g_pulses -= PULSES_PER_ROTATION;
        //} else if (g_pulses < -PULSES_PER_ROTATION) {
            //g_pulses += PULSES_PER_ROTATION;
        //}
    //}
    //portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
//}
//
///**
 //* \brief Computes the initial position of the pendulum
 //* swing based on the period in seconds.
 //*
 //* \param period The full-swing period in seconds
 //*
 //* \return The release angle in rads.
 //*/
//float computeReleaseAngle(float period) {
    ///* To compute the release angle, we use the
    //Legendre polynomial solution for the elliptic integral
    //with a Maclaurin series approximation of the first
    //three terms, which becomes a quadratic problem. */
    //// float a = 1.0;       // No need for this (optimisation)
    //// float b = 192.0 / 11;
    //float b = 17.4545;          // Pre-calculate (optimisation)
    //float b_squared = 304.66;   // Pre-calculate (optimisation)
//
    ///* NOTE: This solution is generally valid only
    //for t > 1.2, however, by observation, we see that
    //the period of oscillation is almost always > 1.2
    //with small exceptions coming in at a minimum of
    //t=1.6. The oscillation generally ends at a period
    //of 1.21 seconds.*/
//
    //// Set minimum value in case of small exceptions
    //if (period < MIN_PERIOD) {
        //period = MIN_PERIOD;
    //}
//
    //// float c = 3072.0 / 11 * (1 - period / (2 * M_PI) * CONSTANT_OMEGA);
    //float c = 279.273 - 235.315 * period;       // Pre-calculate (optimisation)
//
    //// Find the quadratic solution.
    //return sqrt((-b + sqrt(b_squared - 4 * c)) / 2);
//}
//
///**
 //* \brief Fills the positional values [0, 98]
 //* in g_setpointPosition within a specified
 //* period.
 //*
 //* \param releaseAngle The release angle
 //* \param period The swing period in seconds
 //*
 //* \return The positional value at corresponding time.
 //*/
//void fillPosValues(float releaseAngle, float period) {
    //// Ignore if 0
    //if (releaseAngle < 0.01) {
        //memset(g_setpointPosition, PULSES_PER_ROTATION_2, sizeof(g_setpointPosition));
        //return;
    //}
//
    //// Convert angle to position [0, M_PI] -> [0, PULSES_PER_ROTATION]
    //int16_t releasePos = (int16_t) (releaseAngle / M_PI * PULSES_PER_ROTATION);
//
    //const int midpointIndex = BUFFER_SIZE / 2;
    //const float durationSpacing = period / 2 / (BUFFER_SIZE - 1);
//
    //// Fill up easiest parts of the buffer
    //g_setpointPosition[0] = PULSES_PER_ROTATION_2;
    //g_setpointPosition[BUFFER_SIZE - 1] = PULSES_PER_ROTATION_2;
//
    //// Fill in the midpoint (i.e. its highest point)
    //if (g_dir == right) {
        //g_setpointPosition[midpointIndex] = releasePos;
    //} else {
        //g_setpointPosition[midpointIndex] = releasePos + PULSES_PER_ROTATION_2;
    //}
//
    //// Fill up all other values in between
    //for (int i = 1; i < BUFFER_SIZE - 1; i++) {
        //if (i != midpointIndex) {
            //float t = durationSpacing * i;
            //g_setpointPosition[i] = (int16_t) round(sin(CONSTANT_OMEGA * t) * releasePos) + PULSES_PER_ROTATION_2;
//
            //if (g_dir == right) {
                //g_setpointPosition[i] = PULSES_PER_ROTATION - g_setpointPosition[i];
            //}
        //}
    //}
//}
//
//void error(enum err errorCode) {
    //g_stop = true;
    //printf("Stop flag asserted! Error code: %d\n", errorCode);
//};
//
//void trap(enum err errorCode) {
    //// Print globals
    //printf("Software trap called! Error code: %d\n", errorCode);
    //printf("Global variables when trap was called:\n");
    //// printf("g_setpointPosition: %d\n", g_setpointPosition);
    //printf("g_duration: %lu\n", g_duration);
    //printf("g_duration_store[0]: %lu\n", g_duration_store[0]);
    //printf("g_duration_store[1]: %lu\n", g_duration_store[1]);
    //printf("g_refTick: %lu\n", g_refTick);
    //printf("g_releaseAngle: %.2f\n", g_releaseAngle);
    //printf("g_dir: %d\n", g_dir);
//
    //// Stop motor
    //for (;;) {pwm_channel_update_duty(PWM, &g_pwm_channel_pin8, 0);};
//}
//
//void warning(enum warn warningCode) {
    //printf("Warning code: %d\n", warningCode);
//};
//
//void vApplicationIdleHook( void ) {}
//void vApplicationMallocFailedHook( void ) {for( ;; );}
//void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName ) {for( ;; );}
//void vApplicationTickHook( void ) {}