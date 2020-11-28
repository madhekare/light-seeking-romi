// /* EECS 149/249A Final Project Fall 2020
//   FlowerBot
//   Code partially taken from dronebotworkshop.com
//   tutorial on "Using the HC-SR04 Ultrasonic Distance Sensor with Arduino"
// */
//
// /* Notes:
// Timer: https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v15.0.0%2Flib_timer.html
// */
// #include <stdbool.h>
// #include <stdint.h>
// #include <stdio.h>
// #include "app_error.h"
// #include "app_timer.h"
// #include "nrf.h"
// #include "nrf_delay.h"
// #include "nrf_drv_clock.h"
// #include "nrf_drv_gpiote.h"
// #include "nrf_drv_rtc.h"
// #include "nrf_gpio.h"
// #include "nrf_log.h"
// #include "nrf_log_ctrl.h"
// #include "nrf_log_default_backends.h"
// #include "nrf_pwr_mgmt.h"
// #include "nrf_serial.h"
// #include "buckler.h"
// #include "gpio.h"
//
// #define echoPin 3
// #define trigPin 4
//
// typedef struct {
//   uint32_t RESERVED0;
//   uint32_t OUT;
//   uint32_t OUTSET;
//   uint32_t OUTCLR;
//   uint32_t IN;
//   uint32_t DIR;
//   uint32_t DIRSET;
//   uint32_t DIRCLR;
//   uint32_t LATCH;
//   uint32_t DETECTMODE;
//   uint32_t RESERVED1[118];
//   uint32_t PIN_CNF[32];
// } gpio_registers;
//
// // Declare an instance of nrf_drv_rtc for RTC2
// // softdevice BLE uses RTO0 and app_timer uses RTC1
// const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);
// double duration, distance;
// uint32_t ticks1, ticks2, ticks3;
// uint32_t ticks_duration1, ticks_duration2, ticks_duration3;
// ret_code_t error_code;
//
// double ticks_to_distance(uint32_t ticks) {
//   double time_from_ticks =  ticks / APP_TIMER_CLOCK_FREQ;
//   return time_from_ticks * 343 * 100; // 343 m/s for speed of sound, multiply by 100 to get result in cm
// }
//
// // void GPIOTE_IRQHandler(void) {
// //   printf("GPIOTE Interrupt Occurred\n");
// //   // printf("EVENTS_IN[0] %d\n", NRF_GPIOTE->EVENTS_IN[0]);
// //   // printf("EVENTS_IN[1] %d\n", NRF_GPIOTE->EVENTS_IN[1]);
// //   if (NRF_GPIOTE->EVENTS_IN[0] == 1) { // LotoHi Ultrasonic Sensor 1 detected
// //     // start clock
// //     printf("ticks1: %ld\n", ticks1);
// //     ticks1 = app_timer_cnt_get();
// //     printf("ticks1: %ld\n", ticks1);
// //     NRF_GPIOTE->EVENTS_IN[0] = 0; // Clear event so interrupt doesn't happen again
// //     // NRF_GPIOTE->EVENTS_IN[1] = 0;
// //   } else if (NRF_GPIOTE->EVENTS_IN[1] == 1) { // HitoLo Ultrasonic Sensor 1 detected
// //     // stop clock
// //     printf("stop clock");
// //     // if (ticks1 == 0) {
// //     //   NRF_GPIOTE->EVENTS_IN[0] = 0;
// //     //   NRF_GPIOTE->EVENTS_IN[1] = 0;
// //       return;
//     // }
//     // ticks_duration1 = app_timer_cnt_get() - ticks1;
//     // // ticks_duration1 = app_timer_cnt_diff_compute(app_timer_cnt_get(), ticks1);
//     // distance = ticks_to_distance(ticks_duration1);
//     // printf("Distance = ");
//     // printf("%f", distance);
//     // printf(" cm\n");
//     // ticks1 = 0;
//     // NRF_GPIOTE->EVENTS_IN[1] = 0;
//   // }
//     // printf("GPIOTE Interrupt Finished\n");
// // }
//
// /**@brief Function starting the internal LFCLK oscillator.
//  *
//  * @details This is needed by RTC1 which is used by the Application Timer
//  *          (When SoftDevice is enabled the LFCLK is always running and this is not needed).
//  */
// // obtained from tutorial:
// // https://devzone.nordicsemi.com/nordic/short-range-guides/b/software-development-kit/posts/application-timer-tutorial
// static void lfclk_request(void)
// {
//     error_code = nrf_drv_clock_init();
//     APP_ERROR_CHECK(error_code);
//     nrf_drv_clock_lfclk_request(NULL);
// }
//
// /***** SHOULD BE CALLED AT THE VERY START OF MAIN SO EVERYTHING INVOLVING app_timer IS SET UP *****/
// static void set_up_app_timer(void)
// {
//   lfclk_request(); // need to call this to make the oscillator start so timers actually work
//   error_code = app_timer_init();
//   APP_ERROR_CHECK(error_code);
// }
//
// /*@brief Function initialization and configuration of RTC driver instance*/
// static void rtc_config(void) {
//   // uint32_t err_code;
//
//   //Initialize RTC instnace
//   // nrf_drv_rtc_config_t rtc2Config = NRF_DRV_RTC_DEFAULT_CONFIG;
//   // rtc2Config.prescaler = 4095;
//   // err_code = nrf_drv_rtc_init(&rtc, &rtc2Config, rtc_handler);
//   // APP_ERROR_CHECK(err_code);
//
//   //Enable tick event & Interrupt
//   nrf_drv_rtc_tick_enable(&rtc, false);// false to disable interrupt
//   // err_code = nrf_drv_rtc_cc_set(&rtc,0,COMPARE_COUNTERTIME * 8, true);
//   // APP_ERROR_CHECK(err_code);
//
//   //Power on RTC instnace
//   nrf_drv_rtc_enable(&rtc);
// }
//
// // Convert tick values to microseconds
// uint32_t app_timer_ticks_to_usec(uint32_t ticks) {
//   return ((uint32_t)ROUNDED_DIV(
//                   (ticks) * 1000000 * (APP_TIMER_CONFIG_RTC_FREQUENCY + 1),
//                   (uint64_t)APP_TIMER_CLOCK_FREQ));
// }
//
// int main(void) {
//   if (!nrf_drv_gpiote_is_init()) {
//     error_code = nrf_drv_gpiote_init();
//   }
//   set_up_app_timer();
//   // error_code = NRF_SUCCESS;
//   // initialize RTT library
//   error_code = NRF_LOG_INIT(NULL);
//   APP_ERROR_CHECK(error_code);
//   NRF_LOG_DEFAULT_BACKENDS_INIT();
//   printf("Log initialized!\n");
//
//   // Pin configurations
//   // gpio_config(echoPin, 0); // configure echoPin to input
//   // gpio_config(trigPin, 1); // configure trigPin to output
//   // gpio_clear(echoPin);
//   // gpio_clear(trigPin);
//   // nrf_gpio_cfg_input(echoPin, NRF_GPIO_PIN_NOPULL);
//   // nrf_gpio_cfg_output(trigPin);
//   // nrf_gpio_pin_clear(trigPin); // trigPin not outputting signal
//   // Indicates that the pin will be driven by driver function calls and not by
//   // a task. Pin is initially clear.
//   nrf_drv_gpiote_out_config_t trigConfig = GPIOTE_CONFIG_OUT_SIMPLE(false);
//   // trigConfig.init_state = false; // Pin is initially clear
//   error_code = nrf_drv_gpiote_out_init(trigPin, &trigConfig);
//
//   // gpio_registers* registers = 0x50000500;
//
//   // Setup event for LotoHi HiToLo transition on echoPin
//   // NRF_GPIOTE->CONFIG[0] = 0x00010301; // Setup Polarity to LotoHi, PSEL to 3, and Mode to Event
//   // NRF_GPIOTE->CONFIG[1] = 0x00020301; // Setup Polarity to HiToLo, PSEL to 3, and Mode to Event
//
//   // Enable GPIOTE Interrupt for Event[0] and Event[1]
//   // NRF_GPIOTE->INTENSET |= 1;
//   // NRF_GPIOTE->INTENSET |= 2;
//
//   // Enable GPIOTE Interrupt in the Nested Vector Interrupt Controller (NVIC)
//   NVIC_EnableIRQ(GPIOTE_IRQn);
//   NVIC_SetPriority(GPIOTE_IRQn, 5);
//
//   /* Timer Setup,if in GPIOTE_IRQHandler ticks is -1, we detected a
//   HitoLo event before a LotoHi and therefore should ignore the interrupt
//   */
//   ticks1 = 1000;
//   ticks2 = 0;
//   ticks3 = 0;
//
//   while (1) {
//     printf("Looping\n");
//     nrf_delay_ms(1000);
//     ticks1 = app_timer_cnt_get();
//     printf("ticks1: %ld\n", ticks1);
//     // Writing a pulse to the HC-SR04 Trigger Pin
//     gpio_clear(trigPin);
//     nrf_delay_ms(200);
//     gpio_set(trigPin);
//     nrf_delay_ms(100);
//     // gpio_clear(trigPin);
//     // nrf_gpio_pin_set(trigPin);
//     // nrf_delay_ms(1000);
//     // nrf_gpio_pin_clear(trigPin);
//     // Measure the response from the HC-SR04 Echo Pin
//     // duration = pulseIn(echoPin, 1);
//
//     // Determine distance from duration
//     // Use 34m per second as speed of sound
//     // distance =  (duration/2) * 0.0343;
//
//     // Print results
//     // printf("Distance = ");
//     // if (distance >=400 || distance <=2) {
//     //   printf("Out of range\n");
//     // } else {
//     //   printf("%f", distance);
//     //   printf(" cm\n");
//     // }
//     // nrf_delay_ms(1000);
//   }
//
// }
