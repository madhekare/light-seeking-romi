/*
   HC-SR04 test /main.c:
   Talking to the HC-SR04 ultrasonic sensor.
   Uses Timer1 to measure pulse width from sensor. Sends distance over
   BLE via NUS (Nordic UART Service)

   Connections:
   VCC   --> 5V
   GND   --> GND
   Trig  --> P0.04
   Echo  --> P0.03 via resistor divider to 3.3V TTL

   Author: Mahesh Venkitachalam
   Website: electronut.in
   Reference:
   http://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk51.v9.0.0%2Findex.html
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include "app_error.h"
#include "app_timer.h"
// #include "app_uart.h"
// #include "app_pwm.h"
#include "app_util_platform.h"
// #include "buckler.h"
#include "nordic_common.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_serial.h"
#include "nrfx_timer.h"

int timer_instance = 2;
const nrfx_timer_t TIMER_ULTRASONIC = NRFX_TIMER_INSTANCE(2);

// #include "nrf51_bitfields.h"
// #include "ble_hci.h"
// #include "ble_advdata.h"
// #include "ble_advertising.h"
// #include "ble_conn_params.h"
// #include "ble_nus.h"
// #include "softdevice_handler.h"
// #include "boards.h"
// #include "pstorage.h"
// #include "pstorage_platform.h"

// static ble_nus_t m_nus;
// static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;

// Function for assert macro callback.
// void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
// {
//   app_error_handler(0xDEADBEEF, line_num, p_file_name);
// }

// void app_error_handler(uint32_t error_code, uint32_t line_num,
//                        const uint8_t * p_file_name)
// {
//   printf("Error code: %lu line num: %lu\n", error_code, line_num);
// }


// Function for the GAP initialization.
// static void gap_params_init(void)
// {
//     uint32_t                err_code;
//     ble_gap_conn_params_t   gap_conn_params;
//     ble_gap_conn_sec_mode_t sec_mode;
//
//     BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
//
//     const char deviceName[] = "HC-SR04";
//
//     err_code = sd_ble_gap_device_name_set(&sec_mode,
//                                           (const uint8_t *) deviceName,
//                                           strlen(deviceName));
//     APP_ERROR_CHECK(err_code);
//
//     memset(&gap_conn_params, 0, sizeof(gap_conn_params));
//
//     gap_conn_params.min_conn_interval = MSEC_TO_UNITS(20, UNIT_1_25_MS);
//     gap_conn_params.max_conn_interval = MSEC_TO_UNITS(75, UNIT_1_25_MS);
//     gap_conn_params.slave_latency     = 0;
//     gap_conn_params.conn_sup_timeout  = MSEC_TO_UNITS(4000, UNIT_10_MS);
//
//     err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
//     APP_ERROR_CHECK(err_code);
// }

// Function for handling the data from the Nordic UART Service.
// static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data,
//                              uint16_t length)
// {
//     for (uint32_t i = 0; i < length; i++)
//     {
//         while(app_uart_put(p_data[i]) != NRF_SUCCESS);
//     }
//     while(app_uart_put('\n') != NRF_SUCCESS);
// }

// Function for initializing services that will be used by the application.
// static void services_init(void)
// {
//     uint32_t       err_code;
//     ble_nus_init_t nus_init;
//
//     memset(&nus_init, 0, sizeof(nus_init));
//
//     nus_init.data_handler = nus_data_handler;
//
//     err_code = ble_nus_init(&m_nus, &nus_init);
//     APP_ERROR_CHECK(err_code);
// }


// Function for handling an event from the Connection Parameters Module.
// static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
// {
//     uint32_t err_code;
//
//     if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
//     {
//         err_code = sd_ble_gap_disconnect(m_conn_handle,
//                                          BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
//         APP_ERROR_CHECK(err_code);
//     }
// }

// Function for handling errors from the Connection Parameters module.
// static void conn_params_error_handler(uint32_t nrf_error)
// {
//     APP_ERROR_HANDLER(nrf_error);
// }


// Function for initializing the Connection Parameters module.
// static void conn_params_init(void)
// {
//     uint32_t               err_code;
//     ble_conn_params_init_t cp_init;
//
//     memset(&cp_init, 0, sizeof(cp_init));
//
//     cp_init.p_conn_params                  = NULL;
//     cp_init.first_conn_params_update_delay = APP_TIMER_TICKS(5000, 0);
//     cp_init.next_conn_params_update_delay  = APP_TIMER_TICKS(30000, 0);
//     cp_init.max_conn_params_update_count   = 3;
//     cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
//     cp_init.disconnect_on_fail             = false;
//     cp_init.evt_handler                    = on_conn_params_evt;
//     cp_init.error_handler                  = conn_params_error_handler;
//
//     err_code = ble_conn_params_init(&cp_init);
//     APP_ERROR_CHECK(err_code);
// }

// Function for handling advertising events.
// static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
// {
//     switch (ble_adv_evt)
//     {
//         case BLE_ADV_EVT_FAST:
//              break;
//         case BLE_ADV_EVT_IDLE:
//              break;
//         default:
//             break;
//     }
// }


// Function for the Application's S110 SoftDevice event handler.
// static void on_ble_evt(ble_evt_t * p_ble_evt)
// {
//     uint32_t                         err_code;
//
//     switch (p_ble_evt->header.evt_id)
//     {
//         case BLE_GAP_EVT_CONNECTED:
//             m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
//             break;
//
//         case BLE_GAP_EVT_DISCONNECTED:
//             m_conn_handle = BLE_CONN_HANDLE_INVALID;
//             break;
//
//         case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
//             // Pairing not supported
//             err_code =
//               sd_ble_gap_sec_params_reply(m_conn_handle,
//                                           BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
//                                           NULL, NULL);
//             APP_ERROR_CHECK(err_code);
//             break;
//
//         case BLE_GATTS_EVT_SYS_ATTR_MISSING:
//             // No system attributes have been stored.
//             err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
//             APP_ERROR_CHECK(err_code);
//             break;
//
//         default:
//             // No implementation needed.
//             break;
//     }
// }


// Function for dispatching a S110 SoftDevice event to all modules
// with a S110 SoftDevice event handler.
// static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
// {
//     ble_conn_params_on_ble_evt(p_ble_evt);
//     ble_nus_on_ble_evt(&m_nus, p_ble_evt);
//     on_ble_evt(p_ble_evt);
//     ble_advertising_on_ble_evt(p_ble_evt);
// }

// Function for the S110 SoftDevice initialization.
// static void ble_stack_init(void)
// {
//     uint32_t err_code;
//
//     // Initialize SoftDevice.
//     SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, NULL);
//
//     // Enable BLE stack.
//     ble_enable_params_t ble_enable_params;
//     memset(&ble_enable_params, 0, sizeof(ble_enable_params));
//
//     ble_enable_params.gatts_enable_params.service_changed = 0;
//     err_code = sd_ble_enable(&ble_enable_params);
//     APP_ERROR_CHECK(err_code);
//
//     // Subscribe for BLE events.
//     err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
//     APP_ERROR_CHECK(err_code);
// }

// Function for handling app_uart events.
// void uart_event_handle(app_uart_evt_t * p_event)
// {
//     static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//     static uint8_t index = 0;
//     uint32_t       err_code;
//
//     switch (p_event->evt_type)
//     {
//         case APP_UART_DATA_READY:
//             UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//             index++;
//
//             if ((data_array[index - 1] == '\n') ||
//                 (index >= (BLE_NUS_MAX_DATA_LEN)))
//             {
//                 err_code = ble_nus_string_send(&m_nus, data_array, index);
//                 if (err_code != NRF_ERROR_INVALID_STATE)
//                 {
//                     APP_ERROR_CHECK(err_code);
//                 }
//
//                 index = 0;
//             }
//             break;
//
//         case APP_UART_COMMUNICATION_ERROR:
//             APP_ERROR_HANDLER(p_event->data.error_communication);
//             break;
//
//         case APP_UART_FIFO_ERROR:
//             APP_ERROR_HANDLER(p_event->data.error_code);
//             break;
//
//         default:
//             break;
//     }
// }
//
// // Function for initializing the UART module.
// static void uart_init(void)
// {
//     uint32_t                     err_code;
//     const app_uart_comm_params_t comm_params =
//     {
//         RX_PIN_NUMBER,
//         TX_PIN_NUMBER,
//         RTS_PIN_NUMBER,
//         CTS_PIN_NUMBER,
//         APP_UART_FLOW_CONTROL_ENABLED,
//         false,
//         UART_BAUDRATE_BAUDRATE_Baud38400
//     };
//
//     APP_UART_FIFO_INIT( &comm_params,
//                        256,
//                        256,
//                        uart_event_handle,
//                        APP_IRQ_PRIORITY_LOW,
//                        err_code);
//     APP_ERROR_CHECK(err_code);
// }
//
//
// // Function for initializing the Advertising functionality.
// static void advertising_init(void)
// {
//     uint32_t      err_code;
//     ble_advdata_t advdata;
//     ble_advdata_t scanrsp;
//     ble_uuid_t m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE,
//                                  BLE_UUID_TYPE_VENDOR_BEGIN}};
//
//     // Build advertising data struct to pass into @ref ble_advertising_init.
//     memset(&advdata, 0, sizeof(advdata));
//     advdata.name_type          = BLE_ADVDATA_FULL_NAME;
//     advdata.include_appearance = false;
//     advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
//
//     memset(&scanrsp, 0, sizeof(scanrsp));
//     scanrsp.uuids_complete.uuid_cnt =
//       sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
//     scanrsp.uuids_complete.p_uuids  = m_adv_uuids;
//
//     ble_adv_modes_config_t options = {0};
//     options.ble_adv_fast_enabled  = BLE_ADV_FAST_ENABLED;
//     options.ble_adv_fast_interval = 64;
//     options.ble_adv_fast_timeout  = 180;
//
//     err_code = ble_advertising_init(&advdata, &scanrsp, &options,
//                                     on_adv_evt, NULL);
//     APP_ERROR_CHECK(err_code);
// }

// counter
static volatile uint32_t tCount = 0;

// // HC-SR04 Trigger - P0.04
// uint32_t pinTrigFront = 4;
// // HC-SR04 Echo - P0.03
// uint32_t pinEchoFront = 3;

// count to us (micro seconds) conversion factor
// set in start_timer()
static volatile float countToUs = 1;

// get distance measurement from HC-SR04:
// Send a 10us HIGH pulse on the Trigger pin.
// The sensor sends out a “sonic burst” of 8 cycles.
// Listen to the Echo pin, and the duration of the next HIGH
// signal will give you the time taken by the sound to go back
// and forth from sensor to target.
// returns true only if a valid distance is obtained
bool getDistance(float* dist, int pinTrig, int pinEcho) {
  // send 12us trigger pulse
  //    _
  // __| |__
  nrf_gpio_pin_clear(pinTrig);
  nrf_delay_us(20);
  nrf_gpio_pin_set(pinTrig);
  nrf_delay_us(12);
  nrf_gpio_pin_clear(pinTrig);
  nrf_delay_us(20);

  // listen for echo and time it
  //       ____________
  // _____|            |___

  // wait till Echo pin goes high
  while(!nrf_gpio_pin_read(pinEcho));
  // reset counter
  tCount = 0;
  // wait till Echo pin goes low
  while(nrf_gpio_pin_read(pinEcho));

  // calculate duration in us
  float duration = countToUs*tCount;

  // dist = duration * speed of sound * 1/2
  // dist in cm = duration in us * 10^-6 * 340.29 * 100 * 1/2
  float distance = duration*0.017;

  // check value
  if(distance < 400.0) {

    // save
    *dist = distance;

    return true;
  }
  else {
    return false;
  }
}

float calc_theta(float frontDist, float backDist, float distBetweenSensors) {
	// printf("Inside calc theta\n");
	// printf("frontDist: %f\n", frontDist);
	// printf("backDist: %f\n", backDist);
	// printf("distBetweenSensors: %f\n", distBetweenSensors);
	// printf("before atan2: %f\n", (backDist-frontDist)/distBetweenSensors);
	// printf("theta in calc_theta: %f\n", atan2f(distBetweenSensors, backDist-frontDist) * 180/M_PI);
	// printf("theta2 no f: %f\n", atan2(distBetweenSensors, backDist-frontDist) * 180/M_PI);
	// printf("theta3 swapped: %f\n", atan2f(backDist-frontDist, distBetweenSensors) * 180/M_PI);
	// printf("theta4 swapped no f: %f\n", atan2(backDist-frontDist, distBetweenSensors) * 180/M_PI);
	// printf("ending calc_theta\n");
	return atan2f(backDist-frontDist, distBetweenSensors) * 180/M_PI;
}

float getDistanceKalman(float* dist, int pinTrig, int pinEcho, int iters) {
  assert(iters>0);
  float gain;
  float estimate = 20; // initial estimate
  float error_estimate = 5; // initial error estimate
  float error_measurement = 5; // assumed measurement error
  float measurement;
  for (int i=0; i<iters; i++) {
    while(!getDistance(dist, pinTrig, pinEcho));
    measurement = *dist;
    gain = error_estimate/(error_estimate + error_measurement);
    estimate = (1-gain)*estimate + gain*measurement;
    error_estimate = (1-gain)*error_estimate;
    printf("iteration: %d, estimate: %f\n", i, estimate);
  }
  printf("\n");
  return estimate;
}

int compareFloat(const void * a, const void * b) {
  float fa = *(const float*) a;
  float fb = *(const float*) b;
  return (fa > fb) - (fa < fb);
}

float getMedian(float distances[], int num_elems) {
  qsort(distances, num_elems, sizeof(distances[0]), compareFloat);
  if (num_elems % 2 == 0) {
    int index1 = num_elems / 2;
    int index2 = index1 - 1;
    return (distances[index1]+distances[index2])/2;
  } else {
    int index = (int) floor(num_elems/2);
    return distances[index];
  }
}

float getDistanceMedian(float* dist, int pinTrig, int pinEcho, int iters) {
  float distances[iters];
  for (int i = 0; i<iters; i++) {
    while(!getDistance(dist, pinTrig, pinEcho));
    distances[i] = *dist;
    // printf("iteration: %d, distance: %f\n", i, *dist);
  }
  // printf("median: %f\n", getMedian(distances, iters));
  // printf("\n");
  return getMedian(distances, iters);
}

float getDistanceDifference(float* dist1, int pinTrig1, int pinEcho1, float* dist2, int pinTrig2, int pinEcho2) {
  while(!getDistance(dist1, pinTrig1, pinEcho1));
  while(!getDistance(dist2, pinTrig2, pinEcho2));
  return fabs(*dist1-*dist2);
}

float getThetaMedian(float* frontDist, int pinTrig1, int pinEcho1, float* backDist, int pinTrig2, int pinEcho2, int iters, float distBetweenSensors) {
  float thetas[iters];
  for (int i=0; i<iters; i++) {
    float frontDistMedian = getDistanceMedian(frontDist, pinTrig1, pinEcho1, 1);
    float backDistMedian = getDistanceMedian(backDist, pinTrig2, pinEcho2, 1);

    printf("frontDistMedian: %f\n", frontDistMedian);
    printf("backDistMedian: %f\n", backDistMedian);
    thetas[i] = calc_theta(frontDistMedian, backDistMedian, distBetweenSensors);

    // printf("iteration: %d, theta: %f\n", i, thetas[i]);
  }
  // printf("\n");
  printf("iters: %d", iters);
  for (int j=0; j<iters; j++) {
    printf("iteration: %d, theta: %f\n", j, thetas[j]);
  }
  printf("median theta: %f\n", getMedian(thetas, iters));
  return getMedian(thetas, iters);
}

float getDistanceDifferenceKalman(float* dist1, int pinTrig1, int pinEcho1, float* dist2, int pinTrig2, int pinEcho2, int iters) {
  // May need to implement Kalman filter here to increase reliability
  assert(iters>0);
  float gain;
  float estimate = 2; // initial estimate
  float error_estimate = 2; // initial error estimate
  float error_measurement = 3; // assumed measurement error
  float measurement;
  for (int i=0; i<iters; i++) {
    measurement = getDistanceDifference(dist1, pinTrig1, pinEcho1, dist2, pinTrig2, pinEcho2);
    gain = error_estimate/(error_estimate + error_measurement);
    estimate = (1-gain)*estimate + gain*measurement;
    error_estimate = (1-gain)*error_estimate;
    printf("iteration: %d, estimate: %f\n", i, estimate);
  }
  printf("\n");
  return estimate;
}

void timer_ultrasonic_custom_event_handler(void) {
	// if (NRF_TIMER1->EVENTS_COMPARE[1] &&
  //     NRF_TIMER1->INTENSET & TIMER_INTENSET_COMPARE1_Msk) {
  //
  //   // clear compare register event
  //   NRF_TIMER1->EVENTS_COMPARE[1] = 0;
  //
  //   // increment count
  //   tCount++;
  // }
  tCount++;
  if (timer_instance == 2) {
    NRF_TIMER2->EVENTS_COMPARE[1] = 0;
  } else {
    NRF_TIMER1->EVENTS_COMPARE[1] = 0;
  }
}

void start_timer_rev1() {
  nrfx_timer_config_t timer_cfg = NRFX_TIMER_DEFAULT_CONFIG;
  ret_code_t err_code = NRF_SUCCESS;

  timer_cfg.bit_width = 0; // 0 for 16 bit, 1 for 8 bit, 2 for 24 bit, 3 for 32 bit
  timer_cfg.mode = 0; // 0 for timer, 1 for counter

  err_code = nrfx_timer_init(&TIMER_ULTRASONIC, &timer_cfg, (nrfx_timer_event_handler_t) timer_ultrasonic_custom_event_handler);
  APP_ERROR_CHECK(err_code);

  // set compare
  uint16_t comp1 = 500;
  nrfx_timer_extended_compare(&TIMER_ULTRASONIC, NRF_TIMER_CC_CHANNEL1, comp1, NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);

  // set conversion factor
  countToUs = 0.0625*comp1;

  printf("timer tick = %f us\n", countToUs);

  nrfx_timer_enable(&TIMER_ULTRASONIC);
}

// Application main function.
// int get_distance(void)
// {
//     uint32_t err_code;
//
//     /* set up timers, see link below for argument specifications
//     arg0: Prescaler
//     arg1: Size of queues holding timer operations
//     arg2: Pointer to memory buffer
//     arg3: Function for passing time-out events to the scheduler.
//     Set to NULL to make the timer module call the time-out handler directly
//     from the timer interrupt handler.
//     https://infocenter.nordicsemi.com/index.jsp?topic=%2Fcom.nordic.infocenter.sdk5.v11.0.0%2Fgroup__app__timer.html
//     */
//     // APP_TIMER_INIT(0, 4, 4, false);
//     app_timer_init();
//     initlialize BLE
//     ble_stack_init();
//     gap_params_init();
//     services_init();
//     advertising_init();
//     conn_params_init();
//     err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
//     APP_ERROR_CHECK(err_code);
//
//     intialize UART
//     uart_init();
//
//     start_timer();
//
//     prints to serial port
//     printf("starting...\n");
//
//     // set up HC-SR04 pins
//     nrf_gpio_pin_dir_set(pinTrigFront, NRF_GPIO_PIN_DIR_OUTPUT);
//     nrf_gpio_pin_dir_set(pinEchoFront, NRF_GPIO_PIN_DIR_INPUT);
//
//     counter = 0;
//     // main loop:
//     while(counter == 0) {
//       counter++;
//       // get HC-SR04 distance
//       float dist;
//       if(getDistance(&dist)) {
//
//         // enable to print to serial port
//         printf("dist = %f cm\n", dist);
//
//         // send distance via NUS
//         // uint8_t str[4];
//         // sprintf((char*)str, "%f cm", dist);
//         // ble_nus_string_send(&m_nus, str, strlen((char*)str));
//       }
//
//       // delay
//       nrf_delay_ms(250);
//     }
// }
