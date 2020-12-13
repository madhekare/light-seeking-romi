// BLE advertisement template variant from EECS149 lab
//
// Advertises device name: EE149

#include <stdbool.h>
#include <stdint.h>
#include <ble_advdata.h>
#include "nrf.h"
#include "app_util.h"
#include "nrf_twi_mngr.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "nrf_log_default_backends.h"
#include "ultrasonic.h"
#include "buckler.h"
#include "simple_ble.h"


#include "opt3004.h"


// Create a timer
//APP_TIMER_DEF(adv_timer);

// I2C manager
//NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0000,  // TODO: replace with your lab bench number
        .adv_name          = "EE149", // Note that this name is not displayed to save room in the advertisement for data.
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};


/*******************************************************************************
 *   State for this application
 ******************************************************************************/
// Main application state
static simple_ble_service_t sens_service = {{
    .uuid128 = {0x70,0x6C,0x98,0x41,0xCE,0x43,0x14,0xA9,
                0xB5,0x4D,0x22,0x2B,0x89,0x10,0xE6,0x32}
}};

static simple_ble_char_t sens_state_char = {.uuid16 = 0x108a};
static simple_ble_char_t ult_state_char_front = {.uuid16 = 0x108b};
static simple_ble_char_t ult_state_char_right = {.uuid16 = 0x108c};
static float light_data = 1234.12345456;
static float ult_data_front = 1.0;
static float ult_data_right = 1.0;
simple_ble_app_t* simple_ble_app;



void light_timer_callback() {
    printf("Light timer fired!\n");
    light_data = opt3004_read_result();
    printf("Reading (lux): %f\n", opt3004_read_result());
    // Use Simple BLE function to read light sensor and put data in advertisement
}

//float light_data;

int get_ble_adv(void) {
  
  ret_code_t error_code = NRF_SUCCESS;
  // HC-SR04 Trigger and Echo Pins
  uint32_t pinTrigFront = 4;
  uint32_t pinEchoFront = 3;
  uint32_t pinTrigLeft = 5;
  uint32_t pinEchoLeft = 2;
  uint32_t pinTrigRight = 13; // for some reason, can't drive when using pin 19
  uint32_t pinEchoRight = 16; // for some reason, can't drive when using pin 20

  // Set up timer
  app_timer_init();
  start_timer_rev1();

  // Set up HC-SR04 pins
  nrf_gpio_pin_dir_set(pinTrigFront, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEchoFront, NRF_GPIO_PIN_DIR_INPUT);
  nrf_gpio_pin_dir_set(pinTrigLeft, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEchoLeft, NRF_GPIO_PIN_DIR_INPUT);
  nrf_gpio_pin_dir_set(pinTrigRight, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEchoRight, NRF_GPIO_PIN_DIR_INPUT);

  // Initialize

  /*// initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);

  opt3004_config_t config = {
    .range_number = OPT3004_AUTORANGE,
    .conversion_time = OPT3004_CONVERSION_100MS,
    .latch_interrupt = 1,
    .interrupt_polarity = OPT3004_INTERRUPT_ACTIVE_LO,
    .fault_count = OPT3004_FAULT_COUNT_1,
  };

  // initialize opt3004 driver
  opt3004_init(&twi_mngr_instance);
  error_code = opt3004_config(config);
  // set up opt3004 to read continuously
  opt3004_continuous();

  printf("opt3004 initialized: %ld\n", error_code);*/

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&sens_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(light_data), (uint8_t*)&light_data,
      &sens_service, &sens_state_char);

  simple_ble_add_characteristic(1, 1, 0, 0,
          sizeof(ult_data_front), (uint8_t*)&ult_data_front,
          &sens_service, &ult_state_char_front);

  simple_ble_add_characteristic(1, 1, 0, 0,
          sizeof(ult_data_right), (uint8_t*)&ult_data_right,
          &sens_service, &ult_state_char_right);

  // ble_advdata_service_data_t serv_data = {.uuid16 = 0x108a, .data = (uint8_t*)&light_data};
  // ble_advdata_t data= {.p_service_data_array = &serv_data};
  // simple_ble_set_adv(data, NULL);
  // start_advertising();


  // TODO replace this with advertisement sending light data
  simple_ble_adv_only_name();

  // Set a timer to read the light sensor and update advertisement data every second.
  // app_timer_init();
  // app_timer_create(&adv_timer, APP_TIMER_MODE_REPEATED, (app_timer_timeout_handler_t) light_timer_callback);
  // app_timer_start(adv_timer, APP_TIMER_TICKS(1000), NULL); // 1000 milliseconds

  while(1) {
    // Sleep while SoftDevice handles BLE
    
    light_data = opt3004_read_result();
    printf("HELLO");
    getDistance(&ult_data_front, pinTrigFront, pinEchoFront);
    getDistance(&ult_data_right, pinTrigRight, pinEchoRight);
    //light_data = ((int)(data * 100 + .5) / 100.0);
    printf("Reading (lux): %f\n", light_data);
    printf("Front dist %f\n", ult_data_front);
    printf("Right dist %f\n", ult_data_right);
    //nrf_delay_ms(1);
  //  power_manage();
  }
}