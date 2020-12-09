/* main function for EE 149/249A Final Project: flowerBot
Contributors: Albert Loekman, Esha Madhekar, Nidhi Kakulawaram
*/
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

// #include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"

#include "app_util_platform.h"
#include "nrf_gpiote.h"
#include "trace_wall.h"
#include "ultrasonic.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "mpu9250.h"
#include "lsm9ds1.h"
#include "opt3004.h"
#include "helper_functions.h"

#define ROOMLEN 1
#define TURN_DIST 0.2

uint16_t encoder_value = 0;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// HC-SR04 Trigger and Echo Pins
uint32_t pinTrigFront = 4;
uint32_t pinEchoFront = 3;
uint32_t pinTrigLeft = 5;
uint32_t pinEchoLeft = 2;
// uint32_t pinTrigRight = 19;
uint32_t pinTrigRight = 13; // for some reason, can't drive when using pin 19
// uint32_t pinEchoRight = 20;
uint32_t pinEchoRight = 16; // for some reason, can't drive when using pin 20



float frontDist, leftDist, rightDist;


int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  //initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
    .sck_pin = BUCKLER_LCD_SCLK,
    .mosi_pin = BUCKLER_LCD_MOSI,
    .miso_pin = BUCKLER_LCD_MISO,
    .ss_pin = BUCKLER_LCD_CS,
    .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
    .orc = 0,
    .frequency = NRF_DRV_SPI_FREQ_4M,
    .mode = NRF_DRV_SPI_MODE_2,
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  //mpu9250_init(&twi_mngr_instance);
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

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
  opt3004_continuous();
  printf("opt3004 initialized: %ld\n", error_code);

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};
  uint32_t speed = 75;
  float distance = 0;
  //float degrees = 0;
  bool is_up = true;
  bool is_first = true;

  trace_wall();
  explore_room();



}
