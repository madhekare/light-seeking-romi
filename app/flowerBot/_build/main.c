// Robot Template app
//
// Framework for creating applications that control the Kobuki robot
#include "controller.h"
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

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "lsm9ds1.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
  const float CONVERSION = 0.0006108;
  float distance = 0;
  if (current_encoder < previous_encoder) {
      if (previous_encoder - current_encoder > 30000) {
        distance = (current_encoder - previous_encoder + 655365);
      }
      else {
        distance = current_encoder - previous_encoder;
      }
  }
  else {
      if (current_encoder  - previous_encoder > 30000) {
        distance = (current_encoder - previous_encoder - 655365);
      }
      else {
        distance = current_encoder - previous_encoder;
      }
  }
  float val = CONVERSION * distance;
  if (fabs(val) > 300) {
     val = 0;
  }
  return val;
}

static int16_t check_bump(KobukiSensors_t *sensors){
    if (sensors->bumps_wheelDrops.bumpLeft) {
        return -15;
    }
    else if (sensors->bumps_wheelDrops.bumpCenter) {
        return -15;
    }
    else if (sensors->bumps_wheelDrops.bumpRight) {
        return 15;
    }
    return 0;
}

static int16_t check_cliff(KobukiSensors_t *sensors){
    if (sensors->cliffLeft) {
        return -15;
    }
    else if (sensors->cliffCenter) {
        return -15;
    }
    else if (sensors->cliffRight) {
        return 15;
    }
    return 0;
}

static float read_tilt() {
  lsm9ds1_measurement_t measurement = lsm9ds1_read_accelerometer();
  float x = measurement.x_axis;
  float y = measurement.y_axis;
  float z = measurement.z_axis;
  return 57.296 * atan2(y, sqrt(x * x + z * z));
}
int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs
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
  lsm9ds1_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
KobukiSensors_t sensors = {0};
float distance_total = 0;
uint16_t encoder_cur = 0;
uint16_t encoder_prev = 0;
const float target_dist_forward = 2;
const float target_dist_back = -.10;
float target_dist = 2;
float tilt_memory = 0;
float tilt_decay = .85;
int16_t target_angle = 90;
int16_t collide = 0;
int16_t is_cliff = 0;
float target_slope = 15;
float min_slope = 0;
float max_slope = 0;
float first_tilt = 0;
float last_tilt = 0;
uint8_t orient_count = 0;
bool wait = true;
bool up = true;
bool prev_line = false;
bool prev_prev_line = false;
robot_state_t state = OFF;



  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(1);
    // handle states
    switch(state) {
      case OFF: {
        // transition logic
        printf("IN OFF\n");
        if (is_button_pressed(&sensors)) {
          state = ORIENT;
          up = true;
        } else {
          // perform state-specific actions here
          display_write("OFF", DISPLAY_LINE_0);
          kobukiDriveDirect(0, 0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }
      case ORIENT: {
        float tilt = read_tilt();
        printf("IN ORIENT\n");
        int16_t use_slope = up ? -target_slope : target_slope;
        if (is_button_pressed(&sensors)) {
          kobukiDriveDirect(0, 0);
          state = OFF;
          wait = true;
        } 
        else if (wait) {
            kobukiDriveDirect(0, 0);
            nrf_delay_ms(500);
            wait = false;
            state = ORIENT;
        }
        else if (fabs(tilt - use_slope) < 1.5) { //(orient_count >= 12 && fabs(tilt - use_slope) < 1.5) {
          state = DRIVING;
          tilt_memory = tilt;
          wait = true;
        }
        else {
          // perform state-specific actions here
         // if (first_tilt == 0) {
         //     first_tilt = tilt;
         // } else if ((tilt > first_tilt) && (first_tilt > last_tilt)){
         //     orient_count++;
         // }
         // if (orient_count < 12){
         //     if (tilt < min_slope) {
         //         min_slope = tilt;
         //     }
         //     if (tilt > max_slope) {
         //         max_slope = tilt;
         //     }
         //     target_slope = (fabs(min_slope) + max_slope) / 2;
         // }
          display_write("ORIENTING", DISPLAY_LINE_0);
          char buf[16];
          snprintf(buf, 16, "%f", tilt);
          display_write(buf, DISPLAY_LINE_1);
          int16_t speed = 80;
          kobukiDriveDirect(-speed, speed);
          state = ORIENT;
          last_tilt = tilt;
        }
        break;
      }
      case DRIVING: {
      printf("IN DRIVING\n");
        // transition logic
        float tilt = read_tilt();
        int16_t use_slope = up ? -target_slope : target_slope;
        tilt_memory = tilt_decay * tilt_memory + (1 - tilt_decay) * tilt;
        collide = check_bump(&sensors);
//        bool cliff_loc = false;
//        bool is_cliff = check_cliff(&sensors, &cliff_loc);
        is_cliff = check_cliff(&sensors);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          wait = true;
        } 
        // else if (is_cliff && (fabs(tilt - use_slope) > 5)){
        //   if (prev_prev_line) {
        //     if (prev_line) {
        //       distance_total = 0;
        //       encoder_prev = sensors.rightWheelEncoder;
        //       target_dist = target_dist_back;
        //       //target_angle = collide;
        //       target_angle = is_cliff;
        //       state = AVOID_DRIVING;
        //       wait = true;
        //     }
        //     prev_line = true;
        //   }
        //   prev_prev_line = true;
        // }
        else if (is_cliff) {
            distance_total = 0;
            encoder_prev = sensors.rightWheelEncoder;
            target_dist = target_dist_back;
            //target_angle = collide;
            target_angle = is_cliff;
            state = AVOID_DRIVING;
            wait = true;
        }
        else if (wait) {
            kobukiDriveDirect(0, 0);
            nrf_delay_ms(500);
            wait = false;
            state = DRIVING;
        }
        else if (up && fabs(tilt_memory) < 2){
          state = TURNING;
          distance_total = 0;
          target_angle = 180;
          wait = true;
        }
        else {
          // perform state-specific actions here
          prev_line = false;
          prev_prev_line = false;
          display_write("DRIVING", DISPLAY_LINE_0);
          char buf[16];
          snprintf(buf, 16, "%f", distance_total);
          display_write(buf, DISPLAY_LINE_1);
          int16_t speed = 110;
          if (!up) {
            display_write("DRIVING DOWN", DISPLAY_LINE_0);
            int16_t speed = 20;
          }
          kobukiDriveDirect(speed, speed);
          state = DRIVING;
        }
        break; // each case needs to end with break!
    }
      case AVOID_DRIVING: {
        printf("IN AVOID_DRIVING\n");
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = OFF;
          distance_total = 0;
          wait = true;
        } 
        else if (wait) {
            kobukiDriveDirect(0, 0);
            nrf_delay_ms(500);
            wait = false;
            state = AVOID_DRIVING;
            encoder_prev = sensors.rightWheelEncoder;
        }
        else if (fabs(distance_total - target_dist) < .01){
          state = AVOID_TURNING;
          distance_total = 0;
          wait = true;
        }
        else {
          // perform state-specific actions here
          display_write("AVOID_DRIVING", DISPLAY_LINE_0);
          encoder_cur = sensors.rightWheelEncoder;
          distance_total += measure_distance(encoder_cur, encoder_prev);
          encoder_prev = encoder_cur;
          char buf[16];
          snprintf(buf, 16, "%f", distance_total);
          display_write(buf, DISPLAY_LINE_1);
    float diff = target_dist - distance_total;
          //float diff = distance_total - target_dist;
          int8_t sign = (2 * (diff > 0)) - 1;
          // int16_t speed = sign * fmax(220 * fabs(diff), 120);
          int16_t speed = -50;
          kobukiDriveDirect(speed, speed);
          state = AVOID_DRIVING;
        }
        break; // each case needs to end with break!
      }
      case TURNING: {
        // transition logic
        printf("IN TURNING\n");
        collide = check_bump(&sensors);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          lsm9ds1_stop_gyro_integration();
          wait = true;
        } 
        else if (collide) {
          distance_total = 0;
          encoder_prev = sensors.rightWheelEncoder;
          target_dist = target_dist_back;
          target_angle = collide;
          state = AVOID_DRIVING;
          lsm9ds1_stop_gyro_integration();
          wait = true;
        }
        else if (wait) {
            kobukiDriveDirect(0, 0);
            nrf_delay_ms(500);
            lsm9ds1_start_gyro_integration();
            wait = false;
            state = TURNING;
        }
        else if (fabs(lsm9ds1_read_gyro_integration().z_axis - target_angle) < 3){
          up = false;
          state = DRIVING;
          lsm9ds1_stop_gyro_integration();
          wait = true;
        }
        else {
          // perform state-specific actions here
          display_write("TURNING", DISPLAY_LINE_0);
          char buf[16];
          snprintf(buf, 16, "%f", lsm9ds1_read_gyro_integration().z_axis);
          display_write(buf, DISPLAY_LINE_1);
          float diff = target_angle - lsm9ds1_read_gyro_integration().z_axis;
          int8_t sign = (2 * (diff > 0)) - 1;
          int16_t speed = sign * fmax(.8 * fabs(diff), 50);
          kobukiDriveDirect(-speed, speed);
          state = TURNING;
        }
        break; // each case needs to end with break!
      }
      case AVOID_TURNING: {
        // transition logic
        printf("IN AVOID_TURNING\n");
        printf("TARGET ANGLE %d\n", target_angle);
        printf("CURRENT ANGLE %f\n", lsm9ds1_read_gyro_integration().z_axis);
        if (is_button_pressed(&sensors)) {
          state = OFF;
          target_dist = target_dist_forward;
          target_angle = 90;
          distance_total = 0;
          encoder_prev = sensors.rightWheelEncoder;
          lsm9ds1_stop_gyro_integration();
          wait = true;
        } 
        else if (wait) {
            kobukiDriveDirect(0, 0);
            nrf_delay_ms(500);
            wait = false;
            lsm9ds1_start_gyro_integration();
            state = AVOID_TURNING;
        }
        else if (fabs(lsm9ds1_read_gyro_integration().z_axis - target_angle) < 3){
          state = DRIVING;
          lsm9ds1_stop_gyro_integration();
         // target_dist = target_dist_forward;
         // target_angle = 90;
         // distance_total = 0;
         // encoder_prev = sensors.rightWheelEncoder;
         // lsm9ds1_stop_gyro_integration();
          wait = true;
        }
        else {
          // perform state-specific actions here
          display_write("AVOID_TURNING", DISPLAY_LINE_0);
          char buf[16];
          snprintf(buf, 16, "%f", lsm9ds1_read_gyro_integration().z_axis);
          // display_write(buf, DISPLAY_LINE_1);
          float diff = target_angle - lsm9ds1_read_gyro_integration().z_axis;
          int8_t sign = (2 * (diff > 0)) - 1;
          int16_t speed = sign * fmax(.8 * fabs(diff), 50);
          kobukiDriveDirect(-speed, speed);
          state = AVOID_TURNING;
        }
        break; // each case needs to end with break!
      }
      // add other cases here
    }
    }}