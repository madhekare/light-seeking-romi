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
#include "app_util_platform.h"
#include "nrf_gpiote.h"
#include "trace_wall.h"
#include "ultrasonic.h"
#include "opt3004.h"
#include "helper_functions.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "mpu9250.h"

#include "lsm9ds1.h"


#define TURN_DIST 0.1
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))

//Implemented PID control for Kobuki to drive straight
void drive_straight(void) {


  printf("HELLOO");
  //float frontDist, leftDist, rightDist;

  // HC-SR04 Trigger and Echo Pins
  /*uint32_t pinTrigFront = 4;
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
  nrf_gpio_pin_dir_set(pinEchoRight, NRF_GPIO_PIN_DIR_INPUT);*/


  // configure initial state
  uint16_t encoder_value = 0;
  uint16_t right_encoder_value = 0;
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};
  uint32_t speed = 50;
  float distance = 0;
  float right_distance = 0;
  float room_len = 0;
  bool is_up = true;
  bool is_first = true;
  float kp = 0.075;
  float kd = 0.0375;
  float ki = 0.0125;
  uint16_t left_speed = 65;
  uint16_t right_speed = 65;
  float e_left_prev = 0;
  float e_right_prev = 0;
  float e_sum_left = 0;
  float e_sum_right = 0;
  //getDistance(&frontDist, pinTrigFront, pinEchoFront);



  while (1) {
    printf("Looping\n");
    kobukiSensorPoll(&sensors);
    nrf_delay_ms(1);
    float lux = opt3004_read_result();

    switch(state) {
      case OFF: {
        kobukiDriveDirect(0,0);
        printf("Off\n");
        if (is_button_pressed(&sensors)) {
          state = DRIVING;
          encoder_value = sensors.leftWheelEncoder;
          printf("Starting left encoder value: %f\n",encoder_value);
          right_encoder_value = sensors.rightWheelEncoder;
          printf("Starting right encoder value: %f\n", right_encoder_value);
          distance = 0;
          right_distance = 0;
        } else {
          kobukiDriveDirect(0,0);
          state = OFF;
          distance = 0;
          display_write("OFF", DISPLAY_LINE_0);
          display_write("", DISPLAY_LINE_1);
        }
        break;
      }

      case DRIVING: {
        printf("Driving\n");
        if (is_button_pressed(&sensors)) {
          state = OFF;
          encoder_value = 0;
          right_encoder_value = 0;
          distance = 0;
        } else {
          uint16_t upd_encoder  = sensors.leftWheelEncoder;
          distance += measure_distance(upd_encoder , encoder_value);
          //printf("leftEncoder: %d\n", encoder_value);
          int e_left = 15 - (upd_encoder-encoder_value);
          encoder_value = upd_encoder;
          //printf("leftEncoderUpd: %d\n", encoder_value);
          //printf("leftdistance %d\n", distance);

          uint16_t right_upd_encoder  = sensors.rightWheelEncoder;
          right_distance += measure_distance(right_upd_encoder , right_encoder_value);
          //printf("rightEncoder: %d\n", right_encoder_value);
          int e_right = 15 - (right_upd_encoder-right_encoder_value);
          right_encoder_value = right_upd_encoder;
          //printf("rightEncoderUpd: %d\n", right_encoder_value);
          //printf("rightDist %f\n", right_distance);
          
          left_speed += e_left*kp + e_left_prev*kd + e_sum_left*ki;
          left_speed = MAX(MIN(60,left_speed),0);
          right_speed += e_right*kp + e_right_prev*kd + e_sum_right*ki;
          right_speed = MAX(MIN(60,right_speed),0);
          printf("e_left %d\n",e_left);
          printf("e_right %d\n", e_right);
          printf("left speed: %d\n",left_speed);
          printf("right speed %d\n", right_speed);
          //uint16_t speed_adj_right = 
          e_left_prev = e_left;
          e_right_prev = e_right;
          e_sum_left += e_left;
          e_sum_right += e_right;
          kobukiDriveDirect(left_speed,right_speed);
          state = DRIVING;
        }
        break;
      }

    }
  }
}