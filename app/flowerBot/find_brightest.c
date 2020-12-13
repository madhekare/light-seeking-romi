#include "app_error.h"
#include "app_timer.h"
#include "display.h"
#include "helper_functions.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include <math.h>
#include <stdio.h>
#include "ultrasonic.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "lsm9ds1.h"

#include "orient_test.h"


// Configure intial state
char buf[16]; // Used for display_write
char buf2[16];

void find_brightest(float frontDistGoal, float rightDistGoal) {
  printf("Beginning Find Brightest ...\n");
  KobukiSensors_t sensors = {0};
  float frontDist, leftDist, rightDist;
  float distance, goalDistance;
  bool turned_left = false;
  int orient_check = 0;
  uint16_t encoder_value = 0;

  int16_t left_speed, right_speed;

  // HC-SR04 Trigger and Echo Pins
  uint32_t pinTrigFront = 4;
  uint32_t pinEchoFront = 3;
  uint32_t pinTrigLeft = 5;
  uint32_t pinEchoLeft = 2;
  // uint32_t pinTrigRight = 19;
  uint32_t pinTrigRight = 13; // for some reason, can't drive when using pin 19
  // uint32_t pinEchoRight = 20;
  uint32_t pinEchoRight = 16; // for some reason, can't drive when using pin 20

  robot_state_t state = OFF;
  float difference_tolerance = 25;
  uint16_t upd_encoder;

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

  while (1) {
    kobukiSensorPoll(&sensors);
    switch(state) {
      case OFF: {
        if (is_button_pressed(&sensors)) {
          turned_left = false;
          state = CALIBRATE;
        } else {
          display_write("OFF: FIND BRIGHT", DISPLAY_LINE_0);
          kobukiDriveDirect(0, 0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case CALIBRATE: {
        kobukiDriveDirect(0, 0);
        display_write("CALIBRATE", DISPLAY_LINE_0);
        orient_test();
        float currRightDist = getDistanceMedian(&rightDist, pinTrigRight, pinEchoRight, 10);
        float currFrontDist = getDistanceMedian(&frontDist, pinTrigFront, pinEchoFront, 10);
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (fabs(rightDistGoal-currRightDist) > difference_tolerance) {
          goalDistance = rightDistGoal - currRightDist;
          lsm9ds1_start_gyro_integration();
          state = TURN_LEFT;
        } else if (fabs(currFrontDist-frontDistGoal) > difference_tolerance) {
          goalDistance = fabs(currFrontDist-frontDistGoal);
          if (currFrontDist-frontDistGoal >= 0) {
            distance = 0;
            encoder_value = sensors.leftWheelEncoder;
            kobukiDriveDirect(0, 0);
            state = DRIVING;
          } else {
            distance = 0;
            encoder_value = sensors.leftWheelEncoder;
            kobukiDriveDirect(0, 0);
            state = BACKWARDS;
          }
        } else {
          display_write("COMPLETE!", DISPLAY_LINE_0);
          state = OFF;
        }
        break;
      }

      case TURN_LEFT: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (fabs(angle) >= 85) {
          turned_left = true;
          lsm9ds1_stop_gyro_integration();
          if (goalDistance >=0) {
            distance = 0;
            encoder_value = sensors.leftWheelEncoder;
            kobukiDriveDirect(0, 0);
            state = DRIVING;
          } else {
            goalDistance = fabs(goalDistance);
            distance = 0;
            encoder_value = sensors.leftWheelEncoder;
            kobukiDriveDirect(0, 0);
            state = BACKWARDS;
          }
        } else {
          snprintf(buf, 16, "%f", angle);
          display_write("TURN_LEFT", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          kobukiDriveDirect(-50,50);
          state = TURN_LEFT;
        }
        break;
      }

      case DRIVING: {
        if (orient_check%10 == 0) {
          orient_test();
        }
        orient_check++;
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (distance >= goalDistance) {
          kobukiDriveDirect(0,0);
          if (turned_left) {
            lsm9ds1_start_gyro_integration();
            state = TURN_RIGHT;
          } else {
            state = CALIBRATE;
          }
        } else {
          upd_encoder  = sensors.leftWheelEncoder;
          distance += measure_distance(upd_encoder, encoder_value);
          encoder_value = upd_encoder;
          snprintf(buf, 16, "%f", distance);
          display_write("DRIVING", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          kobukiDriveDirect(55, 50);
        }
        break;
      }

      case BACKWARDS: {
        if (orient_check%10 == 0) {
          orient_test();
        }
        orient_check++;
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (fabs(distance) >= goalDistance) {
          if (turned_left) {
            lsm9ds1_start_gyro_integration();
            state = TURN_LEFT;
          } else {
            state = CALIBRATE;
          }
        } else {
          upd_encoder  = sensors.leftWheelEncoder;
          distance += measure_distance(upd_encoder, encoder_value);
          encoder_value = upd_encoder;
          printf("back distance: %f\n", distance);
          snprintf(buf, 16, "%f", distance);
          display_write("BACKWARDS", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          kobukiDriveDirect(-50, -50);
        }
        break;
      }

      case TURN_RIGHT: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (fabs(angle) >= 85) {
          lsm9ds1_stop_gyro_integration();
          turned_left = false;
          state = CALIBRATE;
        } else {
          left_speed = 50;
          right_speed = -50;
          kobukiDriveDirect(left_speed, right_speed);
          snprintf(buf, 16, "%f", angle);
          display_write("TURN_RIGHT", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          state = TURN_RIGHT;
        }
        break;
      }
    }
  }
}
