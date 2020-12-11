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
#include "trace_wall.h"
#include "ultrasonic.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "lsm9ds1.h"


// Configure intial state
char buf[16]; // Used for display_write
char buf2[16];

void trace_wall(void) {
  KobukiSensors_t sensors = {0};
  printf("Beginning Trace Wall ...\n");
  float frontDist, leftDist, rightDist;
  float frontDistMemory, leftDistMemory, rightDistMemory;
  float difference;

  int16_t speed, left_speed, right_speed;

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
  float distance_from_wall = 20.0;
  float difference_tolerance = 4.5;
  float target_angle = 25;
  float orientation_turning_max = 35;

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
          frontDistMemory = leftDistMemory = rightDistMemory = 0; //setting Memory to 0;
          state = DRIVING;
        } else {
          // if(getDistance(&rightDist, pinTrigRight, pinEchoRight)) {
          //   snprintf(buf, 16, "%f", rightDist);
          //   display_write(buf, DISPLAY_LINE_1);
          // }
          // if (getDistance(&frontDist, pinTrigFront, pinEchoFront)) {
          //   snprintf(buf2, 16, "%f", frontDist);
          //   display_write(buf2, DISPLAY_LINE_0);
          // }
          // nrf_delay_ms(200);

          display_write("OFF: TRACE WALL", DISPLAY_LINE_0);
          display_write("", DISPLAY_LINE_1);
          kobukiDriveDirect(0, 0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }
      case DRIVING: {
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (getDistanceDifference(&frontDist, pinTrigFront, pinEchoFront, &rightDist, pinTrigRight, pinEchoRight) >= difference_tolerance) {
          if (frontDist > rightDist) {
            lsm9ds1_start_gyro_integration();
            state = ORIENT_CLOCKWISE;
          } else {
            lsm9ds1_start_gyro_integration();
            state = ORIENT_COUNTERCLOCKWISE;
          }
        } else {
          printf("frontDist: %f\n", frontDist);
          printf("rightDist: %f\n", rightDist);
          snprintf(buf, 16, "%f", fabs(frontDist-rightDist));
          display_write("DRIVING", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          left_speed = 70;
          right_speed = 70;
          kobukiDriveDirect(left_speed, right_speed);
          state = DRIVING;
        }
        break;
      }
      case ORIENT: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else {
          left_speed = 0;
          right_speed = 0;
          kobukiDriveDirect(left_speed, right_speed);
          snprintf(buf, 16, "%f", angle);
          display_write("ORIENT", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
        }
        break;
      }
      case ORIENT_CLOCKWISE: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (getDistanceDifference(&frontDist, pinTrigFront, pinEchoFront, &rightDist, pinTrigRight, pinEchoRight) < difference_tolerance) {
          lsm9ds1_stop_gyro_integration();
          state = DRIVING;
        } else if (fabs(angle) >= orientation_turning_max) {
          lsm9ds1_stop_gyro_integration();
          state = ORIENT_COUNTERCLOCKWISE;
        } else {
          left_speed = 35;
          right_speed = -35;
          kobukiDriveDirect(left_speed, right_speed);
          snprintf(buf, 16, "%f", angle);
          display_write("ORIENT_CLOCKWISE", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          state = ORIENT_CLOCKWISE;
        }
        break;
      }
      case ORIENT_COUNTERCLOCKWISE: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (getDistanceDifference(&frontDist, pinTrigFront, pinEchoFront, &rightDist, pinTrigRight, pinEchoRight) < difference_tolerance) {
          lsm9ds1_stop_gyro_integration();
          state = DRIVING;
        } else if (fabs(angle) >= orientation_turning_max) {
          lsm9ds1_stop_gyro_integration();
          state = ORIENT_CLOCKWISE;
        } else {
          left_speed = -35;
          right_speed = 35;
          kobukiDriveDirect(left_speed, right_speed);
          snprintf(buf, 16, "%f", angle);
          display_write("COUNTER_CLOCKWISE", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          state = ORIENT_COUNTERCLOCKWISE;
        }
        break;
      }
    }
  }
}
