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


// Configure intial state
char buf[16]; // Used for display_write
char buf2[16];

void orient_test(void) {
  printf("Beginning Orient Test ...\n");
  KobukiSensors_t sensors = {0};
  float frontDist, leftDist, rightDist;
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

	robot_state_t state = ORIENT;
  float difference_tolerance = 5;
  float max_theta = 30;
  float twitch_angle = 3;

  // Set up timer
  // app_timer_init();
  // start_timer_rev1();

  // Set up HC-SR04 pins
  nrf_gpio_pin_dir_set(pinTrigFront, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEchoFront, NRF_GPIO_PIN_DIR_INPUT);
  nrf_gpio_pin_dir_set(pinTrigLeft, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEchoLeft, NRF_GPIO_PIN_DIR_INPUT);
  nrf_gpio_pin_dir_set(pinTrigRight, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEchoRight, NRF_GPIO_PIN_DIR_INPUT);
  int num_button_presses = 0;

  while (1) {
    kobukiSensorPoll(&sensors);
		switch(state) {
      case OFF: {
        printf("Off");
				if (is_button_pressed(&sensors)) {
          num_button_presses++;
          state = ORIENT;
				} else {
          snprintf(buf2, 16, "%d", num_button_presses);
					display_write(buf2, DISPLAY_LINE_0);
          display_write("", DISPLAY_LINE_1);
					kobukiDriveDirect(0, 0);
					state = OFF;
				}
        return;
				break; // each case needs to end with break!
			}
      case ORIENT: {
        lsm9ds1_stop_gyro_integration();
        printf("orient");
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else {
          printf("orient1");
          float leftMedian = getDistanceMedian(&leftDist, pinTrigLeft, pinEchoLeft, 10);
          printf("orient2");
          float rightMedian = getDistanceMedian(&rightDist, pinTrigRight, pinEchoRight, 10);
          printf("orient3");
          printf("difference: %f\n", leftMedian - rightMedian);
          // snprintf(buf, 16, "%f", leftMedian - rightMedian);
          // display_write(buf, DISPLAY_LINE_1);
          // display_write("ORIENT", DISPLAY_LINE_0);
          if (leftMedian - rightMedian > difference_tolerance) {
            lsm9ds1_start_gyro_integration();
            state = ORIENT_CLOCKWISE;
          } else if (leftMedian - rightMedian < -difference_tolerance) {
            lsm9ds1_start_gyro_integration();
            state = ORIENT_COUNTERCLOCKWISE;
          } else {
            state = OFF;
          }
        }
        break;
      }
      case ORIENT_COUNTERCLOCKWISE: {
        printf("counterclockwise");
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (angle > twitch_angle) {
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(0, 0);
          state = ORIENT;
        } else {
          snprintf(buf, 16, "%f", angle);
          display_write(buf, DISPLAY_LINE_1);
          display_write("COUNTERCLOCKWISE", DISPLAY_LINE_0);
          kobukiDriveDirect(-85, 85);
          state = ORIENT_COUNTERCLOCKWISE;
        }
        break;
      }
      case ORIENT_CLOCKWISE: {
        printf("clockwise");
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (angle < -twitch_angle) {
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(0, 0);
          state = ORIENT;
        } else {
          snprintf(buf, 16, "%f", angle);
          display_write(buf, DISPLAY_LINE_1);
          display_write("CLOCKWISE", DISPLAY_LINE_0);
          kobukiDriveDirect(85, -85);
          state = ORIENT_CLOCKWISE;
        }
        break;
      }
    }
  }
}
