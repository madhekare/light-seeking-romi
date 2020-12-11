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
  float difference_tolerance = 5;
  float target_angle = 25;
  float orientation_turning_max = 35;
  float max_theta = 30;
  float twitch_angle = 5;
  float theta = 0;
  bool max_theta_hit = false;
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
          display_write("CALC THETA", DISPLAY_LINE_0);
          frontDistMemory = leftDistMemory = rightDistMemory = 0; //setting Memory to 0;
          state = ORIENT;
				} else {
					display_write("OFF: ORIENT TEST", DISPLAY_LINE_0);
          display_write("", DISPLAY_LINE_1);
					kobukiDriveDirect(0, 0);
					state = OFF;
				}
				break; // each case needs to end with break!
			}
      case ORIENT: {
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else {
          kobukiDriveDirect(0, 0);
          theta = getThetaMedian(&frontDist, pinTrigFront, pinEchoFront, &rightDist, pinTrigRight, pinEchoRight, 80, (float) 13);
          theta = theta * 0.5;
          snprintf(buf, 16, "%f", theta);
          display_write(buf, DISPLAY_LINE_1);
          display_write("ORIENT", DISPLAY_LINE_0);
          nrf_delay_ms(2500);
          if (theta >= 0) {
            if (fabs(theta) > max_theta) {
              max_theta_hit = true;
              theta = twitch_angle;
              snprintf(buf, 16, "%f", theta);
              display_write(buf, DISPLAY_LINE_1);
              // nrf_delay_ms(2500);
            }
            lsm9ds1_start_gyro_integration();
            state = ORIENT_COUNTERCLOCKWISE;
          } else {
            if (fabs(theta) > max_theta) {
              max_theta_hit = true;
              theta = -twitch_angle;
              snprintf(buf, 16, "%f", theta);
              display_write(buf, DISPLAY_LINE_1);
              nrf_delay_ms(2500);
            }
            lsm9ds1_start_gyro_integration();
            state = ORIENT_CLOCKWISE;
          }
        }
        break;
      }
      case ORIENT_COUNTERCLOCKWISE: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (fabs(angle) > 90) {
          kobukiDriveDirect(0, 0);
          lsm9ds1_stop_gyro_integration();
          if (max_theta_hit) {
            max_theta_hit = false;
            state = ORIENT;
          } else {
            state = PAUSE;
          }
        } else {
          snprintf(buf, 16, "%f", angle);
          display_write(buf, DISPLAY_LINE_1);
          display_write("COUNTERCLOCKWISE", DISPLAY_LINE_0);
          kobukiDriveDirect(-55, 55);
          state = ORIENT_COUNTERCLOCKWISE;
        }
        break;
      }
      case ORIENT_CLOCKWISE: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (fabs(angle) > 90) {
          kobukiDriveDirect(0, 0);
          lsm9ds1_stop_gyro_integration();
          if (max_theta_hit) {
            max_theta_hit = false;
            state = ORIENT;
          } else {
            state = PAUSE;
          }
        } else {
          float angle = lsm9ds1_read_gyro_integration().z_axis;
          snprintf(buf, 16, "%f", angle);
          display_write(buf, DISPLAY_LINE_1);
          display_write("CLOCKWISE", DISPLAY_LINE_0);
          kobukiDriveDirect(55, -55);
          state = ORIENT_CLOCKWISE;
        }
        break;
      }
      case PAUSE: {
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else {
          display_write("PAUSE", DISPLAY_LINE_0);
          kobukiDriveDirect(0, 0);
          state = PAUSE;
        }
      }
    }
  }
}
