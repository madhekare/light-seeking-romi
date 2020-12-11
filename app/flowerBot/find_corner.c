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

void find_corner(void) {
  printf("Beginning Find Corner ...\n");
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
  float difference_tolerance = 3;
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
          lsm9ds1_start_gyro_integration();
          state = ORIENT;
				} else {
					display_write("OFF: FIND CORNER", DISPLAY_LINE_0);
          display_write("", DISPLAY_LINE_1);
					kobukiDriveDirect(0, 0);
					state = OFF;
				}
				break; // each case needs to end with break!
			}
    case ORIENT: {
      float angle = lsm9ds1_read_gyro_integration().z_axis;
      if (is_button_pressed(&sensors)) {
        lsm9ds1_stop_gyro_integration();
        state = OFF;
      } else if (getDistanceDifferenceKalman(&frontDist, pinTrigFront, pinEchoFront, &rightDist, pinTrigRight, pinEchoRight, 15) < difference_tolerance) {
        lsm9ds1_stop_gyro_integration();
        kobukiDriveDirect(0, 0);
        display_write("ORIENT DONE", DISPLAY_LINE_0);
        nrf_delay_ms(5000);
        state = DRIVING;
      } else {
        left_speed = 35;
        right_speed = -35;
        kobukiDriveDirect(left_speed, right_speed);
        snprintf(buf, 16, "%f", angle);
        display_write("ORIENTING", DISPLAY_LINE_0);
        display_write(buf, DISPLAY_LINE_1);
        state = ORIENT;
      }
      break;
    }
    case DRIVING: {
      if (is_button_pressed(&sensors)) {
        lsm9ds1_stop_gyro_integration();
        state = OFF;
      } else if (getDistance(&frontDist, pinTrigFront, pinEchoFront)) {
        if (frontDist < 10) {
          display_write("DRIVE DONE", DISPLAY_LINE_0);
          kobukiDriveDirect(0, 0);
          state = TURNING;
        }
        lsm9ds1_stop_gyro_integration();
        state = OFF;
      } else {
        left_speed = 45;
        right_speed = 45;
        kobukiDriveDirect(left_speed, right_speed);
        display_write("COUNTER_CW", DISPLAY_LINE_0);
        state = ORIENT_COUNTERCLOCKWISE;
      }
      break;
    }
	}
}
}
