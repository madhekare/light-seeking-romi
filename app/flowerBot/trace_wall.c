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

// Configure intial state
KobukiSensors_t sensors = {0};
char buf[16]; // Used for display_write

void trace_wall(void) {
  printf("Beginning Trace Wall ...\n");

  float frontDist, leftDist, rightDist;
  float frontDistMemory, leftDistMemory, rightDistMemory;
  frontDistMemory = leftDistMemory = rightDistMemory = 0; //setting Memory to 0;

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
					state = DRIVE_AWAY;
				} else {
					display_write("OFF: TRACE WALL", DISPLAY_LINE_0);
          display_write("", DISPLAY_LINE_1);
					kobukiDriveDirect(0, 0);
					state = OFF;
				}
				break; // each case needs to end with break!
			}
    case DRIVE_AWAY: {
      if (is_button_pressed(&sensors)) {
        state = OFF;
      } else {
        if(getDistance(&leftDist, pinTrigLeft, pinEchoLeft)) {
          leftDistMemory = update_distance_memory(leftDist, leftDistMemory);
          if (leftDistMemory > distance_from_wall) {
            state = DRIVE_TOWARDS;
          } else {
            state = DRIVE_AWAY;
          }
          snprintf(buf, 16, "%f", leftDistMemory);
          display_write(buf, DISPLAY_LINE_1);
        }
        display_write("DRIVE AWAY", DISPLAY_LINE_0);
        left_speed = 45;
        right_speed = 40;
        kobukiDriveDirect(left_speed, right_speed); // Turn slightly right
        nrf_delay_ms(200);
      }
      break;
    }
    case DRIVE_TOWARDS: {
      if (is_button_pressed(&sensors)) {
        state = OFF;
      } else {
        if(getDistance(&leftDist, pinTrigLeft, pinEchoLeft)) {
          leftDistMemory = update_distance_memory(leftDist, leftDistMemory);
          if (leftDistMemory <= distance_from_wall) {
            state = DRIVE_AWAY;
          } else {
            state = DRIVE_TOWARDS;
          }
          snprintf(buf, 16, "%f", leftDistMemory);
          display_write(buf, DISPLAY_LINE_1);
        }
        display_write("DRIVE TOWARDS", DISPLAY_LINE_0);
        left_speed = 30;
        right_speed = 35;
        kobukiDriveDirect(left_speed, right_speed);
        nrf_delay_ms(200);
      }
      break;
    }
	}
}
  return;
}
