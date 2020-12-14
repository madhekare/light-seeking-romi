#include <stdlib.h>
#include "app_error.h"
#include "app_timer.h"
#include "display.h"
#include "helper_functions.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include <math.h>
#include "ultrasonic.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "lsm9ds1.h"
#include <errno.h>
#include <stdio.h>

// Configure intial state
char buf[16]; // Used for display_write
char buf2[16];

void ultrasonic_turn_data(void) {
  printf("Beginning Ultrasonic Turn Data Collection ...\n");
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
  uint32_t pinTrigRight = 13;
  uint32_t pinEchoRight = 16;

	robot_state_t state = OFF;
  float distance_from_wall = 20.0;
  float difference_tolerance = 5;
  float target_angle = 25;
  float orientation_turning_max = 35;
  float twitch_angle = 3;

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

  bool clockwise = true;
  float currAngle = 0;

  // File writing commands
  // printf("1");
  // FILE *fp = fopen("/home/student/light-seeking-romi/app/flowerBot/ultrasonicTurnData2.txt", "w");
  // if (!fp) {
  //   perror("fopen");
  // }
  // if (fp == NULL){
  //   printf("error opening file");
  //   return;
  // }
  // printf("2");

  while (1) {
    kobukiSensorPoll(&sensors);
		switch(state) {
      case OFF: {
				if (is_button_pressed(&sensors)) {
          float startingDistLeft = getDistanceMedian(&leftDist, pinTrigLeft, pinEchoLeft, 20);
          float startingDistRight = getDistanceMedian(&rightDist, pinTrigRight, pinEchoRight, 20);
          printf("%f,%f,%f\n", currAngle, startingDistLeft, startingDistRight);
          lsm9ds1_start_gyro_integration();
          state = ORIENT_CLOCKWISE;
				} else {
					display_write("OFF: US DATA", DISPLAY_LINE_0);
					kobukiDriveDirect(0, 0);
					state = OFF;
				}
				break; // each case needs to end with break!
			}
      case PAUSE: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          // clockwise = !clockwise; // comment out if 1 direction desired
          if (clockwise) {
            lsm9ds1_start_gyro_integration();
            state = ORIENT_CLOCKWISE;
          } else {
            lsm9ds1_start_gyro_integration();
            state = ORIENT_COUNTERCLOCKWISE;
          }
        } else {
          display_write("PAUSE", DISPLAY_LINE_0);
          // snprintf(buf, 16, "%f", angle);
          // display_write(buf, DISPLAY_LINE_1);
          state = PAUSE;
        }
        break;
      }
      case ORIENT_CLOCKWISE: {
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (angle < -twitch_angle) {
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(0, 0);
          currAngle += 3;
          float leftDistMedian = getDistanceMedian(&leftDist, pinTrigLeft, pinEchoLeft, 20);
          float rightDistMedian = getDistanceMedian(&rightDist, pinTrigRight, pinEchoRight, 20);
          // fprintf(fp, "%f,%f,%f\n", currAngle, leftDistMedian, rightDistMedian);
          printf("%f,%f,%f\n", currAngle, leftDistMedian, rightDistMedian);

          state = PAUSE;
        } else {
          kobukiDriveDirect(35, -35);
          snprintf(buf, 16, "%f", angle);
          display_write("CLOCKWISE", DISPLAY_LINE_0);
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
        } else if (angle > twitch_angle) {
          lsm9ds1_stop_gyro_integration();
          kobukiDriveDirect(0, 0);
          currAngle += 3;
          float leftDistMedian = getDistanceMedian(&leftDist, pinTrigLeft, pinEchoLeft, 20);
          float rightDistMedian = getDistanceMedian(&rightDist, pinTrigRight, pinEchoRight, 20);
          printf("%f,%f,%f\n", currAngle, leftDistMedian, rightDistMedian);

          // fprintf(fp, "%f,%f,%f\n", currAngle, leftDistMedian, rightDistMedian);
          state = PAUSE;
        } else {
          kobukiDriveDirect(-35, 35);
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
