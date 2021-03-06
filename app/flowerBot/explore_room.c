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



#define ROOMLEN 1
#define TURN_DIST 0.1


void explore_room(void) {



  float frontDist, leftDist, rightDist;

  // HC-SR04 Trigger and Echo Pins
  uint32_t pinTrigFront = 4;
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
  nrf_gpio_pin_dir_set(pinEchoRight, NRF_GPIO_PIN_DIR_INPUT);


  // configure initial state
  uint16_t encoder_value = 0;
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};
  uint32_t speed = 50;
  float distance = 0;
  float room_len = 0;
  bool is_up = true;
  bool is_first = true;
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
          frontDist = 50;
          state = DRIVING;
          encoder_value = sensors.leftWheelEncoder;
          distance = 0;
        } else {
          getDistance(&frontDist, pinTrigFront, pinEchoFront);
          kobukiDriveDirect(0,0);
          printf("%f\n", frontDist);
          /*getDistanceMedian(&rightDist, pinTrigFront, pinEchoFront,1);
          printf("%f\n", rightDist);
          getDistanceMedian(&leftDist, pinTrigFront, pinEchoFront,1);
          printf("%f\n", leftDist);*/
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
          distance = 0;
        } else if (frontDist <= 15){
          printf("room_len: %f\n", room_len);
          room_len = distance;
          printf("room_len: %f\n", room_len);
          distance = 0;
          frontDist = 50;
          kobukiDriveDirect(-50,-50);
          state = BACKWARDS;
        } else {
          getDistance(&frontDist, pinTrigFront, pinEchoFront);
          printf("%f\n", frontDist);
          uint16_t upd_encoder  = sensors.leftWheelEncoder;
          distance += measure_distance(upd_encoder , encoder_value);
          encoder_value = upd_encoder;
          char buf [16];
          snprintf(buf, 16, "%f", distance);
          display_write("DRIVING", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          kobukiDriveDirect(50,50);
          state = DRIVING;
        }
        break;
      }

      case TURN_RIGHT: {
        printf("Turn Right\n");
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          encoder_value = 0;
          distance = 0;
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (fabs(angle) >= 87) {
          distance = 0.0;
          lsm9ds1_stop_gyro_integration();
          encoder_value = sensors.leftWheelEncoder;
          state = DRIVING;
        } else {
          char buf [16];
          snprintf(buf, 16, "%f", angle);
          display_write("TURN_RIGHT", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          kobukiDriveDirect(50,-50);
        }
        break;
      }

      case TURN_LEFT: {
        printf("Turn Left\n");
        float angle = lsm9ds1_read_gyro_integration().z_axis;
        if (is_button_pressed(&sensors)) {
          state = OFF;
          encoder_value = 0;
          distance = 0;
          lsm9ds1_stop_gyro_integration();
        } else if (fabs(angle) >= 87) {
          distance = 0.0;
          lsm9ds1_stop_gyro_integration();
          encoder_value = sensors.leftWheelEncoder;
          frontDist = 50;
          state = SHORT_DRIVE;
        } else {
          char buf [16];
          snprintf(buf, 16, "%f", angle);
          display_write("TURN_LEFT", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          kobukiDriveDirect(-50,50);
          state = TURN_LEFT;
        }
        break;
      }

      case SHORT_DRIVE: {
        printf("short_drive\n");
        if (is_button_pressed(&sensors)) {
          distance = 0;
          encoder_value = 0;
          state = OFF;
        } else if (distance >= TURN_DIST) {
          distance = 0.0;
          lsm9ds1_start_gyro_integration();
          kobukiDriveDirect(50,-50);
          state = TURN_RIGHT;
        } else {
          uint16_t upd_encoder  = sensors.leftWheelEncoder;
          distance += measure_distance(upd_encoder , encoder_value);
          encoder_value = upd_encoder;
          char buf [16];
          snprintf(buf, 16, "%f", distance);
          display_write("SHORT_DRIVE", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          kobukiDriveDirect(50, 50);
        }
        break;
      }


      case BACKWARDS: {
        printf("backwards\n");

        if (is_button_pressed(&sensors)) {
          distance = 0;
          encoder_value = 0;
          state = OFF;
        }else if (fabs(distance) >= room_len) {
          distance = 0.0;
          lsm9ds1_start_gyro_integration();
          state = TURN_LEFT;
        } else {
          uint16_t upd_encoder  = sensors.leftWheelEncoder;
          distance += measure_distance(upd_encoder , encoder_value);
          encoder_value = upd_encoder;
          printf("back distance: %f\n", distance);
          char buf [16];
          snprintf(buf, 16, "%f", distance);
          display_write("BACKWARDS", DISPLAY_LINE_0);
          display_write(buf, DISPLAY_LINE_1);
          kobukiDriveDirect(-50, -50);
        }
        break;
      }

    }
  }
}
