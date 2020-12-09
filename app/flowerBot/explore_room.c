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
#define TURN_DIST 0.2


void explore_room(void) {

  uint16_t encoder_value = 0;

  

  float frontDist, leftDist, rightDist;
  // HC-SR04 Trigger and Echo Pins
  uint32_t pinTrigFront = 4;
  uint32_t pinEchoFront = 3;
  uint32_t pinTrigLeft = 5;
  uint32_t pinEchoLeft = 2;
  // uint32_t pinTrigRight = 19;
  uint32_t pinTrigRight = 13; // for some reason, can't drive when using pin 19
  // uint32_t pinEchoRight = 20;
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
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};
  uint32_t speed = 75;
  float distance = 0;
  //float degrees = 0;
  bool is_up = true;
  bool is_first = true;




  while (1) {
    printf("Looping\n");
    // read sensors from robot
    kobukiSensorPoll(&sensors);
    nrf_delay_ms(100);
    float lux = opt3004_read_result();

    //getDistance(&leftDist, pinTrigLeft, pinEchoLeft);
    //printf("left dist = %f cm\n", leftDist);
    /*if(getDistance(&rightDist, pinTrigRight, pinEchoRight)) {
      printf("right dist = %f cm\n", rightDist);
    }
    if(getDistance(&frontDist, pinTrigFront, pinEchoFront)) {
      printf("front dist = %f cm\n", frontDist);
    }*/



    switch(state) {
    case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          encoder_value = sensors.leftWheelEncoder;
          distance = 0;
          state = DRIVING;
          lsm9ds1_start_gyro_integration();
          printf("Started Driving!");
        } else {
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
          display_write("OFF", DISPLAY_LINE_0);
          display_write("", DISPLAY_LINE_1);
          printf("OFF: %f\n", distance);
          state = OFF;
        }
        break; // each case needs to end with break!
      }
    case DRIVING: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          printf("stopped driving \n");
          distance = 0;
          state = OFF;
        }
        else if (distance >= ROOMLEN) {
          distance = 0.0;
          lsm9ds1_start_gyro_integration();

          is_up = !is_up;
          is_first = true;
          state = TURNING;
        }
        else {
         printf("driving \n");
          kobukiDriveDirect(speed, speed);
          uint16_t upd_encoder  = sensors.leftWheelEncoder;
          distance += measure_distance(upd_encoder , encoder_value);
          encoder_value = upd_encoder ;
          printf("   LUX: %f\n",lux);
          display_write("LUX: ", DISPLAY_LINE_0);
          char buf [16];
          snprintf(buf, 16, "%f", lux);
          display_write(buf, DISPLAY_LINE_1);
          state = DRIVING;
        }
        break;
      }
    case SHORT_DRIVE: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          printf("stopped driving \n");
          distance = 0;
          state = OFF;
        }
        else if (distance >= TURN_DIST) {
          distance = 0.0;
          lsm9ds1_start_gyro_integration();
          is_first = false;
          state = TURNING;
        }
        else {
          printf("driving \n");
          kobukiDriveDirect(speed, speed);
          uint16_t upd_encoder  = sensors.leftWheelEncoder;
          distance += measure_distance(upd_encoder , encoder_value);
          encoder_value = upd_encoder ;
          display_write("DRIVING", DISPLAY_LINE_0);
          char buf [16];
          snprintf ( buf , 16 , "%f", distance );
          display_write ( buf , DISPLAY_LINE_1 );
          state = SHORT_DRIVE;
        }
        break;
      }
      case TURNING: {

        float angle = lsm9ds1_read_gyro_integration().z_axis;

        // transition logic
        if (is_button_pressed(&sensors)) {
          distance = 0.0;
          lsm9ds1_stop_gyro_integration();
          state = OFF;
        } else if (fabs(angle) >= 87) {
          lsm9ds1_stop_gyro_integration();
          distance = 0.0;
          encoder_value = sensors.leftWheelEncoder;
          if (is_first){
            state = SHORT_DRIVE;
          } else{
            state = DRIVING;
          }
        } else {
          if (is_up){
            kobukiDriveDirect(50, -50);
          } else{
            kobukiDriveDirect(-50, 50);
          }
          display_write("TURNING", DISPLAY_LINE_0);
          char buf [16];
          snprintf(buf, 16, "%f", angle);
          display_write(buf, DISPLAY_LINE_1);
          state = TURNING;
        }
        break;
      }
    }
  }
}
