#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "app_error.h"
#include "app_timer.h"
#include "app_util_platform.h"
#include "buckler.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "nrf_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_serial.h"
#include "ultrasonic.h"
// #include "kobukiActuator.h"
// #include "nrf_twi_mngr.h"
// #include "max44009.h"

// HC-SR04 Trigger and Echo Pins
uint32_t pinTrigFront = 4;
uint32_t pinEchoFront = 3;
uint32_t pinTrigLeft = 5;
uint32_t pinEchoLeft = 2;
uint32_t pinTrigRight = 19;
uint32_t pinEchoRight = 20;

float frontDist, leftDist, rightDist;

void ultrasonic_setup() {
  app_timer_init();
  start_timer_rev1();

  // Set up HC-SR04 pins
  nrf_gpio_pin_dir_set(pinTrigFront, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEchoFront, NRF_GPIO_PIN_DIR_INPUT);
  nrf_gpio_pin_dir_set(pinTrigLeft, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEchoLeft, NRF_GPIO_PIN_DIR_INPUT);
  nrf_gpio_pin_dir_set(pinTrigRight, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(pinEchoRight, NRF_GPIO_PIN_DIR_INPUT);

}

int main(void) {
  ultrasonic_setup();

  while (1) {
    printf("Looping\n");
    // Getting front distance
    if(getDistance(&frontDist, pinTrigFront, pinEchoFront)) {
      printf("front dist = %f cm\n", frontDist);
    }

    // Getting left distance
    if(getDistance(&leftDist, pinTrigLeft, pinEchoLeft)) {
      printf("left dist = %f cm\n", leftDist);
    }

    // Getting right distance
    if(getDistance(&rightDist, pinTrigRight, pinEchoRight)) {
      printf("right dist = %f cm\n", rightDist);
    }
    nrf_delay_ms(1000);
    printf("\n");
  }
}
