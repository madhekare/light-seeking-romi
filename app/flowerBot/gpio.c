#include "gpio.h"

typedef struct {
  uint32_t RESERVED0;
  uint32_t OUT;
  uint32_t OUTSET;
  uint32_t OUTCLR;
  uint32_t IN;
  uint32_t DIR;
  uint32_t DIRSET;
  uint32_t DIRCLR;
  uint32_t LATCH;
  uint32_t DETECTMODE;
  uint32_t RESERVED1[118];
  uint32_t PIN_CNF[32];
} gpio_registers;

gpio_registers* registers = 0x50000500;

// Inputs:
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir) {
  if (dir==1) {
    registers->PIN_CNF[gpio_num] |= 1;
  } else {
    registers->PIN_CNF[gpio_num] &= ~3;
  }
}

// Set gpio_num high
// Inputs:
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num) {
  registers->OUT &= ~(1<<gpio_num);
}

// Set gpio_num low
// Inputs:
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num) {
  registers->OUT |= 1<<gpio_num;
}

// Inputs:
//  gpio_num - gpio number 0-31
bool gpio_read(uint8_t gpio_num) {
    // should return pin state
    return (registers->IN >> gpio_num) & 1;
}
