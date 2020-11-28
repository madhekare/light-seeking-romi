// Wrapper for ultrasonic sensor

#pragma once
#include "buckler.h"
#include <math.h>
#include "nrf.h"
#include "nrfx_gpiote.h"
#include <stdio.h>

// get distance measurement from HC-SR04:
// Send a 10us HIGH pulse on the Trigger pin.
// The sensor sends out a “sonic burst” of 8 cycles.
// Listen to the Echo pin, and the duration of the next HIGH
// signal will give you the time taken by the sound to go back
// and forth from sensor to target.
// returns true only if a valid distance is obtained
bool getDistance(float* dist, int pinTrig, int pinEcho);
void start_timer(void); // Setup and start Timer1
void TIMER1_IRQHandler(void);
