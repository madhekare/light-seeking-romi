// Wrapper for ultrasonic sensor

#pragma once

// get distance measurement from HC-SR04:
// Send a 10us HIGH pulse on the Trigger pin.
// The sensor sends out a “sonic burst” of 8 cycles.
// Listen to the Echo pin, and the duration of the next HIGH
// signal will give you the time taken by the sound to go back
// and forth from sensor to target.
// returns true only if a valid distance is obtained
bool getDistance(float* dist, int pinTrig, int pinEcho);
float getDistanceKalman(float* dist, int pinTrig, int pinEcho, int iters);
float getDistanceMedian(float* dist, int pinTrig, int pinEcho, int iters);
float getMedian(float distances[], int num_elems);
float getThetaMedian(float* frontDist, int pinTrig1, int pinEcho1, float* backDist, int pinTrig2, int pinEcho2, int iters, float distBetweenSensors);
int compareFloat(const void * a, const void * b);
float calc_theta(float frontDist, float backDist, float distBetweenSensors);
float getDistanceDifference(float* dist1, int pinTrig1, int pinEcho1, float* dist2, int pinTrig2, int pinEcho2);
float getDistanceDifferenceKalman(float* dist1, int pinTrig1, int pinEcho1, float* dist2, int pinTrig2, int pinEcho2, int iters);
void start_timer_rev1(void);
void TIMER1_IRQHandler(void);
void timer_ultrasonic_custom_event_handler(void);
