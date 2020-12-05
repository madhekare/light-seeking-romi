#pragma once

// Define state machine states
typedef enum {
  OFF,
  DRIVING,
  TURNING,
  SHORT_DRIVE,
	ORIENT
} robot_state_t;

float read_tilt();
static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder);
