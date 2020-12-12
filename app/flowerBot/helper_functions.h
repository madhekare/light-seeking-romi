#pragma once

// Define state machine states
typedef enum {
  OFF,
  DRIVING,
  TURNING,
  SHORT_DRIVE,
	ORIENT,
  DRIVE_TOWARDS,
  DRIVE_AWAY,
  ORIENT_CLOCKWISE,
  ORIENT_COUNTERCLOCKWISE,
  PAUSE,
  TURN_RIGHT,
  TURN_LEFT,
  BACKWARDS
} robot_state_t;

float read_tilt();
float measure_distance(uint16_t current_encoder, uint16_t previous_encoder);
float update_distance_memory(float new_distance, float memory_distance);
