#pragma once

// Define state machine states
typedef enum {
  OFF,
  DRIVING,
  BACKWARDS,
  TURNING,
  TURN_LEFT,
  TURN_RIGHT,
  SHORT_DRIVE,
  ORIENT,
  DRIVE_TOWARDS,
  DRIVE_AWAY,
  ORIENT_CLOCKWISE,
  ORIENT_COUNTERCLOCKWISE,
  PAUSE,
  CALIBRATE
} robot_state_t;

float read_tilt();
float measure_distance(uint16_t current_encoder, uint16_t previous_encoder);
float measure_distance_cm(uint16_t current_encoder, uint16_t previous_encoder);
float update_distance_memory(float new_distance, float memory_distance);
