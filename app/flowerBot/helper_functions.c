#include "lsm9ds1.h"
#include <math.h>

// Read accelerometer value and calculate and return tilt (along axis corresponding to climbing the hill)
float read_tilt() {
	lsm9ds1_measurement_t measurement = lsm9ds1_read_accelerometer();
	float x = measurement.x_axis;
	float y = measurement.y_axis;
	float z = measurement.z_axis;
	return 57.296 * atan2(y, sqrt(x * x + z * z));
}

// Return distance traveled between two encoder values
float measure_distance(uint16_t current_encoder, uint16_t previous_encoder) {
  //const float CONVERSION = 0.0006108;
  if (current_encoder >= previous_encoder) {
    if ((float) (previous_encoder - current_encoder) >= (float) 65535 / 2) {
      uint32_t total_ticks = previous_encoder + 65535  - current_encoder;
      return 0.0006108 * (float) total_ticks;
    }
    return 0.0006108 * (float) (current_encoder - previous_encoder);
  } else {
    if ((float) (previous_encoder - current_encoder) <= (float) 65535 / 2) {
      return 0.0006108 * (float) (current_encoder - previous_encoder);
    }
    uint32_t total_ticks = current_encoder + 65535 - previous_encoder;
    return 0.0006108 * (float) total_ticks;
  }
}

float update_distance_memory(float new_distance, float memory_distance) {
	// Memory distance is initialized to 0, so on first update set to new_distance
	// // Ultrasonic sensors can't detect distances < 4cm
	if (memory_distance < 4) {
		return new_distance;
		// If new_distance is much larger than memory distance, assume reading is faulty and ignore
	} else if (new_distance > memory_distance*1.8) {
		return memory_distance;
	} else {
		return 0.1*new_distance + 0.9*memory_distance;
	}
}
