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
