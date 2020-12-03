#include "controller.h"
#include <math.h>
#include <stdio.h>
#include "kobukiSensorTypes.h"
#include "display.h"

// Configure intiial state
KobukiSensors_t sensors = {0};
char buf[16]; // Used for display_write

// Read accelerometer value and calculate and return tilt (along axis corresponding to climbing the hill)
static float read_tilt() {
	lsm9ds1_measurement_t measurement = lsm9ds1_read_accelerometer();
	float x = measurement.x_axis;
	float y = measurement.y_axis;
	float z = measurement.z_axis;
	return 57.296 * atan2(y, sqrt(x * x + z * z));
}

robot_state_t controller(robot_state_t state) {
  kobukiSensorPoll(&sensors);
  // char buf[16];
  // snprintf(buf, 16, "%f", tilt);
  // display_write("TILT", DISPLAY_LINE_0);
  // display_write(buf, DISPLAY_LINE_1);
    switch(state) {
      case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = ORIENT;
          up = true;
        } else {
          // perform state-specific actions here
          // display_write("OFF", DISPLAY_LINE_0);
          kobukiDriveDirect(0, 0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }
    }
    case ORIENT: {
      float tilt = read_tilt();
      if (is_button_presssed(&sensors)) {
        state = OFF;
      } else {
        snprintf(buf, 16, "%f", read_tilt());
        display_write(buf, DISPLAY_LINE1);
        int16_t speed = 30;
        kobukiDriveDirect(speed, -speed); // Turning counterclockwise
        state = ORIENT;
      }
    }
    return state;
  }
