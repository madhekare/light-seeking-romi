#include "display.h"
#include "helper_functions.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include <math.h>
#include <stdio.h>
#include "trace_wall.h"

// Configure intiial state
KobukiSensors_t sensors = {0};
char buf[16]; // Used for display_write

void trace_wall(void) {
  kobukiSensorPoll(&sensors);
  // char buf[16];
  // snprintf(buf, 16, "%f", tilt);
  // display_write("TILT", DISPLAY_LINE_0);
  // display_write(buf, DISPLAY_LINE_1);
	robot_state_t state = OFF;
	while (1) {
		switch(state) {
			case OFF: {
				// transition logic
				if (is_button_pressed(&sensors)) {
					state = ORIENT;
				} else {
					// perform state-specific actions here
					display_write("OFF", DISPLAY_LINE_0);
					// kobukiDriveDirect(0, 0);
					state = OFF;
				}
				break; // each case needs to end with break!
			}
		case ORIENT: {
			float tilt = read_tilt();
			if (is_button_pressed(&sensors)) {
				state = OFF;
			} else {
				snprintf(buf, 16, "%f", tilt);
        display_write("ORIENT", DISPLAY_LINE_0);
				display_write(buf, DISPLAY_LINE_1);
				int16_t speed = 30;
				kobukiDriveDirect(speed, -speed); // Turning counterclockwise
				state = ORIENT;
				}
			break;
		}
		return;
	}
}
}
