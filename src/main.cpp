#include <Arduino.h>
#include <SPI.h>
#include "Status.h"
#include "SENSORS/ICM20948.h"
#include "SENSORS/BMP390.h"
#include "SENSORS/GPS.h"
#include "MovementController.h"
#include "COMMS/RFD900.h"
#include "LED.h"
#include "Motor.h"

// PARAMETERS
constexpr int speed {100};	// Vehicle speed
constexpr int P {1};		// Proportional parameter
constexpr int I {1};		// Integral parameter
constexpr int D {1};		// Derivative parameter
constexpr unsigned long SENSOR_INTERVAL_FAST = 1000/200;
constexpr unsigned long SENSOR_INTERVAL_SLOW = 1000/1;
constexpr unsigned long interval2 = 1000/10;

unsigned long prevFAST = 0;
unsigned long prevSLOW = 0;

Status status;
// ICM20948 icm20948(status);
// BMP390 bmp390(status);
// GPS gps(status);
MovementController movementController(status);
RFD900 rfd900(status, movementController);
// LED led(status);
Motor motor(movementController, status);


// ----------------- //
//       SETUP       //
// ----------------- //
void setup() {
	// icm20948.begin();
	// bmp390.begin();
	motor.begin();
	rfd900.begin();
}


// ---------------- //
//       LOOP       //
// ---------------- //
void loop() {
	unsigned long current = millis();
	
	rfd900.loop();
	if (current - prevFAST >= SENSOR_INTERVAL_FAST) {
		prevFAST = current;
		// icm20948.loop();
		// bmp390.loop();
		rfd900.sendStatus();
		movementController.update();
		motor.loop();
	}
	
	if (current - prevSLOW >= SENSOR_INTERVAL_SLOW) {
		prevSLOW = current;
		// gps.loop();
		// led.loop();
	}
}

