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
constexpr unsigned long SENSOR_INTERVAL_FAST = 1000/1000;
constexpr unsigned long SENSOR_INTERVAL_SLOW = 1000/1;
constexpr unsigned long interval2 = 1000/10;

unsigned long prevFAST = 0;
unsigned long prevSLOW = 0;

Status status;
ICM20948 icm20948(status);
BMP390 bmp390(status);
GPS gps(status);
RFD900 rfd900(status);
MovementController movementController(status, rfd900);
LED led(status);
Motor motor(movementController, status);


// ----------------- //
//       SETUP       //
// ----------------- //
void setup() {
	icm20948.begin();
	bmp390.begin();
	rfd900.begin();
	motor.begin();
}


// ---------------- //
//       LOOP       //
// ---------------- //
void loop() {
	unsigned long current = millis();
	
	if (current - prevFAST >= SENSOR_INTERVAL_FAST) {
		// Serial.print(1000/(current-prevFAST));
		// Serial.println("Hz");
		prevFAST = current;
		icm20948.loop();
		// bmp390.loop(); // This thing is SUPER SLOW
		movementController.update();
		motor.loop();
	}

	if (current - prevSLOW >= SENSOR_INTERVAL_SLOW) {
		prevSLOW = current;
		gps.loop();
		led.loop();
	}
	// delayMicroseconds(1);
}

