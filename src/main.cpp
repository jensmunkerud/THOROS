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
// GPS gps(status);
RFD900 rfd900(status);
MovementController movementController(status, rfd900);
LED led(status, movementController);
Motor motor(movementController, status);

void initDevice(const char* name, uint8_t statusFlag, std::function<void()> beginFunc) {
	beginFunc();
	Serial.print(name);
	Serial.println(statusFlag ? " Success" : " Failed");
	while (!statusFlag) {}
}

// ----------------- //
//       SETUP       //
// ----------------- //
void setup() {
	Serial.begin(115200);
	Serial.println("==== SETUP BEGUN! ====");
	initDevice("ICM20948", status.ICM20948, [](){ icm20948.begin(); });
	// initDevice("BMP390", status.BMP390, [](){ bmp390.begin(); });
	initDevice("RFD900", status.RFD900, [](){ rfd900.begin(); });
	initDevice("Motor", status.motorArmed, [](){ motor.begin(); });
	Serial.println("==== SETUP COMPLETE ====");
}

// ---------------- //
//       LOOP       //
// ---------------- //
void loop() {
	unsigned long current = millis();
	
	if (current - prevFAST >= SENSOR_INTERVAL_FAST) {
		prevFAST = current;
		icm20948.loop();
		// Serial.print(1000/(current-prevFAST));
		// Serial.println("Hz");
		// bmp390.loop(); // This thing is SUPER SLOW
		movementController.update();
		motor.loop();
		Serial.println("we looped");
	}

	if (current - prevSLOW >= SENSOR_INTERVAL_SLOW) {
		prevSLOW = current;
		// gps.loop();
		led.loop();
	}
	// delayMicroseconds(1);
}

