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
#include "COMMS/PidTuningReceiver.h"

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

PidTuningReceiver pidTuningReceiver(Serial, applyPidTuningsToMotor, &motor);

void initDevice(const char* name, std::function<uint8_t()> statusGetter, std::function<void()> beginFunc) {
	beginFunc();
	uint8_t statusFlag = statusGetter();
	Serial.print(name);
	Serial.println(statusFlag ? " Success" : " Failed");
	while (!statusGetter()) {}
}

// ----------------- //
//       SETUP       //
// ----------------- //
void setup() {
	Serial.begin(115200);
	Serial.println("==== SETUP BEGUN! ====");
	initDevice("ICM20948", [](){ return status.ICM20948; }, [](){ icm20948.begin(); });
	// initDevice("BMP390", [](){ return status.BMP390; }, [](){ bmp390.begin(); });
	initDevice("RFD900", [](){ return status.RFD900; }, [](){ rfd900.begin(); });
	initDevice("Motor", [](){ return status.motorArmed; }, [](){ motor.begin(); });
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
		pidTuningReceiver.loop();
		motor.loop();
	}

	if (current - prevSLOW >= SENSOR_INTERVAL_SLOW) {
		prevSLOW = current;
		// gps.loop();
		led.loop();
	}
	// delayMicroseconds(1);
}

