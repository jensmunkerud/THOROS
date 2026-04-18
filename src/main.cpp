#include <Arduino.h>
#include <SPI.h>
#include "Datatypes.h"
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

unsigned long prevFAST = 0;
unsigned long prevSLOW = 0;

Telemetry telemetry;
Drone drone;
ICM20948 icm20948(telemetry, drone);
BMP390 bmp390(telemetry, drone);
// GPS gps(telemetry);
RFD900 rfd900(telemetry, drone);
MovementController movementController(telemetry, drone, rfd900);
LED led(telemetry, drone, movementController);
Motor motor(movementController, telemetry, drone);

PidTuningReceiver pidTuningReceiver(Serial, applyPidTuningsToMotor, &motor);

void initDevice(const char* name, std::function<bool()> statusGetter, std::function<void()> beginFunc) {
	beginFunc();
	bool statusFlag = statusGetter();
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
	initDevice("ICM20948", [](){ return drone.IMU_OK; }, [](){ icm20948.begin(); });
	// initDevice("BMP390", [](){ return drone.BMP390; }, [](){ bmp390.begin(); });
	initDevice("RFD900", [](){ return drone.RADIO_OK; }, [](){ rfd900.begin(); });
	initDevice("Motor", [](){ return drone.MOTOR_ARMED; }, [](){ motor.begin(); });
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
	}
	led.loop();
}

