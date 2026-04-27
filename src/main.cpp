#include <Arduino.h>
#include <SPI.h>
#include "MISC/Datatypes.h"
#include "SENSORS/ICM20948.h"
#include "SENSORS/BMP390.h"
#include "SENSORS/GPS.h"
#include "MISC/MovementController.h"
#include "COMMS/RFD900.h"
#include "MISC/Logger.h"
#include "MISC/LED.h"
#include "MISC/Motor.h"
#include "COMMS/PidTuningReceiver.h"

// PARAMETERS
constexpr unsigned long SENSOR_INTERVAL_FAST = 1000/1000;
constexpr unsigned long SENSOR_INTERVAL_SLOW = 1000/0.1;

unsigned long prevFAST = 0;
unsigned long prevSLOW = 0;

Drone drone;
Telemetry telemetry;
ICM20948 icm20948(telemetry, drone);
BMP390 bmp390(telemetry, drone);
// GPS gps(telemetry);
RFD900 rfd900(telemetry, drone);
MovementController movementController(telemetry, drone, rfd900);
LED led(telemetry, drone);
Motor motor(movementController, drone);
Logger logger(drone, telemetry, motor);

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
	rfd900.setPidApplyCallback(applyPidTuningsToMotor, &motor);
	initDevice("ICM20948", [](){ DroneLockGuard droneLock(drone); return drone.IMU_OK; }, [](){ icm20948.begin(); });
	// initDevice("BMP390", [](){ return drone.BMP390; }, [](){ bmp390.begin(); });
	initDevice("RFD900", [](){ DroneLockGuard droneLock(drone); return drone.RADIO_OK; }, [](){ rfd900.begin(); });
	initDevice("Motor", [](){ DroneLockGuard droneLock(drone); return drone.MOTOR_OK; }, [](){ motor.begin(); });
	initDevice("Logger", [](){ DroneLockGuard droneLock(drone); return drone.LOGGER_OK; }, [](){ logger.begin(); });
	Serial.println("==== SETUP COMPLETE ====");
}
bool began = false;
// ---------------- //
//       LOOP       //
// ---------------- //
void loop() {
	unsigned long current = millis();
	
	if (!began) {
		began = !began;
		logger.startLog();
	}

	if (current - prevFAST >= SENSOR_INTERVAL_FAST) {
		prevFAST = current;
		icm20948.loop();
		// bmp390.loop(); // This thing is SUPER SLOW
		movementController.update();
		pidTuningReceiver.loop();
		motor.loop();
		logger.loop();
	}



	if (current - prevSLOW >= SENSOR_INTERVAL_SLOW) {
		prevSLOW = current;
		// gps.loop();
		logger.stopLog();
	}
	led.loop();
}

