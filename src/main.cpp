#include <Arduino.h>
#include <SPI.h>
// #include <WiFi.h>
// #include <ArduinoOTA.h>
#include "MISC/Datatypes.h"
#include "SENSORS/ICM20948.h"
#include "SENSORS/BMP390.h"
#include "SENSORS/GPS.h"
#include "MISC/MovementController.h"
#include "COMMS/RFD900.h"
#include "MISC/Logger.h"
#include "MISC/LED.h"
#include "MISC/Motor.h"
#include "../secret.h"

// PARAMETERS
constexpr unsigned long SENSOR_INTERVAL_FAST = 1000/1000;
constexpr unsigned long SENSOR_INTERVAL_SLOW = 1000/1;

unsigned long prevFAST = 0;
unsigned long prevSLOW = 0;

Drone drone;
Telemetry telemetry;
ICM20948 icm20948(telemetry, drone);
BMP390 bmp390(telemetry, drone);
// GPS gps(telemetry);
LED led(telemetry, drone);
RFD900 rfd900(telemetry, drone);
MovementController movementController(telemetry, drone, rfd900);
Logger logger(drone, telemetry);
Motor motor(movementController, drone);

void initDevice(const char* name, std::function<bool()> statusGetter, std::function<void()> beginFunc) {
	beginFunc();
	bool statusFlag = statusGetter();
	Serial.print(name);
	Serial.println(statusFlag ? " Success" : " waiting...");
	while (!statusGetter()) {}
	Serial.print(name);
	Serial.println(" Success");
}

// ----------------- //
//       SETUP       //
// ----------------- //
void setup() {
	Serial.begin(115200);
	Serial.println("==== SETUP BEGUN! ====");
	// WiFi.begin(ssid, password);
	// while (WiFi.status() != WL_CONNECTED) {delay(500); Serial.println("connecting...");}
	// Serial.println("Connected! IP: " + WiFi.localIP().toString());
	// ArduinoOTA.onStart([]() { Serial.println("OTA Start"); });
	// ArduinoOTA.onEnd([]()   { Serial.println("OTA End");   });
	// ArduinoOTA.onError([](ota_error_t e) {
		// Serial.printf("OTA Error[%u]\n", e);
	// });
	// ArduinoOTA.begin();

	initDevice("ICM20948", [](){ DroneLockGuard droneLock(drone); return drone.IMU_OK; }, [](){ icm20948.begin(); });
	// initDevice("BMP390", [](){ return drone.BMP390; }, [](){ bmp390.begin(); });
	initDevice("RFD900", [](){ DroneLockGuard droneLock(drone); return drone.RADIO_OK; }, [](){ rfd900.begin(); });
	initDevice("Motor", [](){ DroneLockGuard droneLock(drone); return drone.MOTOR_OK; }, [](){ motor.begin(); });
	initDevice("Logger", [](){ DroneLockGuard droneLock(drone); return drone.LOGGER_OK; }, [](){ logger.begin(); });
	Serial.println("==== SETUP COMPLETE ====");
}

// ---------------- //
//       LOOP       //
// ---------------- //
void loop() {
	// ArduinoOTA.handle();
	unsigned long current = millis();

	if (current - prevFAST >= SENSOR_INTERVAL_FAST) {
		prevFAST = current;
		icm20948.loop();
		// bmp390.loop(); // This thing is SUPER SLOW
		movementController.update();
		motor.loop();
	}



	if (current - prevSLOW >= SENSOR_INTERVAL_SLOW) {
		prevSLOW = current;
		// gps.loop();
	}
	led.loop();
}

