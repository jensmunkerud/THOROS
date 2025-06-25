#include <Wire.h>
#include <SPI.h>
#include <ICM_20948.h>	// Correct header for SparkFun library

ICM_20948_SPI myIMU;

#define CS_PIN 10	// Chip Select for SPI

// === TOGGLE SENSORS ON/OFF ===
bool useAccel	= false;
bool useGyro	= false;
bool useMag		= true;

void setup() {
	Serial.begin(115200);
	while (!Serial);

	Serial.println("ICM-20948 Selective Sensor Output Demo");

	SPI.begin();
	if (myIMU.begin(CS_PIN, SPI) != ICM_20948_Stat_Ok) {
		Serial.println("IMU initialization failed");
		while (1);
	}

	Serial.println("IMU initialized successfully!");
}

void loop() {
	if (myIMU.dataReady()) {
		myIMU.getAGMT();	// Refresh all sensor values
		myIMU.
		if (useAccel) {
			Serial.print("Accel (mg): ");
			Serial.print(myIMU.accX(), 2); Serial.print(", ");
			Serial.print(myIMU.accY(), 2); Serial.print(", ");
			Serial.print(myIMU.accZ(), 2); Serial.print(" | ");
		}

		if (useGyro) {
			Serial.print("Gyro (dps): ");
			Serial.println(myIMU.gyrX(), 2); Serial.print(", ");
			Serial.print(myIMU.gyrY(), 2); Serial.print(", ");
			Serial.print(myIMU.gyrZ(), 2); Serial.print(" | ");
		}

		if (useMag) {
			Serial.print("Mag (uT): ");
			Serial.print(myIMU.magX(), 2); Serial.print(", ");
			Serial.print(myIMU.magY(), 2); Serial.print(", ");
			Serial.print(myIMU.magZ(), 2);
		}

		Serial.println();
	} else {
		delay(10);	// Give time between checks
	}
}
