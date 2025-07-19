#include "ICM20948.h"
#include "string"


ICM20948::ICM20948(Status& status) : status{status} {}

void ICM20948::begin() {
	SPI.begin();
	if (icm20948.begin(ICM20948_CS, SPI) != ICM_20948_Stat_Ok) {
		State = false;
		return;
	}
	State = true;
}

void ICM20948::loop() {
	if (icm20948.dataReady()) {
		icm20948.getAGMT();	// Refresh all sensor values
		if (true) {
			// Serial.print("Accel (mg): ");
			// Serial.print(icm20948.accX(), 2); Serial.print(", ");
			// Serial.print(icm20948.accY(), 2); Serial.print(", ");
			// Serial.print(icm20948.accZ(), 2); Serial.print(" | ");

			status.accelX = (int16_t)(icm20948.accX());
			status.accelY = (int16_t)(icm20948.accY());
			status.accelZ = (int16_t)(icm20948.accZ());
		}
		
		if (true) {
			// 	Serial.print("Gyro (dps): ");
			// 	Serial.println(icm20948.gyrX(), 2); Serial.print(", ");
			// 	Serial.print(icm20948.gyrY(), 2); Serial.print(", ");
			// 	Serial.print(icm20948.gyrZ(), 2); Serial.print(" | ");
			
			status.gyroX = (int16_t)(icm20948.gyrX());
			status.gyroY = (int16_t)(icm20948.gyrY());
			status.gyroZ = (int16_t)(icm20948.gyrZ());
		}
		
		// if (false) {
			// 	Serial.print("Mag (uT): ");
			// 	Serial.print(icm20948.magX(), 2); Serial.print(", ");
			// 	Serial.print(icm20948.magY(), 2); Serial.print(", ");
			// 	Serial.print(icm20948.magZ(), 2);
		// }

		// Serial.println();
	}
}
