#include "ICM20948.h"
#include "string"


ICM20948::ICM20948(Status& status) : status{status} {}

void ICM20948::begin() {
	SPI.begin();
	status.ICM20948 = icm20948.begin(ICM20948_CS, SPI) == ICM_20948_Stat_Ok;
	// icm20948.startupMagnetometer();
	status.ICM20948 &= icm20948.initializeDMP() == ICM_20948_Stat_Ok;
	status.ICM20948 &= icm20948.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok;
	status.ICM20948 &= icm20948.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok;
	status.ICM20948 &= icm20948.enableFIFO() == ICM_20948_Stat_Ok;
	status.ICM20948 &= icm20948.enableDMP() == ICM_20948_Stat_Ok;
	status.ICM20948 &= icm20948.resetFIFO() == ICM_20948_Stat_Ok;
	status.ICM20948 &= icm20948.resetDMP() == ICM_20948_Stat_Ok;
	delay(500);
}

void ICM20948::loop() {
	icm20948.readDMPdataFromFIFO(&data);
	if ((icm20948.status == ICM_20948_Stat_Ok) || (icm20948.status == ICM_20948_Stat_FIFOMoreDataAvail)) {
		if ((data.header & DMP_header_bitmap_Quat6) > 0) {
			q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
			q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
			q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
			q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
			// Serial.print(q1, 8);
			// Serial.print("\t\t|\t");
			// Serial.print(q2, 8);
			// Serial.print("\t\t|\t");
			// Serial.print(q3, 8);
			// Serial.print("\t\t|\t");
			// Serial.println(q0, 8);
			status.attitude = {q1, q2, q3, q0};

		}
	}
}
