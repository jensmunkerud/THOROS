#include "ICM20948.h"
#include "string"


ICM20948::ICM20948(Status& status) : status{status} {}

void ICM20948::begin() {
	SPI.begin();
	status.ICM20948 = icm20948.begin(ICM20948_CS, SPI) == ICM_20948_Stat_Ok;
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

			double qw = q0;
			double qx = q2;
			double qy = q1;
			double qz = -q3;

			// roll (x-axis rotation)
			double t0 = +2.0 * (qw * qx + qy * qz);
			double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
			double roll = atan2(t0, t1) * 180.0 / PI;

			// pitch (y-axis rotation)
			double t2 = +2.0 * (qw * qy - qx * qz);
			t2 = t2 > 1.0 ? 1.0 : t2;
			t2 = t2 < -1.0 ? -1.0 : t2;
			double pitch = asin(t2) * 180.0 / PI;

			// yaw (z-axis rotation)
			double t3 = +2.0 * (qw * qz + qx * qy);
			double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
			double yaw = atan2(t3, t4) * 180.0 / PI;
			// Serial.println(pitch);
			// Serial.print("\t\t|\t");
			// Serial.print(yaw);
			// Serial.print("\t\t|\t");
			// Serial.println(roll);
			status.attitude = {pitch, yaw, roll};
		}
	}
}
