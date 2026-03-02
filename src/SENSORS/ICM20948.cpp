#include "ICM20948.h"
#include "string"


ICM20948::ICM20948(Status& status) : status{status}, lastTime{0} {}

void ICM20948::begin() {
	SPI.begin();
	status.ICM20948 = icm20948.begin(ICM20948_CS, SPI) == ICM_20948_Stat_Ok;
	ICM_20948_fss_t myFSS;
	myFSS.a = gpm4;
	myFSS.g = dps500;
	icm20948.setFullScale(ICM_20948_Internal_Acc, myFSS);
	icm20948.setFullScale(ICM_20948_Internal_Gyr, myFSS);

	icm20948.setSampleRate(ICM_20948_Internal_Gyr, ICM_SAMPLERATE);
	icm20948.setSampleRate(ICM_20948_Internal_Acc, ICM_SAMPLERATE);
	filter.begin(0.05f);
	delay(500);
}
 
void ICM20948::loop() {
	uint32_t now = micros();
	float dt = (now - lastTime) * 1e-6f;
	lastTime = now;

	agmt = icm20948.getAGMT();

	float gx = agmt.gyr.axes.x * DEG_TO_RAD;
	float gy = agmt.gyr.axes.y * DEG_TO_RAD;
	float gz = agmt.gyr.axes.z * DEG_TO_RAD;

	float ax = agmt.acc.axes.x;
	float ay = agmt.acc.axes.y;
	float az = agmt.acc.axes.z;

	lastTime = now;

	filter.update(gx, gy, gz, ax, ay, az, dt);

	status.attitude = {
		filter.getPitch(),
		filter.getYaw(),
		filter.getRoll()
	};
	Serial.print(status.attitude.pitch);
	Serial.print("/");
	Serial.print(status.attitude.yaw);
	Serial.print("/");
	Serial.println(status.attitude.roll);
// 	Serial.print(gx);
// 	Serial.print("/");
// 	Serial.print(gy);
// 	Serial.print("/");
// 	Serial.println(gz);
}

