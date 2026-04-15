#include "ICM20948.h"

static Vec3 gravityBodyFromAttitude(float rollDeg, float pitchDeg) {
	float roll = rollDeg * DEG2RAD;
	float pitch = pitchDeg * DEG2RAD;
	float cr = cosf(roll);
	float sr = sinf(roll);
	float cp = cosf(pitch);
	float sp = sinf(pitch);

	return {
		-sp,
		cp * sr,
		cp * cr
	};
}
 
ICM20948::ICM20948(Status& status) : status{status} {
	sampleRate.a = (1000 / ICM_SAMPLERATE) - 1;
	sampleRate.g = (1000 / ICM_SAMPLERATE) - 1;
}

void ICM20948::begin() {
	SPI.begin();
	status.ICM20948 = icm20948.begin(ICM20948_CS, SPI) == ICM_20948_Stat_Ok;
	if (!status.ICM20948) {
		Serial.println("failed first");
		return;
	}
	status.ICM20948 = icm20948.startupMagnetometer() == ICM_20948_Stat_Ok;
	if (!status.ICM20948) {
		Serial.println("failed first");
		return;
	}
	ICM_20948_fss_t fss;
	fss.a = gpm8;
	fss.g = dps2000;

	icm20948.setFullScale(ICM_20948_Internal_Acc, fss);
	icm20948.setFullScale(ICM_20948_Internal_Gyr, fss);

	icm20948.setSampleRate(ICM_20948_Internal_Acc, sampleRate);
	icm20948.setSampleRate(ICM_20948_Internal_Gyr, sampleRate);

	ICM_20948_dlpcfg_t dlpfCfg;
	dlpfCfg.a = acc_d23bw9_n34bw4;
	dlpfCfg.g = gyr_d23bw9_n35bw9;
	icm20948.setDLPFcfg(ICM_20948_Internal_Acc, dlpfCfg);
	icm20948.setDLPFcfg(ICM_20948_Internal_Gyr, dlpfCfg);
	icm20948.enableDLPF(ICM_20948_Internal_Acc, true);
	icm20948.enableDLPF(ICM_20948_Internal_Gyr, true);

	delay(500);
	calibrateIMU();
	status.ICM20948 = 1;
}

void ICM20948::loop() {
	if (icm20948.dataReady()) {
		icm20948.getAGMT();
		dt = fusion.deltatUpdate();

		Vec3 gyr = {
			(icm20948.gyrX() - gyroBias.x) * DEG2RAD,
			(icm20948.gyrY() - gyroBias.y) * DEG2RAD,
			(icm20948.gyrZ() - gyroBias.z) * DEG2RAD
		};
		
		float accDivision = 1000.0f;
		Vec3 acc = {
			(icm20948.accX()) / accDivision,
			(icm20948.accY()) / accDivision,
			(icm20948.accZ()) / accDivision
		};
		
		Vec3 mag = {
			icm20948.magX() - MAG_BIAS_X * MAG_SCALE_X,
			icm20948.magY() - MAG_BIAS_Y * MAG_SCALE_Y,
			icm20948.magZ() - MAG_BIAS_Z * MAG_SCALE_Z
		};

		gyr = compensateForMountingRotation(gyr);
		acc = compensateForMountingRotation(acc);
		mag = compensateForMountingRotation(mag);

		if (!accelFilterInitialized) {
			accelFiltered = acc;
			accelFilterInitialized = true;
		} else {
			accelFiltered.x += ACCEL_LPF_ALPHA * (acc.x - accelFiltered.x);
			accelFiltered.y += ACCEL_LPF_ALPHA * (acc.y - accelFiltered.y);
			accelFiltered.z += ACCEL_LPF_ALPHA * (acc.z - accelFiltered.z);
		}
		acc = accelFiltered;

		if (!gyroFilterInitialized) {
			gyroFiltered = gyr;
			gyroFilterInitialized = true;
		} else {
			gyroFiltered.x += GYRO_LPF_ALPHA * (gyr.x - gyroFiltered.x);
			gyroFiltered.y += GYRO_LPF_ALPHA * (gyr.y - gyroFiltered.y);
			gyroFiltered.z += GYRO_LPF_ALPHA * (gyr.z - gyroFiltered.z);
		}
		gyr = gyroFiltered;

		// ============================================
		// RAW SENSOR DATA IS TUNED FROM THIS POINT ON
		// Gyro -> rad/s
		// Acc  -> Gs
		// Mag  -> micro teslas
		// ============================================

		fusion.MahonyUpdate(gyr.x, gyr.y, gyr.z, acc.x, acc.y, acc.z, dt);  //mahony is suggested if there isn't the mag and the mcu is slow
		// fusion.MadgwickUpdate(gyr.x, gyr.y, gyr.z, acc.x, acc.y, acc.z, mag.x, mag.y, mag.z, deltat);  //else use the magwick, it is slower but more accurate
		float fusedRoll = fusion.getRoll();
		float fusedPitch = fusion.getPitch();
		float fusedYaw = fusion.getYaw() - 184.0f;

		status.attitude.pitch = -fusedRoll;
		status.attitude.roll  = fusedPitch;
		status.attitude.yaw   = fusedYaw;

		// Remove gravity using current attitude, then convert to world frame.
		Vec3 gBody = gravityBodyFromAttitude(fusedRoll, fusedPitch);
		Vec3 linBody = {
			acc.x - gBody.x,
			acc.y - gBody.y,
			acc.z - gBody.z
		};
		status.linearAccel.x = linBody.x;
		status.linearAccel.y = linBody.y;
		status.linearAccel.z = linBody.z;

		// Print: 9axis_debug
		// Serial.print(acc.x, 4); Serial.print("/");
		// Serial.print(acc.y, 4); Serial.print("/");
		// Serial.print(acc.z, 4); Serial.print("/");
		// Serial.print(gyr.x, 4); Serial.print("/");
		// Serial.print(gyr.y, 4); Serial.print("/");
		// Serial.print(gyr.z, 4); Serial.print("/");
		// Serial.print(mag.x, 4); Serial.print("/");
		// Serial.print(mag.y, 4); Serial.print("/");
		// Serial.println(mag.z, 4);
	}
}

// Sensor calibration sequence, also compensates mountingRotation - Time: 2000ms
void ICM20948::calibrateIMU() {
	const int samples = 2000;
	const unsigned long sampleTimeoutUs = 20000;
	Vec3 gyr = {0, 0, 0};
	Vec3 acc = {0, 0, 0};

	for (int i = 0; i < samples; i++) {
		unsigned long waitStartUs = micros();
		while (!icm20948.dataReady()) {
			if ((unsigned long)(micros() - waitStartUs) > sampleTimeoutUs) {
				status.ICM20948 = 0;
				return;
			}
			delayMicroseconds(100);
		}
		icm20948.getAGMT();

		gyr.x += icm20948.gyrX();
		gyr.y += icm20948.gyrY();
		gyr.z += icm20948.gyrZ();
		
		acc.x += icm20948.accX();
		acc.y += icm20948.accY();
		acc.z += icm20948.accZ();

		// dt = fusion.deltatUpdate();
		// fusion.MahonyUpdate(gyr.x, gyr.y, gyr.z, acc.x, acc.y, acc.z, dt);

		delayMicroseconds(1000);
	}

		// Reset the fusion timer so the first runtime update uses a normal dt.
	fusion.deltatUpdate();

	// yawOffset = fusion.getYaw();

	gyroBias.x = gyr.x / samples;
	gyroBias.y = gyr.y / samples;
	gyroBias.z = gyr.z / samples;

	acc = normalize(acc);

	Vec3 target = {0, 0, 1};

	Vec3 axis = cross(acc, target);
	float axis_norm = sqrtf(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);

	if(axis_norm < 1e-6f)
	{
		// Already aligned
		R_mount[0][0]=1; R_mount[0][1]=0; R_mount[0][2]=0;
		R_mount[1][0]=0; R_mount[1][1]=1; R_mount[1][2]=0;
		R_mount[2][0]=0; R_mount[2][1]=0; R_mount[2][2]=1;
		return;
	}

	axis.x /= axis_norm;
	axis.y /= axis_norm;
	axis.z /= axis_norm;

	float cosA = dot(acc, target);
	cosA = constrain(cosA, -1.0f, 1.0f);
	float angle = acos(cosA);
	float sinA = sin(angle);

	float K[3][3] = {
		{0, -axis.z, axis.y},
		{axis.z, 0, -axis.x},
		{-axis.y, axis.x, 0}
	};

	// Rodrigues formula: R = I + sinA*K + (1-cosA)*K^2
	float K2[3][3];

	for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	{
		K2[i][j] = 0;
		for(int k=0;k<3;k++)
			K2[i][j] += K[i][k]*K[k][j];
	}

	for(int i=0;i<3;i++)
	for(int j=0;j<3;j++)
	{
		R_mount[i][j] =
			(i==j ? 1.0f : 0.0f)
			+ sinA * K[i][j]
			+ (1 - cosA) * K2[i][j];
	}
}

Vec3 ICM20948::normalize(Vec3 v)
{
	float n = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
	return { v.x/n, v.y/n, v.z/n };
}

Vec3 ICM20948::cross(Vec3 a, Vec3 b)
{
	return {
		a.y*b.z - a.z*b.y,
		a.z*b.x - a.x*b.z,
		a.x*b.y - a.y*b.x
	};
}

float ICM20948::dot(Vec3 a, Vec3 b)
{
	return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 ICM20948::compensateForMountingRotation(Vec3 v)
{
	Vec3 r;
	r.x = R_mount[0][0]*v.x + R_mount[0][1]*v.y + R_mount[0][2]*v.z;
	r.y = R_mount[1][0]*v.x + R_mount[1][1]*v.y + R_mount[1][2]*v.z;
	r.z = R_mount[2][0]*v.x + R_mount[2][1]*v.y + R_mount[2][2]*v.z;
	return r;
}

