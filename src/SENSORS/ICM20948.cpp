#include "ICM20948.h"
#include "string"

#define PRINT_FILTERED_ORIENTATION 1
#define PRINT_RAW_ORIENTATION 0

ICM20948::ICM20948(Status& status) : status{status}, lastTime{0} {
	filter = new MadgwickFilter(0.1f);

	sampleRate.a = (1000 / ICM_SAMPLERATE) - 1;
	sampleRate.g = (1000 / ICM_SAMPLERATE) - 1;
}

void ICM20948::begin() {
	SPI.begin();
	status.ICM20948 = icm20948.begin(ICM20948_CS, SPI) == ICM_20948_Stat_Ok;
	status.ICM20948 &= icm20948.startupMagnetometer();
	ICM_20948_fss_t fss;
	fss.a = gpm8;
	fss.g = dps2000;

	icm20948.setFullScale(ICM_20948_Internal_Acc, fss);
	icm20948.setFullScale(ICM_20948_Internal_Gyr, fss);

	icm20948.setSampleRate(ICM_20948_Internal_Acc, sampleRate);
	icm20948.setSampleRate(ICM_20948_Internal_Gyr, sampleRate);

	// madgwickFilter.begin(ICM_SAMPLERATE);

	delay(500);
	calibrateIMU();
	computeMountingRotation();
	lastTime = micros();
}

void ICM20948::loop() {
	if (icm20948.dataReady()) {
		icm20948.getAGMT();

		uint32_t now = micros();
		uint32_t delta = now - lastTime;

		if (delta == 0) return;

		float dt = delta * 1e-6f;
		lastTime = now;

		float mx = icm20948.magX() - magBiasX;
		float my = icm20948.magY() - magBiasY;
		float mz = icm20948.magZ() - magBiasZ;

		Vec3 acc = {
			(icm20948.accX()) / 1000.0f,
			(icm20948.accY()) / 1000.0f,
			(icm20948.accZ()) / 1000.0f
		};

		Vec3 gyr = {
			(icm20948.gyrX()) * DEG2RAD,
			(icm20948.gyrY()) * DEG2RAD,
			(icm20948.gyrZ()) * DEG2RAD
		};

		acc = rotate(acc);

		// Serial.print(mx);
		// Serial.print("/");
		// Serial.print(my);
		// Serial.print("/");
		// Serial.println(mz);

		// filter->update(gyr.x, gyr.y, gyr.z,
		// 				acc.x, acc.y, acc.z,
		// 				mx, my, mz, dt);

		// MadgwickQuaternionUpdate(acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z, mx, my, mz, dt);
		// MahonyQuaternionUpdateIMU(acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z, dt);
		// Attitude a = getAttitude();

		Attitude a = UpdateIMU(acc.x, acc.y, acc.z, gyr.x, gyr.y, gyr.z, dt);
		status.attitude.pitch = a.pitch;
		status.attitude.roll  = a.roll;
		status.attitude.yaw   = a.yaw;
		// Serial.print(getQ()[0]);
		// Serial.print("/");
		// Serial.print(getQ()[1]);
		// Serial.print("/");
		// Serial.print(getQ()[2]);
		// Serial.print("/");
		// Serial.println(getQ()[3]);

		// ekf.predict(gyr.x, gyr.y, gyr.z, dt);
		// ekf.updateAccel(acc.x, acc.y, acc.z);
		// ekf.updateMag(mx, my, mz);

		// status.attitude.pitch = filter->getPitch();
		// status.attitude.roll  = filter->getRoll();
		// status.attitude.yaw   = filter->getYaw();

		#if PRINT_FILTERED_ORIENTATION
			Serial.print(status.attitude.pitch);
			Serial.print("/");
			Serial.print(status.attitude.yaw);
			Serial.print("/");
			Serial.println(status.attitude.roll);
		#endif

		#if PRINT_RAW_ORIENTATION
			// Serial.print(gx);
			// Serial.print("/");
			// Serial.print(gy);
			// Serial.print("/");
			// Serial.println(gz);
			Serial.print(acc.x, 8);
			Serial.print("		/		");
			Serial.print(acc.y, 8);
			Serial.print("		/		");
			Serial.println(acc.z, 8);

			// GYRO
			// pitch = -float(items[1]); // ->gy
			// yaw   = float(items[2])-90; // ->gz
			// roll  = -float(items[0])+180; // ->gx
		#endif
	}
}

void ICM20948::calibrateIMU() {
	const int samples = 1000;
	float sumGx = 0, sumGy = 0, sumGz = 0;
	float sumAx = 0, sumAy = 0, sumAz = 0;
	float mx = 0, my = 0, mz = 0;
	float magMin[3] = {100000,100000,100000};
	float magMax[3] = {-100000,-100000,-100000};


	for (int i = 0; i < samples; i++) {
		while (!icm20948.dataReady());
		icm20948.getAGMT();

		sumGx += icm20948.gyrX();
		sumGy += icm20948.gyrY();
		sumGz += icm20948.gyrZ();
		
		sumAx += icm20948.accX();
		sumAy += icm20948.accY();
		sumAz += icm20948.accZ();

		mx = icm20948.magX();
		my = icm20948.magY();
		mz = icm20948.magZ();

		magMin[0] = min(magMin[0], mx);
		magMin[1] = min(magMin[1], my);
		magMin[2] = min(magMin[2], mz);

		magMax[0] = max(magMax[0], mx);
		magMax[1] = max(magMax[1], my);
		magMax[2] = max(magMax[2], mz);

		delayMicroseconds(1000);
	}

	gyroBiasX = sumGx / samples;
	gyroBiasY = sumGy / samples;
	gyroBiasZ = sumGz / samples;

	// accBiasX = sumAx / samples;
	// accBiasY = sumAy / samples;
	// accBiasZ = sumAz / samples;

	magBiasX = (magMax[0] + magMin[0]) * 0.5f;
	magBiasY = (magMax[1] + magMin[1]) * 0.5f;
	magBiasZ = (magMax[2] + magMin[2]) * 0.5f;
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

Vec3 ICM20948::rotate(Vec3 v)
{
	Vec3 r;
	r.x = R_mount[0][0]*v.x + R_mount[0][1]*v.y + R_mount[0][2]*v.z;
	r.y = R_mount[1][0]*v.x + R_mount[1][1]*v.y + R_mount[1][2]*v.z;
	r.z = R_mount[2][0]*v.x + R_mount[2][1]*v.y + R_mount[2][2]*v.z;
	return r;
}

void ICM20948::computeMountingRotation()
{
	const int samples = 1000;
	Vec3 g = {0,0,0};

	for(int i=0;i<samples;i++)
	{
		while(!icm20948.dataReady());
		icm20948.getAGMT();

		g.x += icm20948.accX();
		g.y += icm20948.accY();
		g.z += icm20948.accZ();

		delayMicroseconds(1000);
	}

	g.x /= samples;
	g.y /= samples;
	g.z /= samples;

	g = normalize(g);

	Vec3 target = {0, 0, -1};

	Vec3 axis = cross(g, target);
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

	float cosA = dot(g, target);
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