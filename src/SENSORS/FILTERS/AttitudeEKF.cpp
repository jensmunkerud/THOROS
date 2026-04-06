#include "AttitudeEKF.h"

AttitudeEKF::AttitudeEKF()
{
	q0 = 1.0f; q1 = 0; q2 = 0; q3 = 0;
	bgx = bgy = bgz = 0;

	for(int i=0;i<6;i++)
		P[i] = 0.01f;
}

void AttitudeEKF::setGyroBias(float bx,float by,float bz)
{
	bgx = bx; bgy = by; bgz = bz;
}

void AttitudeEKF::normalize(float &x,float &y,float &z)
{
	float n = sqrtf(x*x + y*y + z*z);
	if(n == 0) return;
	n = 1.0f/n;
	x *= n; y *= n; z *= n;
}

void AttitudeEKF::normalizeQuat()
{
	float n = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if(n == 0) return;
	n = 1.0f/n;
	q0 *= n; q1 *= n; q2 *= n; q3 *= n;
}

//
// ================== Prediction ==================
//
void AttitudeEKF::predict(float gx,float gy,float gz,float dt)
{
	gx -= bgx;
	gy -= bgy;
	gz -= bgz;

	float q0_old=q0, q1_old=q1, q2_old=q2, q3_old=q3;

	float qDot0 = 0.5f * (-q1_old*gx - q2_old*gy - q3_old*gz);
	float qDot1 = 0.5f * ( q0_old*gx + q2_old*gz - q3_old*gy);
	float qDot2 = 0.5f * ( q0_old*gy - q1_old*gz + q3_old*gx);
	float qDot3 = 0.5f * ( q0_old*gz + q1_old*gy - q2_old*gx);

	q0 += qDot0 * dt;
	q1 += qDot1 * dt;
	q2 += qDot2 * dt;
	q3 += qDot3 * dt;

	normalizeQuat();
}

//
// ================== Accelerometer Update ==================
//
void AttitudeEKF::updateAccel(float ax,float ay,float az)
{
	normalize(ax,ay,az);
	if(ax==0 && ay==0 && az==0) return;

	// Expected gravity from quaternion
	float gx = 2.0f*(q1*q3 - q0*q2);
	float gy = 2.0f*(q0*q1 + q2*q3);
	float gz = 1.0f - 2.0f*(q1*q1 + q2*q2);

	// Innovation
	float ex = ay*gz - az*gy;
	float ey = az*gx - ax*gz;
	float ez = ax*gy - ay*gx;

	float k = 0.05f; // accel gain

	q0 += -k * (-q1*ex - q2*ey - q3*ez);
	q1 += -k * ( q0*ex + q2*ez - q3*ey);
	q2 += -k * ( q0*ey - q1*ez + q3*ex);
	q3 += -k * ( q0*ez + q1*ey - q2*ex);

	normalizeQuat();
}

//
// ================== Magnetometer Update ==================
//
void AttitudeEKF::updateMag(float mx,float my,float mz)
{
	normalize(mx,my,mz);
	if(mx==0 && my==0 && mz==0) return;

	// Yaw correction only
	float yaw = atan2f(
		2.0f*(q0*q3 + q1*q2),
		1.0f - 2.0f*(q2*q2 + q3*q3)
	);

	float magYaw = atan2f(my,mx);
	float error = magYaw - yaw;

	float k = 0.02f;

	q0 += -k * error * q0;
	q1 += -k * error * q1;
	q2 += -k * error * q2;
	q3 += -k * error * q3;

	normalizeQuat();
}

float AttitudeEKF::getRoll() const
{
	return atan2f(2.0f*(q0*q1 + q2*q3),
				  1.0f - 2.0f*(q1*q1 + q2*q2))
		   * 180.0f/M_PI;
}

float AttitudeEKF::getPitch() const
{
	return asinf(2.0f*(q0*q2 - q3*q1))
		   * 180.0f/M_PI;
}

float AttitudeEKF::getYaw() const
{
	return atan2f(2.0f*(q0*q3 + q1*q2),
				  1.0f - 2.0f*(q2*q2 + q3*q3))
		   * 180.0f/M_PI;
}