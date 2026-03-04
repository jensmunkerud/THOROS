#include "MadgwickFilter.h"

MadgwickFilter::MadgwickFilter(float b)
	: beta(b),
	  q0(1.0f), q1(0.0f),
	  q2(0.0f), q3(0.0f)
{
}

void MadgwickFilter::normalize(float &x, float &y, float &z)
{
	float norm = sqrtf(x*x + y*y + z*z);
	if (norm == 0.0f)
		return;

	norm = 1.0f / norm;
	x *= norm;
	y *= norm;
	z *= norm;
}

void MadgwickFilter::update(float gx, float gy, float gz,
							float ax, float ay, float az,
							float mx, float my, float mz,
							float dt)
{
	if (dt <= 0.0f)
		return;

	// Normalize accelerometer
	normalize(ax, ay, az);
	if (ax == 0.0f && ay == 0.0f && az == 0.0f)
		return;

	// Normalize magnetometer
	normalize(mx, my, mz);

	float q0 = this->q0;
	float q1 = this->q1;
	float q2 = this->q2;
	float q3 = this->q3;

	//
	// --- Auxiliary variables ---
	//
	float _2q0 = 2.0f * q0;
	float _2q1 = 2.0f * q1;
	float _2q2 = 2.0f * q2;
	float _2q3 = 2.0f * q3;

	//
	// --- Reference magnetic field ---
	//
	float hx =
		mx * (0.5f - q2*q2 - q3*q3) +
		my * (q1*q2 - q0*q3) +
		mz * (q1*q3 + q0*q2);

	float hy =
		mx * (q1*q2 + q0*q3) +
		my * (0.5f - q1*q1 - q3*q3) +
		mz * (q2*q3 - q0*q1);

	float bx = sqrtf(hx*hx + hy*hy);
	float bz =
		mx * (_2q1*q3 - _2q0*q2) +
		my * (_2q0*q1 + _2q2*q3) +
		mz * (0.5f - q1*q1 - q2*q2);

	//
	// --- Gradient Descent (Full 9DOF Objective) ---
	//

	float f1 = 2.0f*(q1*q3 - q0*q2) - ax;
	float f2 = 2.0f*(q0*q1 + q2*q3) - ay;
	float f3 = 2.0f*(0.5f - q1*q1 - q2*q2) - az;

	float f4 =
		2.0f*bx*(0.5f - q2*q2 - q3*q3) +
		2.0f*bz*(q1*q3 - q0*q2) - mx;

	float f5 =
		2.0f*bx*(q1*q2 - q0*q3) +
		2.0f*bz*(q0*q1 + q2*q3) - my;

	float f6 =
		2.0f*bx*(q0*q2 + q1*q3) +
		2.0f*bz*(0.5f - q1*q1 - q2*q2) - mz;

	//
	// Jacobian transpose (mag + accel)
	//

	float J_11 = -_2q2;
	float J_12 =  _2q3;
	float J_13 = -_2q0;
	float J_14 =  _2q1;

	float J_21 =  _2q1;
	float J_22 =  _2q0;
	float J_23 =  _2q3;
	float J_24 =  _2q2;

	float s0 =
		J_14*f1 + J_24*f2 +
		(2.0f*bx*q3 + 2.0f*bz*q1)*f4 +
		(2.0f*bx*q2 - 2.0f*bz*q0)*f5;

	float s1 =
		J_13*f1 + J_23*f2 +
		(2.0f*bx*q2 - 2.0f*bz*q0)*f4 +
		(2.0f*bx*q3 + 2.0f*bz*q1)*f5;

	float s2 =
		J_12*f1 + J_22*f2 -
		4.0f*bx*q2*f6;

	float s3 =
		J_11*f1 + J_21*f2 -
		4.0f*bx*q3*f6;

	//
	// Normalize gradient
	//
	float norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
	if (norm > 0.0f) {
		norm = 1.0f / norm;
		s0 *= norm;
		s1 *= norm;
		s2 *= norm;
		s3 *= norm;
	}

	//
	// --- Quaternion rate from gyro ---
	//
	float qDot0 =
		0.5f * (-q1*gx - q2*gy - q3*gz) - beta * s0;

	float qDot1 =
		0.5f * ( q0*gx + q2*gz - q3*gy) - beta * s1;

	float qDot2 =
		0.5f * ( q0*gy - q1*gz + q3*gx) - beta * s2;

	float qDot3 =
		0.5f * ( q0*gz + q1*gy - q2*gx) - beta * s3;

	//
	// --- Integrate once ---
	//
	q0 += qDot0 * dt;
	q1 += qDot1 * dt;
	q2 += qDot2 * dt;
	q3 += qDot3 * dt;

	//
	// --- Normalize quaternion ---
	//
	norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if (norm > 0.0f) {
		norm = 1.0f / norm;
		q0 *= norm;
		q1 *= norm;
		q2 *= norm;
		q3 *= norm;
	}

	this->q0 = q0;
	this->q1 = q1;
	this->q2 = q2;
	this->q3 = q3;
}

float MadgwickFilter::getRoll() const
{
	return atan2f(2.0f*(q0*q1 + q2*q3),
				  1.0f - 2.0f*(q1*q1 + q2*q2))
		   * 180.0f / M_PI;
}

float MadgwickFilter::getPitch() const
{
	return asinf(2.0f*(q0*q2 - q3*q1))
		   * 180.0f / M_PI;
}

float MadgwickFilter::getYaw() const
{
	return atan2f(2.0f*(q0*q3 + q1*q2),
				  1.0f - 2.0f*(q2*q2 + q3*q3))
		   * 180.0f / M_PI;
}