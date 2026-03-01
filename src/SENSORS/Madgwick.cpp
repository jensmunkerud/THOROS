#include "Madgwick.h"

void Madgwick::begin(float b) {
	beta = b;
}

void Madgwick::update(float gx, float gy, float gz,
					  float ax, float ay, float az,
					  float dt)
{
	float norm = sqrt(ax * ax + ay * ay + az * az);
	if (norm == 0.0f) return;
	norm = 1.0f / norm;
	ax *= norm;
	ay *= norm;
	az *= norm;

	float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	float f1 = 2*(q1*q3 - q0*q2) - ax;
	float f2 = 2*(q0*q1 + q2*q3) - ay;
	float f3 = 2*(0.5f - q1*q1 - q2*q2) - az;

	float J_11or24 = 2 * q2;
	float J_12or23 = 2 * q3;
	float J_13or22 = 2 * q0;
	float J_14or21 = 2 * q1;
	float J_32 = 2 * J_14or21;
	float J_33 = 2 * J_11or24;

	float s0 = -J_14or21 * f2 + J_11or24 * f1;
	float s1 = J_13or22 * f2 + J_12or23 * f1 - J_32 * f3;
	float s2 = J_13or22 * f1 - J_33 * f3 - J_12or23 * f2;
	float s3 = J_14or21 * f1 + J_11or24 * f2;

	norm = sqrt(s0*s0 + s1*s1 + s2*s2 + s3*s3);
	norm = 1.0f / norm;
	s0 *= norm;
	s1 *= norm;
	s2 *= norm;
	s3 *= norm;

	qDot1 -= beta * s0;
	qDot2 -= beta * s1;
	qDot3 -= beta * s2;
	qDot4 -= beta * s3;

	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	norm = 1.0f / norm;
	q0 *= norm;
	q1 *= norm;
	q2 *= norm;
	q3 *= norm;
}

float Madgwick::getRoll() {
	return atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180.0f / PI;
}

float Madgwick::getPitch() {
	float s = 2*(q0*q2 - q3*q1);
	if (s > 1) s = 1;
	if (s < -1) s = -1;
	return asin(s) * 180.0f / PI;
}

float Madgwick::getYaw() {
	return atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)) * 180.0f / PI;
}