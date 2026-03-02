#include "Kalman.h"

void Kalman::begin(float processNoise, float measurementNoise) {
	Q_angle   = processNoise;
	Q_bias    = processNoise * 0.1f;
	R_measure = measurementNoise;

	for (int i = 0; i < 3; i++) {
		angle[i] = 0.0f;
		axes[i].bias = 0.0f;
		axes[i].P[0][0] = 1.0f; axes[i].P[0][1] = 0.0f;
		axes[i].P[1][0] = 0.0f; axes[i].P[1][1] = 1.0f;
	}
}

// Core 2-state Kalman (angle + gyro bias) for one axis
float Kalman::kalmanUpdate(AxisKalman& k, float newAngle, float newRate, float dt) {
	// --- Predict ---
	float rate = newRate - k.bias;
	float predicted = angle[0]; // caller sets this before calling

	// Update covariance
	k.P[0][0] += dt * (dt * k.P[1][1] - k.P[0][1] - k.P[1][0] + Q_angle);
	k.P[0][1] -= dt * k.P[1][1];
	k.P[1][0] -= dt * k.P[1][1];
	k.P[1][1] += Q_bias * dt;

	// --- Update ---
	float S = k.P[0][0] + R_measure; // Innovation covariance
	float K0 = k.P[0][0] / S;        // Kalman gain
	float K1 = k.P[1][0] / S;

	float y = newAngle - predicted;   // Innovation

	float updated = predicted + K0 * y;
	k.bias += K1 * y;

	// Update error covariance
	float P00_temp = k.P[0][0];
	float P01_temp = k.P[0][1];
	k.P[0][0] -= K0 * P00_temp;
	k.P[0][1] -= K0 * P01_temp;
	k.P[1][0] -= K1 * P00_temp;
	k.P[1][1] -= K1 * P01_temp;

	return updated;
}

void Kalman::update(float gx, float gy, float gz,
					float ax, float ay, float az,
					float dt)
{
	// --- Normalize accelerometer ---
	float norm = sqrtf(ax*ax + ay*ay + az*az);
	if (norm < 1e-6f) return; // Skip bad readings
	ax /= norm; ay /= norm; az /= norm;

	// --- Accel-derived roll & pitch (measurement) ---
	float rollAcc  =  atan2f(ay, az);
	float pitchAcc = -asinf(constrain(ax, -1.0f, 1.0f));

	// --- Gyro-predicted roll & pitch (prediction step) ---
	// Convert gyro rates from sensor frame to Euler rates
	float sinRoll = sinf(angle[0]), cosRoll = cosf(angle[0]);
	float cosPitch = cosf(angle[1]);
	float tanPitch = tanf(angle[1]);

	float rollRate  = gx + sinRoll * tanPitch * gy + cosRoll * tanPitch * gz;
	float pitchRate =                cosRoll        * gy -        sinRoll * gz;
	float yawRate   = (sinRoll / cosPitch) * gy + (cosRoll / cosPitch) * gz;

	// Predict angles using gyro
	angle[0] += rollRate  * dt;
	angle[1] += pitchRate * dt;
	angle[2] += yawRate   * dt;

	// --- Kalman correct roll & pitch with accel ---
	angle[0] = kalmanUpdate(axes[0], rollAcc,  rollRate,  dt);
	angle[1] = kalmanUpdate(axes[1], pitchAcc, pitchRate, dt);
	// Yaw: no accel correction (no magnetometer), gyro only
	// angle[2] already integrated above

	// Wrap yaw to [-π, π]
	while (angle[2] >  M_PI) angle[2] -= 2.0f * M_PI;
	while (angle[2] < -M_PI) angle[2] += 2.0f * M_PI;
}

float Kalman::getRoll()  { return angle[0] * RAD_TO_DEG; }
float Kalman::getPitch() { return angle[1] * RAD_TO_DEG; }
float Kalman::getYaw()   { return angle[2] * RAD_TO_DEG; }