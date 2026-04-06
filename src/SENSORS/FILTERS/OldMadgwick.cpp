#include "OldMadgwick.h"

void Madgwick2::begin(float b, float z) {
	beta = b;
	zeta = z;
	q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
	bx = 1.0f; bz = 0.0f;
	w_bx = 0.0f; w_by = 0.0f; w_bz = 0.0f;
}

void Madgwick2::update(float gx, float gy, float gz,
					   float ax, float ay, float az,
					   float mx, float my, float mz,
					   float dt)
{
	// ── 1. Normalise accelerometer ────────────────────────────────────────
	float norm = sqrtf(ax*ax + ay*ay + az*az);
	if (norm < 1e-10f) return;
	norm = 1.0f / norm;
	ax *= norm; ay *= norm; az *= norm;

	// ── 2. Normalise magnetometer ─────────────────────────────────────────
	norm = sqrtf(mx*mx + my*my + mz*mz);
	if (norm < 1e-10f) return;
	norm = 1.0f / norm;
	mx *= norm; my *= norm; mz *= norm;

	// ── 3. Auxiliary variables ────────────────────────────────────────────
	float q0q0 = q0*q0, q0q1 = q0*q1, q0q2 = q0*q2, q0q3 = q0*q3;
	float q1q1 = q1*q1, q1q2 = q1*q2, q1q3 = q1*q3;
	float q2q2 = q2*q2, q2q3 = q2*q3;
	float q3q3 = q3*q3;

	// ── 4. Reference magnetic field in Earth frame ────────────────────────
	// Rotate magnetometer measurement into Earth frame
	float hx = 2.0f*(mx*(0.5f - q2q2 - q3q3) + my*(q1q2 - q0q3) + mz*(q1q3 + q0q2));
	float hy = 2.0f*(mx*(q1q2 + q0q3) + my*(0.5f - q1q1 - q3q3) + mz*(q2q3 - q0q1));

	// Constrain to XZ plane to correct only yaw (not pitch/roll)
	// bx = horizontal component, bz = vertical component
	bx = sqrtf(hx*hx + hy*hy);
	bz = 2.0f*(mx*(q1q3 - q0q2) + my*(q2q3 + q0q1) + mz*(0.5f - q1q1 - q2q2));

	// ── 5. Gradient objective function & Jacobian ─────────────────────────
	// f_acc: gravity in body frame should match measured accel
	float f1 = 2.0f*(q1q3 - q0q2)       - ax;
	float f2 = 2.0f*(q0q1 + q2q3)       - ay;
	float f3 = 2.0f*(0.5f - q1q1 - q2q2)- az;

	// f_mag: reference B field in body frame should match measured mag
	float f4 = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2) - mx;
	float f5 = 2.0f*bx*(q1q2 - q0q3)         + 2.0f*bz*(q0q1 + q2q3) - my;
	float f6 = 2.0f*bx*(q0q2 + q1q3)         + 2.0f*bz*(0.5f - q1q1 - q2q2) - mz;

	// Jacobian rows (J^T * f):
	float s0 = -2.0f*q2*f1 + 2.0f*q1*f2
			   - 2.0f*bz*q2*f4
			   + (-2.0f*bx*q3 + 2.0f*bz*q1)*f5
			   + 2.0f*bx*q2*f6;

	float s1 =  2.0f*q3*f1 + 2.0f*q0*f2 - 4.0f*q1*f3
			   + 2.0f*bz*q3*f4
			   + (2.0f*bx*q2 + 2.0f*bz*q0)*f5
			   + (2.0f*bx*q3 - 4.0f*bz*q1)*f6;

	float s2 = -2.0f*q0*f1 + 2.0f*q3*f2 - 4.0f*q2*f3
			   + (-4.0f*bx*q2 - 2.0f*bz*q0)*f4
			   + (2.0f*bx*q1 + 2.0f*bz*q3)*f5
			   + (2.0f*bx*q0 - 4.0f*bz*q2)*f6;  // fixed sign on last term

	float s3 =  2.0f*q1*f1 + 2.0f*q2*f2
			   + (-4.0f*bx*q3 + 2.0f*bz*q1)*f4  // corrected coefficient
			   + (-2.0f*bx*q0 + 2.0f*bz*q2)*f5
			   + 2.0f*bx*q1*f6;

	// Normalise gradient
	norm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
	if (norm < 1e-10f) return;
	norm = 1.0f / norm;
	s0 *= norm; s1 *= norm; s2 *= norm; s3 *= norm;

	// ── 6. Gyro bias correction (zeta term) ───────────────────────────────
	// The bias error is proportional to the cross product of q̂_est and ŝ
	w_bx += 2.0f*dt * (q0*s1 - q1*s0 - q2*s3 + q3*s2) * zeta;  // corrected signs
	w_by += 2.0f*dt * (q0*s2 + q1*s3 - q2*s0 - q3*s1) * zeta;
	w_bz += 2.0f*dt * (q0*s3 - q1*s2 + q2*s1 - q3*s0) * zeta;

	gx -= w_bx;
	gy -= w_by;
	gz -= w_bz;

	// ── 7. Rate of change of quaternion from gyro ─────────────────────────
	float qDot0 = 0.5f*(-q1*gx - q2*gy - q3*gz) - beta*s0;
	float qDot1 = 0.5f*( q0*gx + q2*gz - q3*gy) - beta*s1;
	float qDot2 = 0.5f*( q0*gy - q1*gz + q3*gx) - beta*s2;
	float qDot3 = 0.5f*( q0*gz + q1*gy - q2*gx) - beta*s3;

	// ── 8. Integrate ──────────────────────────────────────────────────────
	q0 += qDot0 * dt;
	q1 += qDot1 * dt;
	q2 += qDot2 * dt;
	q3 += qDot3 * dt;

	// ── 9. Normalise quaternion ───────────────────────────────────────────
	norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if (norm < 1e-10f) return;
	norm = 1.0f / norm;
	q0 *= norm; q1 *= norm; q2 *= norm; q3 *= norm;
}

float Madgwick2::getRoll() {
	return atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * (180.0f / PI);
}

float Madgwick2::getPitch() {
	float s = 2.0f*(q0*q2 - q3*q1);
	s = constrain(s, -1.0f, 1.0f);
	return asinf(s) * (180.0f / PI);
}

float Madgwick2::getYaw() {
	return atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * (180.0f / PI);
}