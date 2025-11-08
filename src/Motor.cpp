#include "Motor.h"

Motor::Motor(MovementController& mc, Status& s) : movementController{mc}, status{s}, target{0, 0, 0, 1}, 
motor1(MOTOR1),
motor2(MOTOR2),
motor3(MOTOR3),
motor4(MOTOR4)
{}

void Motor::begin() {
	motor1.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor2.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor3.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);
	motor4.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);

	for(int i = 0; i < INITILIZE_ESC_TIME; i++) {
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		delay(1);
	}
}

Quaternion quatConjugate(const Quaternion &q) {
	return { -q.x, -q.y, -q.z, q.w, };
}

Quaternion quatMultiply(const Quaternion& a, const Quaternion& b) {
	return {
	a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
	a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
	a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
	a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
	};
}

void quatNormalize(Quaternion& q) {
	double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
	if (norm > 0.0) {
		q.w /= norm;
		q.x /= norm;
		q.y /= norm;
		q.z /= norm;
	}
}

Quaternion quatError(const Quaternion& q_des, const Quaternion& q_meas) {
	Quaternion q_inv = quatConjugate(q_meas);
	Quaternion q_err = quatMultiply(q_des, q_inv);
	// Ensure shortest path
	if (q_err.w < 0) {
		q_err.w = -q_err.w;
		q_err.x = -q_err.x;
		q_err.y = -q_err.y;
		q_err.z = -q_err.z;
	}
	quatNormalize(q_err);
	return q_err;
}

void rotationVectorFromQuaternion(const Quaternion& q_err, double& ex, double& ey, double& ez) {
	ex = 2.0 * q_err.x;
	ey = 2.0 * q_err.y;
	ez = 2.0 * q_err.z;
}

void Motor::loop() {
	// if (status.RFD900 != 1) {
	// 	motor1.send_dshot_value(0);
	// 	motor2.send_dshot_value(0);
	// 	motor3.send_dshot_value(0);
	// 	motor4.send_dshot_value(0);
	// 	return;
	// }
	error = quatError(target, status.attitude);
	double ex, ey, ez;
	rotationVectorFromQuaternion(error, ex, ey, ez);
	
	// ---- Outer loop: attitude PID (P only) ----
	double p_des = Kp_att * ex;
	double q_des_rate = Kp_att * ey;
	double r_des = Kp_att * ez;

	// ---- Inner loop: rate PID ----
	double p_err = p_des - status.gyroX;
	double q_err_rate = q_des_rate - status.gyroY;
	double r_err_rate = r_des - status.gyroZ;

	// integrate
	p_int += p_err * dt;
	q_int += q_err_rate * dt;
	r_int += r_err_rate * dt;

	// derivative
	double dp = (p_err - p_prev) / dt;
	double dq = (q_err_rate - q_prev) / dt;
	double dr = (r_err_rate - r_prev) / dt;

	// PID output (torques or rate commands)
	double p_out = Kp_rate * p_err + Ki_rate * p_int + Kd_rate * dp;
	double q_out = Kp_rate * q_err_rate + Ki_rate * q_int + Kd_rate * dq;
	double r_out = Kp_rate * r_err_rate + Ki_rate * r_int + Kd_rate * dr;

	// store for next step
	p_prev = p_err;
	q_prev = q_err_rate;
	r_prev = r_err_rate;


	// Step 7: Mixer to get motor commands
	double m1 = MINIMUM_MOTOR_SPEED - p_out + q_out - r_out;
	double m2 = MINIMUM_MOTOR_SPEED + p_out + q_out + r_out;
	double m3 = MINIMUM_MOTOR_SPEED - p_out - q_out + r_out;
	double m4 = MINIMUM_MOTOR_SPEED + p_out - q_out - r_out;

	// Step 8: Constrain and send
	m1 = constrain(m1, 0, 500);
	m2 = constrain(m2, 0, 500);
	m3 = constrain(m3, 0, 500);
	m4 = constrain(m4, 0, 500);

	motor1.send_dshot_value((int)m1);
	motor2.send_dshot_value((int)m2);
	motor3.send_dshot_value((int)m3);
	motor4.send_dshot_value((int)m4);
	Serial.print((int)m1);
	Serial.print("\t\t|\t");
	Serial.print((int)m2);
	Serial.print("\t\t|\t");
	Serial.print((int)m3);
	Serial.print("\t\t|\t");
	Serial.println((int)m4);
}