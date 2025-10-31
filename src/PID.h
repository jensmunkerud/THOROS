class PID {
public:
	PID(float kp, float ki, float kd)
		: Kp(kp), Ki(ki), Kd(kd), integral(0), lastError(0) {}

	float compute(float target, float actual, float dt = 0.005f) {
		float error = target - actual;
		integral += error * dt;
		float derivative = (error - lastError) / dt;
		lastError = error;
		return Kp * error + Ki * integral + Kd * derivative;
	}

private:
	float Kp, Ki, Kd;
	float integral, lastError;
};
