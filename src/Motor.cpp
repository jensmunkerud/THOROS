#include "Motor.h"

Motor::Motor(MovementController& mc, Status& s) : movementController{mc}, status{s}, motor1(MOTOR1) {
}

void Motor::begin() {
	motor1.begin(DSHOT_TYPE, NO_BIDIRECTION, 14);

	for(int i = 0; i < 5000; i++) {
		motor1.send_dshot_value(0);
		delay(1);
	}
}


void Motor::loop() {
	// float pitchOut = pidPitch.compute(target.pitch, pitch);
	// float rollOut  = pidRoll.compute(target.roll, roll);
	// float yawOut   = pidYaw.compute(target.yaw, yaw);
	motor1.send_dshot_value(100);
}