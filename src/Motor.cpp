#include "Motor.h"

Motor::Motor(MovementController& mc, Status& s) : movementController{mc}, status{s} {
}


void Motor::loop() {
	// float pitchOut = pidPitch.compute(target.pitch, pitch);
	// float rollOut  = pidRoll.compute(target.roll, roll);
	// float yawOut   = pidYaw.compute(target.yaw, yaw);

}