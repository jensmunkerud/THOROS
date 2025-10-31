#include "Motor.h"
#include "Servo.h"

Motor::Motor(MovementController& mc, Status& s) : movementController{mc}, status{s} {
	esc1.attach(MOTOR1);  // example pins
	esc2.attach(MOTOR2);
	esc3.attach(MOTOR3);
	esc4.attach(MOTOR4);
	esc1.writeMicroseconds(1000);
	esc2.writeMicroseconds(1000);
	esc3.writeMicroseconds(1000);
	esc4.writeMicroseconds(1000);
	delay(2000); // Wait for ESCs to arm
}


void Motor::loop() {
	// float pitchOut = pidPitch.compute(target.pitch, pitch);
	// float rollOut  = pidRoll.compute(target.roll, roll);
	// float yawOut   = pidYaw.compute(target.yaw, yaw);

	int speed = map(status.speed, 0, 255, 1000, 2000); // convert to microseconds
	esc1.writeMicroseconds(speed);
	esc2.writeMicroseconds(speed);
	esc3.writeMicroseconds(speed);
	esc4.writeMicroseconds(speed);
}