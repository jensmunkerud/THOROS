#include "Motor.h"

Motor::Motor(MovementController& mc, Status& s) : movementController{mc}, status{s}, 
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


void Motor::loop() {
	if (status.RFD900 != 1) {
		motor1.send_dshot_value(0);
		motor2.send_dshot_value(0);
		motor3.send_dshot_value(0);
		motor4.send_dshot_value(0);
		return;
	}
	// float pitchOut = pidPitch.compute(target.pitch, pitch);
	// float rollOut  = pidRoll.compute(target.roll, roll);
	// float yawOut   = pidYaw.compute(target.yaw, yaw);
	motor1.send_dshot_value(max(0, movementController.isToggled * MINIMUM_MOTOR_SPEED + status.speed));
	motor2.send_dshot_value(max(0, movementController.isToggled * MINIMUM_MOTOR_SPEED + status.speed));
	motor3.send_dshot_value(max(0, movementController.isToggled * MINIMUM_MOTOR_SPEED + status.speed));
	motor4.send_dshot_value(max(0, movementController.isToggled * MINIMUM_MOTOR_SPEED + status.speed));
	// Serial.print("WROTE ");
	// Serial.print(max(0, MINIMUM_MOTOR_SPEED + status.speed));
	// Serial.print(" TO MOTORS ON CORE: ");
	// Serial.println(xPortGetCoreID());
}