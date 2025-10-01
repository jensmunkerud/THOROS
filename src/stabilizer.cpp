// #include "stabilizer.h"

// Stabilizer::Stabilizer() {
// 	if (!IMU.begin()) {
// 		Serial.println("Failed to initialize IMU!");
// 		while (1);
// 	}
// }

// pos Stabilizer::getAlignment() {
// 	if (IMU.gyroscopeAvailable()) {
// 		IMU.readGyroscope(Gx, Gy, Gz);

// 		// Absolute orientation:
// 		roll += Gx / sizeFactor;
// 		pitch += Gy / sizeFactor;
// 		yaw += Gz / sizeFactor;
// 		Serial.print("Roll: ");
// 		Serial.print(roll);
// 		Serial.print("\t\t Pitch: ");
// 		Serial.print(pitch);
// 		Serial.print("\t\t Yaw: ");
// 		Serial.println(yaw);
// 	}
// 	pos p = {0, 0, 0};
// 	return p;
// }