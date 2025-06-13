#pragma once
#include <Arduino_LSM6DSOX.h>
#include "types.h"

class Stabilizer {
	public:
	Stabilizer();
	pos getAlignment();

	private:
	const int sizeFactor = 67;
	float Ax, Ay, Az;
	float Gx, Gy, Gz;
	float roll, pitch, yaw;
};