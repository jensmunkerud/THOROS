#include "Complementary.h"

ComplementaryFilter::ComplementaryFilter(float alpha) : alpha(alpha) {}

Attitude ComplementaryFilter::update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
	attitude.pitch = alpha * (attitude.pitch + gx * dt) + (1-alpha) * ax;
	attitude.roll = alpha * (attitude.roll + gy * dt) + (1-alpha) * ay;
	return attitude;
}