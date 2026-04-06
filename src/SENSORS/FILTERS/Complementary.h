#pragma once

#include <math.h>
#include "Status.h"

class ComplementaryFilter {
public:
	ComplementaryFilter(float alpha = 0.98);

	Attitude update(float gx, float gy, float gz,
				float ax, float ay, float az,
				float dt);

private:
	void normalize(float &x, float &y, float &z);
	Attitude attitude{};
	float alpha;
};