#pragma once

#include <math.h>
#include "Datatypes.h"

namespace Filters {

inline float LowPass(float current, float target, float alpha) {
	if (!isfinite(current)) {
		return target;
	}

	if (alpha < 0.0f) {
		alpha = 0.0f;
	} else if (alpha > 1.0f) {
		alpha = 1.0f;
	}
	
	float filtered = current + alpha * (target - current);
	float lo = (current < target) ? current : target;
	float hi = (current < target) ? target : current;
	if (filtered < lo) {
		return lo;
	}
	if (filtered > hi) {
		return hi;
	}
	return filtered;
}

inline Vec3 LowPass(const Vec3& previous, const Vec3& sample, float alpha) {
	if (!isfinite(previous.x) || !isfinite(previous.y) || !isfinite(previous.z)) {
		return sample;
	}

	return {
		LowPass(previous.x, sample.x, alpha),
		LowPass(previous.y, sample.y, alpha),
		LowPass(previous.z, sample.z, alpha)
	};
}

} // namespace Filters