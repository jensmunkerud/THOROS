#include "OpticalFlow.h"

OpticalFlow::OpticalFlow(Drone& drone) :
	drone{drone},
	flow{OPTICAL_FLOW_CS},
	dx{0},
	dy{0},
	ready{false}
{}

void OpticalFlow::begin() {
	ready = flow.begin();
	{
		DroneLockGuard droneLock(drone);
		drone.OPTICAL_OK = ready;
	}
}

void OpticalFlow::loop() {
	if (!ready) {
		return;
	}

	flow.readMotionCount(&dx, &dy);
}

int16_t OpticalFlow::getDx() const {
	return dx;
}

int16_t OpticalFlow::getDy() const {
	return dy;
}