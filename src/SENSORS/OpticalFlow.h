#pragma once
#include "MISC/Datatypes.h"
#include "Bitcraze_PMW3901.h"

constexpr int OPTICAL_FLOW_CS {4};
constexpr int OPTICAL_FLOW_SCK {18};
constexpr int OPTICAL_FLOW_MISO {19};
constexpr int OPTICAL_FLOW_MOSI {23};


class OpticalFlow{
public:
	OpticalFlow(Drone& drone);
	void begin();
	void loop();
	int16_t getDx() const;
	int16_t getDy() const;

private:
	Drone& drone;
	Bitcraze_PMW3901 flow;
	int16_t dx;
	int16_t dy;
	bool ready;
};