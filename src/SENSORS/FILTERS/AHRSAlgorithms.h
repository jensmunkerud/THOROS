#ifndef _AHRSALOGRITHMS_H_
#define _AHRSALOGRITHMS_H_

#include <Arduino.h>
#include "Status.h"

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
							  float gz, float mx, float my, float mz,
							  float deltat);
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy,
							float gz, float mx, float my, float mz,
							float deltat);
void MadgwickQuaternionUpdateIMU(float ax, float ay, float az,
							float gx, float gy, float gz,
							float deltat);

void MahonyQuaternionUpdateIMU(float ax, float ay, float az,
							float gx, float gy, float gz,
							float deltat);
Attitude UpdateIMU(float ax, float ay, float az,
							float gx, float gy, float gz,
							float dt);
							
const float * getQ();
Attitude getAttitude();

#endif // _AHRSALOGRITHMS_H_
