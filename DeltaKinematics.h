/**
 * Delta X Firmware
 * Copyright (c) 2020 DeltaXFirmware [https://github.com/deltaxrobot/Delta-X-Firmware]
 *
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
// DeltaKinematics.h

#ifndef DELTAKINEMATICS_h_
#define DELTAKINEMATICS_h_

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include "WProgram.h"
#endif

#include "Constants.h"

//trigonometric constants
const double tan30 = 1.0 / sqrt(3.0);
const double tan30x05 = 0.5 / sqrt(3.0);
const double tan60 = sqrt(3.0);
constexpr double sin30 = 0.5;
const double cos30 = sqrt(3.0) / 2.0;
constexpr double cos120 = -0.5;
const double sin120 = sqrt(3.0) / 2.0;

class DeltaKinematicsClass {
public:
	void init();
	bool ForwardKinematicsCalculations(Angle angleposition, Point &point);
	bool InverseKinematicsCalculations(Point point, Angle &angleposition);

private:
	bool AngleThetaCalculations(float x0, float y0, float z0, float &theta) const;
	float _y0_;
	float _y1_;
	float RD_RF_Pow2;
	float RD_RE_Pow2;
};

extern DeltaKinematicsClass DeltaKinematics;

#endif

