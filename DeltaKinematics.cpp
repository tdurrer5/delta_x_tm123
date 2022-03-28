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

#include "DeltaKinematics.h"

void DeltaKinematicsClass::init()
{
	_y0_ = 0.5f * tan30 * Data.RD_E;
	_y1_ = -0.5f * tan30 * Data.RD_F;
	RD_RF_Pow2 = Data.RD_RF*Data.RD_RF;
	RD_RE_Pow2 = Data.RD_RE*Data.RD_RE;
}

bool DeltaKinematicsClass::ForwardKinematicsCalculations(Angle angleposition, Point &point)
{
	const float theta1 = DEG_TO_RAD * angleposition.Theta1;
	const float theta2 = DEG_TO_RAD * angleposition.Theta2;
	const float theta3 = DEG_TO_RAD * angleposition.Theta3;

	const float t = (Data.RD_F - Data.RD_E) * tan30 / 2.0f;

	const float y1 = -(t + Data.RD_RF * cos(theta1));
	const float z1 = -Data.RD_RF * sin(theta1);

	const float y2 = (t + Data.RD_RF * cos(theta2)) * sin30;
	const float x2 = y2 * tan60;
	const float z2 = -Data.RD_RF * sin(theta2);

	const float y3 = (t + Data.RD_RF * cos(theta3)) * sin30;
	const float x3 = -y3 * tan60;
	const float z3 = -Data.RD_RF * sin(theta3);

	const float dnm = (y2 - y1) * x3 - (y3 - y1) * x2;

	const float w1 = hypot(y1, z1);
	const float w2 = hypot(x2, y2) + z2 * z2;
	const float w3 = hypot(x3, y3) + z3 * z3;

	// x = (a1*z + b1)/dnm
	const float a1 = (z2 - z1)*(y3 - y1) - (z3 - z1)*(y2 - y1);
	const float b1 = -((w2 - w1)*(y3 - y1) - (w3 - w1)*(y2 - y1)) / 2.0f;

	// y = (a2*z + b2)/dnm;
	const float a2 = -(z2 - z1) * x3 + (z3 - z1) * x2;
	const float b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0f;

	// a*z^2 + b*z + c = 0
	const float a = hypot(a1, a2) + dnm * dnm;
	const float b = 2 * (a1*b1 + a2 * (b2 - y1 * dnm) - z1 * dnm*dnm);
	const float c = hypot(b2 - y1 * dnm, dnm) * (z1*z1 - Data.RD_RE * Data.RD_RE);

	const float discriminant = b * b - 4.0f * a * c;
	if (discriminant < 0) {
	    return false;
	}

	point.Z = -0.5f * (b + sqrt(discriminant)) / a;
	point.X = (a1 * point.Z + b1) / dnm;
	point.Y = (a2 * point.Z + b2) / dnm;

	return true;
}

bool DeltaKinematicsClass::AngleThetaCalculations(float x0, float y0, float z0, float &theta) const
{
	//float y1 = -0.5 * tan30 * Data.RD_F;
	//y0 -= 0.5 * tan30 * Data.RD_E;
	float y1 = _y1_;
	y0 -= _y0_;

	// z = a + b*y
	float a = (hypot(x0, y0) + z0 * z0 + RD_RF_Pow2 - RD_RE_Pow2 - y1 * y1) / (2.0f * z0);
	float b = (y1 - y0) / z0;

	float discriminant = -(a + b * y1)*(a + b * y1) + b*b*RD_RF_Pow2 + RD_RF_Pow2;

	if (discriminant < 0.0f) {
	    return false;
	}

	float yj = (y1 - a * b - sqrt(discriminant)) / (b * b + 1.0f);
	float zj = a + b * yj;

	theta = atan(-zj / (y1 - yj)) * RAD_TO_DEG + ((yj > y1) ? 180.0f : 0.0f);

	return true;
}

bool DeltaKinematicsClass::InverseKinematicsCalculations(Point point, Angle &angleposition)
{	
	if (!AngleThetaCalculations(point.X, point.Y, point.Z, angleposition.Theta1))
	{
		return false;
	}

	{
	    const auto x = point.X * cos120 + point.Y * sin120;
	    const auto y = point.Y * cos120 - point.X * sin120;
        if (!AngleThetaCalculations(x, y, point.Z, angleposition.Theta2))
        {
            return false;
        }
	}

    const auto x = point.X * cos120 - point.Y * sin120;
    const auto y = point.Y * cos120 + point.X * sin120;
	return AngleThetaCalculations(x, y, point.Z, angleposition.Theta3);
}

DeltaKinematicsClass DeltaKinematics;

