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

#include "MultiServo.h"

void MultiServoClass::init()
{
//tdu	CreateServo(AXIS_4, AXIS_4_SERVO_PIN);
	CreateServo(CLAMP, CLIP_SERVO_PIN);
}

void MultiServoClass::CreateServo(AXIS axisname, uint8_t pin)
{
	Servo_t buffer;

	buffer.lastPosition = 0;
	buffer.Desired = 0;
	buffer.servo.attach(pin);
	buffer.servo.write(90);
	buffer.Name = axisname;

	AddServo(buffer);
}

void MultiServoClass::AddAngle(AXIS axisname, uint8_t angle) {
	for (auto& servo : servos) {
		if (servo.Name == axisname) {
		    servo.Desired = angle;
            break;
		}
	}
}

void MultiServoClass::Running() {
	for (auto& servo : servos) {
		if (servo.lastPosition != servo.Desired) {
		    servo.lastPosition = servo.Desired;
		    servo.servo.write(servo.lastPosition);
		}
	}
}

void MultiServoClass::AddServo(Servo_t servo) {
	servos.push_back(servo);
}

MultiServoClass MultiServo;

