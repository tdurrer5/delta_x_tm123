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
// Stepper.h

#ifndef _STEPPER_h
#define _STEPPER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include "WProgram.h"
#endif

//tdu #include <ArduinoSTL.h>
#include "deltax_vector.h"

#include <avr/interrupt.h>
#include "config.h"
#include "Constants.h"
#include "enum.h"

#include "pin_tdu.h"
//tdu #include "fastio.h"

#include "EndStops.h"
#include "Planner.h"

#define COMPARE_VALUE_TIMER5 OCR5A
#define COMPARE_VALUE_TIMER2 OCR2A


#ifdef SMOOTHSTEP

#endif // SMOOTHSTEP

#if SMOOTHSTEP == 0
#define SmoothStep(x) (x)
#endif

#if SMOOTHSTEP == 1
#define SmoothStep(x) (x*x*(3 - 2*x))
#endif

#if SMOOTHSTEP == 2
#define SmoothStep(x) (x*x*x*(10 + x*(6*x - 15)))
#endif

#if SMOOTHSTEP == 3
#define SmoothStep(x) (x*x*x*x*(35 + x*(x*(70 - 20*x) - 84)))
#endif

typedef struct
{
	float InterruptNumberNextStep;
	AXIS Name;
	float NumberInterrupt2Step;
}Motor;

class StepperClass
{
 protected:

 public:
	void init(std::vector<Segment>* SegmentQueue);
	void CreateMotor();

	bool GetStateMotor();

	void Running();
	void Homing();

	void Isr_Execute_Velocity();
	void Isr_Execute_Movinghome();
	void Isr_Turn_Pulse_Pin();

	void DisanableStepper();
	void EnableStepper();

	std::vector<Segment>* SegmentQueue;

 private:
	 Motor ThetaStepMotor[3];
	 Motor ExtrustionStepMotor;

	 Segment CurrentMoveSegment;

	 uint32_t	IsStoppedMoving;
	 uint32_t   stepper_timeout;
	 uint32_t   stepper_max_movtime;

	 float Timer;
	 uint32_t NumberTnterrupt;
	 uint32_t TotalTnterruptNumber;
	 uint32_t NumberTnterruptAtMinCycle;

	 uint32_t IsRunningHome;
	 uint32_t home_bypass;

	 float CurrentCycle;
	 float MaxCycle;

	 void setPinMode(AXIS axisname);
	 void UpdateMoveSegment();
	 void SetIntCycle(float intCycle);
	 void ClearMotorArray();

	 void setMotorDirection(AXIS axisname, bool variation);
	 void writePulsePin(AXIS axisname, bool ison);
};

extern StepperClass Stepper;

#endif

