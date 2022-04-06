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
//#include <math.h>
#include <cmath>

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"

#include "Stepper.h"

#define TurnOnTimer5    TimerEnable(TIMER0_BASE, TIMER_A);  //(TIMSK5 |= (1 << OCIE5A))     // enable to create a stepper pulse
#define TurnOffTimer5   TimerDisable(TIMER0_BASE, TIMER_A);  //(TIMSK5 &= ~(1 << OCIE5A))

#define TurnOnTimer2    TimerEnable(TIMER1_BASE, TIMER_A);  //(TIMSK2 |= (1 << OCIE2A))     // time for pulse duration fix 50 us
#define TurnOffTimer2   TimerDisable(TIMER1_BASE, TIMER_A); //(TIMSK2 &= ~(1 << OCIE2A))



void Timer0IntHandler(void);
void Timer1IntHandler(void);

int64_t rti_counter_i64;

void StepperClass::init(std::vector<Segment>* SegmentQueue)
{
	this->SegmentQueue = SegmentQueue;

	//noInterrupts();
    IntMasterDisable();

/*
	// Reset register relate to Timer 5
	// Reset register relate
	TCCR5A = TCCR5B = TCNT5 = 0;
	// Set CTC mode to Timer 5
	TCCR5B |= (1 << WGM52);
	// Set prescaler 1 to Timer 5
	TCCR5B |= (1 << CS50);
	//Normal port operation, OCxA disconnected
	TCCR5A &= ~((1 << COM5A1) | (1 << COM5A0) | (1 << COM5B1) | (1 << COM5B0));
	

	// Setup for Timer 2
	// Reset register relate
	TCCR2A = TCCR2B = TCNT2 = 0;
	// Set CTC mode
	TCCR2A |= (1 << WGM21);
	// Set prescaler 1
	TCCR2B |= (1 << CS20);
	//Normal port operation, OC2A disconnected
	TCCR2A &= ~((1 << COM2A1) | (1 << COM2A0) | (1 << COM2B1) | (1 << COM2B0));
	COMPARE_VALUE_TIMER2 = 80; //5us
	                       4000 50x5us = 250us
*/

    //
    // Enable the peripherals used by this example.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);  //pulse interval dynamic
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);  //pulse lenth static

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the two 32-bit periodic timers.
    //
//    TimerConfigure(TIMER0_BASE,  TIMER_CFG_ONE_SHOT);
    TimerConfigure(TIMER0_BASE,  TIMER_CFG_PERIODIC);
//    TimerConfigure(TIMER1_BASE, TIMER_CFG_ONE_SHOT);
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);

    TimerLoadSet(TIMER0_BASE, TIMER_A, ROM_SysCtlClockGet() /  1000); // initially 1ms
    TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet() / 200000); // 1ms/200  = 5us

    TimerIntRegister(TIMER0_BASE, INT_TIMER0A, Timer0IntHandler);
    TimerIntRegister(TIMER1_BASE, INT_TIMER1A, Timer1IntHandler);

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	//interrupts();
    IntMasterEnable();

	CreateMotor();

	IsRunningHome = false;
	IsStoppedMoving = true;

	stepper_max_movtime = 10000; // hope its ~10 sec?

    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER0A); // this one
    IntEnable(INT_TIMER1A); // fixed: in ISR clear Timer0 instead Timer1
}

void StepperClass::CreateMotor()
{
	ThetaStepMotor[0].Name = THETA_1;
	setPinMode(THETA_1);

	ThetaStepMotor[1].Name = THETA_2;
	setPinMode(THETA_2);
	// TODO ^^^ this call is causing a hardware fault
	// Thomas says we can likely sidestep it with a ntive implementation once we have things more
	// figured out

	ThetaStepMotor[2].Name = THETA_3;
	setPinMode(THETA_3);

}

bool StepperClass::GetStateMotor()
{
	if(!IsStoppedMoving)
	    stepper_timeout++;
	if(stepper_timeout > stepper_max_movtime){
	    stepper_timeout=0;
	    IsStoppedMoving =1;  // force stepper to stop
	    SERIAL_PORT.println("Err_Timeout Stepper");
	}
	// org code
    return IsStoppedMoving;
}

void StepperClass::UpdateMoveSegment()
{
	if (SegmentQueue->size() == 0)
	{
		IsStoppedMoving = true;
		NumberTnterrupt = 0;
		Planner.NumberIntRoad = 0;
		Timer = 0;
		TurnOffTimer5; // move done cut timer ISR
		return;
	}	

	CurrentMoveSegment = SegmentQueue->operator[](0);
	SegmentQueue->erase(SegmentQueue->begin());

	for (std::ptrdiff_t i = 0; i < 3; i++) // for each joint
	{
		setMotorDirection(ThetaStepMotor[i].Name, CurrentMoveSegment.StepperArray[i].Direction);
		ThetaStepMotor[i].InterruptNumberNextStep = 0;
		if (CurrentMoveSegment.StepperArray[i].StepsToJump == 0)
		{
			ThetaStepMotor[i].NumberInterrupt2Step = STEP_NULL;
		}
		else
		{
			ThetaStepMotor[i].NumberInterrupt2Step = (float)CurrentMoveSegment.NumberINT / CurrentMoveSegment.StepperArray[i].StepsToJump;
		}
	}

	NumberTnterrupt = 0;
}
int seg=0;
void StepperClass::Running()
{	
	seg = SegmentQueue->size();
    if (SegmentQueue->size() == 0)
	{
		return;
	}

	CurrentMoveSegment = SegmentQueue->operator[](0);
	SegmentQueue->erase(SegmentQueue->begin());

	for (uint8_t i = 0; i < 3; i++) // for each joint
	{
		setMotorDirection(ThetaStepMotor[i].Name, CurrentMoveSegment.StepperArray[i].Direction);
		ThetaStepMotor[i].InterruptNumberNextStep = 0;

		if (CurrentMoveSegment.StepperArray[i].StepsToJump == 0)
		{
			ThetaStepMotor[i].NumberInterrupt2Step = STEP_NULL;
		}
		else
		{
			ThetaStepMotor[i].NumberInterrupt2Step = (float)CurrentMoveSegment.NumberINT / CurrentMoveSegment.StepperArray[i].StepsToJump;
		}
	}

	if (Data.End_Effector == USE_PRINTER)
	{
		WRITE(EXTRUSDER_DIRECTION_PIN, Planner.ExtrustionStepsDirection);
		ExtrustionStepMotor.InterruptNumberNextStep = 0;

		if (Planner.ExtrustionStepsToJump == 0)
		{
			ExtrustionStepMotor.NumberInterrupt2Step = STEP_NULL;
		}
		else
		{
			ExtrustionStepMotor.NumberInterrupt2Step = (float)Planner.NumberIntRoad / Planner.ExtrustionStepsToJump;
		}
	}

	CurrentCycle = Planner.BeginEndIntCycle;
	MaxCycle = Planner.BeginEndIntCycle;
	SetIntCycle(CurrentCycle);  // pulse spacing: running trj gen
	

	NumberTnterruptAtMinCycle = 0;
	TotalTnterruptNumber = 0;
	IsStoppedMoving = false;
	TurnOnTimer5; // start next step interval

    rti_counter_i64++; //tdu dummy instruction

}

void StepperClass::Homing()
{
	if (SegmentQueue->size() == 0)
	{
		IsStoppedMoving = true;
		IsRunningHome = 0;
		TurnOffTimer5; // homing done
		SegmentQueue->clear();
		return;
	}

	IsRunningHome++;

	CurrentMoveSegment = SegmentQueue->operator[](0);
	SegmentQueue->erase(SegmentQueue->begin());

	for (uint8_t i = 0; i < 3; i++) // for each joint
	{
		setMotorDirection(ThetaStepMotor[i].Name, CurrentMoveSegment.StepperArray[i].Direction);
	}

	if (IsRunningHome == 1 || IsRunningHome == 2)
	{
		CurrentCycle = Planner.HomingIntCycle;
	}
	if (IsRunningHome == 3)
	{
		CurrentCycle = Planner.HomingIntCycle * 8;
	}

	SetIntCycle(CurrentCycle);  // stepper interval update: homing call

	IsStoppedMoving = false;
	TurnOnTimer5;  //next interval to pulse

    rti_counter_i64++; //tdu dummy instruction

}

void StepperClass::Isr_Execute_Velocity()  // timer5_isr
{
	// Movinghome **********
	if (IsRunningHome > 0)
	{
		Isr_Execute_Movinghome();   // rather do homing than traj gen for point to point
		return;
	}
	//**********************

	NumberTnterrupt++;
	TotalTnterruptNumber++;

	//Execute accel

	if (TotalTnterruptNumber < Planner.AccelerationUntil)  // accel phase
	{
		Timer += CurrentCycle;
		CurrentCycle = MaxCycle - (Timer * Planner.a);
		if (CurrentCycle < INTERRUPT_CYCLE_MIN)
		{
			CurrentCycle = INTERRUPT_CYCLE_MIN;
		}
		else
		{
			NumberTnterruptAtMinCycle = TotalTnterruptNumber;
		}
	}
	else if (TotalTnterruptNumber > Planner.DecelerateAfter) // decel phase
	{
		if (TotalTnterruptNumber > Planner.NumberIntRoad - NumberTnterruptAtMinCycle)
		{
			Timer += CurrentCycle;
		}	
		CurrentCycle = MaxCycle + (Timer * Planner.a);
	}
	else  //max speed, no accel
	{
		Timer = 0;
		MaxCycle = CurrentCycle;
	}

	SetIntCycle(CurrentCycle);      // next pulse interval: exec_velocity ISR
	//Serial.println(CurrentCycle);

	//Checking jump step and Accumulated error
	for (uint8_t i = 0; i < 3; i++) // for each joint
	{
		if (ThetaStepMotor[i].InterruptNumberNextStep >= NumberTnterrupt)
		{
			continue;
		}
		ThetaStepMotor[i].InterruptNumberNextStep += ThetaStepMotor[i].NumberInterrupt2Step;
		if (CurrentMoveSegment.StepperArray[i].StepsToJump != 0)
		{
			CurrentMoveSegment.StepperArray[i].StepsToJump--;
			writePulsePin(ThetaStepMotor[i].Name, 1);
		}
	}

	if (Data.End_Effector == USE_PRINTER)
	{
		if (ExtrustionStepMotor.InterruptNumberNextStep < TotalTnterruptNumber)
		{
			ExtrustionStepMotor.InterruptNumberNextStep += ExtrustionStepMotor.NumberInterrupt2Step;
			if (Planner.ExtrustionStepsToJump != 0)
			{
				Planner.ExtrustionStepsToJump--;
				WRITE(EXTRUSDER_PULSE_PIN, HIGH);
			}
		}
	}

	TurnOnTimer2;  //make step of 50us
	
	if (CurrentMoveSegment.StepperArray[0].StepsToJump == 0 &&
	        CurrentMoveSegment.StepperArray[1].StepsToJump == 0 &&
	        CurrentMoveSegment.StepperArray[2].StepsToJump == 0)
	//if (NumberTnterrupt >= CurrentMoveSegment.NumberINT - 1)
	{
		UpdateMoveSegment();
	}
} //Isr_Execute_Velocity()

void StepperClass::Isr_Execute_Movinghome()
{
	EndStops.UpdateStates();
	for (uint8_t i = 0; i < 3; i++)
	{
		if (CurrentMoveSegment.StepperArray[i].StepsToJump > 1000)
		{
			if (EndStops.States(ThetaStepMotor[i].Name) == false)
			{
				writePulsePin(ThetaStepMotor[i].Name, 1);
			}
		}
		else
		{
			if (CurrentMoveSegment.StepperArray[i].StepsToJump > 0)
			{
				writePulsePin(ThetaStepMotor[i].Name, 1);
				CurrentMoveSegment.StepperArray[i].StepsToJump--;
			}
					
		}	
	}

	SetIntCycle(CurrentCycle);  // while homing move ISR_timer5
	TurnOnTimer2;
	if (IsRunningHome == 2)
	{
		for (uint8_t i = 0; i < 3; i++)
		{
			if (CurrentMoveSegment.StepperArray[i].StepsToJump > 0)
			{
				return;
			}		
		}
	}
	else if(IsRunningHome == 1 || IsRunningHome == 3)
	{
		if (EndStops.HomingCheck() == false)  //PC5, PC6, PC4 as input pull up so active LOW
		{
			return; // homing done!
		}
	}
	Homing();
    rti_counter_i64++; //tdu dummy instruction

}

//turning pulse pin low
void StepperClass::Isr_Turn_Pulse_Pin()  // time2_isr
{
	for (uint8_t i = 0; i < 3; i++) //for each joint
	{
		writePulsePin(ThetaStepMotor[i].Name, 0);  // turn off step pulse
	}

	if (Data.End_Effector == USE_PRINTER)
	{
		WRITE(EXTRUSDER_PULSE_PIN, LOW);  // if 3D printer
	}
	TurnOffTimer2; // clear so no immediate repeat of 50us
}

void StepperClass::SetIntCycle(float intCycle) // dynamic timer delay for pulse gen and traj gen update
{
	int compare_t0 = intCycle * (1 * 5) *16;  // 62.5us * 5 * 16

    TimerLoadSet(TIMER0_BASE, TIMER_A, compare_t0); //

}

void StepperClass::ClearMotorArray() {
	for (auto& motor : ThetaStepMotor) {
		motor.InterruptNumberNextStep = 0;
	}
}

void StepperClass::setPinMode(AXIS axisname)
{
	switch (axisname)
	{
	case THETA_1:
		pinMode(THETA1_ENABLE_PIN, OUTPUT);
		pinMode(THETA1_PULSE_PIN, OUTPUT);      // ?
		pinMode(THETA1_DIRECTION_PIN, OUTPUT);  // ?

		WRITE(THETA1_ENABLE_PIN, LOW);  // ?
		WRITE(THETA1_PULSE_PIN, LOW);
		WRITE(THETA1_DIRECTION_PIN, DECREASE);
		break;
	case THETA_2:
		pinMode(THETA2_ENABLE_PIN, OUTPUT); // 8
		pinMode(THETA2_PULSE_PIN, OUTPUT);
		pinMode(THETA2_DIRECTION_PIN, OUTPUT);

		WRITE(THETA2_ENABLE_PIN, LOW);
		WRITE(THETA2_PULSE_PIN, LOW);
		WRITE(THETA2_DIRECTION_PIN, DECREASE);
		break;
	case THETA_3:
		pinMode(THETA3_ENABLE_PIN, OUTPUT);
		pinMode(THETA3_PULSE_PIN, OUTPUT);
		pinMode(THETA3_DIRECTION_PIN, OUTPUT);

		WRITE(THETA3_ENABLE_PIN, LOW);
		WRITE(THETA3_PULSE_PIN, LOW);
		WRITE(THETA3_DIRECTION_PIN, DECREASE);
		break;

#ifdef USING_STEPER_FOR_AXIS4
	case AXIS_4:
		pinMode(AXIS_4_ENABLE_PIN, OUTPUT);
		pinMode(AXIS_4_PULSE_PIN, OUTPUT);
		pinMode(AXIS_4_DIRECTION_PIN, OUTPUT);

		WRITE(AXIS_4_ENABLE_PIN, LOW);
		WRITE(AXIS_4_PULSE_PIN, LOW);
		WRITE(AXIS_4_DIRECTION_PIN, DECREASE);
		break;
#endif // USING_STEPER_FOR_AXIS4

#ifdef USING_STEPER_FOR_AXIS5
	case AXIS_5:
		pinMode(AXIS_5_ENABLE_PIN, OUTPUT);
		pinMode(AXIS_5_PULSE_PIN, OUTPUT);
		pinMode(AXIS_5_DIRECTION_PIN, OUTPUT);

		WRITE(AXIS_5_ENABLE_PIN, LOW);
		WRITE(AXIS_5_PULSE_PIN, LOW);
		WRITE(AXIS_5_DIRECTION_PIN, DECREASE);
		break;
#endif // USING_STEPER_FOR_AXIS5

	default:
		break;
	}
}

void StepperClass::setMotorDirection(AXIS axisname, bool variation)
{
	switch (axisname)
	{
	case THETA_1:
		WRITE(THETA1_DIRECTION_PIN, variation);
		break;
	case THETA_2:
		WRITE(THETA2_DIRECTION_PIN, variation);
		break;
	case THETA_3:
		WRITE(THETA3_DIRECTION_PIN, variation);
		break;

#ifdef USING_STEPER_FOR_AXIS4
	case AXIS_4:
		WRITE(AXIS_4_DIRECTION_PIN, variation);
		break;
#endif // USING_STEPER_FOR_AXIS4

#ifdef USING_STEPER_FOR_AXIS5
	case AXIS_5:
		WRITE(AXIS_5_DIRECTION_PIN, variation);
		break;
#endif // USING_STEPER_FOR_AXIS5

	default:
		break;
	}
}

void StepperClass::writePulsePin(AXIS axisname, bool ison)
{
	switch (axisname)
	{
	case THETA_1:
		WRITE(THETA1_PULSE_PIN, ison);
		break;
	case THETA_2:
		WRITE(THETA2_PULSE_PIN, ison);
		break;
	case THETA_3:
		WRITE(THETA3_PULSE_PIN, ison);
		break;

#ifdef USING_STEPER_FOR_AXIS4
	case AXIS_4:
		WRITE(AXIS_4_PULSE_PIN, ison);
		break;
#endif // USING_STEPER_FOR_AXIS4

#ifdef USING_STEPER_FOR_AXIS5
	case AXIS_5:
		WRITE(AXIS_5_PULSE_PIN, ison);
		break;
#endif // USING_STEPER_FOR_AXIS5

	default:
		break;
	}
}

void StepperClass::DisanableStepper()
{
	WRITE(THETA1_ENABLE_PIN, HIGH);
	WRITE(THETA2_ENABLE_PIN, HIGH);
	WRITE(THETA3_ENABLE_PIN, HIGH);
	if (Data.End_Effector == USE_PRINTER)
	{
		WRITE(EXTRUSDER_ENABLE_PIN, HIGH);
	}	
}

void StepperClass::EnableStepper()
{
	WRITE(THETA1_ENABLE_PIN, LOW);
	WRITE(THETA2_ENABLE_PIN, LOW);
	WRITE(THETA3_ENABLE_PIN, LOW);
	if (Data.End_Effector == USE_PRINTER)
	{
		WRITE(EXTRUSDER_ENABLE_PIN, LOW);
	}
}

void Timer0IntHandler()  // 1ms
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);


    Stepper.Isr_Execute_Velocity();


    //MAP_IntMasterEnable();

    rti_counter_i64++;
}

void Timer1IntHandler(void)  // 1ms
{
    //
    // Clear the timer interrupt.
    //
    ROM_TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    Stepper.Isr_Turn_Pulse_Pin();
}

/*
ISR(TIMER5_COMPA_vect)
{
	Stepper.Isr_Execute_Velocity();
}

ISR(TIMER2_COMPA_vect)
{
	Stepper.Isr_Turn_Pulse_Pin();
}
*/

StepperClass Stepper;   //
