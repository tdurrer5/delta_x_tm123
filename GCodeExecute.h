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
// GCodeExecute.h

#ifndef GCODEEXECUTE_h_
#define GCODEEXECUTE_h_

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include "WProgram.h"
#endif

//tdu #include <ArduinoSTL.h>
#include "deltax_vector.h"
#include "enum.h"
#include "Motion.h"
#include "Control.h"

#define NULL_NUMBER 98765

struct KeyValue {
	char Key;
	float Value;
};

struct NoArgumentFunction {
	String Code;
	void(*Function)();
};

class GCodeExecuteClass {
public:
	void Init(std::vector<String>*);
	void Run();
	void WhenFinishMove();

	std::vector<String>* GCodeQueue;
	bool IsRunning;
	int queue_size; //tdu
	int Gcode_cnt;  //tdu

	float X, Y, Z, E, S, A, I, J, F, P, R, Q, W;
private:
	std::vector<KeyValue> getKeyValues(String code);
	void checkAndRunFunction(KeyValue keyValue);
	void ResetValue();
	std::vector<NoArgumentFunction> NoArgumentFunctions;
};


#endif

