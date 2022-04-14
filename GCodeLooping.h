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
// inheret from GCodeReceiver.h

#ifndef GCODELOOPING_h_
#define GCODELOOPING_h_

#if defined(ARDUINO) && ARDUINO >= 100
	#include <Arduino.h>
#else
	#include "WProgram.h"
#endif

//tdu #include <ArduinoSTL.h>
#include "deltax_vector.h"
#include "ConnectionState.h"
#include "WifiSettings.h"
#include "Constants.h"
#include "Temperature.h"

class GCodeLoopingClass {
public:
	//void Init(std::vector<String>* gCodeQueue, HardwareSerial* serial, unsigned long baudrate);
	void Init(std::vector<String>* gCodeQueue, int *pEnab_exec, char * pch_gcodebuff);
	//void Init(std::vector<String>* gCodeQueue); //, int *pEnab_exec, char * pch_gcodebuff);

	void Execute();

//	HardwareSerial* ReceiveSerial;
//	unsigned long Baudrate;

	std::vector<String>* GCodeQueue;

private:
	
	String receiveString;
	bool isStringComplete;
	int rd_inx_str;
	int *pEnable_exec;      //reference to global
	int process_oneLoop;
	char * pch_gcodeloop;   //reference to global
	char inChar;
};

#endif

