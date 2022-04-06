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

#include "config.h"
#include "Constants.h"

//tdu #include "fastio.h"

#include "Storage.h"
//#include <ArduinoSTL/src/ArduinoSTL.h> //tdu from Arduina ext-lib
//#include "tdu/math_tdu.h"
#include "deltax_vector.h"
#include "GCodeReceiver.h"
#include "GCodeExecute.h"
#include "Planner.h"
#include "Motion.h"
#include "Tool.h"
#include "EndStops.h"
#include "EndEffector.h"
#include "MultiServo.h"
#include "Temperature.h"
#include "ConnectionState.h"

using namespace std;

GCodeReceiverClass GcodeReceiver;
GCodeExecuteClass GcodeExecute;

vector<String> GCodeQueue;
vector<Segment> SegmentQueue;

void setup_t() {
	SERIAL_PORT.begin(BAUDRATE);
	Data.init();
	Storage.init();

	DeltaKinematics.init();
	EndEffector.init();
	EndStops.init();
	Planner.init(&SegmentQueue);
	Stepper.init(&SegmentQueue); // run 2 IRQ 5us and
	MultiServo.init();
	ConnectionState.Init();
	Motion.init();
	Temperature.init();

	GcodeReceiver.Init(&GCodeQueue, &SERIAL_PORT, BAUDRATE);
	GcodeExecute.Init(&GCodeQueue);

	Serial.println("Init Success!");
	//Motion.G28();
	Motion.G0(1.2F,Data.CurrentPoint.Y,Data.CurrentPoint.Z,0.0F); // Z <> 0.0 ! otherwise singularity in IK
}

void loop_t() {
	GcodeReceiver.Execute();
	GcodeExecute.Run();
	ConnectionState.Execute();
	Temperature.ISR_EXECUTE();
}
