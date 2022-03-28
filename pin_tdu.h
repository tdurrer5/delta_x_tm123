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
#pragma once

#ifdef BOARD_RAMPS_14
//pin theta
#define THETA1_PULSE_PIN 39 //tdu 54
#define THETA2_PULSE_PIN 40 //tdu 60
#define THETA3_PULSE_PIN 30 //tdu 46
#define THETA4_PULSE_PIN 15 //tdu 46

#define THETA1_DIRECTION_PIN 32 //55
#define THETA2_DIRECTION_PIN 33 //61
#define THETA3_DIRECTION_PIN 34 //48
#define THETA4_DIRECTION_PIN 35 //48

#define THETA1_ENABLE_PIN 38
#define THETA2_ENABLE_PIN 36 //56
#define THETA3_ENABLE_PIN 37 //62
#define THETA4_ENABLE_PIN 27 //62

#define THETA1_ENDSTOP_PIN 3
#define THETA2_ENDSTOP_PIN 14
#define THETA3_ENDSTOP_PIN 18

#define ENDSTOP_FOR_CHECK_Z_AREA_PIN 18

extern int dummy_IO;

//pin axis
#ifdef USING_SERVO_FOR_AXIS4
 #define AXIS_4_SERVO_PIN 11
#else
#ifdef USING_STEPER_FOR_AXIS4
 #define AXIS_4_PULSE_PIN       26
 #define AXIS_4_DIRECTION_PIN   28
 #define AXIS_4_ENABLE_PIN      24
 #define AXIS_4_ENDSTOP_PIN     2
#endif // USING_STEPER_FOR_AXIS4
#endif // USING_SERVO_FOR_AXIS4


#ifdef USING_SERVO_FOR_AXIS5
 #define AXIS_5_SERVO_PIN 11
#else
#ifdef USING_STEPER_FOR_AXIS5
 #define AXIS_5_PULSE_PIN 26
 #define AXIS_5_DIRECTION_PIN 28
 #define AXIS_5_ENABLE_PIN 24
 #define AXIS_5_ENDSTOP_PIN 2
#endif // USING_STEPER_FOR_AXIS5
#endif // USING_SERVO_FOR_AXIS5


#define CHANNEL_A_ENCODER_PIN	20
#define CHANNEL_B_ENCODER_PIN	21

#define VACCUM_PIN      10

#define CLIP_SERVO_PIN 6

#define SPINDLE_LASER_ENABLE_PIN  10   // Pin should have a pullup/pulldown!
#define SPINDLE_LASER_PWM_PIN     5   // MUST BE HARDWARE PWM

#define CUSTOM_PWM_PIN	4
#define CUSTOM_DIR_PIN	7   //16

#define LED_R_PIN	30 //25
#define LED_G_PIN	39 //23
#define LED_B_PIN	40 //17

#define EXTRUSDER_PULSE_PIN     26
#define EXTRUSDER_DIRECTION_PIN 28
#define EXTRUSDER_ENABLE_PIN    24

//thermistor
//tdu #define THERMISTOR_PIN	PIN_A13
#define THERMISTOR_PIN	A0  //tdu from pins_energia.h
#define HEATER_PIN	8


//########################################################################################################
// ######## tdu added till EOF

#ifndef MASK
/// MASKING- returns \f$2^PIN\f$
#define MASK(PIN)  (1 << PIN)
#endif

/// Read a pin
#define _READ(IO) dummy_IO //((bool)(DIO ## IO ## _RPORT & MASK(DIO ## IO ## _PIN)))
/// write to a pin
// On some boards pins > 0x100 are used. These are not converted to atomic actions. An critical section is needed.

#define _WRITE_NC(IO, v)  do {dummy_IO = v; } while (0) //do { if (v) {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); } else {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }; } while (0)

#define _WRITE_C(IO, v)  do {dummy_IO = v; } while (0) //do { if (v) {DIO ##  IO ## _WPORT |= MASK(DIO ## IO ## _PIN); } else  {DIO ##  IO ## _WPORT &= ~MASK(DIO ## IO ## _PIN); }; } while (0)

#define _WRITE(IO, v)  do {dummy_IO = v; } while (0) //do {  if (&(DIO ##  IO ## _RPORT) >= (uint8_t *)0x100) {_WRITE_C(IO, v); } else {_WRITE_NC(IO, v); }; } while (0)

/// toggle a pin
#define _TOGGLE(IO)  do {DIO ##  IO ## _RPORT = MASK(DIO ## IO ## _PIN); } while (0)

/// set pin as input
#define _SET_INPUT(IO) do {DIO ##  IO ## _DDR &= ~MASK(DIO ## IO ## _PIN); } while (0)
/// set pin as output
#define _SET_OUTPUT(IO) do {DIO ##  IO ## _DDR |=  MASK(DIO ## IO ## _PIN); } while (0)

/// check if pin is an input
#define _GET_INPUT(IO)  ((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) == 0)
/// check if pin is an output
#define _GET_OUTPUT(IO)  ((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) != 0)

/// check if pin is an timer
#define _GET_TIMER(IO)  ((DIO ## IO ## _PWM)

//  why double up on these macros? see http://gcc.gnu.org/onlinedocs/cpp/Stringification.html

/// Read a pin wrapper
#define READ(IO)  _READ(IO)
/// Write to a pin wrapper
#define WRITE(IO, v)  _WRITE(IO, v)

/// toggle a pin wrapper
#define TOGGLE(IO)  _TOGGLE(IO)

/// set pin as input wrapper
#define SET_INPUT(IO)  _SET_INPUT(IO)
/// set pin as output wrapper
#define SET_OUTPUT(IO)  _SET_OUTPUT(IO)

/// check if pin is an input wrapper
#define GET_INPUT(IO)  _GET_INPUT(IO)
/// check if pin is an output wrapper
#define GET_OUTPUT(IO)  _GET_OUTPUT(IO)

/// check if pin is an timer wrapper
#define GET_TIMER(IO)  _GET_TIMER(IO)


#endif // BOARD_RAMPS_14


