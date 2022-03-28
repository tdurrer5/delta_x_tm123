/*
 * timer_init.cpp
 *
 *  Created on: Mar 8, 2022
 *      Author: tdurrer
 */

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
#include "driverlib/uart.h"



void timer_init(void){

//    ConfigureUART();

//    UARTprintf("\033[2JTimers example\n");
//    UARTprintf("T1: 0  T2: 0");

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
//tdu    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pins for the LED (PF1 & PF2).
    //
//tdu    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_1);


    //
    // Enable the peripherals used by this example.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
//    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);

    //
    // Enable processor interrupts.
    //
    IntMasterEnable();

    //
    // Configure the two 32-bit periodic timers.
    //
    TimerConfigure(TIMER2_BASE,  TIMER_CFG_ONE_SHOT);
    //TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, ROM_SysCtlClockGet());
    //TimerLoadSet(TIMER1_BASE, TIMER_A, ROM_SysCtlClockGet() / 2);

    //
    // Setup the interrupts for the timer timeouts.
    //
    IntEnable(INT_TIMER2A);
    //IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    //TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Enable the timers.
    //
    TimerEnable(TIMER2_BASE, TIMER_A);
    //TimerEnable(TIMER1_BASE, TIMER_A);

    //
    // Loop forever while the timers run.
    //
}



/*
        MAP_GPIOPinTypeTimer(DAC_LDAC_GPIO_BASE, DAC_LDAC_FORCER_PIN);
        MAP_GPIOPinConfigure(DAC_LDAC_FORCER_PIN_CONFIG);
        MAP_SysCtlPeripheralEnable(DAC_LDAC_FORCER_TIMER_PERIPH);
        MAP_TimerConfigure(DAC_LDAC_FORCER_TIMER_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_B_ONE_SHOT | TIMER_CFG_B_ACT_CLRSETTO);
        MAP_TimerLoadSet(DAC_LDAC_FORCER_TIMER_BASE, DAC_LDAC_FORCER_TIMER, pulseWidth);
*/


