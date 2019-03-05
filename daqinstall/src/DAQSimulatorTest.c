/*
 *   Author:           Jessica Programmer
 *   Student Number:   12345678
 *   Lab Section:      L1X
 *   Date:             October 13, 2017
 *
 *   Purpose:          This program loads the DAQ simulator and runs
 *                     a simple program involving a switch and LED.
 */

#define _CRT_SECURE_NO_WARNINGS

/* headers */
#include <stdio.h>
#include <DAQlib.h>   /* DAQ library and simulator */

/* symbolic constants */
#define TRUE     1
#define FALSE    0
#define ON       1
#define OFF      0

/* simulator 1 has 3 LEDs and 2 switches */
#define DAQ_SIMULATOR 1
#define SWITCH0  0
#define LED0     0

/* work function */
void runSwitches(void);

int main(void) {

	printf("Example Switch and LED Program\n");

	if (setupDAQ(DAQ_SIMULATOR) == TRUE) {
		runSwitches();
	} else {
		printf("ERROR: failed to initialize DAQ\n");
	}

	return 0;
}

/* work function */
void runSwitches() {

	/* continue looping while the device is powered on */
	while (continueSuperLoop()) {
		
		/* check switch and turn LED ON/OFF */
		if (digitalRead(SWITCH0) == ON) {
			digitalWrite(LED0, ON);
		} else {
			digitalWrite(LED0, OFF);
		}
		
	}

}

