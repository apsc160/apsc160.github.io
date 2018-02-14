/*
 *   Author:           Jessica Programmer
 *   Student Number:   12345678
 *   Lab Section:      L1X
 *   Date:             October 13, 2017
 *
 *   Purpose:          This program loads the DAQ simulator
 */

#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>
#include <stdlib.h>

#include <DAQlib.h>  /* DAQ library and simulator */

/* simulator 1 has 3 LEDs and 2 switches */
#define DAQ_SIMULATOR_1 1
#define SWITCH0  0
#define LED0     0

/* main work function */
void workFunction(void);

int main() {

  if (setupDAQ(DAQ_SIMULATOR_1)) {
    workFunction();
  } else {
    printf("Failed to initialize DAQ\n");
  }

  return 0;
}

/* main work function */
void workFunction() {

  /* continue looping while the device is powered on */
  while (continueSuperLoop()) {
    /* do something interesting here */
    int val0 = digitalRead(SWITCH0);
    digitalWrite(LED0, val0);
  }

}
