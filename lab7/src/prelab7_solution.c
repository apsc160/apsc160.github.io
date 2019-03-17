/*
 * Sample solution for prelab 7.
 *
 * Purpose:
 *
 * This program implements a simple counter, that counts the
 * number of elapsed seconds.  The counter has both a start
 * value and a stop value.
 */
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#include <DAQlib.h>

 /* Sleep on Windows systems */
#include <Windows.h>
/* usleep on macOS or Linux systems */
/* #include <unistd.h> */

#define TRUE  1
#define FALSE 0

/* The input channels for the two switches */
#define RUN_SWITCH   0
#define RESET_SWITCH 1

/* The states of the switches */
#define SWITCH_ON  1
#define SWITCH_OFF 0

/* Number of LED displays in the LED screen */
#define NUM_DISPLAYS 8

/* Value to write to the LED display to turn it off (space) */
#define SPACE_CHAR 0

/* 1 second = 1000 miliseconds for the delay */
#define DELAY_TIME 1000

/* function prototypes for all the functions implemented here */
void runCounter();
void writeNumber(int number);
void writeDigit(int digit, int position);

/* main function */
int main(void)
{
	int setupNum;

	printf("Enter configuration type (0 for the device or 4 for the simulator): ");
	scanf("%d", &setupNum);

	if (setupDAQ(setupNum) == TRUE) {
		runCounter();
	}
	else {
		printf("ERROR: Cannot initialize system\n");
	}

	system("PAUSE");
	return 0;
}

/* Implements the counter logic. */
/* Read the two switches and act accordingly. */
void runCounter()
{
	/*  variables for the states of the two switches */
	int runSwitch, resetSwitch;

	/*  The current value of the counter, along with the start and stop values. */
	int counter;
	int counterStart, counterStop;

	printf("Enter the counter start value: ");
	scanf("%d", &counterStart);
	counter = counterStart;

	printf("Enter the counter stop value: ");
	scanf("%d", &counterStop);

	printf("\nInitially, both switches should be off to proceed.\n");
	printf("Please make sure both switches are off.\n\n");
	do {
		runSwitch = digitalRead(RUN_SWITCH);
		resetSwitch = digitalRead(RESET_SWITCH);
	} while (runSwitch == SWITCH_ON || resetSwitch == SWITCH_ON);

	printf("Both switches are now off, proceeding ... \n");
	printf("To start/stop the counter, use the RUN switch (#0). \n");
	printf("To reset the counter, use the RESET switch (#1). \n");

	/* display the initial counter value on the LED screen */
	writeNumber(counter);

	while (continueSuperLoop() == TRUE) {
		/* read the two switches */
		runSwitch = digitalRead(RUN_SWITCH);
		resetSwitch = digitalRead(RESET_SWITCH);

		/*  decide what to do, based on the values of the switches  */
		if (resetSwitch == SWITCH_ON) {
			/*
			 * If the reset switch is on, and the counter is currently
			 * running, then reset the counter
			 *
			 * If the counter is not currently running, then do nothing,
			 * so that the screen does not continuously flicker.
			 */
			if (counter > counterStart) {
				counter = counterStart;
				writeNumber(counter);
			}
		}
		/*
		 * if the reset switch is off, then check the run switch, so that
		 * we know whether to increment the counter or not.
		 */
		else if (runSwitch == SWITCH_ON) {
			/*
			 * Increment the counter only if we have not reached
			 * the final value yet.
			 */
			if (counter < counterStop) {
				/*
				 * NOTE: there are better ways to handle the timing to
				 *       minimize delay when flipping the reset switch
				 */
				 /* windows sleep function */
				Sleep(DELAY_TIME);
				/* macOS/Linux sleep function */
				/* usleep(DELAY_TIME*1000) */

				/* check reset again prior to showing number */
				resetSwitch = digitalRead(RESET_SWITCH);
				if (resetSwitch == SWITCH_ON) {
					counter = counterStart;
				} else {
					counter++;
				}

				writeNumber(counter);
			}
		}
	}
}

/*
 * Writes the number to the LED screen.
 * Only write as many digits of the number as will fit on the
 * LED screen.
 */
void writeNumber(int number)
{
	int pos = 0; /* starting at the rightmost 7-segment */
	int digit;


	/*
	 * extracting one digit at a time, until we have
	 * no more digits left (i.e. number==0)
	 * or
	 * we have exhausted our LED displays (i.e. pos >= NUM_DISPLAYS ).
	 */
	do {
		digit = number % 10;
		number = number / 10;

		writeDigit(digit, pos);

		pos++; /* determining the position for the next digit, if any */
	} while (pos < NUM_DISPLAYS && number != 0);


	/*
	 * if there are any LED displays left that were not used above, then turn
	 * those off
	 */
	while (pos < NUM_DISPLAYS) {
		displayWrite(SPACE_CHAR, pos);
		pos++;
	}
}

/*
 * The following function writes the given "digit"
 * to the LED display at "position"
 */
void writeDigit(int digit, int position)
{
	static int digits_table[10] = {
		252, 96, 218, 242, 102, 182, 190, 224, 254, 246 };

	displayWrite(digits_table[digit], position);
}