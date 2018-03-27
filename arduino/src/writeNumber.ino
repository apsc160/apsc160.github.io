/*
 * Program:  writeNumber.ino
 * Purpose:  writes a number to a set of 7-segment displays
 */

/* headers */
#include <HardwareSerial.h>  /* serial communication over USB */
#include "DisplayWrite.h"    /* hardware 7-segment displays   */

/* I/O pins */
#define DATA_PIN  2
#define LATCH_PIN 3
#define CLOCK_PIN 4

/* number of displays on the DAQ */
#define NUM_DISPLAYS 8

/* function prototypes */
long readLong(void);
void writeNumber(long number);

/* initialization */
void setup(void)
{
	/* initialize serial communication */
	Serial.begin(9600);

	/* initialize pins */
	setupDisplay(DATA_PIN, LATCH_PIN, CLOCK_PIN);
}

/* control loop */
void loop(void)
{
	/* number to write to display*/
	long num = readLong();

	/* write number to displays */
	writeNumber(num);

}

/* reads an integer from Serial connection */
long readLong(void)
{
	long num = 0;  /* number to read */

	Serial.print("Enter an integer: ");     /* prompt user */
	while (!Serial.available()) {}        	/* wait for input */
	num = Serial.parseInt();                /* read integer */
	Serial.println(num, DEC);               /* echo to Serial */

	return num;
}

/* displays "number" on the eight 7-segment displays */
void writeNumber(long number)
{
	/* constant digits table */
	static const int DIGITS_TABLE[10] = { 252, 96, 218, 242, 102,
		182, 190, 224, 254, 246 };
	static const int OFF = 0;      /* turning display off */

	/* current digit */
	int digit = 0;

	/* start at the right-most 7-segment display */
	int pos = 0;

	/*
	 * extract one digit at a time until no digits remain (i.e. number == 0)
	 * or we have exhausted our LED displays (i.e. pos >= NUM_DISPLAYS ).
	 */
	do {
		digit = (int)(number % 10);
		number = number / 10;

		/* write the digit to the 7 segment-display */
		displayWrite(DIGITS_TABLE[digit], pos);

		/* determine the position for the next digit */
		pos++;
	} while (pos < NUM_DISPLAYS && number != 0);

	/* turn off any displays not used above */
	while (pos < NUM_DISPLAYS) {
		displayWrite(OFF, pos);
		pos++;
	}
}
