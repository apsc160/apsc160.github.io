/*
 * Program:  thermostat.c
 * Purpose:  complete program to create a thermostat
 */

/* headers */
#include "DisplayWrite.h"
#include <math.h>

/* I/O channels */
#define DATA_PIN     2
#define LATCH_PIN    3
#define CLOCK_PIN    4
#define ON_SWITCH    5
#define HEATER       6
#define INPUT_SWITCH 7

/* analog input channel for temperature sensor */
#define TEMPERATURE_DIAL   A0
#define TEMPERATURE_SENSOR A1

/* temperature dial settings */
#define MAX_TEMPERATURE_SETTING   30.0
#define MIN_TEMPERATURE_SETTING    0.0

/* symbolic constants for temperature conversion */
#define VOFFSET 0.5
#define VSCALE  100

/* digital values */
#define ON  HIGH
#define OFF LOW

/* sleep time */
#define ONE_SECOND 1000

/* displays settings */
#define TEMPERATURE_LENGTH 4
#define MESSAGE_LENGTH 3
#define MESSAGE_OFFSET 5

/* reference voltage */
#define VREF  5.0
#define AREF  1023

/* function prototypes */
double convertToVoltage(double aval);
double convertToCelsius(double voltage);
double readTemperature(void);
void writeNumber(int number, int length, int offset);
void writeMessage(const int message[], int length, int offset);
double readTemperatureDial(void);
void controlThermostat(double temperature);

/* initialization */
void setup(void)
{
	/* display pins */
	setupDisplay(DATA_PIN, LATCH_PIN, CLOCK_PIN);

	/* input pins */
	pinMode(ON_SWITCH, INPUT);
	pinMode(INPUT_SWITCH, INPUT);
	pinMode(TEMPERATURE_DIAL, INPUT);
	pinMode(TEMPERATURE_SENSOR, INPUT);

	/* output pin */
	pinMode(HEATER, OUTPUT);
}

/* control loop */
void loop(void)
{
	/* initialize temperature */
	static double temperature = 21.0;

	/* check if setting temperature or running thermostat */
	int setting = digitalRead(INPUT_SWITCH);
	if (setting) {
		temperature = readTemperatureDial();
	} else {
		controlThermostat(temperature);
	}

}

/* controls temperature via thermostat */
void controlThermostat(double temperature)
{
	/* define messages */
	static const int OFF_MESSAGE[] = { 0xFC, 0x8E, 0x8E };
	static const int ON_MESSAGE[] = { 0xFC, 0x2A, 0x00 };
	static const int CLEAR_MESSAGE[] = { 0x00, 0x00, 0x00 };

	/* read and display current temperature */
	double currentTemperature = readTemperature();
	int roundedTemperature = (int)round(currentTemperature);

	/* ignore negative temperatures */
	if (roundedTemperature < 0) {
		roundedTemperature = 0;
	}
	writeNumber(roundedTemperature, TEMPERATURE_LENGTH, 0);

	/* check thermostat ON switch */
	int onswitch = digitalRead(ON_SWITCH);
	if (onswitch == ON) {
		/* control temperature */
		if (currentTemperature < temperature) {
			/* turn on heater and display "On " */
			digitalWrite(HEATER, ON);
			writeMessage(ON_MESSAGE, MESSAGE_LENGTH, MESSAGE_OFFSET);
		} else {
			/* turn off heater and display "OFF" */
			digitalWrite(HEATER, OFF);
			writeMessage(OFF_MESSAGE, MESSAGE_LENGTH, MESSAGE_OFFSET);
		}
	} else {
		/* ensure heater is off and clear message */
		digitalWrite(HEATER, OFF);
		writeMessage(CLEAR_MESSAGE, MESSAGE_LENGTH, MESSAGE_OFFSET);
	}
	delay(ONE_SECOND/2);

}

/* converts an analog reading to voltage */
double convertToVoltage(double aval)
{
	return aval * VREF / AREF;
}

/* convert voltage from a temperature sensor to degrees celsius */
double convertToCelsius(double voltage)
{
	double celsius = (voltage - VOFFSET) * VSCALE;
	return celsius;
}

/* reads and returns the current temperature */
double readTemperature(void)
{
	double aval = analogRead(TEMPERATURE_SENSOR);
	double voltage = convertToVoltage(aval);
	double celsius = convertToCelsius(voltage);
	return celsius;
}

/* reads the temperature dial and displays value */
double readTemperatureDial()
{
	static const int SET_MESSAGE[] = { 0xB6, 0x9E, 0x1E };

	int aval = analogRead(TEMPERATURE_DIAL);
	/* interpolate between min and max settings */
	double temperature = (double)aval / AREF * (MAX_TEMPERATURE_SETTING - MIN_TEMPERATURE_SETTING)
		+ MIN_TEMPERATURE_SETTING;

	int roundedTemperature = (int)round(temperature);
	if (roundedTemperature < 0) {
		return 0;
	}

	/* write values to displays */
	writeNumber(roundedTemperature, TEMPERATURE_LENGTH, 0);
	writeMessage(SET_MESSAGE, MESSAGE_LENGTH, MESSAGE_OFFSET);

	delay(ONE_SECOND/2);

	return temperature;
}

/*
 * Writes a message to a set of 7-segment displays
 * The message is assumed to be read from left-to-right,
 * so that message[0] is the left-most character.
 *
 * Parameters:
 *     message - array of 8-bit values to write to the displays
 *     length - number of values in the message
 *     offset - right-most display index
 */
void writeMessage(const int message[], int length, int offset)
{
	int i; /* loop counter */

	/* loop through displays */
	for (i = 0; i < length; ++i) {
		/* write the correct character to the correct position */
		displayWrite(message[length - 1 - i], offset + i);
	}
}

/*
 * Writes a number to a set of 7-segment displays
 * Parameters:
 *     number - non-negative integer to display
 *     length - number of displays to write to
 *     offset - starting index of number
 */
void writeNumber(int number, int length, int offset)
{
	/* constant digits table */
	static const int DIGITS_TABLE[10] = { 252, 96, 218, 242, 102,
		182, 190, 224, 254, 246 };

	/* space character */
	static const int SPACE_CHAR = 0;

	int pos = 0;        /* starting position */
	int digit = 0;      /* next digit */

	/* if no displays to write to, exit immediately */
	if (length == 0) {
		return;
	}

	/* extract and write one digit at a time */
	do {
		digit = number % 10;
		number = number / 10;
		displayWrite(DIGITS_TABLE[digit], pos + offset);
		/* move to next digit */
		pos++;
	/* loop while still displays left and number is not zero */
	} while (pos < length && number != 0);

	/* clear any remaining displays */
	while (pos < length) {
		displayWrite(SPACE_CHAR, pos + offset);
		pos++;
	}
}
