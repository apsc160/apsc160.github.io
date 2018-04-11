/*
 * Program: analog.ino
 * Purpose: demonstrates the use of analog sensors by reading and displaying values
 *          from a temperature sensor and light sensor
 */

#include "DisplayWrite.h"
#include <math.h>

/* display pins */
#define DATA_PIN  2
#define LATCH_PIN 3
#define CLOCK_PIN 4
#define NUM_DISPLAYS 8

/* digital inputs */
#define SWITCH0   5

/* analog inputs */
#define TEMPERATURE_SENSOR  A1
#define LIGHT_SENSOR        A0

/* constants */
#define VREF  5.0   /* reference voltage */
#define AREF  1023  /* analog maximum    */

/* symbolic constants for temperature conversion */
#define VOFF_TEMP  0.5
#define M_TEMP     100
#define ZERO_CELIUS_IN_KELVIN 273.15

/* symbolic constants for light conversion */
#define M_LUX  (-1.4)
#define B_LUX  15.76
#define VD_LUX VREF
#define RD_LUX 10000

/* function prototypes */
double convertAnalogToVoltage(int aval);
double convertToKelvin(double voltage);
double readTemperature(void);
double convertToLux(double voltage);
double readIllumination(void);
void writeNumber(long number);

/* initialize */
void setup(void)
{
	/* initialize display pins */
	setupDisplay(DATA_PIN, LATCH_PIN, CLOCK_PIN);
}

/* control loop */
void loop(void)
{
	/* read switch to determine which sensor to read */
	int switch0 = digitalRead(SWITCH0);

	/* determine which sensor to read */
	if (switch0) {
		/* read and display illumination */
		double lux = readIllumination();
		int roundedLux = (int)round(lux);
		writeNumber(roundedLux);
	} else {
		/* read and display temperature */
		double kelvin = readTemperature();
		int roundedKelvin = (int)round(kelvin);
		writeNumber(roundedKelvin);
	}
	delay(1000);
}

/* converts an analog reading to a voltage */
double convertAnalogToVoltage(int aval)
{
	return aval * VREF / AREF;
}

/* convert voltage from a temperature sensor to degrees Celsius */
double convertToKelvin(double voltage)
{
	double kelvin = (voltage - VOFF_TEMP) * M_TEMP + ZERO_CELIUS_IN_KELVIN;
	return kelvin;
}

/* reads and returns the current temperature */
double readTemperature(void)
{
	int    val = analogRead(TEMPERATURE_SENSOR);
	double voltage = convertAnalogToVoltage(val);
	double kelvin = convertToKelvin(voltage);
	return kelvin;
}

/* converts a voltage to illumination (lux) */
double convertToLux(double voltage)
{
	double resistance = RD_LUX * (VD_LUX - voltage) / voltage;
	double lux = exp(M_LUX * log(resistance) + B_LUX);
	return lux;
}

/* reads and returns the current illumination */
double readIllumination(void)
{
	int val = analogRead(LIGHT_SENSOR);
	double voltage = convertAnalogToVoltage(val);
	double lux = convertToLux(voltage);
	return lux;
}

/* displays "number" on the eight 7-segment displays */
void writeNumber(long number)
{
	/* constant digits table */
	static const int DIGITS_TABLE[10] = { 252, 96, 218, 242, 102,
		182, 190, 224, 254, 246 };
	static const int OFF = 0;      /* turning display off */

	int digit = 0;                 /* current digit */

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
