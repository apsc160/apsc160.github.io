/*
 * Program: controlLEDs.ino
 * Purpose: controls the state of three LEDs based on
 *          values read from two buttons or switches
 *          and a truth table
 *            Inputs:       Outputs:
 *            {OFF, OFF} -> {OFF, OFF, OFF}
 *            { ON, OFF} -> { ON, OFF, OFF}
 *            {OFF,  ON} -> {OFF,  ON, OFF}
 *            { ON,  ON} -> {OFF, OFF,  ON}
 */

/* I/O pins */
#define SWITCH0  2
#define SWITCH1  3
#define LED0     4
#define LED1     5
#define LED2     6

/* function prototypes */
void controlLEDs(void);
void setLEDs(int led0, int led1, int led2);

/* initialize system */
void setup(void)
{
	/* set pin modes */
	pinMode(SWITCH0, INPUT);
	pinMode(SWITCH1, INPUT);
	pinMode(LED0, OUTPUT);
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
}

/* main control loop */
void loop(void)
{
	controlLEDs();
}

/* controls LEDs based on switch values */
void controlLEDs(void)
{
	/* read switches */
	int switch0 = digitalRead(SWITCH0);
	int switch1 = digitalRead(SWITCH1);

	/* set LEDs according to truth table */
	setLEDs(
		switch0 && !switch1,
		!switch0 && switch1,
		switch0 && switch1 );
}

/* turn on/off LEDs */
void setLEDs(int led0, int led1, int led2)
{
	digitalWrite(LED0, led0);
	digitalWrite(LED1, led1);
	digitalWrite(LED2, led2);
}
