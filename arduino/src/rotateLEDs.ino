/*
 * Name:    rotateLEDs.ino
 * Purpose: demonstrate timing on the arduino
 */

/* I/O pins */
const int RESET = 2;
const int LED0 = 3;
const int NUM_LEDS = 6;

/* states */
const int OFF = LOW;
const int ON = HIGH;

/* timing (ms) */
const unsigned long DELAY = 1000;

/* initialization */
void setup(void)
{
	int i = 0;  /* loop counter */

	/* setup I/O pins */
	pinMode(RESET, INPUT);
	for (i = 0; i < NUM_LEDS; ++i) {
		pinMode(LED0 + i, OUTPUT);
	}
}

/* main control loop */
void loop() {

	/* initialize LED index, shared between loop iterations */
	static int led = 0;
	/* initialize time variable for last modification */
	static unsigned long last_modification = millis();

	int button0 = OFF;  /* button state */
	int i = 0;          /* loop counter */
	unsigned long current_time = millis(); /* time */

	/* rotate LED if time elapsed */
	if (current_time - last_modification >= DELAY) {
		/* increment LED index */
		led = (led + 1) % NUM_LEDS;
		last_modification = current_time;
	}

	/* check reset button */
	button0 = digitalRead(RESET);
	if (button0 == ON) {
		led = 0;
		/* reset timing */
		last_modification = current_time;
	}

	/* turn on/off LEDs */
	for (i = 0; i < NUM_LEDS; ++i) {
		/* only turn on LED corresponding to 'led' */
		digitalWrite(LED0 + i, i == led);
	}

}
