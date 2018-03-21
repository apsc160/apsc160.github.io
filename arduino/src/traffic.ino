/*
 * Program:  traffic.ino
 * Purpose:  demonstrate timings on arduino with a traffic light controller
 */

/* output pins for East-West lights */
#define EW_GREEN  2
#define EW_YELLOW 3
#define EW_RED    4

/* output pins for North-South lights */
#define NS_GREEN  10
#define NS_YELLOW 11
#define NS_RED    12

/* values of the lights */
#define OFF 0
#define ON  1

/* value for delay function */
#define ONE_SECOND 1000

/* function prototypes */
void setLightsEastWest(int green, int yellow, int red);
void setLightsNorthSouth(int green, int yellow, int red);

/* initialization */
void setup(void)
{
	pinMode(EW_GREEN, OUTPUT);
	pinMode(EW_YELLOW, OUTPUT);
	pinMode(EW_RED, OUTPUT);

	pinMode(NS_GREEN, OUTPUT);
	pinMode(NS_YELLOW, OUTPUT);
	pinMode(NS_RED, OUTPUT);
}

/* control loop */
void loop(void)
{
	/* EW Green, NS Red for 5 seconds */
	setLightsEastWest(ON, OFF, OFF);
	setLightsNorthSouth(OFF, OFF, ON);
	delay(5 * ONE_SECOND);

	/* EW Yellow for 2 seconds */
	setLightsEastWest(OFF, ON, OFF);
	setLightsNorthSouth(OFF, OFF, ON);
	delay(2 * ONE_SECOND);

	/* Red for one second */
	setLightsEastWest(OFF, OFF, ON);
	setLightsNorthSouth(OFF, OFF, ON);
	delay(ONE_SECOND);

	/* EW Red, NS Green for 4 seconds */
	setLightsEastWest(OFF, OFF, ON);
	setLightsNorthSouth(ON, OFF, OFF);
	delay(4 * ONE_SECOND);

	/* NS Yellow for 2 seconds */
	setLightsEastWest(OFF, OFF, ON);
	setLightsNorthSouth(OFF, ON, OFF);
	delay(2 * ONE_SECOND);

	/* Red for one second */
	setLightsEastWest(OFF, OFF, ON);
	setLightsNorthSouth(OFF, OFF, ON);
	delay(ONE_SECOND);
}

/*
 * Set the values of the lights in the East-West direction.
 * The input parameters indicate the value to the written to
 * the corresponding light.
 */
void setLightsEastWest(int green, int yellow, int red)
{
	digitalWrite(EW_GREEN, green);
	digitalWrite(EW_YELLOW, yellow);
	digitalWrite(EW_RED, red);
}

/*
 * Set the values of the lights in the North-South direction.
 * The input parameters indicate the value to the written to the
 * corresponding light.
 */
void setLightsNorthSouth(int green, int yellow, int red)
{
	digitalWrite(NS_GREEN, green);
	digitalWrite(NS_YELLOW, yellow);
	digitalWrite(NS_RED, red);
}
