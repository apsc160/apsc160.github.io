/*
 * Name:    button_led.ino
 * Purpose: Basic program demonstrating a button/LED
 */

/* digital I/O pins */
#define LED_PIN    5
#define BUTTON_PIN 6

/* the setup function runs once when you press reset or power the board */
void setup(void)
{
  /* initialize digital I/O */
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
}

/* the loop function runs over and over again until power down or reset */
void loop(void)
{
  /* read from button pin, write to LED */
  int val = digitalRead(BUTTON_PIN);
  digitalWrite(LED_PIN, val);

}
