/*
 * Name:  button_led_display.ino
 * Purpose: Basic program demonstrating a button/LED, and 7-segment display
 */

/* header with displayWrite(...) implementation */
#include "DisplayWrite.h"

/* # 7-segment displays */
#define NUM_DISPLAYS 8

/* display pins */
#define DATA_PIN  2
#define LATCH_PIN 3
#define CLOCK_PIN 4

/* digital I/O pins */
#define LED_PIN    5
#define BUTTON_PIN 6

/* the setup function runs once when you press reset or power the board */
void setup(void)
{
  /* initialize display */
  setupDisplay(DATA_PIN, LATCH_PIN, CLOCK_PIN);

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

  /* write message to displays */
  const int message[] = { 252, 190, 96, 0, 156, 182, 206, 238 };
  for (int i = 0; i < NUM_DISPLAYS; ++i) {
    displayWrite(message[i], i);
  }
}
