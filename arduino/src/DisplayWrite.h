/**
 * Definition of displayWrite(...) function
 *
 * Allows persistent displaying of content to a set of eight 7-segment displays
 *
 * Content is shifted out through a pair of 74HC595 shift registers
 * See schematic layout for wiring details
 *
 */
#ifndef DISPLAYWRITE_ARDUINO_H
#define DISPLAYWRITE_ARDUINO_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Initialize the displays with certain pins
 * @param dataPin   data pin
 * @param latchPin  latch pin
 * @param clockPin  clock pin
 */
void setupDisplay(int dataPin, int latchPin, int clockPin);

/**
 * Writes a character to the set of 8 displays
 * @param data 8-bit data value
 * @param pos  display position index
 */
void displayWrite(int data, int pos);

#ifdef __cplusplus
}
#endif

#endif /* DISPLAYWRITE_ARDUINO_H */
