/*
 LEDmeter
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on between 0 and 12 LEDs to show the relative level of the voltage. 
 
 The circuit:
 * Potentiometer attached to analog input 0 (PC0 on Gertboard)
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +3.3V
 * digital pin 13 (PB5) connected to BUF1 (at top of board)
 * digital pin 12 (PB4) connected to BUF2
 * digital pin 11 (PB3) connected to BUF3
 * digital pin 10 (PB2) connected to BUF4
 * digital pin  9 (PB1) connected to BUF5
 * digital pin  8 (PB0) connected to BUF6
 * digital pin  7 (PD7) connected to BUF7
 * digital pin  6 (PD6) connected to BUF8
 * digital pin  5 (PD5) connected to BUF9
 * digital pin  4 (PD4) connected to BUF10
 * digital pin  3 (PD3) connected to BUF11
 * digital pin  2 (PD2) connected to BUF12
 
 Created by Myra VanInwegen, 6 January 2013
 Based on AnalogInput
 
 This example code is in the public domain.
 */

// to avoid wires crossing, we'll make pin 13 (PB5) go to LED 1, pin 12 (PB4)
// to LED2, etc. We put 0 as the first element in the array so that the indexes
// match the LED number: if we want to turn on or off LED n, we acess led_pins[n]
// We do this because C arrays start at 0
int led_pins[] = { 0, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1 };

void setup() {
  int i;

  // declare the led pins as OUTPUT:
  for (i = 1; i <= 12; i++)
    pinMode(led_pins[i], OUTPUT);
  // set them all LOW
  for (i = 1; i <= 12; i++)
    digitalWrite(led_pins[i], LOW);
}

void turn_on_leds (int old_max, int new_max) {
  int i;

  // turn on all LEDs btween old_max + 1 and new_max
  for (i = old_max + 1; i <= new_max; i++)
    digitalWrite(led_pins[i], HIGH);
}

void turn_off_leds (int old_max, int new_max) {
  int i;

  // turn off all LEDs between new_max + 1 and old_max
  for (i = old_max; i > new_max; i--)
    digitalWrite(led_pins[i], LOW);
}

void loop() {
  int sensor_value, max_led;
  static int old_max_led = 0;

  // read the value from the sensor:
  sensor_value = analogRead(A0);
  // scale so we get a number between 0 and 12, max_led is the highest
  // one we want turned on (but if max_led == 0, all LEDs will be off)
  max_led = sensor_value / 79;
  
  // adjust LEDs
  if (max_led > old_max_led)
    turn_on_leds(old_max_led, max_led);
  else if (max_led < old_max_led)
    turn_off_leds(old_max_led, max_led);
  // if it's the same, don't do anything
    
  // save max LED for next time around
  old_max_led = max_led;
}
