//=============================================================================
//
//
// Button LED test
//

#include "../common.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <assert.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <unistd.h>

//
// Set GPIO pins to the right mode
// button-led test GPIO mapping:
//         Function            Mode
// GPIO0=  unused
// GPIO1=  unused
// GPIO4=  unused
// GPIO7=  unused
// GPIO8=  unused
// GPIO9=  unused
// GPIO10= unused
// GPIO11= unused
// GPIO14= unused
// GPIO15= unused
// GPIO17= unused
// GPIO18= unused
// GPIO21= unused
// GPIO22= LED                 Output
// GPIO23= Pushbutton (B3)     Input
// GPIO24= unused
// GPIO25= unused
//
// Always call INP_GPIO(x) first
// as that is how the macros work
void setup_gpio() {
   // for this test we are only using GP22, & 23
  INP_GPIO(22);
  INP_GPIO(23);

  // enable pull-up on GPIO 23 set pull to 2 (code for pull high)
  GPIO_PULL = 2;
  short_wait();
  // setting bit 23 below means that the GPIO_PULL is applied to GPIO 23
  GPIO_PULLCLK0 = 0x00800000;
  short_wait();
  GPIO_PULL = 0;
  GPIO_PULLCLK0 = 0;
} // setup_gpio

// remove pulling on pins so they can be used for somnething else next time
// gertboard is used
void unpull_pins() {
   // to disable pull-up on GPIO 23, set pull to 0 (code for no pull)
   GPIO_PULL = 0;
   short_wait();
   // setting bit 23 below means that the GPIO_PULL is applied to GPIO 23
   GPIO_PULLCLK0 = 0x00800000;
   short_wait();
   GPIO_PULL = 0;
   GPIO_PULLCLK0 = 0;
} // unpull_pins


int main(void) {
  int r,d;
  unsigned int b,prev_b;
  char str [3];

  printf ("These are the connections for the button-LED:\n");
  printf ("GP23 in J2 --- B3 in J3\n");
  printf ("GP22 in J2 --- B6 in J3\n");
  printf ("U3-out-B3 pin 1 --- BUF6 in top header\n");
  printf ("jumper on U4-in-B6\n");
  printf ("When ready hit enter.\n");
  (void) getchar();

  // Map the I/O sections
  setup_io();

  // Set GPIO pins 23, 24, and 25 to the required mode
  setup_gpio();

  // read the switches a number of times and print out the result

  /* below we set prev_b to a number which will definitely be different
     from what the inputs return, as after shift & mask, b can only
     be in range 0..3 */
  prev_b = 4;

  r = 20; // number of repeats

  while (r) {
    b = GPIO_IN0;
    b = (b >> 22 ) & 0x03; // keep only bits 22 & 23
    if (b^prev_b) { // button or other input changed
      make_binary_string(2, b, str);
      printf("%s\n", str);
      prev_b = b;
      r--;
    } // change
  } // while

  // disable pull up on pins & unmap gpio
  unpull_pins();
  restore_io();

  return 0;
} // main

