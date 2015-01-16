//=============================================================================
//
//
// Buttons_test
//
// main file
//


#include "gb_common.h"

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
// button test GPIO mapping:
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
// GPIO22= unused
// GPIO23= Pushbutton (B3)     Input
// GPIO24= Pushbutton (B2)     Input
// GPIO25= Pushbutton (B1)     Input
//
// Always call INP_GPIO(x) first
// as that is how the macros work
void setup_gpio()
{
   // for this test we are only using GP23, 24, & 25
   INP_GPIO(23);
   INP_GPIO(24);
   INP_GPIO(25);

   // enable pull-up on GPIO 23,24&25, set pull to 2 (code for pull high)
   GPIO_PULL = 2;
   short_wait();
   // setting bits 23, 24 & 25 below means that the GPIO_PULL is applied to
   // GPIO 23, 24, & 25
   GPIO_PULLCLK0 = 0x03800000; // 11100000000000000000000000
   short_wait(); // Let the system catch up
   GPIO_PULL = 0;
   GPIO_PULLCLK0 = 0;
} // setup_gpio

// remove pulling on pins so they can be used for somnething else next time
// gertboard is used
void unpull_pins()
{
   // disable pull-up on GPIO 23,24&25, set pull to 0 (code for no pull)
   GPIO_PULL = 0;
   short_wait();
   // setting bits 23, 24 & 25 below means that the GPIO_PULL is applied to
   // GPIO 23, 24, & 25
   GPIO_PULLCLK0 = 0x03800000; // 11100000000000000000000000
   short_wait(); // let the system catch up
   GPIO_PULL = 0;
   GPIO_PULLCLK0 = 0;
} // unpull_pins

int main(void)
{ int r,d;
  unsigned int b,prev_b;
  char str [4];

  printf("Attach tactile buttons to GPIO 23, 24, and 25 (BCM identifications)");
  printf("When ready hit enter.\n");
  (void)getchar(); // Yes, some compilers complain about having nothing to do with the outp

   // Map the I/O sections
   setup_io(); // DON'T EDIT THIS CODE IN COMMON.C!!!

   // Set GPIO pins 23, 24, and 25 to the required mode
   setup_gpio(); // This is the user defined setup function

   // read the switches a number of times and print out the result

   /* below we set prev_b to a number which will definitely be different
      from what the switches returns, as after shift & mask, b can only
      be in range 0..7 */
   prev_b = 8;

   r = 20; // number of button changes - 1

  while (r) {
    b = GPIO_IN0;
    // As there are 26 pins, 23,24, and 25 are the last three
    // Therefore we can right shift 23 bits to isolate them
    // And then ignore all of the other random bits that we found
    // & 0x07 will return the status of the last three bits
    b = (b >> 23 ) & 0x07; // keep only bits 23, 24 & 25
    if (b ^ prev_b) { // one or more buttons changed
      make_binary_string(3, b, str);
      printf("%s\n", str);
      prev_b = b;
      r--;
    } // change
  } // while

  // disable pull up on pins & unmap gpio
  unpull_pins();
  restore_io(); // THIS MUST BE CALLED AT THE END OF EVERY PROGRAM

  return 0;
} // main

