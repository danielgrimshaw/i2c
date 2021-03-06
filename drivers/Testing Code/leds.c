//
// These program walks the LEDs
//
// Try to strike a balance between keep code simple for
// novice programmers but still have reasonable quality code
//

#include "../common.h"

// Use defines for the LEDS. In the GPIO code, GPIO pins n is controlled
// by bit n. The idea is here is that for example L1 will refer
// to the first LED, which is controlled by GPIO25 (because we will
// put a strap between GP25 anb B1). This gives a more intuitive
// name to use for the LEDs in the patterns.
//
// For novice users: don't worry about the complexity
// The compiler will optimise out all constant expressions and you
// will end up with a single constant value in your table.
#define L1 (1<<25)
#define L2 (1<<24)
#define L3 (1<<23)
#define L4 (1<<22)
// Led 5 is controlled by GP21 - on the rev2 Pi, this is now controlled
// by GPIO27, not GP21.
#define L5_1 (1<<21)
#define L5_2 (1<<27)
#define L6 (1<<18)
#define L7 (1<<17)
#define L8 (1<<11)
#define L9 (1<<10)
#define L10 (1<<9)
#define L11 (1<<8)
#define L12 (1<<7)

// This will be assigned the OR of all the bits corresponding to the
// GPIO pins we are using. It will be used to turn all the LEDs off.
static int ALL_LEDS;

// LEDs test GPIO mapping:
//         Function            Mode
// GPIO0=  unused
// GPIO1=  unused
// GPIO4=  unused
// GPIO7=  LED                 Output
// GPIO8=  LED                 Output
// GPIO9=  LED                 Output
// GPIO10= LED                 Output
// GPIO11= LED                 Output
// GPIO14= unused (preset to be UART)
// GPIO15= unused (preset to be UART)
// GPIO17= LED                 Output
// GPIO18= LED                 Output
// GPIO21 (27 on rev2) = LED   Output
// GPIO22= LED                 Output
// GPIO23= LED                 Output
// GPIO24= LED                 Output
// GPIO25= LED                 Output

// pin21 is set to the GPIO pin number driving GP21: GPIO21 on rev1 Pi,
// GPIO27 on rev2 Pi
void setup_gpio(int pin21) {
  INP_GPIO(7);  OUT_GPIO(7);
  INP_GPIO(8);  OUT_GPIO(8);
  INP_GPIO(9);  OUT_GPIO(9);
  INP_GPIO(10);  OUT_GPIO(10);
  INP_GPIO(11);  OUT_GPIO(11);
  // 14 and 15 are already set to UART mode
  // by Linux. Best if we don't touch them
  INP_GPIO(17);  OUT_GPIO(17);
  INP_GPIO(18);  OUT_GPIO(18);
  INP_GPIO(pin21);  OUT_GPIO(pin21);
  INP_GPIO(22);  OUT_GPIO(22);
  INP_GPIO(23);  OUT_GPIO(23);
  INP_GPIO(24);  OUT_GPIO(24);
  INP_GPIO(25);  OUT_GPIO(25);
} // setup_gpio

//
// Define the various patterns.
// The idea here is that each number in the arrays below specifies
// a collection of LEDs to turn on. The last element in each array is
// -1 so we can run through the patter with a a loop and detect when
// we are at the last item in the pattern. pattern0 and pattern1
// have only one LED on at a time. pattern2 starts with one on
// then turns on 2 of themm then 3, etc. Since each LED is controlled by
// a bit, we | (or) them together to turn on more than one LED as a time.
//

static int pattern0_rev1[] =
  {L1, L2, L3, L4, L5_1, L6, L7, L8, L9, L10, L11, L12, -1 };
static int pattern1_rev1[] =
  {L1, L2, L3, L4, L5_1, L6, L7, L8, L9, L10, L11, L12,
   L12, L11, L10, L9, L8, L7, L6, L5_1, L4, L3, L2, L1, -1 };
static int pattern2_rev1[] =
  {0x0,
   L1,
   L1|L2,
   L1|L2|L3,
   L1|L2|L3|L4,
   L1|L2|L3|L4|L5_1,
   L1|L2|L3|L4|L5_1|L6,
   L1|L2|L3|L4|L5_1|L6|L7,
   L1|L2|L3|L4|L5_1|L6|L7|L8,
   L1|L2|L3|L4|L5_1|L6|L7|L8|L9,
   L1|L2|L3|L4|L5_1|L6|L7|L8|L9|L10,
   L1|L2|L3|L4|L5_1|L6|L7|L8|L9|L10|L11,
   L1|L2|L3|L4|L5_1|L6|L7|L8|L9|L10|L11|L12,
   L2|L3|L4|L5_1|L6|L7|L8|L9|L10|L11|L12,
   L3|L4|L5_1|L6|L7|L8|L9|L10|L11|L12,
   L4|L5_1|L6|L7|L8|L9|L10|L11|L12,
   L5_1|L6|L7|L8|L9|L10|L11|L12,
   L6|L7|L8|L9|L10|L11|L12,
   L7|L8|L9|L10|L11|L12,
   L8|L9|L10|L11|L12,
   L9|L10|L11|L12,
   L10|L11|L12,
   L11|L12,
   L12,
   -1};
static int pattern0_rev2[] = 
  {L1, L2, L3, L4, L5_2, L6, L7, L8, L9, L10, L11, L12, -1 };
static int pattern1_rev2[] = 
  {L1, L2, L3, L4, L5_2, L6, L7, L8, L9, L10, L11, L12, 
   L12, L11, L10, L9, L8, L7, L6, L5_2, L4, L3, L2, L1, -1 };
static int pattern2_rev2[] = 
  {0x0,
   L1,
   L1|L2,
   L1|L2|L3,
   L1|L2|L3|L4,
   L1|L2|L3|L4|L5_2,
   L1|L2|L3|L4|L5_2|L6,
   L1|L2|L3|L4|L5_2|L6|L7,
   L1|L2|L3|L4|L5_2|L6|L7|L8,
   L1|L2|L3|L4|L5_2|L6|L7|L8|L9,
   L1|L2|L3|L4|L5_2|L6|L7|L8|L9|L10,
   L1|L2|L3|L4|L5_2|L6|L7|L8|L9|L10|L11,
   L1|L2|L3|L4|L5_2|L6|L7|L8|L9|L10|L11|L12,
   L2|L3|L4|L5_2|L6|L7|L8|L9|L10|L11|L12,
   L3|L4|L5_2|L6|L7|L8|L9|L10|L11|L12,
   L4|L5_2|L6|L7|L8|L9|L10|L11|L12,
   L5_2|L6|L7|L8|L9|L10|L11|L12,
   L6|L7|L8|L9|L10|L11|L12,
   L7|L8|L9|L10|L11|L12,
   L8|L9|L10|L11|L12,
   L9|L10|L11|L12,
   L10|L11|L12,
   L11|L12,
   L12,
   -1};

// these below will point to either the _rev1 or the _rev2 versions of the
// patterns
static int *pattern0, *pattern1, *pattern2;

// Local (to this file) variables
static int *pattern;  // current pattern
static int step;  // which pattern element we are showing

void show_LEDs(int value) {
  // first turn off all LEDs - GPIO_CLR0 selects which output pins
  // will be set up 0
  GPIO_CLR0 = ALL_LEDS;
  // now light up the ones for this value - GPIO_SET0 selects which 
  // output pins will be set up 1
  GPIO_SET0 = value;
} // set_pattern

void leds_off(void) {
  GPIO_CLR0 = ALL_LEDS;
}

//
// Start anew with one of the available patterns
//
void start_new_pattern(int p) {
   switch (p) {
     case 0 : pattern = pattern0; break;
     case 1 : pattern = pattern1; break;
     case 2 : pattern = pattern2; break;
     default: return;
   }
} // start_new_pattern

//
// Do single pattern step
// return 1 on last pattern
// return 0 on all others
//
int led_step() {
   step++;
   show_LEDs(pattern[step]);
   return (pattern[step+1] == -1) ? 1 : 0; // are we at last value?
} // led_step

//
// Quick play all patterns
//
int main(void) {
  int p, r, last, rev, pin21;

  printf ("These are the connections for the LEDs test:\n");
  printf ("jumpers in every out location (U3-out-B1, U3-out-B2, etc)\n");
  printf ("GP25 in J2 --- B1 in J3\n");
  printf ("GP24 in J2 --- B2 in J3\n");
  printf ("GP23 in J2 --- B3 in J3\n");
  printf ("GP22 in J2 --- B4 in J3\n");
  printf ("GP21 in J2 --- B5 in J3\n");
  printf ("GP18 in J2 --- B6 in J3\n");
  printf ("GP17 in J2 --- B7 in J3\n");
  printf ("GP11 in J2 --- B8 in J3\n");
  printf ("GP10 in J2 --- B9 in J3\n");
  printf ("GP9 in J2 --- B10 in J3\n");
  printf ("GP8 in J2 --- B11 in J3\n");
  printf ("GP7 in J2 --- B12 in J3\n");
  printf ("(If you don't have enough straps and jumpers you can install\n");
  printf ("just a few of them, then run again later with the next batch.)\n");
  printf ("When ready hit enter.\n");
  (void) getchar();

  // first find out which rev of RPi we have
  rev = pi_revision();
  // on a rev2 p1, GPIO27 drives the pin connected to GP21 in J2
  if (rev == 1){ // GP21 on Gertboard is controller by GPIO21
    pin21 = 21;
    ALL_LEDS = (L1|L2|L3|L4|L5_1|L6|L7|L8|L9|L10|L11|L12);
    pattern0 = pattern0_rev1;
    pattern1 = pattern1_rev1;
    pattern2 = pattern2_rev1;
  } else { // GP21 on Gertboard is controller by GPIO27
    pin21 = 27;
    ALL_LEDS = (L1|L2|L3|L4|L5_2|L6|L7|L8|L9|L10|L11|L12);
    pattern0 = pattern0_rev2;
    pattern1 = pattern1_rev2;
    pattern2 = pattern2_rev2;
  }

  // set the current pattern to pattern 0
  pattern = pattern0;
  step = 0;

  // Map the I/O sections
  setup_io();

  // Set 12 GPIO pins to output mode
  setup_gpio(pin21);

  for (p=0; p<3; p++) {
    // run pattern several times
    start_new_pattern(p);
    for (r=0; r<2; r++) {
      step = -1; // we will incrment this before showing any patterns
      do {
        last = led_step();
        long_wait(3);
      } while (!last);
    } // run the pattern 2 times
  } // loop over patterns

  leds_off();
  restore_io();
} // main



