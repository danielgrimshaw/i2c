//
// Raspberry Pi GPIO C library
//
// main header file
//
// KEEP AS SIMPLE AS POSSIBLE FOR NOVINCE PROGRAMMERS
// WHILE RETAINING AT LEAST SOME REAL PROGRAMMING
//


#include <stdio.h>

// I/O access
extern volatile unsigned *gpio;
extern volatile unsigned *pwm;
extern volatile unsigned *clk;
extern volatile unsigned *spi0;
extern volatile unsigned *uart;

void short_wait();
void long_wait(int v);

void setup_io();
void restore_io();
void make_binary_string(int, int, char *);
int pi_revision();

// GPIO setup macros.
// Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
// Otherwise bad things happen.
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET0   *(gpio+7)  // Set GPIO high bits 0-31
#define GPIO_CLR0   *(gpio+10) // Set GPIO low bits 0-31

#define GPIO_IN0   *(gpio+13)  // Reads GPIO input bits 0-31

#define GPIO_PULL   *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock


//
//  UART 0
//

#define UART0_BAUD_HI *(uart+9)
#define UART0_BAUD_LO *(uart+10)
