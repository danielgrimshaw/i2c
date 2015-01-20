// i2c_test.c
// Tests the i2c functionality

#include "../i2c.c"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define MODE_READ 0
#define MODE_WRITE 1

#define MAX_LEN 32

char wbuf[MAX_LEN];

typedef enum {
    NO_ACTION,
    I2C_BEGIN,
    I2C_END
} i2c_init;

uint8_t  init = NO_ACTION;
uint16_t clk_div = I2C_CLOCK_DIVIDER_148;
uint8_t slave_address = 0x00;
uint32_t len = 8;
uint8_t  mode = MODE_READ;

char buf[MAX_LEN];
int i;
uint8_t data;

int main(int argc, char **argv) {

    printf("Running ... \n");

    if (!bcm2835_init()) return EXIT_FAILURE;

    // I2C begin
    i2c_begin();

    // If len is 0, no need to continue, but do I2C end if specified
    if (len == 0) {
        i2c_end();
        printf("... done!\n");
        return EXIT_SUCCESS;
    }

    i2c_setSlaveAddress(slave_address);
    i2c_setClockDivider(clk_div);
    fprintf(stderr, "Clock divider set to: %d\n", clk_div);
    fprintf(stderr, "len set to: %d\n", len);
    fprintf(stderr, "Slave address set to: %d\n", slave_address);

    if (mode == MODE_READ) {
        for (i=0; i<MAX_LEN; i++) buf[i] = 'n';
        data = i2c_read(buf, len);
        printf("Read Result = %d\n", data);
        for (i=0; i<MAX_LEN; i++)
    	    if(buf[i] != 'n') printf("Read Buf[%d] = %x\n", i, buf[i]);
    } else if (mode == MODE_WRITE) {
    	data = i2c_write(wbuf, len);
    	printf("Write Result = %d\n", data);
    }

    i2c_end();
    bcm2835_close();
    printf("... done!\n");
    return EXIT_SUCCESS;
}
