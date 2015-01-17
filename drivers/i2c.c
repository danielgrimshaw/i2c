// i2c.c
// This is where we implement i2c.h

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include "i2c.h"

volatile uint32_t *bcm2835_gpio = (volatile uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_bsc0 = (volatile uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_bsc1 = (volatile uint32_t *)MAP_FAILED;
volatile uint32_t *bcm2835_st	= (volatile uint32_t *)MAP_FAILED;
static int i2c_byte_wait_us = 0;

int bcm2835_init(void)
{
    int memfd = -1;
    int ok = 0;
    // Open the master /dev/memory device
    if ((memfd = open("/dev/mem", O_RDWR | O_SYNC) ) < 0) {
	    fprintf(stderr, "bcm2835_init: Unable to open /dev/mem: %s\n", strerror(errno));
	    goto exit;
    }

    // GPIO:
    bcm2835_gpio = (volatile uint32_t *)mapmem("gpio", BCM2835_BLOCK_SIZE, memfd, BCM2835_GPIO_BASE);
    if (bcm2835_gpio == MAP_FAILED) goto exit;
    
    // I2C
    bcm2835_bsc0 = (volatile uint32_t *)mapmem("bsc0", BCM2835_BLOCK_SIZE, memfd, BCM2835_BSC0_BASE);
    if (bcm2835_bsc0 == MAP_FAILED) goto exit;

    bcm2835_bsc1 = (volatile uint32_t *)mapmem("bsc1", BCM2835_BLOCK_SIZE, memfd, BCM2835_BSC1_BASE);
    if (bcm2835_bsc1 == MAP_FAILED) goto exit;
    
    // ST
    bcm2835_st = (volatile uint32_t *)mapmem("st", BCM2835_BLOCK_SIZE, memfd, BCM2835_ST_BASE);
    if (bcm2835_st == MAP_FAILED) goto exit;

    ok = 1;

exit:
    if (memfd >= 0)
        close(memfd);

    if (!ok)
	bcm2835_close();

    return ok;
}

int bcm2835_close(void) {
	unmapmem((void**) &bcm2835_gpio, BCM2835_BLOCK_SIZE);
    unmapmem((void**) &bcm2835_bsc0, BCM2835_BLOCK_SIZE);
    unmapmem((void**) &bcm2835_bsc1, BCM2835_BLOCK_SIZE);
    unmapmem((void**) &bcm2835_st,   BCM2835_BLOCK_SIZE);
    return 1; // Success
}

void gpio_fsel(uint8_t pin, uint8_t mode) {
    // Function selects are 10 pins per 32 bit word, 3 bits per pin
    volatile uint32_t* paddr = bcm2835_gpio + BCM2835_GPFSEL0/4 + (pin/10);
    uint8_t   shift = (pin % 10) * 3;
    uint32_t  mask = BCM2835_GPIO_FSEL_MASK << shift;
    uint32_t  value = mode << shift;
    peri_set_bits(paddr, value, mask);
}

// Set/clear only the bits in value covered by the mask
void peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask) {
    uint32_t v = peri_read(paddr);
    v = (v & ~mask) | (value & mask);
    peri_write(paddr, v);
}

// safe read from peripheral
uint32_t peri_read(volatile uint32_t* paddr) {
	// Make sure we dont return the _last_ read which might get lost
	// if subsequent code changes to a different peripheral
	uint32_t ret = *paddr;
	*paddr; // Read without assigning to an unused variable
	return ret;
}

// safe write to peripheral
void peri_write(volatile uint32_t* paddr, uint32_t value) {
	// Make sure we don't rely on the first write, which may get
	// lost if the previous access was to a different peripheral.
	*paddr = value;
	*paddr = value;
}

// write to peripheral without the write barrier
void peri_write_nb(volatile uint32_t* paddr, uint32_t value) {
	*paddr = value;
}

// read from peripheral without the read barrier
uint32_t peri_read_nb(volatile uint32_t* paddr) {
	return *paddr;
}

void i2c_begin(void) {
#ifdef I2C_V1
    volatile uint32_t* paddr = bcm2835_bsc0 + BCM2835_BSC_DIV/4;
    // Set the I2C/BSC0 pins to the Alt 0 function to enable I2C access on them
    bcm2835_gpio_fsel(RPI_GPIO_P1_03, BCM2835_GPIO_FSEL_ALT0); // SDA
    bcm2835_gpio_fsel(RPI_GPIO_P1_05, BCM2835_GPIO_FSEL_ALT0); // SCL
#else
    volatile uint32_t* paddr = bcm2835_bsc1 + BCM2835_BSC_DIV/4;
    // Set the I2C/BSC1 pins to the Alt 0 function to enable I2C access on them
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_03, BCM2835_GPIO_FSEL_ALT0); // SDA
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_05, BCM2835_GPIO_FSEL_ALT0); // SCL
#endif    

    // Read the clock divider register
    uint16_t cdiv = peri_read(paddr);
    // Calculate time for transmitting one byte
    // 1000000 = micros seconds in a second
    // 9 = Clocks per byte : 8 bits + ACK
    i2c_byte_wait_us = ((float)cdiv / BCM2835_CORE_CLK_HZ) * 1000000 * 9;
}

void i2c_end(void) {
#ifdef I2C_V1
    // Set all the I2C/BSC0 pins back to input
    bcm2835_gpio_fsel(RPI_GPIO_P1_03, BCM2835_GPIO_FSEL_INPT); // SDA
    bcm2835_gpio_fsel(RPI_GPIO_P1_05, BCM2835_GPIO_FSEL_INPT); // SCL
#else
    // Set all the I2C/BSC1 pins back to input
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_03, BCM2835_GPIO_FSEL_INPT); // SDA
    bcm2835_gpio_fsel(RPI_V2_GPIO_P1_05, BCM2835_GPIO_FSEL_INPT); // SCL
#endif
}

void i2c_setSlaveAddress(uint8_t addr) {
	// Set I2C Device Address
#ifdef I2C_V1
	volatile uint32_t* paddr = bcm2835_bsc0 + BCM2835_BSC_A/4;
#else	
	volatile uint32_t* paddr = bcm2835_bsc1 + BCM2835_BSC_A/4;
#endif
	peri_write(paddr, addr);
}

// defaults to 0x5dc, should result in a 166.666 kHz I2C clock frequency.
// The divisor must be a power of 2. Odd numbers
// rounded down.
void i2c_setClockDivider(uint16_t divider) {
#ifdef I2C_V1
    volatile uint32_t* paddr = bcm2835_bsc0 + BCM2835_BSC_DIV/4;
#else
    volatile uint32_t* paddr = bcm2835_bsc1 + BCM2835_BSC_DIV/4;
#endif    
    peri_write(paddr, divider);
    // Calculate time for transmitting one byte
    // 1000000 = micros seconds in a second
    // 9 = Clocks per byte : 8 bits + ACK
    i2c_byte_wait_us = ((float)divider / BCM2835_CORE_CLK_HZ) * 1000000 * 9;
}

// set I2C clock divider by means of a baudrate number
void i2c_set_baudrate(uint32_t baudrate) {
	uint32_t divider;
	// use 0xFFFE mask to limit a max value and round down any odd number
	divider = (BCM2835_CORE_CLK_HZ / baudrate) & 0xFFFE;
	i2c_setClockDivider((uint16_t)divider);
}

// Writes an number of bytes to I2C
uint8_t i2c_write(const char * buf, uint32_t len) {
#ifdef I2C_V1
    volatile uint32_t* dlen    = bcm2835_bsc0 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc0 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc0 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc0 + BCM2835_BSC_C/4;
#else
    volatile uint32_t* dlen    = bcm2835_bsc1 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc1 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc1 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc1 + BCM2835_BSC_C/4;
#endif    

    uint32_t remaining = len;
    uint32_t i = 0;
    uint8_t reason = I2C_REASON_OK;

    // Clear FIFO
    peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );
    // Clear Status
	peri_write_nb(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
	// Set Data Length
    peri_write_nb(dlen, len);
    // pre populate FIFO with max buffer
    while (remaining && (i < BCM2835_BSC_FIFO_SIZE)) {
        peri_write_nb(fifo, buf[i]);
        i++;
        remaining--;
    }
    
    // Enable device and start transfer
    peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
    
    // Transfer is over when BCM2835_BSC_S_DONE
    while (!(peri_read_nb(status) & BCM2835_BSC_S_DONE)) {
        while (remaining && (peri_read_nb(status) & BCM2835_BSC_S_TXD)) {
        	// Write to FIFO, no barrier
        	peri_write_nb(fifo, buf[i]);
        	i++;
        	remaining--;
    	}
    }

    if (peri_read(status) & BCM2835_BSC_S_ERR) // Received a NACK
		reason = I2C_REASON_ERROR_NACK;
    
    else if (peri_read(status) & BCM2835_BSC_S_CLKT) // Received Clock Stretch Timeout
		reason = I2C_REASON_ERROR_CLKT;
    
    else if (remaining) // Not all data is sent
		reason = I2C_REASON_ERROR_DATA;

    peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

    return reason;
}

// Read an number of bytes from I2C
uint8_t i2c_read(char* buf, uint32_t len) {
#ifdef I2C_V1
    volatile uint32_t* dlen    = bcm2835_bsc0 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc0 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc0 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc0 + BCM2835_BSC_C/4;
#else
    volatile uint32_t* dlen    = bcm2835_bsc1 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc1 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc1 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc1 + BCM2835_BSC_C/4;
#endif    

    uint32_t remaining = len;
    uint32_t i = 0;
    uint8_t reason = I2C_REASON_OK;

    // Clear FIFO
    peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );
    // Clear Status
	peri_write_nb(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
	// Set Data Length
    peri_write_nb(dlen, len);
    // Start read
    peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST | BCM2835_BSC_C_READ);
    
    // wait for transfer to complete
    while (!(peri_read_nb(status) & BCM2835_BSC_S_DONE)) {
        // we must empty the FIFO as it is populated and not use any delay
        while (peri_read_nb(status) & BCM2835_BSC_S_RXD) {
    		// Read from FIFO, no barrier
    		buf[i] = peri_read_nb(fifo);
        	i++;
        	remaining--;
    	}
    }
    
    // transfer has finished - grab any remaining stuff in FIFO
    while (remaining && (peri_read_nb(status) & BCM2835_BSC_S_RXD)) {
        // Read from FIFO, no barrier
        buf[i] = peri_read_nb(fifo);
        i++;
        remaining--;
    }

    if (peri_read(status) & BCM2835_BSC_S_ERR) // Received a NACK
		reason = I2C_REASON_ERROR_NACK;

    else if (peri_read(status) & BCM2835_BSC_S_CLKT) // Received Clock Stretch Timeout
		reason = I2C_REASON_ERROR_CLKT;

    else if (remaining) // Not all data is received
		reason = I2C_REASON_ERROR_DATA;

    peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

    return reason;
}

// Read an number of bytes from I2C sending a repeated start after writing
// the required register. Only works if your device supports this mode
uint8_t i2c_read_register_rs(char* regaddr, char* buf, uint32_t len) {   
#ifdef I2C_V1
    volatile uint32_t* dlen    = bcm2835_bsc0 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc0 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc0 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc0 + BCM2835_BSC_C/4;
#else
    volatile uint32_t* dlen    = bcm2835_bsc1 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc1 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc1 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc1 + BCM2835_BSC_C/4;
#endif    
	uint32_t remaining = len;
    uint32_t i = 0;
    uint8_t reason = I2C_REASON_OK;
    
    // Clear FIFO
    peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );
    // Clear Status
	peri_write_nb(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
	// Set Data Length
    peri_write_nb(dlen, 1);
    // Enable device and start transfer
    peri_write_nb(control, BCM2835_BSC_C_I2CEN);
    peri_write_nb(fifo, regaddr[0]);
    peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
    
    // poll for transfer has started
    while (!(peri_read_nb(status) & BCM2835_BSC_S_TA)) {
        // Linux may cause us to miss entire transfer stage
        if(peri_read(status) & BCM2835_BSC_S_DONE)
            break;
    }
    
    // Send a repeated start with read bit set in address
    peri_write_nb(dlen, len);
    peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST  | BCM2835_BSC_C_READ );
    
    // Wait for write to complete and first byte back.	
    bcm2835_delayMicroseconds(i2c_byte_wait_us * 3);
    
    // wait for transfer to complete
    while (!(peri_read_nb(status) & BCM2835_BSC_S_DONE)) {
        // we must empty the FIFO as it is populated and not use any delay
        while (remaining && peri_read_nb(status) & BCM2835_BSC_S_RXD) {
    		// Read from FIFO, no barrier
    		buf[i] = peri_read_nb(fifo);
        	i++;
        	remaining--;
    	}
    }
    
    // transfer has finished - grab any remaining stuff in FIFO
    while (remaining && (peri_read_nb(status) & BCM2835_BSC_S_RXD)) {
        // Read from FIFO, no barrier
        buf[i] = peri_read_nb(fifo);
        i++;
        remaining--;
    }
    
    if (peri_read(status) & BCM2835_BSC_S_ERR) // Received a NACK
		reason = I2C_REASON_ERROR_NACK;

    else if (peri_read(status) & BCM2835_BSC_S_CLKT) // Received Clock Stretch Timeout
		reason = I2C_REASON_ERROR_CLKT;

    else if (remaining) // Not all data is sent
		reason = I2C_REASON_ERROR_DATA;

    peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

    return reason;
}

// Sending an arbitrary number of bytes before issuing a repeated start 
// (with no prior stop) and reading a response. Some devices require this behavior.
uint8_t i2c_write_read_rs(char* cmds, uint32_t cmds_len, char* buf, uint32_t buf_len) {
#ifdef I2C_V1
    volatile uint32_t* dlen    = bcm2835_bsc0 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc0 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc0 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc0 + BCM2835_BSC_C/4;
#else
    volatile uint32_t* dlen    = bcm2835_bsc1 + BCM2835_BSC_DLEN/4;
    volatile uint32_t* fifo    = bcm2835_bsc1 + BCM2835_BSC_FIFO/4;
    volatile uint32_t* status  = bcm2835_bsc1 + BCM2835_BSC_S/4;
    volatile uint32_t* control = bcm2835_bsc1 + BCM2835_BSC_C/4;
#endif    

    uint32_t remaining = cmds_len;
    uint32_t i = 0;
    uint8_t reason = I2C_REASON_OK;
    
    // Clear FIFO
    peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );

    // Clear Status
    peri_write_nb(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);

    // Set Data Length
    peri_write_nb(dlen, cmds_len);
 
    // pre populate FIFO with max buffer
    while(remaining && (i < BCM2835_BSC_FIFO_SIZE )) {
        peri_write_nb(fifo, cmds[i]);
        i++;
        remaining--;
    }

    // Enable device and start transfer 
    peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
    
    // poll for transfer has started (way to do repeated start, from BCM2835 datasheet)
    while (!(peri_read_nb(status) & BCM2835_BSC_S_TA)) {
        // Linux may cause us to miss entire transfer stage
        if(peri_read(status) & BCM2835_BSC_S_DONE)
            break;
    }
    
    remaining = buf_len;
    i = 0;

    // Send a repeated start with read bit set in address
    peri_write_nb(dlen, buf_len);
    peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST  | BCM2835_BSC_C_READ );
    
    // Wait for write to complete and first byte back.	
    bcm2835_delayMicroseconds(i2c_byte_wait_us * (cmds_len + 1));
    
    // wait for transfer to complete
    while (!(peri_read_nb(status) & BCM2835_BSC_S_DONE)) {
        // we must empty the FIFO as it is populated and not use any delay
        while (remaining && peri_read_nb(status) & BCM2835_BSC_S_RXD) {
    		// Read from FIFO, no barrier
    		buf[i] = peri_read_nb(fifo);
        	i++;
        	remaining--;
    	}
    }
    
    // transfer has finished - grab any remaining stuff in FIFO
    while (remaining && (peri_read_nb(status) & BCM2835_BSC_S_RXD)) {
        // Read from FIFO, no barrier
        buf[i] = peri_read_nb(fifo);
        i++;
        remaining--;
    }

    if (peri_read(status) & BCM2835_BSC_S_ERR) // Received a NACK
		reason = I2C_REASON_ERROR_NACK;

    else if (peri_read(status) & BCM2835_BSC_S_CLKT) // Received Clock Stretch Timeout
		reason = I2C_REASON_ERROR_CLKT;

    else if (remaining) // Not all data is sent
		reason = I2C_REASON_ERROR_DATA;

    peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

    return reason;
}

// microseconds
void delayMicroseconds(uint64_t micros) {
    struct timespec t1;
    uint64_t        start;
	
    // Calling nanosleep() takes at least 100-200 us, so use it for
    // long waits and use a busy wait on the System Timer for the rest.
    start =  bcm2835_st_read();
    
    if (micros > 450) {
	    t1.tv_sec = 0;
	    t1.tv_nsec = 1000 * (long)(micros - 200);
	    nanosleep(&t1, NULL);
    }    
  
    bcm2835_st_delay(start, micros);
}

// Read the System Timer Counter (64-bits)
uint64_t bcm2835_st_read(void) {
    volatile uint32_t* paddr;
    uint32_t hi, lo;
    uint64_t st;
    paddr = bcm2835_st + BCM2835_ST_CHI/4;
    hi = peri_read(paddr);

    paddr = bcm2835_st + BCM2835_ST_CLO/4;
    lo = peri_read(paddr);
    
    paddr = bcm2835_st + BCM2835_ST_CHI/4;
    st = peri_read(paddr);
    
    // Test for overflow
    if (st == hi) {
        st <<= 32;
        st += lo;
    } else {
        st <<= 32;
        paddr = bcm2835_st + BCM2835_ST_CLO/4;
        st += peri_read(paddr);
    }
    return st;
}

// Delays for the specified number of microseconds with offset
void bcm2835_st_delay(uint64_t offset_micros, uint64_t micros) {
    uint64_t compare = offset_micros + micros;
    while(bcm2835_st_read() < compare);
}
