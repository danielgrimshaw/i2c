// i2c.c
// This is where we implement i2c.h

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
    uint16_t cdiv = bcm2835_peri_read(paddr);
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
	bcm2835_peri_write(paddr, addr);
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
    bcm2835_peri_write(paddr, divider);
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
	bcm2835_i2c_setClockDivider((uint16_t)divider);
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
    uint8_t reason = BCM2835_I2C_REASON_OK;

    // Clear FIFO
    bcm2835_peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );
    // Clear Status
	bcm2835_peri_write_nb(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
	// Set Data Length
    bcm2835_peri_write_nb(dlen, len);
    // pre populate FIFO with max buffer
    while (remaining && (i < BCM2835_BSC_FIFO_SIZE)) {
        bcm2835_peri_write_nb(fifo, buf[i]);
        i++;
        remaining--;
    }
    
    // Enable device and start transfer
    bcm2835_peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
    
    // Transfer is over when BCM2835_BSC_S_DONE
    while (!(bcm2835_peri_read_nb(status) & BCM2835_BSC_S_DONE)) {
        while (remaining && (bcm2835_peri_read_nb(status) & BCM2835_BSC_S_TXD)) {
        	// Write to FIFO, no barrier
        	bcm2835_peri_write_nb(fifo, buf[i]);
        	i++;
        	remaining--;
    	}
    }

    if (bcm2835_peri_read(status) & BCM2835_BSC_S_ERR) // Received a NACK
		reason = BCM2835_I2C_REASON_ERROR_NACK;
    
    else if (bcm2835_peri_read(status) & BCM2835_BSC_S_CLKT) // Received Clock Stretch Timeout
		reason = BCM2835_I2C_REASON_ERROR_CLKT;
    
    else if (remaining) // Not all data is sent
		reason = BCM2835_I2C_REASON_ERROR_DATA;

    bcm2835_peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

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
    uint8_t reason = BCM2835_I2C_REASON_OK;

    // Clear FIFO
    bcm2835_peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );
    // Clear Status
	bcm2835_peri_write_nb(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
	// Set Data Length
    bcm2835_peri_write_nb(dlen, len);
    // Start read
    bcm2835_peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST | BCM2835_BSC_C_READ);
    
    // wait for transfer to complete
    while (!(bcm2835_peri_read_nb(status) & BCM2835_BSC_S_DONE)) {
        // we must empty the FIFO as it is populated and not use any delay
        while (bcm2835_peri_read_nb(status) & BCM2835_BSC_S_RXD) {
    		// Read from FIFO, no barrier
    		buf[i] = bcm2835_peri_read_nb(fifo);
        	i++;
        	remaining--;
    	}
    }
    
    // transfer has finished - grab any remaining stuff in FIFO
    while (remaining && (bcm2835_peri_read_nb(status) & BCM2835_BSC_S_RXD)) {
        // Read from FIFO, no barrier
        buf[i] = bcm2835_peri_read_nb(fifo);
        i++;
        remaining--;
    }

    if (bcm2835_peri_read(status) & BCM2835_BSC_S_ERR) // Received a NACK
		reason = BCM2835_I2C_REASON_ERROR_NACK;

    else if (bcm2835_peri_read(status) & BCM2835_BSC_S_CLKT) // Received Clock Stretch Timeout
		reason = BCM2835_I2C_REASON_ERROR_CLKT;

    else if (remaining) // Not all data is received
		reason = BCM2835_I2C_REASON_ERROR_DATA;

    bcm2835_peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

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
    uint8_t reason = BCM2835_I2C_REASON_OK;
    
    // Clear FIFO
    bcm2835_peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );
    // Clear Status
	bcm2835_peri_write_nb(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);
	// Set Data Length
    bcm2835_peri_write_nb(dlen, 1);
    // Enable device and start transfer
    bcm2835_peri_write_nb(control, BCM2835_BSC_C_I2CEN);
    bcm2835_peri_write_nb(fifo, regaddr[0]);
    bcm2835_peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
    
    // poll for transfer has started
    while (!(bcm2835_peri_read_nb(status) & BCM2835_BSC_S_TA)) {
        // Linux may cause us to miss entire transfer stage
        if(bcm2835_peri_read(status) & BCM2835_BSC_S_DONE)
            break;
    }
    
    // Send a repeated start with read bit set in address
    bcm2835_peri_write_nb(dlen, len);
    bcm2835_peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST  | BCM2835_BSC_C_READ );
    
    // Wait for write to complete and first byte back.	
    bcm2835_delayMicroseconds(i2c_byte_wait_us * 3);
    
    // wait for transfer to complete
    while (!(bcm2835_peri_read_nb(status) & BCM2835_BSC_S_DONE)) {
        // we must empty the FIFO as it is populated and not use any delay
        while (remaining && bcm2835_peri_read_nb(status) & BCM2835_BSC_S_RXD) {
    		// Read from FIFO, no barrier
    		buf[i] = bcm2835_peri_read_nb(fifo);
        	i++;
        	remaining--;
    	}
    }
    
    // transfer has finished - grab any remaining stuff in FIFO
    while (remaining && (bcm2835_peri_read_nb(status) & BCM2835_BSC_S_RXD)) {
        // Read from FIFO, no barrier
        buf[i] = bcm2835_peri_read_nb(fifo);
        i++;
        remaining--;
    }
    
    if (bcm2835_peri_read(status) & BCM2835_BSC_S_ERR) // Received a NACK
		reason = BCM2835_I2C_REASON_ERROR_NACK;

    else if (bcm2835_peri_read(status) & BCM2835_BSC_S_CLKT) // Received Clock Stretch Timeout
		reason = BCM2835_I2C_REASON_ERROR_CLKT;

    else if (remaining) // Not all data is sent
		reason = BCM2835_I2C_REASON_ERROR_DATA;

    bcm2835_peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

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
    uint8_t reason = BCM2835_I2C_REASON_OK;
    
    // Clear FIFO
    bcm2835_peri_set_bits(control, BCM2835_BSC_C_CLEAR_1 , BCM2835_BSC_C_CLEAR_1 );

    // Clear Status
    bcm2835_peri_write_nb(status, BCM2835_BSC_S_CLKT | BCM2835_BSC_S_ERR | BCM2835_BSC_S_DONE);

    // Set Data Length
    bcm2835_peri_write_nb(dlen, cmds_len);
 
    // pre populate FIFO with max buffer
    while(remaining && (i < BCM2835_BSC_FIFO_SIZE )) {
        bcm2835_peri_write_nb(fifo, cmds[i]);
        i++;
        remaining--;
    }

    // Enable device and start transfer 
    bcm2835_peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST);
    
    // poll for transfer has started (way to do repeated start, from BCM2835 datasheet)
    while (!(bcm2835_peri_read_nb(status) & BCM2835_BSC_S_TA)) {
        // Linux may cause us to miss entire transfer stage
        if(bcm2835_peri_read(status) & BCM2835_BSC_S_DONE)
            break;
    }
    
    remaining = buf_len;
    i = 0;

    // Send a repeated start with read bit set in address
    bcm2835_peri_write_nb(dlen, buf_len);
    bcm2835_peri_write_nb(control, BCM2835_BSC_C_I2CEN | BCM2835_BSC_C_ST  | BCM2835_BSC_C_READ );
    
    // Wait for write to complete and first byte back.	
    bcm2835_delayMicroseconds(i2c_byte_wait_us * (cmds_len + 1));
    
    // wait for transfer to complete
    while (!(bcm2835_peri_read_nb(status) & BCM2835_BSC_S_DONE)) {
        // we must empty the FIFO as it is populated and not use any delay
        while (remaining && bcm2835_peri_read_nb(status) & BCM2835_BSC_S_RXD) {
    		// Read from FIFO, no barrier
    		buf[i] = bcm2835_peri_read_nb(fifo);
        	i++;
        	remaining--;
    	}
    }
    
    // transfer has finished - grab any remaining stuff in FIFO
    while (remaining && (bcm2835_peri_read_nb(status) & BCM2835_BSC_S_RXD)) {
        // Read from FIFO, no barrier
        buf[i] = bcm2835_peri_read_nb(fifo);
        i++;
        remaining--;
    }

    if (bcm2835_peri_read(status) & BCM2835_BSC_S_ERR) // Received a NACK
		reason = BCM2835_I2C_REASON_ERROR_NACK;

    else if (bcm2835_peri_read(status) & BCM2835_BSC_S_CLKT) // Received Clock Stretch Timeout
		reason = BCM2835_I2C_REASON_ERROR_CLKT;

    else if (remaining) // Not all data is sent
		reason = BCM2835_I2C_REASON_ERROR_DATA;

    bcm2835_peri_set_bits(control, BCM2835_BSC_S_DONE , BCM2835_BSC_S_DONE);

    return reason;
}
