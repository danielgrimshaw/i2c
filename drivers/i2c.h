// i2c.h 
//
// These functions let you use I2C (The Broadcom Serial Control bus with the Philips
// I2C bus/interface version 2.1 January 2000.) to interface with an external I2C device.

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// Macros
// Speed of the core clock core_clk
#define BCM2835_CORE_CLK_HZ				250000000	// 250 MHz
// Base Physical Address of the BCM 2835 peripheral registers
#define BCM2835_PERI_BASE               0x20000000
// Base Physical Address of the GPIO registers
#define BCM2835_GPIO_BASE               (BCM2835_PERI_BASE + 0x200000)
// Base Physical Address of the BSC0 registers
#define BCM2835_BSC0_BASE 		(BCM2835_PERI_BASE + 0x205000)
// Base Physical Address of the BSC1 registers
#define BCM2835_BSC1_BASE		(BCM2835_PERI_BASE + 0x804000)
// Base Physical Address of the System Timer registers
#define BCM2835_ST_BASE			(BCM2835_PERI_BASE + 0x3000)
// Size of memory page on RPi
#define BCM2835_PAGE_SIZE               (4*1024)
// Size of memory block on RPi
#define BCM2835_BLOCK_SIZE              (4*1024)
// GPIO register offsets from BCM2835_BSC*_BASE.
// Offsets into the BSC Peripheral block in bytes per 3.1 BSC Register Map
#define BCM2835_BSC_C 							0x0000 // BSC Master Control
#define BCM2835_BSC_S 							0x0004 // BSC Master Status
#define BCM2835_BSC_DLEN						0x0008 // BSC Master Data Length
#define BCM2835_BSC_A 							0x000c // BSC Master Slave Address
#define BCM2835_BSC_FIFO						0x0010 // BSC Master Data FIFO
#define BCM2835_BSC_DIV							0x0014 // BSC Master Clock Divider
#define BCM2835_BSC_DEL							0x0018 // BSC Master Data Delay
#define BCM2835_BSC_CLKT						0x001c // BSC Master Clock Stretch Timeout

// Register masks for BSC_C
#define BCM2835_BSC_C_I2CEN 					0x00008000 // I2C Enable, 0 = disabled, 1 = enabled
#define BCM2835_BSC_C_INTR 						0x00000400 // Interrupt on RX
#define BCM2835_BSC_C_INTT 						0x00000200 // Interrupt on TX
#define BCM2835_BSC_C_INTD 						0x00000100 // Interrupt on DONE
#define BCM2835_BSC_C_ST 						0x00000080 // Start transfer, 1 = Start a new transfer
#define BCM2835_BSC_C_CLEAR_1 					0x00000020 // Clear FIFO Clear
#define BCM2835_BSC_C_CLEAR_2 					0x00000010 // Clear FIFO Clear
#define BCM2835_BSC_C_READ 						0x00000001 // Read transfer

// Register masks for BSC_S
#define BCM2835_BSC_S_CLKT 						0x00000200 // Clock stretch timeout
#define BCM2835_BSC_S_ERR 						0x00000100 // ACK error
#define BCM2835_BSC_S_RXF 						0x00000080 // RXF FIFO full, 0 = FIFO is not full, 1 = FIFO is full
#define BCM2835_BSC_S_TXE 						0x00000040 // TXE FIFO full, 0 = FIFO is not full, 1 = FIFO is full
#define BCM2835_BSC_S_RXD 						0x00000020 // RXD FIFO contains data
#define BCM2835_BSC_S_TXD 						0x00000010 // TXD FIFO can accept data
#define BCM2835_BSC_S_RXR 						0x00000008 // RXR FIFO needs reading (full)
#define BCM2835_BSC_S_TXW 						0x00000004 // TXW FIFO needs writing (full)
#define BCM2835_BSC_S_DONE 						0x00000002 // Transfer DONE
#define BCM2835_BSC_S_TA 						0x00000001 // Transfer Active

#define BCM2835_BSC_FIFO_SIZE   				16 // BSC FIFO size

// GPIO Macros
// The BCM2835 has 54 GPIO pins.
// BCM2835 data sheet, Page 90 onwards.
// GPIO register offsets from BCM2835_GPIO_BASE. Offsets into the GPIO Peripheral block in bytes per 6.1 Register View
#define BCM2835_GPFSEL0                      0x0000 // GPIO Function Select 0
#define BCM2835_GPFSEL1                      0x0004 // GPIO Function Select 1
#define BCM2835_GPFSEL2                      0x0008 // GPIO Function Select 2
#define BCM2835_GPFSEL3                      0x000c // GPIO Function Select 3
#define BCM2835_GPFSEL4                      0x0010 // GPIO Function Select 4
#define BCM2835_GPFSEL5                      0x0014 // GPIO Function Select 5
#define BCM2835_GPSET0                       0x001c // GPIO Pin Output Set 0
#define BCM2835_GPSET1                       0x0020 // GPIO Pin Output Set 1
#define BCM2835_GPCLR0                       0x0028 // GPIO Pin Output Clear 0
#define BCM2835_GPCLR1                       0x002c // GPIO Pin Output Clear 1
#define BCM2835_GPLEV0                       0x0034 // GPIO Pin Level 0
#define BCM2835_GPLEV1                       0x0038 // GPIO Pin Level 1
#define BCM2835_GPEDS0                       0x0040 // GPIO Pin Event Detect Status 0
#define BCM2835_GPEDS1                       0x0044 // GPIO Pin Event Detect Status 1
#define BCM2835_GPREN0                       0x004c // GPIO Pin Rising Edge Detect Enable 0
#define BCM2835_GPREN1                       0x0050 // GPIO Pin Rising Edge Detect Enable 1
#define BCM2835_GPFEN0                       0x0058 // GPIO Pin Falling Edge Detect Enable 0
#define BCM2835_GPFEN1                       0x005c // GPIO Pin Falling Edge Detect Enable 1
#define BCM2835_GPHEN0                       0x0064 // GPIO Pin High Detect Enable 0
#define BCM2835_GPHEN1                       0x0068 // GPIO Pin High Detect Enable 1
#define BCM2835_GPLEN0                       0x0070 // GPIO Pin Low Detect Enable 0
#define BCM2835_GPLEN1                       0x0074 // GPIO Pin Low Detect Enable 1
#define BCM2835_GPAREN0                      0x007c // GPIO Pin Async. Rising Edge Detect 0
#define BCM2835_GPAREN1                      0x0080 // GPIO Pin Async. Rising Edge Detect 1
#define BCM2835_GPAFEN0                      0x0088 // GPIO Pin Async. Falling Edge Detect 0
#define BCM2835_GPAFEN1                      0x008c // GPIO Pin Async. Falling Edge Detect 1
#define BCM2835_GPPUD                        0x0094 // GPIO Pin Pull-up/down Enable
#define BCM2835_GPPUDCLK0                    0x0098 // GPIO Pin Pull-up/down Enable Clock 0
#define BCM2835_GPPUDCLK1                    0x009c // GPIO Pin Pull-up/down Enable Clock 1

#define BCM2835_ST_CS 							0x0000 ///< System Timer Control/Status
#define BCM2835_ST_CLO 							0x0004 ///< System Timer Counter Lower 32 bits
#define BCM2835_ST_CHI 							0x0008 ///< System Timer Counter Upper 32 bits

// Base of the ST (System Timer) registers.
// Available after bcm2835_init has been called
extern volatile uint32_t *bcm2835_st;

// Base of the GPIO registers.
// Available after bcm2835_init has been called
extern volatile uint32_t *bcm2835_gpio;

// Base of the BSC0 registers.
// Available after bcm2835_init has been called
extern volatile uint32_t *bcm2835_bsc0;

// Base of the BSC1 registers.
// Available after bcm2835_init has been called
extern volatile uint32_t *bcm2835_bsc1;

// GPIO Pin Numbers
//
// Here be defined Raspberry Pin GPIO pins on P1 in terms of the underlying BCM GPIO pin numbers.
// These can be passed as a pin number to any function requiring a pin.
// Not all pins on the RPi 26 bin IDE plug are connected to GPIO pins
// and some can adopt an alternate function.
// RPi version 2 has some slightly different pinouts, and these are values RPI_V2_*.
// RPi B+ has yet differnet pinouts and these are defined in RPI_BPLUS_*.
// At bootup, pins 8 and 10 are set to UART0_TXD, UART0_RXD (ie the alt0 function) respectively
// When SPI0 is in use (ie after bcm2835_spi_begin()), SPI0 pins are dedicated to SPI
// and cant be controlled independently.
// If we use the RPi Compute Module, just use the GPIO number: there is no need to use one of these
// symbolic names
typedef enum {
    RPI_GPIO_P1_03        =  0,  // Version 1, Pin P1-03
    RPI_GPIO_P1_05        =  1,  // Version 1, Pin P1-05
    RPI_GPIO_P1_07        =  4,  // Version 1, Pin P1-07
    RPI_GPIO_P1_08        = 14,  // Version 1, Pin P1-08, defaults to alt function 0 UART0_TXD
    RPI_GPIO_P1_10        = 15,  // Version 1, Pin P1-10, defaults to alt function 0 UART0_RXD
    RPI_GPIO_P1_11        = 17,  // Version 1, Pin P1-11
    RPI_GPIO_P1_12        = 18,  // Version 1, Pin P1-12, can be PWM channel 0 in ALT FUN 5
    RPI_GPIO_P1_13        = 21,  // Version 1, Pin P1-13
    RPI_GPIO_P1_15        = 22,  // Version 1, Pin P1-15
    RPI_GPIO_P1_16        = 23,  // Version 1, Pin P1-16
    RPI_GPIO_P1_18        = 24,  // Version 1, Pin P1-18
    RPI_GPIO_P1_19        = 10,  // Version 1, Pin P1-19, MOSI when SPI0 in use
    RPI_GPIO_P1_21        =  9,  // Version 1, Pin P1-21, MISO when SPI0 in use
    RPI_GPIO_P1_22        = 25,  // Version 1, Pin P1-22
    RPI_GPIO_P1_23        = 11,  // Version 1, Pin P1-23, CLK when SPI0 in use
    RPI_GPIO_P1_24        =  8,  // Version 1, Pin P1-24, CE0 when SPI0 in use
    RPI_GPIO_P1_26        =  7,  // Version 1, Pin P1-26, CE1 when SPI0 in use

    // RPi Version 2
    RPI_V2_GPIO_P1_03     =  2,  // Version 2, Pin P1-03
    RPI_V2_GPIO_P1_05     =  3,  // Version 2, Pin P1-05
    RPI_V2_GPIO_P1_07     =  4,  // Version 2, Pin P1-07
    RPI_V2_GPIO_P1_08     = 14,  // Version 2, Pin P1-08, defaults to alt function 0 UART0_TXD
    RPI_V2_GPIO_P1_10     = 15,  // Version 2, Pin P1-10, defaults to alt function 0 UART0_RXD
    RPI_V2_GPIO_P1_11     = 17,  // Version 2, Pin P1-11
    RPI_V2_GPIO_P1_12     = 18,  // Version 2, Pin P1-12, can be PWM channel 0 in ALT FUN 5
    RPI_V2_GPIO_P1_13     = 27,  // Version 2, Pin P1-13
    RPI_V2_GPIO_P1_15     = 22,  // Version 2, Pin P1-15
    RPI_V2_GPIO_P1_16     = 23,  // Version 2, Pin P1-16
    RPI_V2_GPIO_P1_18     = 24,  // Version 2, Pin P1-18
    RPI_V2_GPIO_P1_19     = 10,  // Version 2, Pin P1-19, MOSI when SPI0 in use
    RPI_V2_GPIO_P1_21     =  9,  // Version 2, Pin P1-21, MISO when SPI0 in use
    RPI_V2_GPIO_P1_22     = 25,  // Version 2, Pin P1-22
    RPI_V2_GPIO_P1_23     = 11,  // Version 2, Pin P1-23, CLK when SPI0 in use
    RPI_V2_GPIO_P1_24     =  8,  // Version 2, Pin P1-24, CE0 when SPI0 in use
    RPI_V2_GPIO_P1_26     =  7,  // Version 2, Pin P1-26, CE1 when SPI0 in use

    // RPi Version 2, new plug P5
    RPI_V2_GPIO_P5_03     = 28,  // Version 2, Pin P5-03
    RPI_V2_GPIO_P5_04     = 29,  // Version 2, Pin P5-04
    RPI_V2_GPIO_P5_05     = 30,  // Version 2, Pin P5-05
    RPI_V2_GPIO_P5_06     = 31,  // Version 2, Pin P5-06

    // RPi B+ J8 header
    RPI_BPLUS_GPIO_J8_03     =  2,  // B+, Pin J8-03
    RPI_BPLUS_GPIO_J8_05     =  3,  // B+, Pin J8-05
    RPI_BPLUS_GPIO_J8_07     =  4,  // B+, Pin J8-07
    RPI_BPLUS_GPIO_J8_08     = 14,  // B+, Pin J8-08, defaults to alt function 0 UART0_TXD
    RPI_BPLUS_GPIO_J8_10     = 15,  // B+, Pin J8-10, defaults to alt function 0 UART0_RXD
    RPI_BPLUS_GPIO_J8_11     = 17,  // B+, Pin J8-11
    RPI_BPLUS_GPIO_J8_12     = 18,  // B+, Pin J8-12, can be PWM channel 0 in ALT FUN 5
    RPI_BPLUS_GPIO_J8_13     = 27,  // B+, Pin J8-13
    RPI_BPLUS_GPIO_J8_15     = 22,  // B+, Pin J8-15
    RPI_BPLUS_GPIO_J8_16     = 23,  // B+, Pin J8-16
    RPI_BPLUS_GPIO_J8_18     = 24,  // B+, Pin J8-18
    RPI_BPLUS_GPIO_J8_19     = 10,  // B+, Pin J8-19, MOSI when SPI0 in use
    RPI_BPLUS_GPIO_J8_21     =  9,  // B+, Pin J8-21, MISO when SPI0 in use
    RPI_BPLUS_GPIO_J8_22     = 25,  // B+, Pin J8-22
    RPI_BPLUS_GPIO_J8_23     = 11,  // B+, Pin J8-23, CLK when SPI0 in use
    RPI_BPLUS_GPIO_J8_24     =  8,  // B+, Pin J8-24, CE0 when SPI0 in use
    RPI_BPLUS_GPIO_J8_26     =  7,  // B+, Pin J8-26, CE1 when SPI0 in use
    RPI_BPLUS_GPIO_J8_29     =  5,  // B+, Pin J8-29, 
    RPI_BPLUS_GPIO_J8_31     =  6,  // B+, Pin J8-31, 
    RPI_BPLUS_GPIO_J8_32     =  12, // B+, Pin J8-32, 
    RPI_BPLUS_GPIO_J8_33     =  13, // B+, Pin J8-33, 
    RPI_BPLUS_GPIO_J8_35     =  19, // B+, Pin J8-35, 
    RPI_BPLUS_GPIO_J8_36     =  16, // B+, Pin J8-36, 
    RPI_BPLUS_GPIO_J8_37     =  26, // B+, Pin J8-37, 
    RPI_BPLUS_GPIO_J8_38     =  20, // B+, Pin J8-38, 
    RPI_BPLUS_GPIO_J8_40     =  21, // B+, Pin J8-40, 
} RPiGPIOPin;

// bcm2835FunctionSelect
// Port function select modes for gpio_fsel()
typedef enum {
    BCM2835_GPIO_FSEL_INPT  = 0b000,   // Input
    BCM2835_GPIO_FSEL_OUTP  = 0b001,   // Output
    BCM2835_GPIO_FSEL_ALT0  = 0b100,   // Alternate function 0
    BCM2835_GPIO_FSEL_ALT1  = 0b101,   // Alternate function 1
    BCM2835_GPIO_FSEL_ALT2  = 0b110,   // Alternate function 2
    BCM2835_GPIO_FSEL_ALT3  = 0b111,   // Alternate function 3
    BCM2835_GPIO_FSEL_ALT4  = 0b011,   // Alternate function 4
    BCM2835_GPIO_FSEL_ALT5  = 0b010,   // Alternate function 5
    BCM2835_GPIO_FSEL_MASK  = 0b111    // Function select bits mask
} bcm2835FunctionSelect;

// I2CReasonCodes
// Specifies the reason codes for the i2c_write and i2c_read functions.
typedef enum {
    I2C_REASON_OK   	     = 0x00,      // Success
    I2C_REASON_ERROR_NACK    = 0x01,      // Received a NACK
    I2C_REASON_ERROR_CLKT    = 0x02,      // Received Clock Stretch Timeout
    I2C_REASON_ERROR_DATA    = 0x04,      // Not all data is sent/received
} I2CReasonCodes;

// I2CClockDivider
// Specifies the divider used to generate the I2C clock from the system clock.
// Clock divided is based on nominal base clock rate of 250MHz
typedef enum
{
    I2C_CLOCK_DIVIDER_2500   = 2500,      // 2500 = 10us = 100 kHz
    I2C_CLOCK_DIVIDER_626    = 626,       // 626 = 2.504us = 399.3610 kHz
    I2C_CLOCK_DIVIDER_150    = 150,       // 150 = 60ns = 1.666 MHz (default at reset)
    I2C_CLOCK_DIVIDER_148    = 148,       // 148 = 59ns = 1.689 MHz
} I2CClockDivider;

// Initialise the library by opening /dev/mem and getting pointers to the 
// internal memory for BCM2835 device registers. You must call this (successfully)
// before calling any other 
// functions in this library. 
// If bcm2835_init() fails by returning 0, 
// calling any other function may result in crashes or other failures.
// Prints messages to stderr in case of errors.
// 
// return 1 if successful else 0 (boolean)
extern int bcm2835_init(void);

// Close the library, deallocating any allocated memory and closing /dev/mem
// return 1 if successful else 0 (boolean)
extern int bcm2835_close(void);

// Sets the Function Select register for the given pin, which configures
// the pin as Input, Output or one of the 6 alternate functions.
//
// Parameters:
// pin 		GPIO number, or one of RPI_GPIO_P1_* from RPiGPIOPin.
// mode 	Mode to set the pin to, one of BCM2835_GPIO_FSEL_* from bcm2835FunctionSelect
extern void gpio_fsel(uint8_t pin, uint8_t mode);

// Alters a number of bits in a 32 peripheral regsiter.
// It reads the current value and then alters the bits defined as 1 in mask, 
// according to the bit value in value. 
// All other bits that are 0 in the mask are unaffected.
// Use this to alter a subset of the bits in a register.
// The write is done twice, and is therefore always safe in terms of 
// manual section 1.3 Peripheral access precautions for correct memory ordering
//
// Parameters:
// paddr 	Physical address to read from. See BCM2835_GPIO_BASE etc.
// value 	The 32 bit value to write, masked in by mask.
// mask 	Bitmask that defines the bits that will be altered in the register.
extern void peri_set_bits(volatile uint32_t* paddr, uint32_t value, uint32_t mask);

// Reads 32 bit value from a peripheral address
// The read is done twice, and is therefore always safe in terms of 
// manual section 1.3 Peripheral access precautions for correct memory ordering
//
// Parameter:
// paddr 	Physical address to read from. See BCM2835_GPIO_BASE etc.
//
// Returns the value read from the 32 bit register
extern uint32_t peri_read(volatile uint32_t* paddr);

// Writes 32 bit value from a peripheral address
// The write is done twice, and is therefore always safe in terms of 
// manual section 1.3 Peripheral access precautions for correct memory ordering
//
// Parameters:
// paddr		Physical address to read from. See BCM2835_GPIO_BASE etc.
// value		The 32 bit value to write
extern void peri_write(volatile uint32_t* paddr, uint32_t value);

// Writes 32 bit value from a peripheral address without the write barrier
// You should only use this when your code has previously called peri_write()
// within the same peripheral, and no other peripheral access has occurred since.
//
// Parameters:
// paddr		Physical address to read from. See BCM2835_GPIO_BASE etc.
// value		The 32 bit value to write
extern void peri_write_nb(volatile uint32_t* paddr, uint32_t value);

// Reads 32 bit value from a peripheral address without the read barrier
// You should only use this when your code has previously called bcm2835_peri_read()
// within the same peripheral, and no other peripheral access has occurred since.
//
// Parameter:
// paddr	Physical address to read from. See BCM2835_GPIO_BASE etc.
// Returns the value read from the 32 bit register
extern uint32_t bcm2835_peri_read_nb(volatile uint32_t* paddr);

// Start I2C operations.
// Forces RPi I2C pins P1-03 (SDA) and P1-05 (SCL)
// to alternate function ALT0, which enables those pins for I2C interface.
// You should call i2c_end() when all I2C functions are complete to return the pins to
// their default functions
extern void i2c_begin(void);

// End I2C operations.
// I2C pins P1-03 (SDA) and P1-05 (SCL)
// are returned to their default INPUT behaviour.
extern void i2c_end(void);

// Sets the I2C slave address.
//
// Parameter:
// addr		The I2C slave address.
extern void i2c_setSlaveAddress(uint8_t addr);

// Sets the I2C clock divider and therefore the I2C clock speed.
//
// Parameter:
// divider		The desired I2C clock divider, one of I2C_CLOCK_DIVIDER_*,
// see I2CClockDivider
extern void i2c_setClockDivider(uint16_t divider);

// Sets the I2C clock divider by converting the baudrate parameter to
// the equivalent I2C clock divider. (see i2c_setClockDivider)
// For the I2C standard 100khz you would set baudrate to 100000
// The use of baudrate corresponds to its use in the I2C kernel device
// driver. (Of course, bcm2835 has nothing to do with the kernel driver)
extern void i2c_set_baudrate(uint32_t baudrate);

// Transfers any number of bytes to the currently selected I2C slave.
// (as previously set by i2c_setSlaveAddress)
//
// Parameters:
// buf		Buffer of bytes to send.
// len		Number of bytes in the buf buffer, and the number of bytes to send.
//
// return reason (see I2CReasonCodes)
extern uint8_t i2c_write(const char * buf, uint32_t len);

// Transfers any number of bytes from the currently selected I2C slave.
// (as previously set by i2c_setSlaveAddress)
//
// Parameters:
// buf		Buffer of bytes to receive.
// len		Number of bytes in the buf buffer, and the number of bytes to receive.
//
// return reason (see I2CReasonCodes)
extern uint8_t i2c_read(char* buf, uint32_t len);

// Allows reading from I2C slaves that require a repeated start (without any prior stop)
// to read after the required slave register has been set. For example, the popular
// MPL3115A2 pressure and temperature sensor. Note that your device must support or
// require this mode. If your device does not require this mode then the standard
// combined:
//   i2c_write
//   i2c_read
// are a better choice.
// Will read from the slave previously set by i2c_setSlaveAddress

// Parameters:
// regaddr		Buffer containing the slave register you wish to read from.
// buf			Buffer of bytes to receive.
// len			Number of bytes in the buf buffer, and the number of bytes to received.
//
// return reason (see I2CReasonCodes)
extern uint8_t i2c_read_register_rs(char* regaddr, char* buf, uint32_t len);

// Allows sending an arbitrary number of bytes to I2C slaves before issuing a repeated
// start (with no prior stop) and reading a response.
// Necessary for devices that require such behavior, such as the MLX90620.
// Will write to and read from the slave previously set by i2c_setSlaveAddress
//
// Parameters:
// cmds 		Buffer containing the bytes to send before the repeated start condition.
// cmds_len		Number of bytes to send from cmds buffer
// buf 			Buffer of bytes to receive.
// buf_len		Number of bytes to receive in the buf buffer.
//
// return reason (see I2CReasonCodes)
extern uint8_t i2c_write_read_rs(char* cmds, uint32_t cmds_len, char* buf, uint32_t buf_len);

// Delays for the specified number of microseconds.
// Uses a combination of nanosleep() and a busy wait loop on the BCM2835 system timers,
// However, you are at the mercy of nanosleep(). From the manual for nanosleep():
// If the interval specified in req is not an exact multiple of the granularity  
// underlying  clock  (see  time(7)),  then the interval will be
// rounded up to the next multiple. Furthermore, after the sleep completes, 
// there may still be a delay before the CPU becomes free to once
// again execute the calling thread.
// For times less than about 450 microseconds, uses a busy wait on the System Timer.
// It is reported that a delay of 0 microseconds on RaspberryPi will in fact
// result in a delay of about 80 microseconds. Your mileage may vary.
extern void delayMicroseconds(uint64_t micros);

// Read the System Timer Counter register.
// Returns the value read from the System Timer Counter Lower 32 bits register
extern uint64_t bcm2835_st_read(void);

// Delays for the specified number of microseconds with offset.
//
// Parameters
// offset_micros	Offset in microseconds
// micros			Delay in microseconds
extern void bcm2835_st_delay(uint64_t offset_micros, uint64_t micros);
