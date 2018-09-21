/*************************************************** 
 This is a library for the MCP23017 i2c port expander

 These displays use I2C to communicate, 2 pins are required to
 interface
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries.
 BSD license, all text above must be included in any redistribution
 ****************************************************/

/* ---------------------------------------------------------------
 *
 * Note for the ESP-32 IDF: The i2c subsystem is not initalized 
 * by any of this code.  You must do that before calling the 
 * begin() method of this class.
 *
 * See the generic_i2c_master_init() function in generic_i2c_rw.{h,cpp}
 *
 * ---------------------------------------------------------------
 */

#include <stdint.h>
#include "Adafruit_MCP23017.h"
#include <driver/gpio.h>
#include <driver/i2c.h>
#include "generic_i2c_rw.h"


// minihelper to keep Arduino backward compatibility
/*
static inline void wiresend(uint8_t x) {
    //Wire.write((uint8_t) x);
    //i2c_master_write(cmd, &x, 1, 0);
}
*/

/*
static inline uint8_t wirerecv(void) {

    //return Wire.read();
    return 0;
}
*/

static int bitRead(uint32_t x, uint8_t n) {
    return ((x & 1 << n) != 0);
}

static uint32_t bitWrite(uint32_t x, uint8_t n, uint8_t b) {
    return ((x & (~(1 << n))) | (b << n));
}

/**
 * Bit number associated to a give Pin
 */
uint8_t Adafruit_MCP23017::bitForPin(uint8_t pin){
    return pin % 8;
}

/**
 * Register address, port dependent, for a given PIN
 */
uint8_t Adafruit_MCP23017::regForPin(uint8_t pin, uint8_t portAaddr, uint8_t portBaddr) {
    return (pin < 8) ? portAaddr : portBaddr;
}


/**
 * Read a 16 bit register value
 */
uint16_t Adafruit_MCP23017::readRegisterWord(uint8_t addr) {
	return generic_read_i2c_register_word (MCP23017_ADDRESS | i2caddr, addr, i2c_port);
}

/**
 * Write a 16 bit register value
 */
void Adafruit_MCP23017::writeRegisterWord(uint8_t addr, uint16_t value) {
	generic_write_i2c_register_word (MCP23017_ADDRESS | i2caddr, addr, value, i2c_port);
}

/**
 * Reads a given register
 */
uint8_t Adafruit_MCP23017::readRegister(uint8_t addr){
	return generic_read_i2c_register (MCP23017_ADDRESS | i2caddr, addr, i2c_port);
}

/**
 * Writes a given register
 */
void Adafruit_MCP23017::writeRegister(uint8_t regAddr, uint8_t regValue){
	generic_write_i2c_register (MCP23017_ADDRESS | i2caddr, regAddr, regValue, i2c_port);
}


/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
void Adafruit_MCP23017::updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t portAaddr, uint8_t portBaddr) {
    uint8_t regValue;
    uint8_t regAddr = regForPin(pin, portAaddr, portBaddr);
    uint8_t bit = bitForPin(pin);

    regValue = readRegister(regAddr);

    // set the value for the particular bit
    regValue = bitWrite(regValue, bit, pValue);

    writeRegister(regAddr, regValue);
}

////////////////////////////////////////////////////////////////////////////////

/**
 * Initializes the MCP23017 given its HW selected address, see datasheet for Address selection.
 *
 * Note: Address means the actual address as set by the hardware pins, 0 to 7
 *
 * Just to be nice, only the lower 3 bits of the address are looked at, so if 
 * the user sends in the logical address, they don't get unexpected results.
 *
 */
void Adafruit_MCP23017::begin(uint8_t addr, uint8_t i2cport) {

	i2c_port = i2cport;
    i2caddr = addr & 7;

    //Wire.begin();

    // set defaults!
    // all inputs on port A and B
    writeRegisterWord(MCP23017_IODIRA, 0xffff);

}

/**
 * Initializes the default MCP23017, with 000 for the configurable part of the address
 */
void Adafruit_MCP23017::begin(void) {
    begin(0);
}

/**
 * Sets the pin mode to either INPUT or OUTPUT
 */
void Adafruit_MCP23017::pinMode(uint8_t p, uint8_t d) {
    updateRegisterBit(p, (d==GPIO_MODE_INPUT), MCP23017_IODIRA, MCP23017_IODIRB);
}

/**
 * Reads all 16 pins (port A and B) into a single 16 bits variable.
 */
uint16_t Adafruit_MCP23017::readGPIOAB() {
    return readRegisterWord (MCP23017_GPIOA);
}

/**
 * Read a single port, A or B, and return its current 8 bit value.
 * Parameter b should be 0 for GPIOA, and 1 for GPIOB.
 */
uint8_t Adafruit_MCP23017::readGPIO(uint8_t b) {

    return readRegister(b==0 ? MCP23017_GPIOA : MCP23017_GPIOB);
}

/**
 * Writes all the pins in one go. This method is very useful if you are implementing a multiplexed matrix and want to get a decent refresh rate.
 */
void Adafruit_MCP23017::writeGPIOAB(uint16_t ba) {
    writeRegisterWord(MCP23017_GPIOA, ba);
}

void Adafruit_MCP23017::digitalWrite(uint8_t pin, uint8_t d) {
    uint8_t gpio;
    uint8_t bit = bitForPin(pin);


    // read the current GPIO output latches
    uint8_t regAddr = regForPin(pin, MCP23017_OLATA, MCP23017_OLATB);
    gpio = readRegister(regAddr);

    // set the pin and direction
    gpio = bitWrite(gpio, bit, d);

    // write the new GPIO
    regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
    writeRegister(regAddr, gpio);
}

void Adafruit_MCP23017::pullUp(uint8_t p, uint8_t d) {
    updateRegisterBit(p, d, MCP23017_GPPUA, MCP23017_GPPUB);
}

uint8_t Adafruit_MCP23017::digitalRead(uint8_t pin) {
    uint8_t bit = bitForPin(pin);
    uint8_t regAddr = regForPin(pin, MCP23017_GPIOA, MCP23017_GPIOB);
    return (readRegister(regAddr) >> bit) & 0x1;
}

/**
 * Configures the interrupt system. both port A and B are assigned the same configuration.
 * Mirroring will OR both INTA and INTB pins.
 * Opendrain will set the INT pin to value or open drain.
 * polarity will set LOW or HIGH on interrupt.
 * Default values after Power On Reset are: (false,flase, LOW)
 * If you are connecting the INTA/B pin to arduino 2/3, you should configure the interupt handling as FALLING with
 * the default configuration.
 */
void Adafruit_MCP23017::setupInterrupts(uint8_t mirroring, uint8_t openDrain, uint8_t polarity){
    // configure the port A
    uint8_t ioconfValue=readRegister(MCP23017_IOCONA);
    ioconfValue = bitWrite(ioconfValue,6,mirroring);
    ioconfValue = bitWrite(ioconfValue,2,openDrain);
    ioconfValue = bitWrite(ioconfValue,1,polarity);
    writeRegister(MCP23017_IOCONA,ioconfValue);

    // Configure the port B
    ioconfValue=readRegister(MCP23017_IOCONB);
    ioconfValue = bitWrite(ioconfValue,6,mirroring);
    ioconfValue = bitWrite(ioconfValue,2,openDrain);
    ioconfValue = bitWrite(ioconfValue,1,polarity);
    writeRegister(MCP23017_IOCONB,ioconfValue);
}

/**
 * Sets up a pin for interrupt. uses arduino MODEs: CHANGE, FALLING, RISING.
 *
 * Note that the interrupt condition finishes when you read the information about the port / value
 * that caused the interrupt or you read the port itself. Check the datasheet can be confusing.
 *
 */
void Adafruit_MCP23017::setupInterruptPin(uint8_t pin, uint8_t mode) {

    // set the pin interrupt control (0 means change, 1 means compare against given value);
//  updateRegisterBit(pin,(mode!=CHANGE),MCP23017_INTCONA,MCP23017_INTCONB);
    // if the mode is not CHANGE, we need to set up a default value, different value triggers interrupt

    // In a RISING interrupt the default value is 0, interrupt is triggered when the pin goes to 1.
    // In a FALLING interrupt the default value is 1, interrupt is triggered when pin goes to 0.
//  updateRegisterBit(pin,(mode==FALLING),MCP23017_DEFVALA,MCP23017_DEFVALB);

    // enable the pin for interrupt
//  updateRegisterBit(pin,HIGH,MCP23017_GPINTENA,MCP23017_GPINTENB);

}

uint8_t Adafruit_MCP23017::getLastInterruptPin(){
    uint8_t intf;

    // try port A
    intf=readRegister(MCP23017_INTFA);
    for(int i=0;i<8;i++) if (bitRead(intf,i)) return i;

    // try port B
    intf=readRegister(MCP23017_INTFB);
    for(int i=0;i<8;i++) if (bitRead(intf,i)) return i+8;

    return MCP23017_INT_ERR;

}
uint8_t Adafruit_MCP23017::getLastInterruptPinValue(){
    uint8_t intPin=getLastInterruptPin();
    if(intPin!=MCP23017_INT_ERR){
        uint8_t intcapreg=regForPin(intPin,MCP23017_INTCAPA,MCP23017_INTCAPB);
        uint8_t bit=bitForPin(intPin);
        return (readRegister(intcapreg)>>bit) & (0x01);
    }

    return MCP23017_INT_ERR;
}


