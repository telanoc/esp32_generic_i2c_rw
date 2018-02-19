/*
 * ESP32 IDF Test program for the Adafruit MCP23017 library which was ported
 * over to ESP32 IDF native code by Neil Kolban, and further ported to this
 * "esp32_generic_i2c_rw" library by Pete Cervasio.
 *
 * This test program presumes:
 *  a). MCP23017 wired for address 0
 *
 *  b). Port A and B tied together (bit 0 to bit 0, etc) and also to
 *      the cathode of an LED connected to VCC through a few hundred
 *      ohms.  Only one port is set for output at a time.
 *
 *  c). Using GPIO 19 for SCL (mcp pin 12)
 *      Using GPIO 18 for SDA (mcp pin 13)
 *
 *  d). GPIO 17 is tied to the MCP23017 /RESET pin
 *
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "Adafruit_MCP23017.h"
#include "generic_i2c_rw.h"

#define I2C_MASTER_SCL_IO    19          /* gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO    18          /* gpio number for I2C master data  */
#define I2C_MASTER_TX_BUF_DISABLE   0    /* I2C master does not need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0    /* I2C master does not need buffer */
#define I2C_MASTER_FREQ_HZ    400000     /* I2C master clock frequency */

#define MCP23017_RESET_GPIO 17


// VERBOSE is really 'extra verbose'
//#define VERBOSE

Adafruit_MCP23017 mcp;


void set_directions (int a)
{
    // Set first port pins to input
    for (int i = (a * 8); i < 8 + (a * 8); i++) {
#ifdef VERBOSE
        printf("  -> Pin %d set to input\n", i);
#endif
        mcp.pinMode(i, GPIO_MODE_INPUT);
    }

    // Set second port to output with high level
    for (int i = (1-a) * 8; i < (1-a) * 8 + 8; i++) {
#ifdef VERBOSE
        printf("  -> Pin %d set to output\n", i);
#endif
        mcp.pinMode (i, GPIO_MODE_OUTPUT);
#ifdef VERBOSE
        printf("  -> Pin %d written high\n", i);
#endif
		mcp.digitalWrite (i, 1);
    }

}


void do_led(int start)
{
    uint8_t val;
    uint16_t value = mcp.readRegisterWord(MCP23017_IODIRA);
	printf ("   MCP23017_IODIRA = %04x\n", value);

    for (int i = start * 8; i < start * 8 + 8; i++) {
        mcp.digitalWrite (i, 0); // pull pin low to light LED
        //vTaskDelay (10 / portTICK_RATE_MS);
        val = mcp.readGPIO(1-start);
        printf ("   Write pin %02d  Port %c read=%02x\n", i, start==0?'A':'B', val);
        vTaskDelay (200 / portTICK_RATE_MS);
        mcp.digitalWrite (i, 1);
    }
    printf("\n");
}

void led_march(int a)
{
    if (a == 0) {
        printf("--- Setting port A for write\n");
        set_directions (1);
        printf("*** Lighting port A LEDs\n");
        do_led (0);
    } else {
        printf("--- Setting port B for write\n");
        set_directions (0);
        printf("*** Lighting port B LEDs\n");
        do_led (1);
    }
}

void read_words()
{
	uint16_t value;

    value = mcp.readRegisterWord(MCP23017_IODIRA);   printf ("Value read from MCP23017_IODIRA is %04x\n", value);
    value = mcp.readRegisterWord(MCP23017_IPOLA);    printf ("Value read from MCP23017_IPOLA is %04x\n", value);
    value = mcp.readRegisterWord(MCP23017_GPINTENA); printf ("Value read from MCP23017_GPINTENA is %04x\n", value);
    value = mcp.readRegisterWord(MCP23017_DEFVALA);  printf ("Value read from MCP23017_DEFVALA is %04x\n", value);
    value = mcp.readRegisterWord(MCP23017_INTCONA);  printf ("Value read from MCP23017_INTCONA is %04x\n", value);
    value = mcp.readRegisterWord(MCP23017_IOCONA);   printf ("Value read from MCP23017_IOCONA is %04x\n", value);
    value = mcp.readRegisterWord(MCP23017_GPPUA);    printf ("Value read from MCP23017_GPPUA is %04x\n", value);
    value = mcp.readRegisterWord(MCP23017_INTFA);    printf ("Value read from MCP23017_INTFA is %04x\n", value);
    value = mcp.readRegisterWord(MCP23017_INTCAPA);  printf ("Value read from MCP23017_INTCAPA is %04x\n", value);
    value = mcp.readRegisterWord(MCP23017_GPIOA);    printf ("Value read from MCP23017_GPIOA is %04x\n", value);
    value = mcp.readRegisterWord(MCP23017_OLATA);    printf ("Value read from MCP23017_OLATA is %04x\n", value);
}

void read_bytes()
{
	uint8_t value;

    value = mcp.readRegister(MCP23017_IODIRA);   printf ("Value read from MCP23017_IODIRA is %02x\n", value);
    value = mcp.readRegister(MCP23017_IODIRB);   printf ("Value read from MCP23017_IODIRB is %02x\n", value);
    value = mcp.readRegister(MCP23017_GPINTENA); printf ("Value read from MCP23017_GPINTENA is %02x\n", value);
    value = mcp.readRegister(MCP23017_GPINTENB); printf ("Value read from MCP23017_GPINTENB is %02x\n", value);
    value = mcp.readRegister(MCP23017_DEFVALA);  printf ("Value read from MCP23017_DEFVALA is %02x\n", value);
    value = mcp.readRegister(MCP23017_DEFVALB);  printf ("Value read from MCP23017_DEFVALB is %02x\n", value);
    value = mcp.readRegister(MCP23017_INTCONA);  printf ("Value read from MCP23017_INTCONA is %02x\n", value);
    value = mcp.readRegister(MCP23017_INTCONB);  printf ("Value read from MCP23017_INTCONB is %02x\n", value);
    value = mcp.readRegister(MCP23017_IOCONA);   printf ("Value read from MCP23017_IOCONA is %02x\n", value);
    value = mcp.readRegister(MCP23017_IOCONB);   printf ("Value read from MCP23017_IOCONB is %02x\n", value);
    value = mcp.readRegister(MCP23017_GPPUA);    printf ("Value read from MCP23017_GPPUA is %02x\n", value);
    value = mcp.readRegister(MCP23017_GPPUB);    printf ("Value read from MCP23017_GPPUB is %02x\n", value);
    value = mcp.readRegister(MCP23017_INTFA);    printf ("Value read from MCP23017_INTFA is %02x\n", value);
    value = mcp.readRegister(MCP23017_INTFB);    printf ("Value read from MCP23017_INTFB is %02x\n", value);
    value = mcp.readRegister(MCP23017_INTCAPA);  printf ("Value read from MCP23017_INTCAPA is %02x\n", value);
    value = mcp.readRegister(MCP23017_INTCAPB);  printf ("Value read from MCP23017_INTCAPB is %02x\n", value);
    value = mcp.readRegister(MCP23017_GPIOA);    printf ("Value read from MCP23017_GPIOA is %02x\n", value);
    value = mcp.readRegister(MCP23017_GPIOB);    printf ("Value read from MCP23017_GPIOB is %02x\n", value);
    value = mcp.readRegister(MCP23017_OLATA);    printf ("Value read from MCP23017_OLATA is %02x\n", value);
    value = mcp.readRegister(MCP23017_OLATB);    printf ("Value read from MCP23017_OLATB is %02x\n", value);
}

void reset_mcp23017()
{
    gpio_pad_select_gpio((gpio_num_t)MCP23017_RESET_GPIO);
    gpio_set_direction((gpio_num_t)MCP23017_RESET_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)MCP23017_RESET_GPIO, 0);
    vTaskDelay (1); /* one tick, whatever that's set to */
    gpio_set_level((gpio_num_t)MCP23017_RESET_GPIO, 1);
    vTaskDelay (1);
}


extern "C" void app_main(void)
{

	// Do an initialization of the i2c driver
    generic_i2c_master_init (I2C_NUM_0, I2C_MASTER_SCL_IO, I2C_MASTER_SDA_IO, I2C_MASTER_FREQ_HZ);

	// Pull mcp23017 reset line low for a moment
	reset_mcp23017();	

	// Init the Adafruit device
	mcp.begin(0x20); /* test device address handling. */

#ifdef VERBOSE
	read_bytes();
#endif
	read_words();

    while (true) {
        led_march (0);
        led_march (1);
    }
}

