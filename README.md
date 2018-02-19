Generic i2c register read/write routines

Copyright 2017 by Pete Cervasio.
See the source for license details.

This is a somewhat generic i2c read/write library for the
ESP32.  Five functions are defined:

generic_i2c_master_init 

	An easy function to initialize the hardware as an i2c master

generic_read_i2c_register

	Read from an 8 bit register on an i2c device

generic_read_i2c_register_word

	Read from a 16 bit register on an i2c device

generic_write_i2c_register

	Write to an 8 bit register on an i2c device

generic_write_i2c_register_word

	Write to a 16 bit register on an i2c device

See the .h and .cpp files for specifics.

For a bit of test code, the Adafruit MCP23017 library, which was converted
to ESP32 native code by Neil Kolban, was further converted to use this
library for doing the i2c I/O.  There is also a test program which verifies
that the software works with the hardware.

Check out Neil's ESP-32 snippets at https://github.com/nkolban, and also his
Youtube channel at https://www.youtube.com/channel/UChKn_BlaVrMrhEquPNI6HuQ
which has lots of informative ESP8266 and ESP32 videos.

Also: Thanks to Antonio Fiol for finding bugs and reporting them.
