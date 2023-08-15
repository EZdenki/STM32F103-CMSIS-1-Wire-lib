# STM32F103-CMSIS-1-Wire-lib

A simple 1-Wire library for the STM32F103 (Blue Pill). Includes a sample project to read a single attached DS18B20 1-Wire temperature sensor and display the temperature on a 16x2 LCD connected with an I2C LCD driver module.
## 1-Wire Functions Supported in the Library:
+ uint32_t W1_resetPulse( void )<br>
Sends reset pulse to 1-Wire device and returns 1 if a device is detected.
+ void W1_writeByte( uint8_t data )<br>
Write 8 bits of data (from least significant bit) to the 1-Wire device.
+ uint8_t W1_readByte( void )<br>
Reads a byte of data from the 1-Wire bus.<br>
### Notes:
1. These functions have not been tested on 1-Wire devices other than the DS18B20.<br>
2. Code is hard-coded so that the 1-Wire device IO pin must be connected to GPIO pin B12.<br>
