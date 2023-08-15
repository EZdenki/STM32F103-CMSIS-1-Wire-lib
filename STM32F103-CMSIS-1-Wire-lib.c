//  STM32F103-CMSIS-1-Wire-lib.c
//
//  Simple library to implement the 1-Wire protocol on an STM32F103 (Blue Pill) using CMSIS.
//
//    Mike Shegedin, EZdenki.com
//
//      Version 1.0   15 Aug 2023   Updated core files, cleaned up code, renamed files
//      Version 0.9      Jun 2023   Started
//
//  Target Microcontroller: STM32F103 (Blue Pill)
//
//  Implement the 1-Wire protocol with enough functionality to be able to communicate with a
//  DS18B20 temperature sensor. Once this is working, other 1-Wire devices should be possible.
//  Will focus on having the devices powered directly with VDD -- not "parasite power" via the
//  data pin. Parasite power can/should be attempted later.
//
//  NOTE: This implementation is hardwared for GPIO pin B12. Changing to another pin will
//        require updating various parts of this code! Making the code use an aribtrary
//        IO pin would end up reducing the readability. Maybe do so in a later version!
//
// ================================================================== 
//                ******  HARDWARE SETUP for DS18B20  ******
//                         Hardwired for GPIO B12 !
//                                            ===
//
//              DS18B20  Blue Pill  POWER  
//              =======  =========  =====
//                          GND ---- GND
//                           5V ----  5V
//
//             |  GND -------------- GND
//     DS18B20 | DATA ----- B12 --------------------, 
//             |  VDD --------------  5V -- [5K6] --'  (Pullup)
//
// ================================================================== 

#ifndef __STM32F103_CMSIS_1_WIRE_LIB_C
#define __STM32F103_CMSIS_1_WIRE_LIB_C

#include "stm32f103xb.h"                  // Primary CMSIS header file
#include "STM32F103-Delay-lib.c"          // Library for pause and delay_us functions


// Timing parameters required for 1-Wire interface
#define W1_T_SLOT   80    // Read/Write full time slot length
#define W1_T_REC    5     // Recovery time between bits
#define W1_T_L0     80    // 0-bit low time
#define W1_T_L1     5     // 1-bit low time
#define W1_T_H1     25    // 1-bit high time
#define W1_T_RT     5     // Time to pulse the line low to initiate a read
#define W1_T_RRT    75    // Time to wait for end of the time slot for read
#define W1_T_RSTL   480   // Reset pulse low time
#define W1_T_RSTD   100   // Time after reset low pulse to detect low from device
#define W1_T_RSTR   240   // Time to wait after reset pulse detected for end of reset 

// Command defines -- may or may not be specific to the DS18B20
#define W1_R_ROM        0x33  // Command to read the ID, family ID, and CRC of a single device
#define W1_R_SKIP_ROM   0xCC  // Command to address all devices on the 1-Wire bus
#define W1_R_CNVT       0x44  // Command to initiate a single temperature converstion
#define W1_R_RD_SCRATCH 0xBE  // Command to read scatchpad memory to get temperature and other
                              // scratchpad data, including alarm/user data and CRC byte.

//  void
//  W1_portInput( void )
//  Set the 1-Wire port to be in an input state
void
W1_portInput( void )
{
  GPIOB->CRH &= ~( 0b1111 << GPIO_CRH_MODE12_Pos );   // Clear CNF12[1:0] and MODE12[1:0] bits
  GPIOB->CRH |=  ( 0b0100 << GPIO_CRH_MODE12_Pos );   // Floating input
}


//  void
//  W1_portOutput( void )
//  Set the 1-Wire port to be in an output state
void
W1_portOutput( void )
{
  GPIOB->CRH &= ~( 0b1111 << GPIO_CRH_MODE12_Pos );   // Clear CNF12[1:0] and MODE12[1:0] bits
  GPIOB->CRH |=  ( 0b0101 << GPIO_CRH_MODE12_Pos );   // Open drain output
}


//  void
//  W1_pulse( uint32_t len )
//  Bring the 1-Wire bus low, wait for the specified time in microseconds, and then release
//  the bus back to its high state.
//  Note that this routine does not initially put the GPIO pin into an output state because
//  the routine will be used in serial (repeated output) operations and the repeated GPIO
//  settings would be a waste of resources and time.
void
W1_pulse( uint32_t len )
{
  GPIOB->BSRR |= GPIO_BSRR_BR12;
  delay_us( len );
  GPIOB->BSRR |= GPIO_BSRR_BS12;
}


//  void
//  W1_init( void )
//  Simply enables the GPIO port used for the GPIO pin used for the 1-Wire bus.
//  Note that the input/output state of the pin is not set here as 1-Wire requires changing
//  the state of the pin between input and output during communications.
void
W1_init( void )
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   // Enable GPIO PORT B
}


//  Macro that returns non-zero value if the 1-Wire bus is in a high state.
#define W1_high() (GPIOB->IDR &= GPIO_IDR_IDR12)


//  Macro that returns non-zero value if the 1-Wire bus is in a low state.
#define W1_low() ~(GPIOB->IDR &= GPIO_IDR_IDR12)


//  uint32_t
//  W1_resetPulse()
//  Sends the reset pulse to the 1-Wire device and checks for the resulting detect-pulse
//  low response. Returns 1 if the detect pulse is properly detected.
uint32_t
W1_resetPulse( void )
{
  uint32_t resetOK;

  W1_portOutput();        // Change port to output state
  W1_pulse( W1_T_RSTL );  // Send reset low pulse
  delay_us( W1_T_RSTD );  // Give time for device to respond
  resetOK = W1_low();     // Should detect low from the device if present
  delay_us( W1_T_RSTR );  // Wait for end of reset low pulse
  return resetOK;
}


//  void
//  W1_writeBit( uint32_t bit )
//  If "bit" is non-zero then a 1-bit is sent to the 1-Wire bus, otherwise a 0-bit is sent.
//  Note that this routine does not set the IO port to be in a read state as the routine will
//  generally be used to read bytes of data at a time, making subsequent setting of the
//  port as output between bits a waste of time and resources.
void
W1_writeBit( uint32_t bit )
{
  if( bit )
  {
    W1_pulse( W1_T_L1  );   // Send a short low
    delay_us( W1_T_H1 );    // Fill the rest of the time slot
  }
  else
    W1_pulse( W1_T_L0  );   // Send a long low (length of time slot)
  
  delay_us( W1_T_REC );     // Pause for recovery time
}


//  void
//  W1_writeByte( uint8_t data )
//  Write 8 bits of "data" to the 1-Wire bus from the least significant bit.
void
W1_writeByte( uint8_t data )
{
  for( uint8_t x = 0; x<8; x++)   // Count 8 bits
  {
    W1_writeBit( data & 0x01 );   // Output the least significant bit
    data >>= 1;                   // Shift the data byte to the right to get the next bit
  }                               // The last shift is unneeded, but done to the copy of
}                                 // "data" no not consequential.


//  uint32_t
//  W1_readBit( void )
//  Reads a bit from the 1-Wire bus and returns a 0 if a 0-bit is read, otherwise a non-zero
//  value will be returned.
uint32_t
W1_readBit( void )
{
  uint32_t readResult;

  W1_portOutput();        // Put port in output mode
  W1_pulse( W1_T_RT );    // Pulse line low to tell device we want to read a bit
  W1_portInput();         // Change port to input mode
  readResult = W1_high(); // Read the bit on the port
  delay_us( W1_T_RRT + W1_T_REC );    // Wait till end of time slot plus recovery time
  return readResult;
}


//  uint8_t
//  W1_readByte( void )
//  Reads a byte of data from the 1-Wire bus and returns the byte to the calling routine.
uint8_t
W1_readByte( void )
{
  uint8_t data = 0;

  for( uint8_t x = 0; x<8; x++ )        // Go through 8 bits.
  {                                     // (First right-shift has no meaning)
    data >>= 1;                         // Shift byte 1 bit to right
    if( W1_readBit() ) data |= 0x80;    // If 1 is read, then add 1 to most significant bit
  }
  return data;
}

#endif /* __STM32F103_CMSIS_1_WIRE_LIB_C */
