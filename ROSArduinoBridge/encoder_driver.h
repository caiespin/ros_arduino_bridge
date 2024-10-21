/* *************************************************************
   Encoder driver function definitions - by James Nugen
   ************************************************************ */
   
   
#ifdef ARDUINO_ENC_COUNTER
  //below can be changed, but should be PORTD pins; 
  //otherwise additional changes in the code are required
  #define LEFT_ENC_PIN_A PD2  //pin 2
  #define LEFT_ENC_PIN_B PD3  //pin 3
  
  //below can be changed, but should be PORTC pins
  #define RIGHT_ENC_PIN_A PC4  //pin A4
  #define RIGHT_ENC_PIN_B PC5   //pin A5
#elif defined(ENC_AS5047_SPI)
  #include <AS5047P.h>
  // define the chip select port.
  #define AS5047P_CHIP_SELECT_PORT 7 

  // define the spi bus speed 
  #define AS5047P_CUSTOM_SPI_BUS_SPEED 100000

  // initialize a new AS5047P sensor object.
  AS5047P as5047p(AS5047P_CHIP_SELECT_PORT, AS5047P_CUSTOM_SPI_BUS_SPEED);
#endif
   
long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
