/*####################################
#            LTC2400 Library         #
#          Robert Ayrenschmalz       #
#               17/ 9/2014           #
####################################*/


#include "RAy_LTC2400.h"
#include <SPI.h>
#include <inttypes.h>
#if defined(__AVR__)
# include <avr/io.h>
#endif
#if ARDUINO >= 100
# include "Arduino.h"
#else
# include "WProgram.h"
#endif

byte CS_2400 ;

#if (defined(__AVR_ATmega1280__) || \
     defined(__AVR_ATmega1281__) || \
     defined(__AVR_ATmega2560__) || \
     defined(__AVR_ATmega2561__))      //--- Arduino Mega ---

#  define MOSI_PIN      (51)
#  define MISO_PIN      (50)
#  define CLK_PIN       (52)



#elif (defined(__AVR_ATmega1284__) || \
       defined(__AVR_ATmega1284P__))    //--- Robduino "goldilocks" Pinout---

# define MOSI_PIN       (11)
# define MISO_PIN       (12)
# define CLK_PIN        (13)


#elif defined(__AVR_ATmega32U4__)      //--- Arduino Leonardo ---

# define MOSI_PIN       (16) //PB2
# define MISO_PIN       (14) //PB3
# define CLK_PIN        (15) //PB1

#else                                  //--- Arduino Uno ---

# define MOSI_PIN       (11)
# define MISO_PIN       (12)
# define CLK_PIN        (13)

#endif



RAy_LTC2400::RAy_LTC2400(void)
{ }

//---------------------------------------------------------------------------------------------------------------------

void RAy_LTC2400::begin(byte CsPin)
{

int32_t data=0;

CS_2400 = CsPin;

pinMode(MOSI_PIN, OUTPUT);
pinMode(MISO_PIN, INPUT);
pinMode(CLK_PIN, OUTPUT);
pinMode(CS_2400, OUTPUT);
digitalWrite(CS_2400, HIGH);

SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV2); // 8MHz on a Board running at 16MHz.
SPI.begin();
 
// start first conversion to throw out mismatch from adc

  digitalWrite(CS_2400,LOW);              //! 1) Pull CS low
  delay(1);
  digitalWrite(CS_2400,HIGH);              
  delay(1);
  digitalWrite(CS_2400,LOW);
              
  while (digitalRead(MISO_PIN) == 1)   {;} // wait for Miso going low
  
     data = (long)SPI.transfer(0x0)<<24;
     data |=(long)SPI.transfer(0x0)<<16;
     data |=(long)SPI.transfer(0x0)<<8;
     data |=(long)SPI.transfer(0x0);

   digitalWrite(CS_2400, HIGH);

}

//---------------------------------------------------------------------------------------------------------------------


double RAy_LTC2400::readAnalog(float VREF) 
{
return readAnalogAVG(VREF,24,1);
}

//---------------------------------------------------------------------------------------------------------------------

double RAy_LTC2400::readAnalog(float VREF, byte resolution)
{   
return readAnalogAVG(VREF,resolution,1);
}


//---------------------------------------------------------------------------------------------------------------------


double RAy_LTC2400::readAnalogAVG(float VREF,byte nos)
{
return readAnalogAVG(VREF,24,nos);
}


//---------------------------------------------------------------------------------------------------------------------

/* reducing resolution is done by right shifting LSB´s 
   resolution down to 12 Bits possible but without sense */

double RAy_LTC2400::readAnalogAVG(float VREF, byte resolution,byte nos)

{    double voltage = 0.0; 
     byte sign = 0x0;
     int32_t data,result=0;
     
     byte shiftout = 24 - resolution;// No of lsb´s to shift out
     if ( shiftout > 12) shiftout =12;

        
     for (byte i=0; i< nos; i++) // start loop if nos >1
       {
          digitalWrite(CS_2400,LOW);
          delay(1);
          digitalWrite(CS_2400,HIGH);              
          delay(1);
          digitalWrite(CS_2400,LOW);

        while (digitalRead(MISO_PIN)==1)    //! 2) Wait for SDO (MISO) to go low
        {;}                                 //! 3) If SDO is low, break loop

          data = (long)SPI.transfer(0x0)<<24;
          data |=(long)SPI.transfer(0x0)<<16;
          data |=(long)SPI.transfer(0x0)<<8;
          data |=(long)SPI.transfer(0x0);

        digitalWrite(CS_2400,HIGH);              
  
          data &=0x3FFFFFFF; //mask Bit 31 , 30
		  		  
          data = data >> 4 ; // position Bit 3 (LSB) on correct place 

          result += data;

         } 

  result /= (nos);

  result = result >> shiftout ;
  
  
  /* although LTC 2400 is a unipolar single ended device it has a Sign bit
     and is allowed for Input voltages from (GND - 0.3 V) to (VDD +0.3V).
	 
     LTC2400 has overrange Bit (Bit 28) and Sign bit(Bit 29) 
     MSB is Bit 27 , LSB Bit 4
	 its possible to show code from - 1/8 Vref up to 9/8 Vref
     so we use  overrange Bit as 25th Data Bit plus Sign.
	 LSB Value is calculated as 24Bit (Vref/24Bit)
	 
	 as result is already shifted right to fit LSB Position(line 159)
     Sign bit and overrange Bit also move right 3 Bits
     and so now
	 
	 Sign bit is now BIT 26 ( value 33554432 Decimal) followed by OVR Bit and 24 bit data
  */ 
   result -= 33554432>> shiftout;  // subtract Offset of Sign Bit
  
   voltage =  (double)result *(VREF/(16777216 >> shiftout));


  return(voltage);


} 


