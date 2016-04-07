/*####################################
#            LTC2440 Library         #
#          Robert Ayrenschmalz       #
#               17/ 9/2014           #
####################################*/


#include "RAy_LTC2440.h"
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

byte CS_2440 ;

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



RAy_LTC2440::RAy_LTC2440(void)
{ }

//---------------------------------------------------------------------------------------------------------------------

void RAy_LTC2440::begin(byte CsPin)
{

int32_t data=0;

CS_2440 = CsPin;

pinMode(MOSI_PIN, OUTPUT);
pinMode(MISO_PIN, INPUT);
pinMode(CLK_PIN, OUTPUT);
pinMode(CS_2440, OUTPUT);
digitalWrite(CS_2440, HIGH);

SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV2); // 8MHz on a Board running at 16MHz.
SPI.begin();
 
// start first conversion to throw out mismatch from adc

  digitalWrite(CS_2440,LOW);              //! 1) Pull CS low
  delay(1);
  digitalWrite(CS_2440,HIGH);              
  delay(1);
  digitalWrite(CS_2440,LOW);
              
  while (digitalRead(MISO_PIN) == 1)   {;} // wait for Miso going low
  
     data = (long)SPI.transfer(0x70)<<24;
     data |=(long)SPI.transfer(0x0)<<16;
     data |=(long)SPI.transfer(0x0)<<8;
     data |=(long)SPI.transfer(0x0);

   digitalWrite(CS_2440, HIGH);

}

//---------------------------------------------------------------------------------------------------------------------


double RAy_LTC2440::readAnalog(word OSR,float VREF) 
{
return readAnalogAVG(OSR,VREF,24,1);
}

//---------------------------------------------------------------------------------------------------------------------

double RAy_LTC2440::readAnalog(word OSR,float VREF, byte resolution)
{   
return readAnalogAVG(OSR,VREF,resolution,1);
}


//---------------------------------------------------------------------------------------------------------------------


double RAy_LTC2440::readAnalogAVG(word OSR,float VREF,byte nos)
{
return readAnalogAVG(OSR,VREF,24,nos);
}


//---------------------------------------------------------------------------------------------------------------------

/* reducing resolution is done by right shifting LSB´s 
   resolution down to 12 Bits possible but without sense */

double RAy_LTC2440::readAnalogAVG(word OSR,float VREF, byte resolution,byte nos)

{    double voltage = 0.0; 
     byte CtrlByte = 0;
     int32_t data,result=0;
     
     byte shiftout = 24 - resolution;// No of lsb´s to shift out
     if ( shiftout > 12) shiftout =12;
 
if(nos > 50) nos = 50;

switch (OSR)
     {
     case     64: CtrlByte = 0B00001000; 
                   break;
     case    128: CtrlByte = 0B00010000;
                  break;
     case    256: CtrlByte = 0B00011000;
                  break;
     case    512: CtrlByte = 0B00100000;
                  break;
     case   1024: CtrlByte = 0B00101000;
                  break;
     case   2048: CtrlByte = 0B00110000;
                  break;
     case   4096: CtrlByte = 0B00111000;
                  break;
     case   8192: CtrlByte = 0B01000000;
                  break;
     case  16384: CtrlByte = 0B01001000;
                  break;
     case  32768: CtrlByte = 0B01111000;
                  break;
     default :    CtrlByte = 0B00011000;
     }

digitalWrite(CS_2440,LOW);
delay(1);
digitalWrite(CS_2440,HIGH);              
delay(1);
digitalWrite(CS_2440,LOW);
         


     for (byte i=0; i< nos; i++) // start loop if nos >1
       {
      digitalWrite(CS_2440,LOW);
delay(1);
digitalWrite(CS_2440,HIGH);              
delay(1);
digitalWrite(CS_2440,LOW);

        while (digitalRead(MISO_PIN)==1)    //! 2) Wait for SDO (MISO) to go low
        {;}                                 //! 3) If SDO is low, break loop

          data = (long)SPI.transfer(CtrlByte)<<24;
          data |=(long)SPI.transfer(0x0)<<16;
          data |=(long)SPI.transfer(0x0)<<8;
          data |=(long)SPI.transfer(0x0);

          data &= 0x3FFFFFFF;// masking out bits 31..30
          data = data >> 5 ;

          result += data;
digitalWrite(CS_2440, HIGH);
         } 

     result /= (nos);
    
     result -= (16777216>>shiftout); 
   
   voltage =  result *(VREF / (16777216>>shiftout));

  return (voltage);

} 
