/*####################################
#            MCP3911 Library         #
#          Robert Ayrenschmalz       #
#               22/ 8/2014           #
####################################*/

#include "RAy_MCP3911.h"
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

byte CS_3911 ;


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



RAy_MCP3911::RAy_MCP3911(void)
{   }

//---------------------------------------------------------------------------------------------------------------------

void RAy_MCP3911::begin(byte CsPin)
{

 CS_3911=CsPin;
 pinMode(MOSI_PIN, OUTPUT);
 pinMode(MISO_PIN, INPUT);
 pinMode(CLK_PIN, OUTPUT);
 pinMode(CS_3911, OUTPUT);
 digitalWrite(CS_3911, HIGH);

 SPI.setBitOrder(MSBFIRST);
 SPI.setDataMode(SPI_MODE0);
 SPI.setClockDivider(SPI_CLOCK_DIV2); // 8MHz on a Board running at 16MHz.
 SPI.begin();

 write16BitRegister(CONFIG_REG3911, 0xFE00);
 write16BitRegister(STATUSCOM_REG3911, 0x1FB8);
 write8BitRegister (GAIN_REG3911, B11000000); //Gain and Boost 
//write16BitRegister(STATUSCOM_REG3911, 0B0000001110011000);
//write16BitRegister(CONFIG_REG3911, 0B1110011000000000);
//write8BitRegister (GAIN_REG3911, 0B10000000); //Gain and Boost register current *1 , both channels Gain 1
}

//---------------------------------------------------------------------------------------------------------------------


//read 8 bit data from register

byte RAy_MCP3911::read8BitRegister(byte reg)
{
	byte CtrlByte = MCP3911_ADDR | reg <<1 | 1;
	byte r = 0;

	digitalWrite(CS_3911, LOW);
	SPI.transfer(CtrlByte);
	r = SPI.transfer(0x0);
	digitalWrite(CS_3911, HIGH);
	return r;
}

//---------------------------------------------------------------------------------------------------------------------

//write 8 bit data to register

void RAy_MCP3911::write8BitRegister(byte reg, byte data)
{
	byte CtrlByte = MCP3911_ADDR | reg <<1;
	
	digitalWrite(CS_3911, LOW);
	SPI.transfer(CtrlByte);
	SPI.transfer(data);
	digitalWrite(CS_3911, HIGH);	
}

//---------------------------------------------------------------------------------------------------------------------

//read 16 bit data from register

word RAy_MCP3911::read16BitRegister(byte reg)
{
	byte CtrlByte = MCP3911_ADDR | reg <<1 | 1;
	word result = 0;

	digitalWrite(CS_3911, LOW);
	SPI.transfer(CtrlByte);
	result  = (word) SPI.transfer(0x0) << 8;
	result |=  SPI.transfer(0x0);
	digitalWrite(CS_3911, HIGH);
	return result;
}

//---------------------------------------------------------------------------------------------------------------------

//write 16 bit data to register

void RAy_MCP3911::write16BitRegister(byte reg, word data)
{
	byte CtrlByte = MCP3911_ADDR | reg <<1;
	
	byte b1 = (data & 0xff00) >> 8;
	byte b0 = data & 0x00ff;

	digitalWrite(CS_3911, LOW);
	SPI.transfer(CtrlByte);
	SPI.transfer(b1);
	SPI.transfer(b0);
	digitalWrite(CS_3911, HIGH);	
}


//---------------------------------------------------------------------------------------------------------------------

//read 24 bit data from register

unsigned long RAy_MCP3911::read24BitRegister(byte reg)
{
	byte CtrlByte = MCP3911_ADDR | reg <<1 | 1;
	unsigned long result = 0;

	digitalWrite(CS_3911, LOW);
	SPI.transfer(CtrlByte);
	result =  (unsigned long) SPI.transfer(0x0) << 16;
	result |= (unsigned long) SPI.transfer(0x0) << 8;
	result |= (unsigned long) SPI.transfer(0x0);
	digitalWrite(CS_3911, HIGH);
	return result;
}

//---------------------------------------------------------------------------------------------------------------------

//write 24 bit data to register

void RAy_MCP3911::write24BitRegister(byte reg, unsigned long data)
{
	byte CtrlByte = MCP3911_ADDR | reg <<1;
	
	byte b2 = (data & 0xff0000) >> 16;
	byte b1 = (data & 0x00ff00) >> 8;
	byte b0 = data & 0x0000ff;

	digitalWrite(CS_3911, LOW);
	SPI.transfer(CtrlByte);
	SPI.transfer(b2);
	SPI.transfer(b1);
	SPI.transfer(b0);
	digitalWrite(CS_3911, HIGH);	
}


//---------------------------------------------------------------------------------------------------------------------


double RAy_MCP3911::readAnalog(byte channel)
{
 return readAnalogAVG(channel,24,1);
}


//---------------------------------------------------------------------------------------------------------------------

double RAy_MCP3911::readAnalog(byte channel, byte resolution)
{   
  return readAnalogAVG(channel,resolution,1);   
}


//---------------------------------------------------------------------------------------------------------------------


double RAy_MCP3911::readAnalogAVG(byte channel,byte nos)
{
   return readAnalogAVG(channel,24,nos);
}


//---------------------------------------------------------------------------------------------------------------------

/* reducing resolution is done by setting the last bits to 0
   masking up to 12 Bits possible but without sense */

double RAy_MCP3911::readAnalogAVG(byte channel, byte resolution,byte nos)
{    
    
     double voltage = 0.0;
           
     byte gain = getGain(channel);

     byte channelAddress = 0x00;

     if (channel==1){channelAddress = 0x03;}

     byte shiftout = 24 - resolution;
     if (shiftout > 12) shiftout =12;

     if (nos > 50 ) nos= 50;


 
     long result = read24BitRegister(channelAddress);
     
        for (byte i=1; i< nos; i++) 
         {
          result += read24BitRegister(channelAddress);
         } 
   
     result /= nos;
    
  
    result = result >> shiftout ;

   if (result > (8388607>> shiftout)) result -= (16777216 >> shiftout) ;
   
   voltage =  (double)result *(1.197 /((8388608 >> shiftout)  * 1.5 * gain));// 1,2 V internal VREF

 return voltage;
}

//---------------------------------------------------------------------------------------------------------------------

// sets both channel the same gain , keeps boost settings

void RAy_MCP3911::setGain(byte gain) 
{  
 switch (gain) 
    {// gain settings 0B110 and 0B111 reserved

    case 32: gain = 0B101;
             break;

    case 16: gain = 0B100;
             break;

    case 8 : gain = 0B011;
             break;
    
    case 4 : gain = 0B010;
             break;

    case 2 : gain = 0B001;
             break;

    case 1 : gain = 0B000;
             break;

    default: gain = 0B000 ;// default setting gain = 1
    }


    byte result = read8BitRegister(GAIN_REG3911);

    byte boost = result>>6;
    byte modified = 0;

    modified|= gain ;
    modified|= (gain<<3) ; 
    modified|= boost<<6;
    write8BitRegister(GAIN_REG3911, modified);
}

//---------------------------------------------------------------------------------------------------------------------


// sets single channel gain , keeps boost settings

void RAy_MCP3911::setGain(byte channel, byte gain) 
{ 
    setGain(channel, gain, getBoost());
}


//---------------------------------------------------------------------------------------------------------------------

// sets single channel gain , sets boost for both channels

void RAy_MCP3911::setGain(byte channel, byte gain, float boost)
{
    byte result = read8BitRegister(GAIN_REG3911);
    byte modified = 0;
    
 switch (gain) 
    {
    case 32: gain = 0B101;
             break;

    case 16: gain = 0B100;
             break;

    case 8 : gain = 0B011;
             break;
    
    case 4 : gain = 0B010;
             break;

    case 2 : gain = 0B001;
             break;

    default: gain = 0B000 ;       // default setting gain = 1
    }
        
            if ( channel==0) 
            { modified = result & 0x38;
              modified|= gain; }

            if ( channel==1) 
             { modified = result & 0x07;
               modified|= (gain<<3) ; }

// following just a copy of procedure setBoost code in order of writing register only once

    boost*=100;  
    int bost = boost;
  
  switch (bost) 
    {
    case 50:  bost = 0B00;
              break;

    case 66:  bost = 0B01;
              break;
       
    case 200: bost = 0B11;
              break;

    default:  bost = 0B10 ;// default setting boost current = 1
    }
           
            modified|= bost<<6;
            write8BitRegister(GAIN_REG3911, modified);

}


//---------------------------------------------------------------------------------------------------------------------

// gets gain value from ADC

byte RAy_MCP3911::getGain(byte channel)
{           
  byte gain=0;
 
           if ( channel==0) gain = (read8BitRegister(GAIN_REG3911)>> 0)&7 ;// masking the 3 gain bits to read  
           if ( channel==1) gain = (read8BitRegister(GAIN_REG3911)>> 3)&7 ;// masking the 3 gain bits to read  
            
           gain = 1<<gain;

           return gain;
}

//---------------------------------------------------------------------------------------------------------------------

// sets boost for both channels,keeps gain settings

void RAy_MCP3911::setBoost(float boost)
{
   boost*=100;
   byte bost = (byte) boost;
   switch (bost) 
    {
    case 50:  bost = 0B00;
              break;

    case 66:  bost = 0B01;
              break;
       
    case 200: bost = 0B11;
              break;

    default:  bost = 0B10 ;// default setting boost current = 1
    }
            byte modified =  read8BitRegister(GAIN_REG3911) & 0x3F;
            modified|= bost<<6;
            write8BitRegister(GAIN_REG3911, modified);
}

//---------------------------------------------------------------------------------------------------------------------

float RAy_MCP3911::getBoost()
{  float flboost;
   
    byte boost = read8BitRegister(GAIN_REG3911)>>6;
    switch (boost) 
    {//boost to float Table

    case 0B00: flboost = 0.5;
               break;

    case 0B01: flboost = 0.66;
               break;

    case 0B11: flboost = 2.0;
               break;
    
    default  : flboost = 1.0;
               break;
    }
    return flboost;
}

//---------------------------------------------------------------------------------------------------------------------

// sets Oversampling rate

void RAy_MCP3911::setOSR(word OSR)
{
   switch (OSR) 
    {
    case 4096: OSR= 0B111;
               break;

    case 2048: OSR= 0B110;
               break;

    case 1024: OSR= 0B101;
               break;
    
    case  512: OSR= 0B100;
               break;

    case  256: OSR= 0B011;
               break;

    case  128: OSR= 0B010;
               break;

    case   64: OSR= 0B001;
               break;

    case   32: OSR= 0B000;
               break;

    default:   OSR = 0B011 ;// default setting OSR = 256
    }

            word modified =  read16BitRegister(CONFIG_REG3911) & 0xC7FF;//set OSR Bits ( Bit 11...13) to 0 , keep the others
            modified|= OSR<<11; // set OSR Bits
            write16BitRegister(CONFIG_REG3911, modified);
}

//---------------------------------------------------------------------------------------------------------------------

// gets Oversampling value from ADC

word RAy_MCP3911::getOSR()
{
 word OSR_value=0;

            word OSR = (read16BitRegister(CONFIG_REG3911)>> 11)&7 ;// masking the 3 OSR bits to read  

            OSR += 5;            //add 5 to get offset of decimal32 in next line
            OSR_value = 1<<OSR;  // OSR number = 2^OSR
            return OSR_value;
}


