/*####################################
#            MCP3903 Library         #
#          Robert Ayrenschmalz       #
#               25/ 8/2014           #
####################################*/

#include "RAy_MCP3903.h"
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

byte CS_3903 ;



RAy_MCP3903::RAy_MCP3903(void)
{       }

//---------------------------------------------------------------------------------------------------------------------

void RAy_MCP3903::begin(byte CsPin)
{

CS_3903=CsPin;
pinMode(MOSI_PIN, OUTPUT);
pinMode(MISO_PIN, INPUT);
pinMode(CLK_PIN, OUTPUT);
pinMode(CS_3903, OUTPUT);
digitalWrite(CS_3903, HIGH);

SPI.setBitOrder(MSBFIRST);
SPI.setDataMode(SPI_MODE0);
SPI.setClockDivider(SPI_CLOCK_DIV4); // 4MHz on a Board running at 16MHz.
SPI.begin();

write24BitRegister(CONFIG_REG3903,   0B111111000000111111010000); // RESET
write24BitRegister(CONFIG_REG3903,   0B000000000000111111110100); // SETUP OSR 256
write24BitRegister(STATUSCOM_REG3903,0B100111111100000000111111); // Set to 24 BIT
write24BitRegister(GAIN_REG3903,     0B000000000000000000000000); //Gain and Boost register current *1 , both channels Gain 1
}

//---------------------------------------------------------------------------------------------------------------------

//read 24 bit data from register

unsigned long RAy_MCP3903::read24BitRegister(byte reg)
{
	byte CtrlByte = MCP3903_ADDR | reg <<1 | 1;
	unsigned long result = 0;

	digitalWrite(CS_3903, LOW);
	SPI.transfer(CtrlByte);
	result =  (unsigned long) SPI.transfer(0x0) << 16;
	result |= (unsigned long) SPI.transfer(0x0) << 8;
	result |= (unsigned long) SPI.transfer(0x0);
	digitalWrite(CS_3903, HIGH);
	return result;
}

//---------------------------------------------------------------------------------------------------------------------

//write 24 bit data to register

void RAy_MCP3903::write24BitRegister(byte reg, unsigned long data)
{
	byte CtrlByte = MCP3903_ADDR | reg <<1;
	
	byte b2 = (data & 0xff0000) >> 16;
	byte b1 = (data & 0x00ff00) >> 8;
	byte b0 = data & 0x0000ff;

	digitalWrite(CS_3903, LOW);
	SPI.transfer(CtrlByte);
	SPI.transfer(b2);
	SPI.transfer(b1);
	SPI.transfer(b0);
	digitalWrite(CS_3903, HIGH);	
}


//---------------------------------------------------------------------------------------------------------------------


double RAy_MCP3903::readAnalog(byte channel)  // channel 0-5
{
 return readAnalogAVG(channel,24,1);
}


//---------------------------------------------------------------------------------------------------------------------
/* reducing resolution is done by setting the last bits to 0
   masking up to 12 Bits possible but without sense */

double RAy_MCP3903::readAnalog(byte channel, byte resolution)
{   
   return readAnalogAVG(channel,resolution,1); 
}


//---------------------------------------------------------------------------------------------------------------------


double RAy_MCP3903::readAnalogAVG(byte channel,byte nos)
{
return readAnalogAVG(channel,24,nos);
}


//---------------------------------------------------------------------------------------------------------------------

/* reducing resolution is done by setting the last bits to 0
   masking up to 12 Bits possible but without sense */

double RAy_MCP3903::readAnalogAVG(byte channel, byte resolution,byte nos)
{   
    
     double voltage = 0.0;
     long result =0;      
     byte gain =( getGain(channel));

 
    byte shiftout = 24 - resolution;
    if (shiftout > 12) shiftout = 12;

    if (nos > 50 ) nos= 50;

        for (byte i=0; i< nos; i++) 
         {
          result += read24BitRegister(channel);
         } 
   
     result /= nos;
   
    result = result >> shiftout ;

   if (result > (8388607>> shiftout)) result -= (16777216 >> shiftout) ;

voltage =(double)result *(2.3522 /((8388608 >> shiftout)*3*gain));
// 2.35V internal VREF

return voltage;
}

//---------------------------------------------------------------------------------------------------------------------

// sets all channels the same gain , keeps boost settings

void RAy_MCP3903::setGain( byte gain) 
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


    unsigned long result = read24BitRegister(GAIN_REG3903);

    unsigned long boost = result & 0x181818;
    unsigned long modified = 0;
    unsigned long Gain = gain;

    modified|= Gain<<0;
    modified|= (Gain<<5) ; 
    modified|= (Gain<<8) ; 
    modified|= (Gain<<13) ; 
    modified|= (Gain<<16) ; 
    modified|= (Gain<<21) ;
    modified|= boost; 
 
 
    write24BitRegister(GAIN_REG3903, modified);
}

//---------------------------------------------------------------------------------------------------------------------


// sets single channel gain , keeps boost settings

void RAy_MCP3903::setGain(byte channel, byte gain) 
{
unsigned long result;

if((channel%2)==0)
result =(read24BitRegister(GAIN_REG3903)>>((4*channel)+3))&1;

else
result=(read24BitRegister(GAIN_REG3903)>>(4*channel))&1;

byte boost= (byte)result;
setGain(channel, gain, boost);
}


//---------------------------------------------------------------------------------------------------------------------

// sets single channel gain and boost 

void RAy_MCP3903::setGain(byte channel, byte gain, byte boost)
{
     unsigned long result = read24BitRegister(GAIN_REG3903);
     unsigned long modified, newgain , mask = 0;
     boost&=1; // only last bit necessary
    
 switch (gain) 
    {// gain settings 0B110 and 0B111 reserved

    case 32: newgain = 0B101;
             break;

    case 16: newgain = 0B100;
             break;

    case 8 : newgain = 0B011;
             break;
    
    case 4 : newgain = 0B010;
             break;

    case 2 : newgain = 0B001;
             break;

    default: newgain = 0B000 ;// default setting gain = 1
    }

if((channel%2)==0)
    newgain = newgain |(boost<<3)  ;  
else
    newgain = (newgain <<1) |boost  ;  


switch (channel) 
    {
    case 0:  mask = 0xFFFFF0;
             break;

    case 1:  mask = 0xFFFF0F;
             break;

    case 2 : mask = 0xFFF0FF;
             break;
    
    case 3 : mask = 0xFF0FFF;
             break;

    case 4 : mask = 0xF0FFFF;
             break;

    case 5 : mask = 0x0FFFFF;
             break;

    default: mask = 0xFFFFFF;  //no change on typos
    }

    modified = result & mask;
    modified|= newgain<<(channel*4);

write24BitRegister(GAIN_REG3903, modified);
}


//---------------------------------------------------------------------------------------------------------------------

// gets gain value from ADC

byte RAy_MCP3903::getGain(byte channel)
{
byte gain=0;
unsigned long result =0;
 
if((channel%2)==0)
result = (read24BitRegister(GAIN_REG3903)>> (channel*4)&7 );// masking the 3 gain bits to read 

else
result =(read24BitRegister(GAIN_REG3903)>>((channel*4)+1)&7 );
            gain = 1<<result;

            return gain;
}

//---------------------------------------------------------------------------------------------------------------------

// sets boost for all channels

void RAy_MCP3903::setBoost(byte boost)
{
    int32_t result = read24BitRegister(GAIN_REG3903);

    int32_t modified = result & 0xE7E7E7;
    unsigned long Boost = boost;
    
    modified|=(Boost<<3) ;
    modified|=(Boost<<4) ; 
    modified|=(Boost<<11) ; 
    modified|=(Boost<<12) ; 
    modified|=(Boost<<19) ; 
    modified|=(Boost<<20) ;
     
    write24BitRegister(GAIN_REG3903, modified);
 
}


byte RAy_MCP3903::getBoost(byte channel)
{
unsigned long result=0;
byte boost=0;
 
 if((channel%2)==0)
 result = (read24BitRegister(GAIN_REG3903)>> ((channel*4)+3))&1 ; 

else

 result = (read24BitRegister(GAIN_REG3903)>> (channel*4))&1 ; 
            
boost = (byte)result;
            return boost;
}

//---------------------------------------------------------------------------------------------------------------------

// sets Oversampling rate

void RAy_MCP3903::setOSR(word OSR)
{
   switch (OSR) 
    {
    case  256: OSR= 0B11;
               break;

    case  128: OSR= 0B10;
               break;

    case   64: OSR= 0B01;
               break;

    case   32: OSR= 0B00;
               break;

    default:   OSR = 0B11 ;// default setting OSR = 256
    }

 unsigned long modified = read24BitRegister(CONFIG_REG3903) & 0xFFFFCF;//set OSR Bits ( Bit 4:5) to 0 , keep the others
            modified|= OSR<<4; // set OSR Bits
            write24BitRegister(CONFIG_REG3903, modified);
}

//---------------------------------------------------------------------------------------------------------------------

// gets Oversampling value from ADC

word RAy_MCP3903::getOSR()
{
 word OSR_value=0;

 unsigned long OSR = (read24BitRegister(CONFIG_REG3903)>> 4)&3 ;// masking the 2 OSR bits to read  

            OSR += 5;            //add 5 to get offset of 32 in next line
            OSR_value = 1<<OSR;  // OSR number = 2^OSR
            return OSR_value;
}


