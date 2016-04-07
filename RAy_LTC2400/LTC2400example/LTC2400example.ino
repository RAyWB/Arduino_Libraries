#include <SPI.h>
#include <RAy_LTC2400.h>

RAy_LTC2400  LTC2400;

long oldmillisec;
void setup()

{
pinMode (SS, OUTPUT); // Uno´s SS Pin

 LTC2400.begin(10);

Serial.begin(115200);
Serial.print(" RESET ");
}

void loop()
{
Serial.print("adc 0  24bit :");
oldmillisec=millis();
Serial.println(LTC2400.readAnalogAVG(5.0,22,1),7);
Serial.println(millis()-oldmillisec);
/*
Serial.print("adc 0  16bit :");
oldmillisec=millis();
Serial.println(LTC2400.readAnalog(5.0,16),7);
Serial.println(millis()-oldmillisec);

Serial.print("AVG :");
oldmillisec=millis();
Serial.println(LTC2400.readAnalogAVG(5.0,24,2),7);
Serial.println(millis()-oldmillisec);
*/



}


