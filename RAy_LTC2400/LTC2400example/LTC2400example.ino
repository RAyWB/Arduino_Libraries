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
Serial.print("adc 0  22bit :");
oldmillisec=millis();
Serial.println(LTC2400.readAnalogAVG(5.0,22,1),7);// (VRef 5.0V,22Bit resolution,1 Sample),show 7 Digits
Serial.println(millis()-oldmillisec);
}


