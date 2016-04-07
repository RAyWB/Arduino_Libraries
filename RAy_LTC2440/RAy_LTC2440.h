/*####################################
#            LTC2440 Library         #
#          Robert Ayrenschmalz       #
#               22/ 8/2014           #
####################################*/

#ifndef RAy_LTC2440_H
#define RAy_LTC2440_H

#include <Arduino.h>


class RAy_LTC2440
 {

public:
     RAy_LTC2440(void);
     void begin(byte CsPin);
     double readAnalog(word OSR,float VREF);
     double readAnalog(word OSR,float VREF, byte resolution);
     double readAnalogAVG(word OSR,float VREF,byte nos);
     double readAnalogAVG(word OSR,float VREF, byte resolution,byte nos);

private:
};
#endif
