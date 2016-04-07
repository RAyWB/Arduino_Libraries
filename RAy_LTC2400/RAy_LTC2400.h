/*####################################
#            LTC2400 Library         #
#          Robert Ayrenschmalz       #
#               25/ 8/2014           #
####################################*/

#ifndef RAy_LTC2400_H
#define RAy_LTC2400_H

#include <Arduino.h>


class RAy_LTC2400
 {

public:
     RAy_LTC2400(void);
     void begin(byte CsPin);
     double readAnalog(float VREF);
     double readAnalog(float VREF, byte resolution);
     double readAnalogAVG(float VREF,byte nos);
     double readAnalogAVG(float VREF, byte resolution,byte nos);
private:
};
#endif
