#ifndef LM75_h
#define LM75_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#define LM75_ADDRESS 0x48
#define LM75_TEMP_REGISTER 0


class LM75 {

    public: 
        LM75(TwoWire* _Wire2_LM75);
        float getTemperature(void);

    private: 

       TwoWire* Wire2_LM75; 

       int address;
       word _register16(byte);
       float regdata2float(word);
};

#endif