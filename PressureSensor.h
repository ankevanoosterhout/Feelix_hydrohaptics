#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H

#include <Arduino.h>
#include <SPI.h>
#include "FeelixCoreConfig.h"

class PressureSensor {

    public: 

        PressureSensor(float min, float max);

        bool onTarget = false;            //Boolean flag to measure if the pressure is on the current target.
        float targetPressure_PID = 15.0;  // Desired pressure (PSI)

        float HC_Pressure = 0.0;  // hydraulic Cell pressure reading
        
        float pressureMax = 65.0; //142.5;  // Max pressure in PSI
        float pressureMin = 45.0; //7.5   // Min pressure in PSI      
        
        float pressureFilterAlpha = 0.3;  // 0.3 Smoothing factor (0 to 1)
        
        float zeroPressure = 100;   // Zeroed baseline pressure - Set during PressureZero Function
        float pressureDrift = 0.0;  //Offset for correcting pressure drift

        float Pressure_Kp = 0.13;   // Proportional gain
        float Pressure_Ki = 0.0001;  // Integral gain
        float Pressure_Kd = 0.0;    // Derivative gain

        float Pressure_previousError = 0;
        float Pressure_integral = 0;

        float pressurePIDRange = 20;
        float pressurePIDError = 0.0;
        float pressurePIDOutput = 0.0;
        const float pressurePIDErrorThreshold = 1;
        float VoltagePIDErrorThreshold = 0.45;
        float feedforward = 0.0;
        float intergralGainThreashold = 0.2;

        const float PSI_KPA_Conversion = 6.89476;  //Conversion rate of PSI to KPA
        const int pressureSampleCount = 50;     // Samples for averaging

        const int pressureRawDataLength = 2;     // Bytes returned by sensor

        const uint16_t pressureRawMin = 0x0333;  // 5% of 2^14
        const uint16_t pressureRawMax = 0x3CCC;  // 95% of 2^14

        float targetVoltage = 0.0;    // Desired motor voltage
        float setTargetPressure_PID = 25.0;  // Desired pressure (PSI)
        float action = 1;

        bool pressureOffsetFlag = false;
        float pressureOffset = 0.0;

        bool pressureChangeFlag = false;
        float pressureDriftOffset = 0;
        float previousDisplacementPressureCalc = 0;


        float pressureError_Threashold = 5;  // Pressure Threashold (in Pka) at which the displacement calc swaps from displacment based to Pressure based.
        float displacementpressureCalc;
        float DispTemp;


        uint16_t getData();
        void updatePressure();
        
        float getPressure();
        float calculatePressurePIDError();
        void updateSetPressure(float setTargetPressure_MOD);      
        void setPressure(float setTargetPressure_MOD); 
        float calculateFFPressure();
        float calculate_raw_to_pressure(int output);
        void checkOnTarget(float motorTarget);
        float calculateFeedForward(float FFheight);

        float updatePressurePID(float height);

        SPIClass* spi; 

    private:

       

    
};




#endif