#ifndef HYDROHAPTICS_H
#define HYDROHAPTICS_H


#include "PressureSensor.h"
#include "EffectCore.h"

class HydroHaptics {

    public: 

        HydroHaptics(float _leadScrewPitch, PressureSensor* _pressureSensor, SPIClass* _SPI2_EXT);    

        PressureSensor* pressureSensor;    


        //void initHydro(SPIClass* _spi); 
                
        // ------------------- Home/Offset Motor Variables -------------------

        float RD_Height = 0.0;              // Height from the datum (mm)
        float motorAngleDatum = 0.0;        // Motor base angle - Set during HomeMotor Function
        float leadScrewPitch = 2.0;  // Lead screw pitch (mm/rev) 32mm 16rev

        // ------------------- Height Sweep Variables -------------------
        // Acceptable error margin for stopping
        float heightHoldTolerance = 0.05;  // Acceptable error in height (e.g., 0.05 units)

        // Maximum number of PID iterations before giving up
        int maxIterations = 20000;

        // Number of frames to hold the height for averaging
        int holdFrames_Avg = 100;
        int holdFrames_Seek = 250;

        // PID variables
        float Height_previousError = 0;
        float Height_integral = 0;

        float targetHeight_PID = 0.0;  // Desired height (mm)

        // Motor movement settings
        const float HomeSearchVoltage = -1.5;   // Voltage for downward motion
        const float HomeAngleThreshold = 0.01;  // Minimum angle change to detect stop
        const int HomeStabilityCount = 100;     // Stable readings needed to confirm stop
        const int HomeMotorRunDelay = 500;      // Delay to ensure motor is in contact
        const int HomeAveragingSamples = 20;    // Number of samples for averaging


        // ------------------- Height PID Variables -------------------
        // PID constants (these need to be tuned for your system)
        float Height_Kp = 0.5;   // Proportional gain
        float Height_Ki = 0.0005;  // Integral gain
        float Height_Kd = 0.0;    // Derivative gain

        // Height range and steps
        const float touchMinHeight = 1;
        const float touchMaxHeight = 14;
        const float touchHeightStep = 0.5;

        // Stabilization thresholds
        const int touchMovingAverageWindow = 10;            // Rolling average window
        const float pressureStabilizationThreshold = 0.01;  // Allowable pressure change
        const float heightStabilizationThreshold = 0.003;   // Allowable height change
        const unsigned long stabilizationTimeoutMs = 5000;  // Timeout (ms)

        // Data arrays for touch detection
        //const int touchDataPoints = abs((int)((touchMaxHeight - touchMinHeight) / touchHeightStep) + 1);
        const int touchDataPoints = 26;
        float touchPressureData[26];
        float touchHeightData[26];
        float touchVoltageData[26];


                // Touch detection parameters
        float touchThreshold = 0.3;          // Sensitivity threshold
        float touchModThreshold = 0.00;    // Modified threshold for increases in pressure
        int touchCount = 0;                  // Current touch count
        const int touchCountThreshold = 25;  // Count threshold for confirmed touch

        float pressureOffsetCorrection = 0.0;  // Offset correction value
        bool touchFlag = false;                // Touch detection flag

        long touchStartTime;
        long touchTime;
        float touchStartFlag = false; 

        // Offset calibration flags and values
        bool pressureOffsetFlag = false;
        float pressureOffset = 0.0;
        float heightOffset = 0.0;

        bool pressureChangeFlag = false;

        float pressureDriftOffset = 0;


        float smoothedDisplacement = 0.0;  
        float lastSmoothedDisplacement = 0.0;
        bool inputDirection = false; 
        float inputSpeed = 0;
        float displacementSmoothingAlpha_base = 0.05;

        float calculatedDisplacement = 0.0;
        float displacementHeightCalc;

        // Sin Wave Commands

        // float amplitude = 5;
        // float frequency = 1;  // Default frequency
        // float period = 1;
        // float Frequency = 1;
        unsigned long lastTime = 0;
        float timeInterval = 50;  // How often to update the pressure (in ms)

      
        float calculateFFHeight();
        float interpolateHeight(float inputPressure);
        void updateHeight(float angle);
        void setHeightPID(float _targetHeight_PID);

        float previousDisplacementHeightCalc = 0;

        // const float alpha = 0.1;  // Smoothing factor for EMA
        const float threshold = -0.2;  // Threshold for locking displacementpressureCalc

        bool detectTouch();
        float calculate_targetTouchHeight();
        void calculate_Displacement(Effect effect);
        void updateSpeed(float displacement);
    
};



#endif