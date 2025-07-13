#ifndef EFFECTCORE_H
#define EFFECTCORE_H

#include "Arduino.h"

#define MAX_NR_OF_COPIES  20
#define VAL_NOT_SET -12345


struct df {
    volatile float x;
    volatile float y;
};

struct db {
    volatile bool x;
    volatile bool y;
    volatile float middleline_y;
};

struct dir {
    volatile bool cw;
    volatile bool ccw;
};


enum Effect_type {
    INDEPENDENT  = 0, 
    DEPENDENT    = 1,
    NOTSET       = -12345 
};

enum Control_type {
    POSITION        = 3, 
    TORQUE          = 2,
    VELOCITY        = 1,
    VELOCITY_ANGLE  = 4,
    UNDEFINED       = -12345   //not yet known or invalid state
};

struct EffectConfig_s {
    uint16_t data_size;
    float angle;
    float quality;
    Effect_type effect_type;
    Control_type control_type;
};




class Effect {
  public:

    int8_t isActive(long time);
    float getArrayPointerValue(long ms);

    float getEffectVoltage(float value);  

    float getArrayPointerValueVelocityEffect(long ms);
    float getVelocityOverTime(float value);
    float getAngleOverTime(float value);

    float getPressureOverTime(float value);
    
    uint16_t data_size;
    bool infinite;
    float position;
    df scale;
    db flip;
    
    Control_type control_type;
    uint16_t data_ptr;

    long range;

    float angle;
    Effect_type effect_type; 
    dir direction;
    float copy[MAX_NR_OF_COPIES];
    uint8_t copy_count;
    float quality;
    float start_angle;    
    long start_time;

    uint8_t index;

    float (*heDisplacementEquation)(float); // Function pointer for equation-based effects
    float (*heTimeEquation)(unsigned long); // Function pointer for equation-based effects
    void (*heSOEquation)(int, int); // Function pointer for equation-based effects

    bool heTime;
    bool heDisplacement;
    bool secondOrder;
    bool heEquation;
    bool heIncreasing;
    float displacementSmoothingAlpha;
    float heHeightLimit;
    float* displacementProfile;
};



class FeelixEffectCore : public Effect {

    public:
        bool isHapticEffectActive(float angle_deg, float position, int8_t cw, int range);
        bool isVelocityEffectActive(float angle_deg, long time);

        FeelixEffectCore(EffectConfig_s effect, float d[]);

        float getValueAtPointer(float value, uint8_t step, uint8_t offset);
        uint16_t getPointerValueVelocityEffect(long ms);
        void start();
        void stop();
        
        
        float* data[];
        
};






#endif