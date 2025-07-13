#ifndef LIB_H
#define LIB_H

#include "EffectCore.h"

#define EFFECT_LIST_SIZE    30
#define MAX_ARRAY_SIZE      90

#define MAX_SUB_ARRAY_SIZE  30

struct data {
  float sub[MAX_SUB_ARRAY_SIZE];
};


struct pointer {
  volatile int value;
  volatile int mod;
  volatile int section;
};


class Lib {
  public:

    void init();

    float getValueAtPointer(float value, uint16_t data_ptr, uint8_t step, uint8_t offset, uint16_t datasize);
    float getValueAtPointerInt(uint16_t value);

    float playEffect(int e, long current_time);
    float renderHapticEffect(int e, long current_time);

    void reset();
    void resetVelocityEffects();

    Effect effect[EFFECT_LIST_SIZE];

    data d[MAX_ARRAY_SIZE];  
    pointer ptr;

    volatile uint8_t effect_count;

    float he_OutputPressure = 0.0;
    float he_lastPressure = 0.0;
    float inputMeasure = 0;
    float noTouchPressure = 0;

};



#endif