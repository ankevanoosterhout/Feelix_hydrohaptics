#include "Lib.h"



void Lib::init() {
    reset();
}



float Lib::getValueAtPointer(float value, uint16_t data_ptr, uint8_t step, uint8_t offset, uint16_t data_size) {

    float dX = value - floor(value);
    ptr.value = data_ptr + (floor(value) * step) + offset; 
    ptr.mod = ptr.value % MAX_SUB_ARRAY_SIZE;
    ptr.section = floor(ptr.value / MAX_SUB_ARRAY_SIZE);

    if (ptr.value >= data_ptr + data_size && step == 1) {
        return d[ptr.section].sub[ptr.mod];

    } else {
        float value_1 = d[ptr.section].sub[ptr.mod];
        float value_2 = d[ptr.section].sub[ptr.mod + step];

        return ((value_2 - value_1) * dX) + value_1;
    }

}



float Lib::getValueAtPointerInt(uint16_t value) {

    ptr.mod = value % MAX_SUB_ARRAY_SIZE;
    ptr.section = floor(value / MAX_SUB_ARRAY_SIZE);

    return d[ptr.section].sub[ptr.mod];
}





void Lib::reset() {

    for (int e = 0; e < EFFECT_LIST_SIZE; e++) {
        effect[e].data_size = 0;
        effect[e].infinite = 0;
        effect[e].position = 0.0;
        effect[e].scale.x = 0.0;
        effect[e].scale.y = 0.0;
        effect[e].flip.x = 0;
        effect[e].flip.y = 0;
        effect[e].flip.middleline_y = 0.0;
        effect[e].direction.cw = 0;
        effect[e].direction.ccw = 0;
        effect[e].control_type = Control_type::UNDEFINED;
        effect[e].effect_type = Effect_type::NOTSET;
        effect[e].data_ptr = 0;
        effect[e].quality = 1;
        effect[e].start_angle = VAL_NOT_SET;

        for (int c = 0; c < MAX_NR_OF_COPIES; c++) {
            effect[e].copy[c] = 0.0;
        }

        effect[e].copy_count = 0;
    }

    for (int i = 0; i < MAX_ARRAY_SIZE; i++) {
        for (int j = 0; j < MAX_SUB_ARRAY_SIZE; j++) {
            d[i].sub[j] = 0.0;
        }
    }

    ptr.mod = 0;
    ptr.value = 0;
    ptr.section = 0;
    effect_count = 0;
}


void Lib::resetVelocityEffects() {
    for (int e = 0; e < EFFECT_LIST_SIZE; e++) {
        effect[e].start_angle = VAL_NOT_SET;
    }

}




float Lib::playEffect(int e, long current_time) {
    // for (int e = 0; e < effect_count; e++) {

        float target = 0.0;
        
    //     int8_t active_effect = effect[e].isActive(current_time);

        // if (active_effect > -1) {

            float value = effect[e].getArrayPointerValue(current_time);
            target = effect[e].getPressureOverTime(getValueAtPointer(value, effect[e].data_ptr, 1, 0, effect[e].data_size)); 

            return target;
        // }
        
    // }
    // return 0.0;
}





float Lib::renderHapticEffect(int e, long current_time) { //bool touchFlag, long touchTime, float calculatedDisplacement
  
// This is a generic function for reading a haptic effect. 
// it can be used for time (heTime) or displacment (heDisplacement). 

 
  // if (touchFlag == 0) {

  //   noTouchPressure = effect[e].getPressureOverTime(getValueAtPointer(0, effect[e].data_ptr, 1, 0, effect[e].data_size)); 
  //   return noTouchPressure; // Exit function to prevent unintended behavior
  // }
  // inputMeasure = touchTime != 0 ? touchTime : 0;
  // calculatedDisplacement != 0 ? calculatedDisplacement
  // if(effect[e].heTime){
  //   inputMeasure = touchTime; // Read current touch time
  // }
  // else if (effect[e].heDisplacement){
  //   inputMeasure = calculatedDisplacement; // Read current displacement
  // }
  // else {
  //   // If neither is set to true then default to a low pressure
  //   return 5;
  // }

  he_OutputPressure = playEffect(e, current_time);

  if (effect[e].heIncreasing && he_OutputPressure < he_lastPressure){
    // If the HE is set to increase only
    he_OutputPressure = he_lastPressure;
  }

  he_lastPressure = he_OutputPressure;

  return he_OutputPressure;
}

