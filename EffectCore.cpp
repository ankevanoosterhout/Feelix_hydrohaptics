#include "EffectCore.h"

#include "Arduino.h"
#include <math.h>       /* fmod */

#define _DEG_PI     0.017453292519
#define _PI_DEG     57.29577951308



FeelixEffectCore::FeelixEffectCore(EffectConfig_s effect, float d[]) {
    data_size = effect.data_size;
    angle = effect.angle;
    effect_type = effect.effect_type;
    control_type = effect.control_type;
    quality = (float) effect.quality;

    for (int i = 0; i < data_size; i++) {
        data[i] = &d[i]; 
    }
    
    direction.cw = true;
    direction.ccw = true;
    infinite = false;
    position = 0.0;
    scale.x = 1.0;
    scale.y = 1.0;
    flip.x = false;
    flip.y = false;
    flip.middleline_y = 0.0;
    start_angle = VAL_NOT_SET;
    copy[0] = 0.0;
    copy_count = 1;
}




float FeelixEffectCore::getValueAtPointer(float value, uint8_t step, uint8_t offset) {
    float dX = value - floor(value);
    int index = floor(value) + offset;

    float value_1 = *data[index];
    float value_2 = *data[index + step];

    return ((value_2 - value_1) * dX) + value_1;
}



static float getInfiniteAngle(float angle, float range) {
    angle = fmod(angle, range); 
    if (angle < 0) {
        angle += range;
    }
    return angle;
}




int8_t Effect::isActive(long time) {
           
 
    if (infinite) { return 0; }   

    return (time >= (long) copy[0] && time < (long) (copy[0] + range)) ? 0 : -1;

}


float Effect::getArrayPointerValue(long ms) {

    // Serial.print((String) "#time " + ms);

    float offset = round((ms - (long) copy[0]) * (1.0 / scale.x) / quality); 

    // Serial.println((String) " ptr " + data_ptr + " offset " + offset);
    
    if (flip.x) { offset = data_size - offset; }
   
    return offset;
}








bool FeelixEffectCore::isHapticEffectActive(float angle_deg, float position, int8_t cw, int range) {

    if ((cw == 1 && !direction.cw) || (cw == -1 && !direction.ccw)) {  return false; }

    if (effect_type == Effect_type::INDEPENDENT) { return true; } 

    if (infinite) { angle_deg = getInfiniteAngle(angle_deg, range); }

    return angle_deg >= position && angle_deg <= position + angle ? true : false;
}




bool FeelixEffectCore::isVelocityEffectActive(float angle_deg, long time) {

    if (start_time != VAL_NOT_SET) {

        if (infinite || (time >= start_time && time <= (start_time + (long) angle))) {

            if (control_type == Control_type::VELOCITY_ANGLE && start_angle == VAL_NOT_SET) {
                start_angle = angle_deg * _DEG_PI;
            }

            return true;

        } else {

            start_angle = VAL_NOT_SET;
            if (time > (start_time + (long) angle)) {
                start_time = VAL_NOT_SET;
            }
            return false;
        }
    }
    return false;
}


void FeelixEffectCore::start() {
    start_time = millis();
}

void FeelixEffectCore::stop() {
    start_time = VAL_NOT_SET;
}







float Effect::getEffectVoltage(float value) {

    float voltage_at_angle = value * scale.y + position;

    if (flip.y) {
        voltage_at_angle = flip.middleline_y + (flip.middleline_y - voltage_at_angle);
    }

    return voltage_at_angle;
}



float Effect::getArrayPointerValueVelocityEffect(long ms) {

    float offset = (ms - (long) copy[0]) * (1.0 / scale.x) / quality; 
    
    if (flip.x) { offset = data_size - offset; }
   
    // return data_ptr + offset;
    return offset;

}


uint16_t FeelixEffectCore::getPointerValueVelocityEffect(long ms) {

    int offset = round((ms - start_time) * (1.0 / scale.x) / quality); 
    
    if (flip.x) { offset = data_size - offset; }
   
    return offset;
}



float Effect::getVelocityOverTime(float value) {

    float velocity = value * scale.y;

    if (flip.y) {
        velocity = flip.middleline_y + (flip.middleline_y - velocity);
    }

    return velocity;

 }


float Effect::getAngleOverTime(float value) {

    float angle = value * scale.y;

    if (flip.y) {
        angle = flip.middleline_y + (flip.middleline_y - angle);
    }

    return angle + start_angle;

 }



float Effect::getPressureOverTime(float value) {

    float pressure = value * scale.y;

    if (flip.y) {
        pressure = flip.middleline_y + (flip.middleline_y - pressure);
    }

    return pressure;
}

