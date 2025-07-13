#include "FeelixCore.h"
#include "FeelixCoreCommands.h"


#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <math.h>



#define _PI_DEG     57.29577951308
#define _DEG_PI     .0174532925199


uint8_t ledBrightness = 0;
bool brightnessChangeDirection = 1;
bool STATUS_LED_ENABLED = true;
bool LED_STATE = 0;
volatile bool PROCESSING = false;
volatile bool ML5DATACOLLECTION = false;


volatile uint16_t CURRENT_ARRAY_INDEX = 0;
bool BREAK_LOOP = false;

char charFloat[9];
char charLongFloat[18];

int8_t lastEffectID = -1;
 

int address = 0;

Lib library;

bool endOfDataReceive = false;

/* initialize motor */
BLDCMotor _bldc = BLDCMotor(7);

DRV8316Driver6PWM _driver =
    DRV8316Driver6PWM(
        STM32_INHA,
        STM32_INLA,
        STM32_INHB,
        STM32_INLB,
        STM32_INHC,
        STM32_INLC,
        STM32_SPI1_CS_DRV,
        false);

/* similar for boards with AS5047 and AS5048A */
MagneticSensorSPI _sensor = MagneticSensorSPI(AS5147_SPI, STM32_SPI3_CS_ENC);




FeelixM::FeelixM(char _id) {

    INITIALIZED = false;  
    MOVE = false;
    bldc = &_bldc;
    sensor = &_sensor;
    driver = &_driver;
    current_sense = NULL;
    library.init();
    communication_speed = 50;
    control_type = Control_type::UNDEFINED;
    range = 360.0;
    start_pos = 0.0; //RAD
    start_time = 0;
    id = _id;
    I2C_address = (int)strtol(&_id, NULL, 16);
    state = State::NO_COMMUNICATION;
    dataRequestType = '\0';
    nrOfConnectedDevices = 0;
    blink_time = millis();
    blink_interval = 1000;
    tempRead_time = millis();
    tempRead_interval = 6000;
   
}



static int convert_ID_to_I2C_address(char* ID) {
  int address = strtol(ID, NULL, 16);
  return address;
}




static char* convertData(char* user_command, char* cstr) {
    char* cmd = strtok(user_command, "&");
    // cmd = strtok(NULL, "&");
    std::string str(cmd);
    str = "F" + str;
    str += "&";
    cstr = new char[str.length() + 1];
    strcpy(cstr, str.c_str());

    return cstr;
}


void FeelixM::readSPIBus(byte dataToSend) {

    if (digitalRead(STM32_SPI2_CS_EXT) == LOW) {
        // If the SS pin is LOW, SPI communication is active

        // Read data from the master
        byte receivedData = SPI.transfer(dataToSend);  // Send dataToSend and receive master's data

        // Prepare data to send in the next transfer
        dataToSend = receivedData + 1;  // For example, reply with incremented value

        // Print received data for debugging
        Serial.print("Received data: ");
        Serial.println(receivedData, HEX);
    }

    // Main loop can handle other tasks or remain idle
    delay(100);  // Dummy delay, the SPI communication happens in polling
    
}


bool FeelixM::transferData(char identifier, char* user_command) {
    // Serial.println((String) "&state " + I2C_state + " " + identifier + " " + id);

    if ((state == State::I2C_MASTER) && identifier != id) {
        // Serial.println((String) "&forward data to " + identifier);

        address = convert_ID_to_I2C_address(&identifier);
        // Serial.println((String) "&command " + user_command);
        char *cstr = convertData(user_command, cstr);
        // Serial.println((String) "&data " + cstr);
        transmitDataI2C(address, cstr);

        
        // Serial.println("*");
        
        return true;
    }
    return false;
}





void FeelixM::receiveDataI2C() {

    if (state == State::I2C_MASTER) {
        I2CincomingDataMaster = "";
        endOfDataReceive = false;

        while(Wire.available()) {
            I2CincomingDataMaster = Wire.readString();
            // Serial.print((String) " " + c);  

            // if (!endOfDataReceive) {
                // I2CincomingDataMaster += c;
                // if (c == '&') {
                //     endOfDataReceive = true;
                    // Serial.println();
                // }
            // }
           
                            
        }
        if (I2CincomingDataMaster != "") {
            endOfDataReceive = false;
            Serial.print("received data ");
            Serial.println(I2CincomingDataMaster); 
            
        }
    }
}


void FeelixM::process_data(char* cmd) { 
    char command = cmd[0];
    // char* fullCommand = strtok(cmd, "&");
    // fullCommand = strtok(0, "&");

    // Serial.println((String) "&" + fullCommand);

    switch (command) {
        case 'M':
            BLDC_Data(cmd);
            break;
        case 'E':
            Effect_Data(cmd);
            break;
        case 'D':
            Data_Points(cmd);   
            break; 
        case 'C': 
            BLDC_Config(cmd);            
            break;
        case 'A':
            Cogging_Test();
            break;
        case 'S':
            driverVoltage = calculateDriverVoltage();
            Serial.println((String) "S" + SoftwareVersion_major + "." + SoftwareVersion_minor + "." + SoftwareVersion_patch + "." + driverVoltage);
            Serial.println("*");
            break;
        case 'G':
            Return_Data(cmd);
            break;
        case 'R':
            Serial.println("*");
            break;
        case 'L':
            int devices = listDevices();
            Serial.println((String) "Ldevices " + devices + " " + state);
            break;
    }
  
  delay(5);
};




char* FeelixM::returnDataOnRequest() {
    char *cstr = NULL;

    switch(dataRequestType) {
        case '\0':
            return cstr;
            break;
        case 'A': {
                dataOutStr = "A";
                dataOutStr += id;
                dataOutStr += ":";
                dtostrf(angle, 9, 4, charFloat); 
                dataOutStr += charFloat;
                dataOutStr += ":";
                dtostrf(velocity, 9, 3, charFloat); 
                dataOutStr += charFloat;
                dataOutStr += ":";
                dataOutStr += current_time;
                dataOutStr += ":";
                dtostrf(target_val, 9, 4, charFloat);  
                dataOutStr += charFloat;
                dataOutStr += "&";
                // if (dataOutStr.length() < 39) {
                //     for (int i = dataOutStr.length(); i < 39; i++) {
                //         dataOutStr += '0';
                //     }
                // }
            }
            break;
        case 'C': {
                dataOutStr = "Z";
                dataOutStr += id;
                dataOutStr += ":";
                dtostrf(bldc->zero_electric_angle, 18, 15, charLongFloat); 
                dataOutStr += charLongFloat;
                dataOutStr += ":";
                dataOutStr += bldc->sensor_direction;
                dataOutStr += "&";
            }
            break;
        case 'R': {
            dataOutStr = "R";
            dataOutStr += id;
            dataOutStr += ":A:";
            dtostrf(angle, 18, 15, charLongFloat); 
            dataOutStr += charLongFloat;
            dataOutStr += "&";
        }
    }

    cstr = new char[dataOutStr.length() + 1];
    strcpy(cstr, dataOutStr.c_str());
    dataRequestType = '\0';

    return cstr;
    
}


void FeelixM::BLDC_Config(char* user_command) {

    bldc->init();

    bldc->initFOC();

    
    if (!INITIALIZED) {
      Serial.print((String) "Z" + id + ":");
      Serial.print(bldc->zero_electric_angle, 18);
      Serial.print(":");
      Serial.println(bldc->sensor_direction);

      homeMotor(); 

      delay(100);

      start_pos = sensor->getAngle();

      delay(100);

      height_Sweep_Voltage_PID();
    }

    PROCESSING = false;
    INITIALIZED = true; 

    delay(100);

    start_time = millis();    
    
    
    if (!RUN) {
        bldc->disable();
    }
}





void FeelixM::update() {
    angle = (bldc->shaft_angle * transmission_factor * sensor_dir) - sensor_offset; //(180 / 3.14159);
    velocity = bldc->shaft_velocity * sensor_dir;
    target = 0.0;
    target_val = 0.0;
    voltage = 0.0;
    control_type = Control_type::UNDEFINED;

    current_time = (millis() - start_time);

    getDirection();  
}




void FeelixM::run(uint8_t loop_count) {
   
    angle = bldc->shaft_angle * sensor_dir + sensor_offset; //(180 / 3.14159);
    velocity = bldc->shaft_velocity * sensor_dir;
    angle_deg = angle * _PI_DEG;
    getDirection();  
    target = 0.0;
    voltage = 0.0;
    control_type = Control_type::UNDEFINED;
    BREAK_LOOP = false;


    if (RUN && !PROCESSING) { //&& loop_count % 2 == 0

        current_time = (millis() - start_time);

        hydroHaptics->updateHeight(angle);
        hydroHaptics->pressureSensor->updatePressure();

        if (constrain_range && (angle > (start_pos + range) || angle <= start_pos)) {
            target = 0.0;
        } else {
            for (int e = 0; e < library.effect_count; e++) {

                active_effect = library.effect[e].isActive(current_time);

                if (active_effect > -1) {
                    
                    float targetPressure = library.renderHapticEffect(e, current_time); 

                    hydroHaptics->pressureSensor->setPressure(targetPressure);

                    hydroHaptics->pressureSensor->calculatePressurePIDError();
                    hydroHaptics->pressureSensor->checkOnTarget(target);
                    target = hydroHaptics->pressureSensor->updatePressurePID(hydroHaptics->calculateFFHeight());
                    hydroHaptics->detectTouch();

                    hydroHaptics->calculate_Displacement(library.effect[e]);
                    hydroHaptics->updateSpeed(hydroHaptics->calculatedDisplacement);

                   
                    //Serial.println((String) "#" + target);
                    
                    // switch(library.effect[e].control_type) {

                    //     case Control_type::POSITION: 
                    //         if (control_type == Control_type::UNDEFINED || control_type == Control_type::POSITION) {
                    //             control_type = Control_type::POSITION;
                    //             float value = library.effect[e].getArrayPointerValue(library.effect[e].copy[active_effect], angle_deg, range);
                    //             target += (library.getValueAtPointer(value, library.effect[e].data_ptr, 2, 0, library.effect[e].data_size) * library.effect[e].scale.x * (library.effect[e].flip.x ? -1 : 1));
                    //             voltage += (library.effect[e].getEffectVoltage(library.getValueAtPointer(value, library.effect[e].data_ptr, 2, 1, library.effect[e].data_size)) * driver->voltage_power_supply); 
                    //         }                 
                    //         break;

                    //     case Control_type::TORQUE: 
                    //         if (control_type == Control_type::UNDEFINED || control_type == Control_type::TORQUE) {
                    //             control_type = Control_type::TORQUE;
                    //             float value = library.effect[e].getArrayPointerValue(library.effect[e].copy[active_effect], angle_deg, range);
                    //             target += (library.effect[e].getEffectVoltage(library.getValueAtPointer(value, library.effect[e].data_ptr, 1, 0, library.effect[e].data_size)) * driver->voltage_power_supply); 
                    //             driver->voltage_limit = vol_limit;
                    //         }
                    //         break;

                    //     case Control_type::VELOCITY: {
                    //             control_type = Control_type::VELOCITY;
                    //             float value = library.effect[e].getArrayPointerValueVelocityEffect(current_time);
                    //             target = (library.effect[e].getVelocityOverTime(library.getValueAtPointer(value, library.effect[e].data_ptr, 1, 0, library.effect[e].data_size)) * bldc->velocity_limit); 
                    //             driver->voltage_limit = driver->voltage_power_supply;
                    //         }
                    //         break;

                    //     case Control_type::VELOCITY_ANGLE: {
                    //             control_type = Control_type::VELOCITY_ANGLE;
                    //             float value = library.effect[e].getArrayPointerValueVelocityEffect(current_time);
                    //             target = library.effect[e].getAngleOverTime(library.getValueAtPointer(value, library.effect[e].data_ptr, 1, 0, library.effect[e].data_size)); 
                    //             driver->voltage_limit = driver->voltage_power_supply;
                    //         }
                            // break;

                    // }

                    // if (library.effect[e].effect_type == Effect_type::NOTSET) { BREAK_LOOP = true; }

                    if (!bldc->enabled) { bldc->enable(); }

                    break;
                }    

                if (BREAK_LOOP) { break; }
            }
        }
    }


    
    bldc->loopFOC();

    if (angle < start_pos) { target = 0.0; }
    if (angle > start_pos + range) { target = 0.0; }

    if (!MOVE && RUN && loop_count % 2 == 0) {
        move();

        if (loop && current_time > range) {
            start_time = millis();
        }
    } else if (MOVE && loop_count % 2 == 0) {
        moveTo();
    }
}




void FeelixM::move() {
    target_val = 0.0;

    switch(control_type) {
        case Control_type::POSITION: {
            target_val = bldc->shaft_angle + (target * (1.0 / transmission_factor) * sensor_dir) - sensor_offset;
            //target_val = (bldc->shaft_angle + target) * transmission_factor * sensor_dir - sensor_offset;
            driver->voltage_limit = voltage;
            bldc->controller = MotionControlType::angle;
            bldc->move(target_val);
        }
        break;

        case Control_type::TORQUE:  {
            target_val = (target * sensor_dir);
            bldc->controller = MotionControlType::torque;
            bldc->move(target_val);
        } 
        break;

        case Control_type::VELOCITY:  {
            target_val = target;
            bldc->controller = MotionControlType::velocity;
      // calculate necessary voltage to be set by FOC loop
            bldc->move(target_val);
        }
        break;

        case Control_type::VELOCITY_ANGLE:  {
            target_val = bldc->shaft_angle + (target * (1.0 / transmission_factor) * sensor_dir) - sensor_offset;
            bldc->controller = MotionControlType::angle;
            bldc->move(target_val);
        } 
        break;

        case Control_type::UNDEFINED:  {
            target_val = 0.0;
            bldc->controller = MotionControlType::torque;
            bldc->move(target_val * sensor_dir);
        }
        default : {
            target_val = 0.0;
            bldc->controller = MotionControlType::torque;
            bldc->move(target_val);
        }
        break;
    }  
}






void FeelixM::moveTo() {
    
    if (angle > target_val - 0.02 && angle < target_val + 0.02) {
        MOVE = false;
        library.resetVelocityEffects();
        target_val = 0.0;
        bldc->controller = MotionControlType::torque;
        bldc->move(0.0);
    } else {
        bldc->controller = MotionControlType::angle;
        if (bldc->enabled == 0) { bldc->enable(); }
        bldc->move(target_val); 
    }
}




void FeelixM::velocityLimitProtection() {
    if (bldc->shaft_velocity > bldc->velocity_limit || bldc->shaft_velocity < -bldc->velocity_limit) {
       RUN = false;
       target_val = 0.0;
       bldc->disable();
       Serial.println("V");
   }
}







// void FeelixM::playHapticEffectAtAngle(FeelixEffectCore effect, float position) {
//     effect.copy[0] = angle;

//     if (effect.isHapticEffectActive(angle_deg, position, rotation_dir, range)) {
     
//         switch(effect.control_type) {
//             case Control_type::POSITION: 
//                 if (control_type == Control_type::UNDEFINED || control_type == Control_type::POSITION) {
//                     control_type = Control_type::POSITION;
//                     volatile float value = effect.getArrayPointerValue(position, angle_deg, range);
//                     target += effect.getValueAtPointer(value, 2, 0) * effect.scale.x * (effect.flip.x ? -1 : 1);
//                     voltage += (effect.getEffectVoltage(effect.getValueAtPointer(value, 2, 1)) * driver->voltage_power_supply); 
//                 }                 
//                 break;

//             case Control_type::TORQUE: 
//                 if (control_type == Control_type::UNDEFINED || control_type == Control_type::TORQUE) {
//                     control_type = Control_type::TORQUE;
//                     volatile float value = effect.getArrayPointerValue(position, angle_deg, range);
//                     target += (effect.getEffectVoltage(effect.getValueAtPointer(value, 1, 0)) * driver->voltage_power_supply); 
//                     driver->voltage_limit = vol_limit;
//                 }
//                 break;

//             case Control_type::VELOCITY: 
//                 break;
//             case Control_type::VELOCITY_ANGLE: 
//                 break;
            
//         }
//     }
// }





void FeelixM::playVelocityEffect(FeelixEffectCore effect) {
    
    if (effect.isVelocityEffectActive(angle_deg, current_time)) {
     
        switch(effect.control_type) {
            case Control_type::VELOCITY: 
                if (control_type == Control_type::UNDEFINED) {
                    control_type = Control_type::VELOCITY;
                    volatile uint16_t value = effect.getPointerValueVelocityEffect(current_time);
                    target = (effect.getVelocityOverTime(effect.getValueAtPointer(value, 1, 0)) * bldc->velocity_limit); 
                    driver->voltage_limit = driver->voltage_power_supply;
                }
                break;

            case Control_type::VELOCITY_ANGLE: 
                if (control_type == Control_type::UNDEFINED) {
                    control_type = Control_type::VELOCITY_ANGLE;
                    volatile uint16_t value = effect.getPointerValueVelocityEffect(current_time);
                    target = effect.getAngleOverTime(effect.getValueAtPointer(value, 1, 0)); 
                    driver->voltage_limit = driver->voltage_power_supply;
                }
                break;
            case Control_type::POSITION: 
                break;
            case Control_type::TORQUE: 
                break;
            
        }
    }
}





void FeelixM::calibrateCurrentSenseValues() {
    RUN = false;
    int x = 0;
    bldc->controller = MotionControlType::torque;
    float max_current_sense = 0.0;
    float min_current_sense = 0.0;
    while (x < 200) {
        bldc->loopFOC();
        bldc->move(driver->voltage_power_supply);
        if (x % 5 == 0) {
            currents = current_sense->getPhaseCurrents();
            max_current_sense = currents.a > max_current_sense ? currents.a : max_current_sense; 
            max_current_sense = currents.b > max_current_sense ? currents.b : max_current_sense; 
            min_current_sense = currents.a < min_current_sense ? currents.a : min_current_sense; 
            min_current_sense = currents.b < min_current_sense ? currents.b : min_current_sense; 
        }
        delay(1);
        x++;
    }
    bldc->move(0.0);
    current_threshold = min_current_sense * -1 > max_current_sense ? min_current_sense * -0.75 : max_current_sense * 0.75;
    
    Serial.println((String) "C" + id + ":" + current_threshold);
}





void FeelixM::getDirection() {

    if (velocity > 1.0) {
        rotation_dir = Direction::CW;
    } else if (velocity < -1.0) {
        rotation_dir = Direction::CCW;
    }
}








void FeelixM::BLDC_Data(char* user_command){ 
   
    char identifier = user_command[1];
    
    
    // if (!transferData(identifier, user_command)) {

        char sub_cmd = user_command[2];
        char* value = strtok(user_command, ":");
        switch(sub_cmd){

            case M_DATA_SEQUENCE: {
                    PROCESSING = true;
                    library.reset();
                    lastEffectID = -1;
                    CURRENT_ARRAY_INDEX = 0;
                    RUN = true;
                    delay(50);
                }
                break;
            case M_ID:  
                PROCESSING = true;
                // id = user_command[3];
                break;

            case M_POLE_PAIRS:  
                bldc->pole_pairs = atoi(&user_command[3]);
                break;

            case M_ZERO_EL_ANGLE: 
                bldc->zero_electric_angle = atof(&user_command[3]);
                break;

            case M_SENSOR_DIRECTION: 
                bldc->sensor_direction = atoi(&user_command[3]) == 1 ? Direction::CW : Direction::CCW;
                break;

            case M_SENSOR_OFFSET:  
                sensor_offset = atof(&user_command[3]);
                sensor_offset = sensor_dir == Direction::CW ? sensor_offset : -sensor_offset;
                break;

            case M_MAG_ENC_CLK_SPD: 
                sensor->clock_speed = atol(&user_command[3]);
                break;

            case M_SUPPLY_VOL:  
                driver->voltage_power_supply = atof(&user_command[3]);
                break;

            case M_VOL_LIMIT: 
                bldc->voltage_limit = atof(&user_command[3]); 
                driver->voltage_limit = bldc->voltage_limit;
                vol_limit = bldc->voltage_limit;
                break;

            case M_COMM_SPEED:
                communication_speed = atoi(&user_command[3]);
                break;

            case M_LOOP:
                loop = atoi(&user_command[3]) == 1 ? true : false;
                break;

            case M_SENSOR_DIR: 
                sensor_dir = atoi(&user_command[3]) == 1 ? Direction::CW : Direction::CCW;
                sensor_offset = sensor_dir == Direction::CW ? sensor_offset : -sensor_offset;
                break;

            case M_VEL_LIMIT:  
                bldc->velocity_limit = atof(&user_command[3]);
                break;

            case M_ANGLE_RAMP_FT: {
                    value = strtok(0, ":");
                    bldc->P_angle.output_ramp = atof(value);

                    value = strtok(0, ":");
                    bldc->LPF_angle.Tf = atof(value);
                }
                break;

            case M_ANGLE_PID:  {
                    value = strtok(0, ":");
                    bldc->P_angle.P = atof(value);

                    value = strtok(0, ":");
                    bldc->P_angle.I = atof(value);
                }

                break;

            case M_VEL_RAMP_FT: {
                    value = strtok(0, ":");
                    bldc->PID_velocity.output_ramp = atof(value);

                    value = strtok(0, ":");
                    bldc->LPF_velocity.Tf = atof(value);
                }
                break;

            case M_VEL_PID:  {
                    value = strtok(0, ":");
                    bldc->PID_velocity.P = atof(value);

                    value = strtok(0, ":");
                    bldc->PID_velocity.I = atof(value);
                }
                break;

            case M_RANGE:  
                range = atof(&user_command[3]);
                break;

            case M_STARTPOS:  
                start_pos = atof(&user_command[3]);
                break;

            case M_CONSTRAIN_RANGE:  
                constrain_range = atoi(&user_command[3]) == 1 ? true : false;
                break;

            case M_PLAY: {
                    uint8_t play = atoi(&user_command[3]);
                    RUN = play == 1 ? true : false;
                    if (RUN) { 
                        if (bldc->enabled == 0) { bldc->enable(); }
                        start_time = millis(); 
                        current_time = start_time;
                    } else {
                        if (bldc->enabled == 1) { bldc->disable(); }
                        library.resetVelocityEffects();
                    }
                }
                break;

            case M_RETURN: {
                    value = strtok(0, ":");
                    uint8_t returnType = atoi(value);
                    if (returnType == 0) {
                        if (bldc->enabled == 0) { bldc->enable(); }
                        RUN = false;
                        MOVE = true;
                        float target_angle = atof(&user_command[3]);
                        target_val = target_angle * _DEG_PI * sensor_dir;
                        driver->voltage_limit = driver->voltage_power_supply;

                    } else if (returnType == 1) {
                        return_to_pos = false;
                        start_pos_loop = 0.0;

                    } else if (returnType == 2) {
                        value = strtok(0, ":");
                        return_to_pos = true;
                        start_pos_loop = atof(value);
                    }
                }
                break;

            case M_CALIBRATE:  
                bldc->zero_electric_angle = NOT_SET;
                break;

            case M_CURRENT_SENSE:
                CURRENT_SENSE_ACTIVE = atoi(&user_command[3]) == 1 ? true : false;
                break;

            case M_CALIBRATE_CS: {
                    if (current_sense != NULL) {
                        CURRENT_SENSE_ACTIVE = true;
                        calibrateCurrentSenseValues();
                    }
                }
                break;

            case M_CS_THRESHOLD: 
                current_threshold = atof(&user_command[3]);
                break;

            case M_TRANSMISSION: 
                transmission_factor = atof(&user_command[3]);
                break;

            // case M_I2C_CONNECTION:
                // const int state = atoi(&user_command[3]);
                // I2C_state = I2C_State(state);
                // Serial.println("&update i2c state" + I2C_state);

                // if (I2C_state != NO_COMMUNICATION) { initI2C(I2C_state); }
                // break;
        };
    // }
    Serial.println("*");
}




void FeelixM::Return_Data(char* user_command) {
    char identifier = user_command[1];
    char sub_cmd = user_command[2];
    // Serial.println((String) "&" + identifier + " "+ sub_cmd);

    // if (identifier != id && I2C_state == I2C_State::MASTER) {        
    //     address = convert_ID_to_I2C_address(&identifier);

    //     char *cstr = convertData(user_command, cstr);
    //     // Serial.println((String) "&data " + cstr);
    //     transmitDataI2C(address, cstr);

    //     // delay(100);
    //     // Serial.println((String) "&request from " + address);
    //     Wire.requestFrom(address, 23);
        
    // } else {
        

        switch(sub_cmd){

            case G_ANGLE: {
                    Serial.println((String) "R" + id + ":A:" + angle);
                    // if (state == State::SPI_SLAVE) {
                    //     dataRequestType = 'R';
                    // } 
                }
                break;
        };
    // }
}




void FeelixM::Effect_Data(char* user_command){ 
    char identifier = user_command[1];
    
    // if (!transferData(identifier, user_command)) {

        uint8_t effect_id = atoi(&user_command[2]);
        char sub_cmd = user_command[3];
        char* value = strtok(user_command, ":");
        value = strtok(0, ":");

        if (effect_id > lastEffectID) {
            library.effect_count++;
            lastEffectID = effect_id;
            library.effect[effect_id].index = effect_id;
        }

        switch(sub_cmd){

            case CMD_E_DIR:  
                library.effect[effect_id].direction.cw = atoi(value) == 1 ? true : false;
                value = strtok(0, ":");
                library.effect[effect_id].direction.ccw = atoi(value) == 1 ? true : false;
                break;
            case CMD_E_FLIP:  
                library.effect[effect_id].flip.x = atoi(value);
                value = strtok(0, ":");
                library.effect[effect_id].flip.y = atoi(value);
                value = strtok(0, ":");
                library.effect[effect_id].flip.middleline_y = atof(value);
                break;
            case CMD_E_POS: 
                library.effect[effect_id].position = atof(value);
                break;
            case CMD_E_SCALE:  
                library.effect[effect_id].scale.x = atof(value);
                value = strtok(0, ":");
                library.effect[effect_id].scale.y = atof(value);
                break;
            case CMD_E_ANGLE: 
                library.effect[effect_id].angle = atof(value);
                break;
            case CMD_E_INF:
                library.effect[effect_id].infinite = atoi(value) == 1 ? true : false;
                break;
            case CMD_E_CONTROL_TYPE:
                
                if (atoi(value) == 0) {
                    library.effect[effect_id].control_type = Control_type::TORQUE;
                } else if (atoi(value) == 1) {
                    library.effect[effect_id].control_type = Control_type::POSITION;
                } else if (atoi(value) == 2) {
                    library.effect[effect_id].control_type = Control_type::VELOCITY;
                    RUN = false;
                } else if (atoi(value) == 3) {
                    library.effect[effect_id].control_type = Control_type::VELOCITY_ANGLE;
                    RUN = false;
                }
                break;
            case CMD_E_EFFECT_TYPE: 
                library.effect[effect_id].effect_type = atoi(value) == 0 ? Effect_type::DEPENDENT : Effect_type::INDEPENDENT;
                break;
            case CMD_E_DATA_SIZE: 
                library.effect[effect_id].data_size = atoi(value);
                break;
            case CMD_E_POINTER: 
                library.effect[effect_id].data_ptr = atoi(value);
                break;
            case CMD_E_QUALITY: 
                library.effect[effect_id].quality = atof(value);
                break;
            case CMD_E_COPIES:
                library.effect[effect_id].copy[library.effect[effect_id].copy_count] = atof(value);
                library.effect[effect_id].copy_count++;   
                break;
        }        
    // }

    Serial.println("*");
}


void FeelixM::Data_Points(char* user_command){ 
    // char cmd = user_command[1];
    char identifier = user_command[2];

    // if (!transferData(identifier, user_command)) {
        
        char* value = strtok(user_command, ":");
        value = strtok(0, ":");
        uint8_t i = 0;

        while (value != NULL) {
                
            uint16_t mod = (CURRENT_ARRAY_INDEX % MAX_SUB_ARRAY_SIZE);
            uint16_t section = floor(CURRENT_ARRAY_INDEX / MAX_SUB_ARRAY_SIZE);
            library.d[section].sub[mod] = atof(value);

            CURRENT_ARRAY_INDEX++;
            
            
            if (CURRENT_ARRAY_INDEX > MAX_ARRAY_SIZE * MAX_SUB_ARRAY_SIZE) {
                Serial.println("M");
            }

            value = strtok(0, ":");
            i++;
        } 

    // }

    Serial.println("*");

};


void FeelixM::broadcastRequest() {
    for (int8_t n = 0; n < nrOfConnectedDevices; n++) {
        requestDataI2C(I2C_connections[n]);
    }
}


void FeelixM::send_data() {
    if (!PROCESSING && (RUN || MOVE)) {   

        // if (state == State::I2C_SLAVE) {
        //     dataRequestType = 'A';
        // }

        // } else if (I2C_state == I2C_State::MASTER) {
        

            // if (I2C_state != I2C_State::SLAVE) {
        Serial.println((String) "A" + id + ":" + angle + ":" + velocity + ":" + current_time + ":" + target_val);
            // }
        

        // if (state == State::I2C_MASTER && nrOfConnectedDevices > 0) {
        //     broadcastRequest();
        // }
        // }
    }
} 


int FeelixM::listDevices() {
    if (state == State::I2C_MASTER) {
        PROCESSING = true;
        byte error, address;
        int nDevices;

        // Serial.println("LScanning...");

        nDevices = 0;
        for(address = 1; address < 127; address++ ) {
            // The i2c_scanner uses the return value of
            // the Write.endTransmisstion to see if
            // a device did acknowledge to the address.
            Wire.beginTransmission(address);
            error = Wire.endTransmission(true);
            

            if (error == 0) {
                Serial.print("LI2C device found at address 0x");
                if (address<16)  
                    Serial.print("0");
                    Serial.println(address,HEX);

                nDevices++;
            } else {
                Serial.print("Lerror ");
                Serial.println(error);
            }

        }
        if (nDevices == 0)
            Serial.println("LNo I2C devices found\n");
        else
            Serial.println("Lscane complete");
            PROCESSING = false;
            return nDevices;
    }
    return 0;

    Serial.println("*");
}

void FeelixM::blinkStatusLED() {
    if (STATUS_LED_ENABLED) {
        if (millis() - blink_time > blink_interval) {
            blink_time = millis();
            LED_STATE = !LED_STATE;  

            if (LED_STATE) {
                digitalWrite(STM32_LED_PINK, LOW);  
            } else if (!LED_STATE) {
                digitalWrite(STM32_LED_PINK, HIGH);  
            }
        }
    }
}




void FeelixM::requestDataI2C(int I2C_ADDR) {

  Wire.requestFrom(I2C_ADDR, 40);
}






void FeelixM::transmitDataI2C(int I2C_ADDR, char* str) {
//dtostrf(float_value, min_width, num_digits_after_decimal, where_to_store_string)
//   Serial.println((String) "&Send data over I2C " + str);
  Wire.beginTransmission(I2C_ADDR); 
  Wire.write(str);              
  byte error = Wire.endTransmission(true);    
  if (error != 0) {
    Serial.println((String) "#error " + error);
  }
  
}



void FeelixM::testAlignmentAndCogging(int direction, int sample_count, int shaft_rotation) {

  bldc->move(0);
  _delay(200);

  sensor->update();
  float initialAngle = sensor->getAngle(); 

  float stDevSum = 0;

  float mean = 0.0f;
  float prev_mean = 0.0f;


  for (int i = 0; i < sample_count; i++) {

    float shaftAngle = (float) direction * i * shaft_rotation / sample_count;
    float electricAngle = (float) shaftAngle * bldc->pole_pairs;
    // move and wait
    bldc->move(shaftAngle * PI / 180);
    _delay(5);

    // measure
    sensor->update();
    float sensorAngle = (sensor->getAngle() - initialAngle) * 180 / PI;
    float sensorElectricAngle = sensorAngle * bldc->pole_pairs;
    float electricAngleError = electricAngle - sensorElectricAngle;

    // use knuth standard deviation algorithm so that we don't need an array too big for an Uno
    prev_mean = mean;
    mean = mean + (electricAngleError-mean)/(i+1);
    stDevSum = stDevSum + (electricAngleError-mean)*(electricAngleError-prev_mean);

    Serial.println((String) "P" + sensorAngle + ":" + electricAngle + ":" + sensorElectricAngle + ":" + electricAngleError + ":" + i); 
  }

  Serial.println((String) "Q" + direction + ":" + mean + ":" + (stDevSum/sample_count)); 

}



void FeelixM::Cogging_Test() { //char* user_command
    // char identifier = user_command[1];

    const float prev_voltage_limit = bldc->voltage_limit;
    bldc->voltage_sensor_align = 3;
    bldc->foc_modulation = FOCModulationType::SpaceVectorPWM;
    bldc->controller = MotionControlType::angle_openloop;
    bldc->voltage_limit = bldc->voltage_sensor_align;

    bldc->initFOC();

    const int shaft_rotation = 720; // 720 deg test - useful to see repeating cog pattern
    int sample_count = int(shaft_rotation * bldc->pole_pairs); // test every electrical degree

    testAlignmentAndCogging(1, sample_count, shaft_rotation);

    bldc->move(0);

    Serial.println((String) "T1:" + sample_count); //completed round 1

    _delay(1000);

    testAlignmentAndCogging(-1, sample_count, shaft_rotation);

    bldc->voltage_limit = 0;
    bldc->move(0);
    
    while (true) ;

    Serial.println((String) "T2:" + sample_count); //completed round 2

}


void FeelixM::disable() {
  bldc->move(0);
  RUN = false;
}


void FeelixM::readTemperature() {

  if (!PROCESSING && (RUN || MOVE)) {   

    if (millis() - tempRead_time > tempRead_interval) {
      tempRead_time = millis();

      temperature = temperatureSensor->getTemperature();
      driverVoltage = calculateDriverVoltage();

      Serial.println((String) "W" + id + ":" + temperature + ":" + driverVoltage); 

      if (temperature > 65) {   
        disable();
        digitalWrite(STM32_LED_ORANGE, HIGH); 
      } 
    }
  }
}


float FeelixM::calculateDriverVoltage() {
  return (((float)(analogRead(STM32_Vm_b_adc) + 8)) * 0.00732421875);
}


void FeelixM::printDRV8316Status() {
	DRV8316Status status = driver->getStatus();
	Serial.println("DRV8316 Status:");
	Serial.print("Fault: ");
  Serial.println(status.isFault());
  bool drvOFF = driver->isDriverOffEnabled();
  delayMicroseconds(1); 
	Serial.print("drvOFF: ");
  Serial.println(drvOFF);
  if (drvOFF)
		driver->setDriverOffEnabled(false);
  delayMicroseconds(1); 
	Serial.print("Buck Error: ");
	Serial.print(status.isBuckError());
	Serial.print("  Undervoltage: ");
	Serial.print(status.isBuckUnderVoltage());
	Serial.print("  OverCurrent: ");
	Serial.println(status.isBuckOverCurrent());
	Serial.print("Charge Pump UnderVoltage: ");
	Serial.println(status.isChargePumpUnderVoltage());
	Serial.print("OTP Error: ");
	Serial.println(status.isOneTimeProgrammingError());
	Serial.print("OverCurrent: ");
	Serial.print(status.isOverCurrent());
	Serial.print("  Ah: ");
	Serial.print(status.isOverCurrent_Ah());
	Serial.print("  Al: ");
	Serial.print(status.isOverCurrent_Al());
	Serial.print("  Bh: ");
	Serial.print(status.isOverCurrent_Bh());
	Serial.print("  Bl: ");
	Serial.print(status.isOverCurrent_Bl());
	Serial.print("  Ch: ");
	Serial.print(status.isOverCurrent_Ch());
	Serial.print("  Cl: ");
	Serial.println(status.isOverCurrent_Cl());
	Serial.print("OverTemperature: ");
	Serial.print(status.isOverTemperature());
	Serial.print("  Shutdown: ");
	Serial.print(status.isOverTemperatureShutdown());
	Serial.print("  Warning: ");
	Serial.println(status.isOverTemperatureWarning());
	Serial.print("OverVoltage: ");
	Serial.println(status.isOverVoltage());
	Serial.print("PowerOnReset: ");
	Serial.println(status.isPowerOnReset());
	Serial.print("SPI Error: ");
	Serial.print(status.isSPIError());
	Serial.print("  Address: ");
	Serial.print(status.isSPIAddressError());
	Serial.print("  Clock: ");
	Serial.print(status.isSPIClockFramingError());
	Serial.print("  Parity: ");
	Serial.println(status.isSPIParityError());
	if (status.isFault())
    delay(1);
		driver->clearFault();
	delayMicroseconds(1); // ensure 400ns delay
  
  delayMicroseconds(1); // ensure 400ns delay
	DRV8316_PWMMode val = driver->getPWMMode();
	Serial.print("PWM Mode: ");
	Serial.println(val);
	delayMicroseconds(1); // ensure 400ns delay
	bool lock = driver->isRegistersLocked();
	Serial.print("Lock: ");
	Serial.println(lock);
}




void FeelixM::homeMotor() {
  //Serial.println(F("Homing motor..."));

  sensor->update();
  // Variables to track motor state
  float previousAngle = sensor->getAngle();

  int stableReadings = 0;

  // Initial motor motion to ensure contact with the base
  unsigned long startTime = millis();
  while (millis() - startTime < hydroHaptics->HomeMotorRunDelay) {
    bldc->loopFOC();
    target = hydroHaptics->HomeSearchVoltage;
    bldc->move(-target);

    //delay(2);
  }

  // Main loop for homing
  while (true) {
    // Move the motor downward
    bldc->loopFOC();
    target = hydroHaptics->HomeSearchVoltage;
    bldc->move(-target);

    delay(2);

    // Read and evaluate the current angle
    float currentAngle = sensor->getAngle();

    if (abs(currentAngle - previousAngle) < hydroHaptics->HomeAngleThreshold) {
      stableReadings++;
    } else {
      stableReadings = 0;  // Reset if angle changes significantly
    }

    // Update the previous angle for the next iteration
    previousAngle = currentAngle;

    // Stop the motor if stable readings exceed the threshold
    if (stableReadings >= hydroHaptics->HomeStabilityCount) {
      //Serial.println(F("Bottom position reached using encoder"));

      // Collect multiple readings for averaging
      float angleSum = 0.0;
      for (int i = 0; i < hydroHaptics->HomeAveragingSamples; i++) {
        bldc->loopFOC();  // Ensure FOC continues during averaging
        angleSum += sensor->getAngle();
        delay(2);        // ShorTt delay between samples
      }

      // Compute and set the averaged angle as the motor datum
      hydroHaptics->motorAngleDatum = angleSum / hydroHaptics->HomeAveragingSamples;
      //Serial.print(F("Averaged motor datum: "));
      //Serial.println(hydroHaptics->motorAngleDatum);

      
      target = 0;
      bldc->move(target);
      bldc->loopFOC(); // Ensure torque is removed

      break;
    }
  }
}
  

  

  
  
  
void FeelixM::height_Sweep_Voltage_PID() {
  float targetHeightSweep;
  int Hold_Count;
  int index = 0;
  float avgHeight = 0;
  float avgPressure = 0;
  float avgVoltage = 0;


  // Loop through the height range
  for (targetHeightSweep = hydroHaptics->touchMinHeight; targetHeightSweep <= hydroHaptics->touchMaxHeight; targetHeightSweep += hydroHaptics->touchHeightStep) {
    int iterations = 0;
    Hold_Count = 0;

    hydroHaptics->targetHeight_PID = targetHeightSweep;

    while (Hold_Count < hydroHaptics->holdFrames_Seek) {

      // Phase 1 - Seek the target height using the Height PID
      if (iterations > hydroHaptics->maxIterations){
        // If the device can't reach a height target within a set number of loops, then stop the code and report a message
        while(true){
          // Serial.println("Max Iterations Exceeded - Calibration Failed - Reset Arduino");
          target = 0.0;
          bldc->move(target);
          bldc->loopFOC();
        }
      }

      hydroHaptics->updateHeight(sensor->getAngle());
      hydroHaptics->pressureSensor->updatePressure();

      float HeightPIDError = updateHeightPID();

      bldc->loopFOC();
      bldc->move(-target);

      if (abs(HeightPIDError) < hydroHaptics->heightHoldTolerance){
        // Hold for 200 Frames
        Hold_Count++;
      } else {
        Hold_Count = 0;
      }

      iterations++;
    }

    Hold_Count = 0;

    while (Hold_Count < hydroHaptics->holdFrames_Avg) {
      // Phase 2 - Hold the Target height using PID
      // Hold The achived pressure for 200 frames and take an average of Pressure, height and Voltage at that point

      if (iterations > hydroHaptics->maxIterations){
        while(true){
          // Serial.println("Max Iterations Exceeded - Calibration Failed - Reset Arduino");
          target = 0.0;
          bldc->move(target);
          bldc->loopFOC();
        }
      }

      hydroHaptics->updateHeight(sensor->getAngle());
      hydroHaptics->pressureSensor->updatePressure();
      float HeightPIDError = updateHeightPID();
      bldc->loopFOC();
      bldc->move(-target);

      if (abs(HeightPIDError) < hydroHaptics->heightHoldTolerance){
        // Hold for 200 Frames
        Hold_Count++;
        avgHeight += hydroHaptics->RD_Height;
        avgPressure += hydroHaptics->pressureSensor->HC_Pressure;
        avgVoltage += target;  // Voltage is the same for each iteration

      } else {
        Hold_Count = 0;
        avgHeight = 0;
        avgPressure = 0;
        avgVoltage = 0;
      }
    }

    // Calculate the average values over the hold period
    avgHeight /= hydroHaptics->holdFrames_Avg;
    avgPressure /= hydroHaptics->holdFrames_Avg;
    avgVoltage /= hydroHaptics->holdFrames_Avg;

    // Store the averaged values in the data arrays
    hydroHaptics->touchVoltageData[index] = avgVoltage;  // Store the average voltage
    hydroHaptics->touchPressureData[index] = avgPressure;  // Store the average pressure
    hydroHaptics->touchHeightData[index] = avgHeight;  // Store the average height

    // Output the averaged results for each height step
    Serial.print("T1");
    Serial.print(id);
    Serial.print(":");
    Serial.print(avgHeight, 5); //height
    Serial.print(":");
    Serial.print(avgVoltage, 5); //voltage
    Serial.print(":"); 
    Serial.println(avgPressure, 5); //pressure

    index++;
  }

  range = sensor->getAngle() - start_pos;

  hydroHaptics->targetHeight_PID = 0;

  Serial.print("T2");
  Serial.print(id);
  Serial.print(":");
  Serial.print(start_pos, 8); //height
  Serial.print(":");
  Serial.println(range, 8); 
}
  


float FeelixM::updateHeightPID() {
  
    float error = hydroHaptics->targetHeight_PID - hydroHaptics->RD_Height;
    float output = 0;

    hydroHaptics->Height_integral += error;  //Update Rolling integral

    // Calculate derivative (rate of change of error)
    float derivative = error - hydroHaptics->Height_previousError;

    // Compute PID output (voltage adjustment)
    output = hydroHaptics->Height_Kp * error + hydroHaptics->Height_Ki * hydroHaptics->Height_integral + hydroHaptics->Height_Kd * derivative;

    // Save the current error for the next loop
    hydroHaptics->Height_previousError = error;

    // // Map the PID output to the appropriate voltage range
    // // Assuming your voltage range is between -5V and 5V, adjust as needed
    target = output < 0.00 ? constrain(output, -7, -1.5) : output > 0.00 ? constrain(output, 1.5, 7) : 0.0;

    return error;
}
