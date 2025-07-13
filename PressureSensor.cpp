#include "PressureSensor.h"



PressureSensor::PressureSensor(float _min, float _max) {
    pressureMin = _min;
    pressureMax = _max;
}



float PressureSensor::calculate_raw_to_pressure(int output) {
    return (float(constrain(output, pressureRawMin, pressureRawMax) - pressureRawMin) * (pressureMax - pressureMin) / (pressureRawMax - pressureRawMin) + pressureMin) * PSI_KPA_Conversion;  //This function should return the Atms pressure in Kpa
}
  
float PressureSensor::getPressure() {
    return HC_Pressure;
}
  
float PressureSensor::calculatePressurePIDError() {
    pressurePIDError = targetPressure_PID - HC_Pressure;
    return pressurePIDError;
}
  
    // ------------------------- Pressure Set Functions ---------------------------------
  
void PressureSensor::updateSetPressure(float setTargetPressure_MOD) {
    targetPressure_PID = setTargetPressure_PID + setTargetPressure_MOD;
      // setMotorVoltage(calculateFeedForward());
}
 
  
void PressureSensor::setPressure(float setTargetPressure_MOD) {
  targetPressure_PID = setTargetPressure_MOD;
  // setMotorVoltage(calculateFeedForward());
}



  
float PressureSensor::calculateFFPressure() {
    float pressureFeedForward = targetPressure_PID * 0.022 - 0.1;
    // Serial.print("Pressure_FF ");
    // Serial.print(pressureFeedForward);
    return pressureFeedForward;
}
  

float PressureSensor::updatePressurePID(float height) {

  // pressurePIDError = targetPressure_PID - HC_Pressure;

  float PIDfeedforward = calculateFeedForward(height);//hydroHaptics->calculateFFHeight()

   Pressure_integral +=  pressurePIDError;  //Update Rolling integral

  float integral_max = 150.0;  // Set a reasonable limit, adjust as needed

    if ( Pressure_integral > integral_max) {
       Pressure_integral = integral_max;
    } else if ( Pressure_integral < -integral_max) {
       Pressure_integral = -integral_max;
    }
  // Calculate derivative (rate of change of error)
  float derivative =  pressurePIDError -  Pressure_previousError;

   pressurePIDOutput =  Pressure_Kp * pressurePIDError + Pressure_Ki * Pressure_integral + Pressure_Kd * derivative; //PIDfeedforward

  // Save the current error for the next loop
   Pressure_previousError =  pressurePIDError;

  // Constrain the PID output to the appropriate voltage range
  // Serial.println("Pressure PID Out = ")
  // Serial.printlmn
  
  float target = pressurePIDOutput < 0.00 ? constrain( pressurePIDOutput, -7, -1.5) :
                 pressurePIDOutput > 0.00 ? constrain( pressurePIDOutput, 1.5, 7) : 0.0;

  return  target;

}

  
uint16_t PressureSensor::getData() {

    uint8_t receivedData[pressureRawDataLength];  // Array to store the received data
  
    spi->beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE0)); 
    // // Start SPI communication
    digitalWrite(STM32_SPI2_CS_EXT, LOW);  // Select the sensor (bring CS low)
  
    // Read two bytes of pressure data
    for (int i = 0; i < pressureRawDataLength; i++) {
      receivedData[i] = spi->transfer(0x00);  // Send dummy byte to read data
    }
  
    // Deactivate the sensor
    digitalWrite(STM32_SPI2_CS_EXT, HIGH);
  
    spi->endTransaction();
  
    uint16_t rawValue = (receivedData[0] << 8) | receivedData[1];

    return rawValue;

}


void PressureSensor::updatePressure() {

    uint16_t rawValue = getData();
    
    float rawPressure = calculate_raw_to_pressure(rawValue); // - zeroPressure - pressureDrift

    // Serial.print("rawPressure ");
    // Serial.print(rawPressure);
    // Serial.print("\t");
  
    // Apply the low-pass filter using the exponential moving average
    HC_Pressure = pressureFilterAlpha * rawPressure + (1.0 - pressureFilterAlpha) * HC_Pressure;

    // Serial.print("HC_Pressure ");
    // Serial.println(HC_Pressure);
}



void PressureSensor::checkOnTarget(float motorTarget) {

    bool PIDErrorCheck = (pressurePIDError < pressurePIDErrorThreshold);
    bool VoltageErrorCheck = (motorTarget < VoltagePIDErrorThreshold);
    onTarget = PIDErrorCheck || VoltageErrorCheck;
}
  



float PressureSensor::calculateFeedForward(float FFHeight) {
  if (onTarget) {
      feedforward = calculateFFPressure() - FFHeight;
  } else {
      feedforward = calculateFFPressure();
  }
  // return feedforward;
  return 0; //IGNORE FEEDFORWARD FOR 2.54 PITCH
}