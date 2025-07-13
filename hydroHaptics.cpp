#include "hydroHaptics.h"



HydroHaptics::HydroHaptics(float _leadScrewPitch, PressureSensor* _pressureSensor, SPIClass* _SPI2_EXT) {
    leadScrewPitch = _leadScrewPitch;
    pressureSensor = _pressureSensor;
    pressureSensor->spi = _SPI2_EXT;
}  


// void HydroHaptics::initHydro() {
//     // pressureSensor->spi = _spi;

//     pressureSensor->spi->begin();
//     pressureSensor->spi->setClockDivider(SPI_CLOCK_DIV16); 
//     pressureSensor->spi->setDataMode(SPI_MODE0);           // SPI Mode 0 (CPOL = 0, CPHA = 0)
//     pressureSensor->spi->setBitOrder(MSBFIRST);            // Most Significant Bit first
//     // pressureSensor->spi->beginTransaction(SPISettings(800000, MSBFIRST, SPI_MODE0));
//     pinMode(STM32_SPI2_CS_EXT, OUTPUT);
//     digitalWrite(STM32_SPI2_CS_EXT, HIGH);
// }




float HydroHaptics::calculateFFHeight() {
  float targetHeightOffset = constrain(interpolateHeight(pressureSensor->targetPressure_PID) - RD_Height, 0, 6);
  float heightFeedForward = (constrain((pressureSensor->targetPressure_PID * 0.002) + 0.01, -0.03, 0.08)) * targetHeightOffset;
  // Serial.print("target_Height_Offset ");
  // Serial.print(targetHeightOffset);
  // Serial.print("height_Feed_Forward ");
  // Serial.println(heightFeedForward);
  return heightFeedForward;
}



float HydroHaptics::interpolateHeight(float inputPressure) {
  // Check if the current height is within the range
  for (int i = 0; i < touchDataPoints - 1; i++) {
    if (inputPressure >= touchPressureData[i] && inputPressure <= touchPressureData[i + 1]) {
      // Linear interpolation formula for heights within range:
      float x0 = touchPressureData[i];
      float x1 = touchPressureData[i + 1];
      float y0 = touchHeightData[i];
      float y1 = touchHeightData[i + 1];

      // Interpolate the pressure for the current height
      float interpolatedHeight = y0 + (inputPressure - x0) * (y1 - y0) / (x1 - x0);
      return interpolatedHeight;
    }
  }

  // If the current height is lower than the minimum height
  if (inputPressure < touchPressureData[0]) {
    // Extrapolate using the gradient between the first two points
    float x0 = touchPressureData[0];
    float x1 = touchPressureData[1];
    float y0 = touchHeightData[0];
    float y1 = touchHeightData[1];

    // Calculate the slope and extrapolate
    float slope = (y1 - y0) / (x1 - x0);
    return y0 + slope * (inputPressure - x0);
  }

  // If the current height is higher than the maximum height
  if (inputPressure > touchPressureData[touchDataPoints - 1]) {
    // Extrapolate using the gradient between the last two points
    float x0 = touchPressureData[touchDataPoints - 2];
    float x1 = touchPressureData[touchDataPoints - 1];
    float y0 = touchHeightData[touchDataPoints - 2];
    float y1 = touchHeightData[touchDataPoints - 1];

    // Calculate the slope and extrapolate
    float slope = (y1 - y0) / (x1 - x0);
    return y1 + slope * (inputPressure - x1);
  }

  // Return 0 or an appropriate error value if something goes wrong
  return 0.0;
}






void HydroHaptics::updateHeight(float angle) {
    // Get the current angle from the sensor
    float currentAngle = (angle - motorAngleDatum);
    // Serial.print("current angle ");
    // Serial.println(currentAngle);
    // Calculate the height based on the angle and pitch
    RD_Height = (currentAngle / TWO_PI) * leadScrewPitch;
    // Serial.print("RD_Height ");
    // Serial.println(RD_Height);
}
    

void HydroHaptics::setHeightPID(float _targetHeight_PID){
    targetHeight_PID = _targetHeight_PID;
}





bool HydroHaptics::detectTouch() {
  // NOTE THIS FUNCTION IS BAD - IT DOES TWO STABLE COUNTS FOR NO REASON
  float targetTouchHeight = calculate_targetTouchHeight();
  static int stableTrueCount = 0;  // Counter to track stable 'True' touch state
  static int stableFalseCount = 0; // Counter to track stable 'False' touch state

  // Condition for touch (i.e., RD_Height is below the target with the threshold)
  if (RD_Height < (targetTouchHeight - (touchThreshold + pressureSensor->HC_Pressure * touchModThreshold))) {
    touchCount++;  // Increment the touch counter as touch condition is met

    // If the touch has been detected long enough
    if (touchCount > touchCountThreshold) {
      if (!touchFlag) {
        // If the touchFlag was previously false, it means we detected touch
        stableTrueCount++;  // Increase the stable 'True' counter
        stableFalseCount = 0;  // Reset the stable 'False' counter
        
        // If stableTrueCount is large enough, change the touchFlag
        if (stableTrueCount > touchCountThreshold) {
          touchFlag = true;
          stableTrueCount = 0;  // Reset the stableTrueCount after change
          touchStartTime = millis();
          touchStartFlag = true;    

          // if (touchStartFlag == false){
          //   touchStartTime = millis();
          //   touchStartFlag = true;    
          //   }

        
          if (!pressureSensor->pressureOffsetFlag) {
            pressureSensor->pressureOffsetFlag = true;
            pressureSensor->pressureOffset = pressureSensor->HC_Pressure;
            heightOffset = RD_Height;
          }
          return true;  // Touch detected
        }
      } else {
        touchTime = millis() - touchStartTime;
        return true;
      }
    } else {
      stableTrueCount = 0;  // Reset stableTrueCount if touchCount is not long enough
    }

    // If no touch, ensure counters are reset
    if (touchCount <= touchCountThreshold) {
      touchFlag = false;
      stableTrueCount = 0;
      touchStartFlag = false;
      touchTime = 0;
    }

    return false;
  }

  // Condition for no touch (i.e., RD_Height is above the target)
  if (RD_Height >= (targetTouchHeight - (touchThreshold + pressureSensor->HC_Pressure * touchModThreshold))) {
    touchCount = 0;  // Reset the touchCount
    stableFalseCount++;  // Track how long we've been in the 'False' state

    if (stableFalseCount > touchCountThreshold) {
      // If stableFalseCount is large enough, toggle the touchFlag to 'false'
      touchFlag = false;
      stableFalseCount = 0;  // Reset the stableFalseCount after change
      previousDisplacementHeightCalc = 0;
      pressureSensor->previousDisplacementPressureCalc = 0;
      pressureSensor->pressureOffsetFlag = false;  // Reset flag on no touch
      pressureSensor->pressureOffset = 0;
      heightOffset = 0;
    }
    return false;  // No touch detected
  }

  // Default return false when no touch condition is met
  touchFlag = false;
  touchCount = 0;
  stableTrueCount = 0;
  stableFalseCount = 0;
  previousDisplacementHeightCalc = 0;
  pressureSensor->previousDisplacementPressureCalc =0;
  pressureSensor->pressureOffsetFlag = false;
  pressureSensor->pressureOffset = 0;
  heightOffset = 0;

  return false;  // No touch detected
}


float HydroHaptics::calculate_targetTouchHeight(){
  // Look up HC_Pressure in float touchPressureData[touchDataPoints]; float touchHeightData[touchDataPoints];
  // This approach is not good (Intensive on the processing) This should be addressed through 2 ways.
  // 1 - Swap to a constant Poly Equation which calculates the target height (And can be adjusted based on collected data)
  // 2 - Run this function less often. 
  return interpolateHeight(pressureSensor->HC_Pressure - pressureSensor->pressureDriftOffset);
}

void HydroHaptics::calculate_Displacement(Effect effect){

  float targetTouchHeight = calculate_targetTouchHeight();

  if(touchFlag == false){
    calculatedDisplacement = 0;
    return;
  } else{
    float offsetHeight = RD_Height - targetTouchHeight;
    float offsetPressure = pressureSensor->HC_Pressure - pressureSensor->pressureOffset;

    // Calculate the displacement values
    displacementHeightCalc = offsetHeight * 1.2;
    pressureSensor->displacementpressureCalc = sq(offsetPressure) * 0.1 + 1.1 * offsetPressure;

    // Apply the EMA smoothing for displacementHeightCalc
    displacementHeightCalc = effect.displacementSmoothingAlpha * displacementHeightCalc + (1 - effect.displacementSmoothingAlpha) * 
        previousDisplacementHeightCalc;

    // Apply the EMA smoothing for displacementpressureCalc only if offsetHeight <= threshold
    if (offsetHeight >= threshold) {
      pressureSensor->displacementpressureCalc = effect.displacementSmoothingAlpha * pressureSensor->displacementpressureCalc + 
              (1 - effect.displacementSmoothingAlpha) * pressureSensor->previousDisplacementPressureCalc;
    }

    // Lock displacementpressureCalc if offsetHeight exceeds threshold
    if (offsetHeight < threshold) {
      pressureSensor->displacementpressureCalc = pressureSensor->previousDisplacementPressureCalc;  // Lock at previous value
    }

    // Update the previous displacement values for the next calculation
    previousDisplacementHeightCalc = displacementHeightCalc;
    pressureSensor->previousDisplacementPressureCalc = pressureSensor->displacementpressureCalc;
    
  // calculatedDisplacement = displacementHeightCalc + displacementpressureCalc;
    calculatedDisplacement = -displacementHeightCalc;
  }
}



// Function to update input speed and direction based on displacement
void HydroHaptics::updateSpeed(float displacement) {
    const float smoothingFactor = 0.05;  // Unique smoothing factor for this function
    const float directionThreshold = - 1; // Minimum speed needed to change direction

    // Apply exponential smoothing to remove noise
    smoothedDisplacement = smoothingFactor * displacement + (1 - smoothingFactor) * smoothedDisplacement;

    // Set input speed based on absolute displacement
    inputSpeed = (smoothedDisplacement - lastSmoothedDisplacement) * 1000;

    // Determine direction with threshold
    if ( inputSpeed < directionThreshold) {
        inputDirection = false;
    } else {
        inputDirection = true;  // Default to true if speed is too low
    }
    
    lastSmoothedDisplacement = smoothedDisplacement;
}



