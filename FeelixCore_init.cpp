/*

    Feelix V3.0 IO Init
    3 proto testing versions
    STM32F401RCTx

*/

#include "FeelixCore.h"



TwoWire Wire1_EXT(STM32_I2C1_SDA_EXT, STM32_I2C1_SCL_EXT);
TwoWire Wire2_LM75(STM32_I2C2_SDA_LM75, STM32_I2C2_SCL_LM75);

LM75 _temperatureSensor = LM75(&Wire2_LM75);


// SPI

SPIClass SPI1_DRV(
    STM32_SPI1_MOSI_DRV,
    STM32_SPI1_MISO_DRV,
    STM32_SPI1_CLK_DRV,
    STM32_SPI1_CS_DRV);

SPIClass SPI2_EXT(
    STM32_SPI2_MOSI_EXT,
    STM32_SPI2_MISO_EXT,
    STM32_SPI2_CLK_EXT);

SPIClass SPI3_ENC(
    STM32_SPI3_MOSI_ENC,
    STM32_SPI3_MISO_ENC,
    STM32_SPI3_CLK_ENC);

void SystemClock_Config(void);
void function_STM32F401RCTx_Init_V31(void);


PressureSensor _pressureSensor = PressureSensor(15.0, 135.0);

HydroHaptics _hydroHaptics = HydroHaptics(2.0, &_pressureSensor, &SPI2_EXT);


void FeelixM::init()
{
  function_STM32F401RCTx_Init_V31();

  sensor->init(&SPI3_ENC);
  bldc->linkSensor(sensor);

  angle = bldc->shaftAngle(); //(180 / 3.14159);
  rotation_dir = Direction::CW;

  driver->pwm_frequency = STM32_PWM_FREQUENCY;
  driver->init(&SPI1_DRV);
  driver->enable();

  bldc->linkDriver(driver);

  bldc->controller = MotionControlType::torque;
  bldc->torque_controller = TorqueControlType::voltage;
  bldc->foc_modulation = FOCModulationType::SpaceVectorPWM;

  bldc->voltage_sensor_align = 10.0;

  bldc->PID_velocity.P = 0.5;
  bldc->PID_velocity.I = 10.0;
  bldc->PID_velocity.D = 0.0;
  bldc->PID_velocity.output_ramp = 1000;
  bldc->LPF_velocity.Tf = 0.01;

  bldc->P_angle.P = 14.0;
  bldc->P_angle.I = 0.0;
  bldc->P_angle.D = 0.0;
  bldc->P_angle.output_ramp = 10000;

  bldc->velocity_limit = 22.0;
  bldc->voltage_limit = 12.0;
  vol_limit = 12.0;
  driver->voltage_limit = 12.0;

  DRV_SPI_initialize();

  bldc->init();

  /* initialize i2c temperature sensor */
  temperatureSensor = &_temperatureSensor;
  hydroHaptics = &_hydroHaptics;

  Wire2_LM75.setSDA(STM32_I2C2_SDA_LM75);
  Wire2_LM75.setSCL(STM32_I2C2_SCL_LM75);
  Wire2_LM75.begin();

  initSPI_EXT();
  initI2C_EXT();

}


void FeelixM::DRV_SPI_initialize()
{
  driver->setDriverOffEnabled(false);
  delayMicroseconds(1);
  driver->setPWMMode(PWM6_Mode);
  delayMicroseconds(1);
  driver->setSDOMode(SDOMode_PushPull);
  delayMicroseconds(1);
  driver->setBuckEnabled(false);
}


void FeelixM::initSPI_EXT()
{

  // if (state == State::SPI_SLAVE) {

  SPI2_EXT.begin();
  SPI2_EXT.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  pinMode(STM32_SPI2_CS_EXT, OUTPUT);
  digitalWrite(STM32_SPI2_CS_EXT, HIGH);

  // }
}

void FeelixM::initI2C_EXT()
{

  if (state == State::I2C_SLAVE)
  {
    Wire1_EXT.setSDA(STM32_I2C1_SDA_EXT);
    Wire1_EXT.setSCL(STM32_I2C1_SCL_EXT);

    Wire1_EXT.begin(I2C_address);

    // Wire.onReceive(receiveDataI2C_slave);
    // Wire.onRequest(requestDataI2C_slave);
  }
  else if (state == State::I2C_MASTER)
  {
    Wire1_EXT.begin();
    Wire1_EXT.setClock(100000);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void function_STM32F401RCTx_Init_V31(void)
{
  uint variable_dummy [[maybe_unused]];

  // analogWriteResolution() ?

  // STM32 pin #56
  // Orange LED
  pinMode(STM32_LED_ORANGE, OUTPUT);
  digitalWrite(STM32_LED_ORANGE, LOW);
  // analogWrite

  // STM32 pin #57
  // Pink LED
  pinMode(STM32_LED_PINK, OUTPUT);
  digitalWrite(STM32_LED_PINK, LOW);
  // analogWrite

  // STM32 pin #28
  // Pull Down on this board. No further use as DFU Firmware
  pinMode(STM32_BOOT1, INPUT);

  // DRV Digital In

  // STM32 pin #16
  // Fault indicator.
  // Pulled logic-low with fault condition;
  // Open-drain output requires an external pull-up resistor to 1.8 V to 5.0 V.
  // If external supply is used to pull up nFAULT,
  // ensure that it is pulled to >2.2 V on power up or the device will enter test mode
  pinMode(STM32_nFAULT, INPUT_PULLUP);

  // DRV Digital Out

  // STM32 pin #14
  // Driver nSLEEP.
  // When this pin is logic low,
  // the device goes into a low-power sleep mode.
  // An 20 to 40-μs low pulse can be used to reset fault conditions
  // without entering sleep mode.
  pinMode(STM32_nSLEEP, OUTPUT);
  digitalWrite(STM32_nSLEEP, HIGH);

  // STM32 pin #15
  // When this pin is pulled high,
  // the six MOSFETs in the power stage are turned OFF making all outputs Hi-Z.
  pinMode(STM32_DRVOFF, OUTPUT);
  digitalWrite(STM32_DRVOFF, HIGH); // enabled in later stage, avoid startup glitches

  // ADC

  analogReadResolution(12);

  // STM32 pin #17
  // Init analog read
  // Analog readout Vm_b
  variable_dummy = analogRead(STM32_Vm_b_adc);

  // STM32 pin #8
  // Init analog read
  variable_dummy = analogRead(STM32_SOA);

  // STM32 pin #9
  // Init analog read
  variable_dummy = analogRead(STM32_SOB);

  // STM32 pin #10
  // Init analog read
  variable_dummy = analogRead(STM32_SOC);

  // AUX GPIO/ADC External connector

  // STM32 pin #27
  // ADC and GPIO pin
  variable_dummy = analogRead(STM32_J1_GPIO_PIN2);

  // STM32 pin #26
  // ADC and GPIO pin
  variable_dummy = analogRead(STM32_J1_GPIO_PIN3);

  // STM32 pin #25
  // ADC and GPIO pin
  variable_dummy = analogRead(STM32_J1_GPIO_PIN4);

  // STM32 pin #24
  // ADC and GPIO pin
  variable_dummy = analogRead(STM32_J1_GPIO_PIN5);

  // PWM

  analogWriteResolution(16);
  analogWriteFrequency(STM32_PWM_FREQUENCY);

  // STM32 pin #41
  analogWrite(STM32_INLA, 0);
  // STM32 pin #42
  analogWrite(STM32_INHA, 0);

  // STM32 pin #39
  analogWrite(STM32_INLB, 0);
  // STM32 pin #40
  analogWrite(STM32_INHB, 0);

  // STM32 pin #37
  analogWrite(STM32_INLC, 0);
  // STM32 pin #38
  analogWrite(STM32_INHC, 0);

  // N.C.

  // STM32 pin #43
  pinMode(STM32_PA10_NC, OUTPUT);
  digitalWrite(STM32_PA10_NC, LOW);

  // STM32 pin #50
  pinMode(STM32_PA15_NC, OUTPUT);
  digitalWrite(STM32_PA15_NC, LOW);

  // STM32 pin #61
  pinMode(STM32_PB8_NC, OUTPUT);
  digitalWrite(STM32_PB8_NC, LOW);

  // STM32 pin #62
  pinMode(STM32_PB9_NC, OUTPUT);
  digitalWrite(STM32_PB9_NC, LOW);

  // STM32 pin #11
  pinMode(STM32_PC3_NC, OUTPUT);
  digitalWrite(STM32_PC3_NC, LOW);

  // STM32 pin #2
  pinMode(STM32_PC13_NC, OUTPUT);
  digitalWrite(STM32_PC13_NC, LOW);

  // STM32 pin #3
  pinMode(STM32_PC14_NC, OUTPUT);
  digitalWrite(STM32_PC14_NC, LOW);

  // STM32 pin #4
  pinMode(STM32_PC15_NC, OUTPUT);
  digitalWrite(STM32_PC15_NC, LOW);

  // Enable MOSFETS
  digitalWrite(STM32_DRVOFF, LOW);
}
