// Board

/*
    https://docs.simplefoc.com/low_side_current_sense

    PWM frequency considerations

    As the ADC conversion takes some time to finish and as this conversion has to happen only during the specific time window
    ( when all the phases are grounded - low-side mosfets are ON )
    it is important to use an appropriate PWM frequency.
    PWM frequency will determine how long each period of the PWM is and in term how much time the low-side switches are ON.
    Higher PWM frequency will leave less time for the ADC to read the current values.

    On the other hand, having higher PWM frequency will produce smoother operation, so there is definitely a trade-off here.

    RULE OF THUMB: PWM frequency
    The rule of thumb is to stay arround 20kHz. driver.pwm_frequency = 20000;
*/


#define SoftwareVersion_major 3
#define SoftwareVersion_minor 1
#define SoftwareVersion_patch 2

#define STM32_PWM_FREQUENCY 20000 // 20000 recommended when using ADC current

#define CS_MAGNETIC_SENSOR_ADDRESS 0x3FFF
// Designed for GPIO pins on header. Not recommended for use
// void now. V3.1 redesign.

#define STM32_SWDIO PA13
#define STM32_SWDCLK PA14

// BOOT pins
#define STM32_BOOT1 PB2

// On board LED's

#define STM32_LED_ORANGE PB4
#define STM32_LED_PINK PB5

// DRV Digital In

#define STM32_nFAULT PA2

// DRV Digital Out

#define STM32_nSLEEP PA0
#define STM32_DRVOFF PA1

// ADC

#define STM32_Vm_b_adc A3 // PA3
#define STM32_SOA A10     // PC0
#define STM32_SOB A11     // PC1
#define STM32_SOC A12     // PC2

// AUX GPIO/ADC External connector
// Digital
#define STM32_J1_GPIO_PIN2 PB1
#define STM32_J1_GPIO_PIN3 PB0
#define STM32_J1_GPIO_PIN4 PC5
#define STM32_J1_GPIO_PIN5 PC4
// Analog
#define STM32_J1_ADC_PIN2 A9
#define STM32_J1_ADC_PIN3 A8
#define STM32_J1_ADC_PIN4 A15
#define STM32_J1_ADC_PIN5 A14

// PWM

#define STM32_INLA PA8
#define STM32_INHA PA9
#define STM32_INLB PC8
#define STM32_INHB PC9
#define STM32_INLC PC6
#define STM32_INHC PC7

// I2C

#define STM32_I2C1_SCL_EXT PB6
#define STM32_I2C1_SDA_EXT PB7

#define STM32_I2C2_SDA_LM75 PB3
#define STM32_I2C2_SCL_LM75 PB10

// SPI

#define STM32_SPI1_CS_DRV PA4
#define STM32_SPI1_CLK_DRV PA5
#define STM32_SPI1_MISO_DRV PA6
#define STM32_SPI1_MOSI_DRV PA7

#define STM32_SPI2_CS_EXT PB12
#define STM32_SPI2_CLK_EXT PB13
#define STM32_SPI2_MISO_EXT PB14
#define STM32_SPI2_MOSI_EXT PB15

#define STM32_SPI3_CS_ENC PD2
#define STM32_SPI3_CLK_ENC PC10
#define STM32_SPI3_MISO_ENC PC11
#define STM32_SPI3_MOSI_ENC PC12

// N.C.

#define STM32_PA10_NC PA10
#define STM32_PA15_NC PA15

#define STM32_PB8_NC PB8
#define STM32_PB9_NC PB9

#define STM32_PC3_NC PC3
#define STM32_PC13_NC PC13
#define STM32_PC14_NC PC14
#define STM32_PC15_NC PC15
