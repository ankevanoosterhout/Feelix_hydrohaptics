#ifndef FeelixCore_h
#define FeelixCore_h

#include "FeelixCoreConfig.h"

#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include <drivers/drv8316/drv8316.h>

#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>

#include "Lib.h"
#include "LM75.h"
#include "HydroHaptics.h"



enum State {
    NO_COMMUNICATION  = 0, 
    SPI_MASTER        = 1,
    SPI_SLAVE         = 2,
    I2C_MASTER        = 3,
    I2C_SLAVE         = 4  
};





class FeelixM
{
  public:

    FeelixM(char _id);


    void init();
    void initSPI_EXT();
    void initI2C_EXT();
    void DRV_SPI_initialize();

    void readSPIBus(byte dataToSend);   

    int listDevices();
    void requestDataI2C(int I2C_ADDR);
    void transmitDataI2C(int I2C_ADDR, char* str);
    void receiveDataI2C();


    void send_data();
    void blinkStatusLED();

    void run(uint8_t loop_count);
    void move();
    void moveTo();
    void disable();

    void getDirection();

    void process_data(char* cmd);


    void update();
    void playHapticEffectAtAngle(FeelixEffectCore effect, float angle);
    void playVelocityEffect(FeelixEffectCore effect);

    void calibrateCurrentSenseValues();

    char* returnDataOnRequest();
    
    BLDCMotor* bldc;
    DRV8316Driver6PWM* driver;
    MagneticSensorSPI* sensor;
    InlineCurrentSense* current_sense;
    HydroHaptics* hydroHaptics;

    float angle;
    float angle_deg;
    Direction sensor_dir;
    Direction rotation_dir;
    float sensor_offset;
    float velocity;
    float f_velocity;
    float target;
    float voltage;
    float target_val;
    float vol_limit;
    PhaseCurrent_s currents;
    float range;
    float start_pos;
    bool constrain_range;
    Control_type control_type;

    uint16_t communication_speed;
    unsigned long current_time;
    unsigned long start_time;
    bool loop;
    bool return_to_pos;
    float start_pos_loop;
  
    char id;

    bool INITIALIZED;
    bool RUN;   
    bool MOVE;
    bool CURRENT_SENSE_ACTIVE;
    bool OVERHEAT_PROTECTION;
    uint16_t overheat_count;
    float current_threshold;
    
    float transmission_factor = 1.0;

    State state;
    int I2C_address;
    int8_t nrOfConnectedDevices;
    int I2C_connections[10];

    String I2CincomingData = "";
    String I2CincomingDataMaster = "";
    String dataOutStr = "";   

    char dataRequestType;
    
    LM75* temperatureSensor;
    void readTemperature(); 
    float temperature;
    
    void printDRV8316Status();

    void homeMotor();
    void height_Sweep_Voltage_PID();
    float updateHeightPID();

  private: 
    void broadcastRequest();
    unsigned long blink_time;
    long blink_interval;
    unsigned long tempRead_time;
    long tempRead_interval = 6000;
    
    float calculateDriverVoltage();
    float driverVoltage = 0.0;

    int8_t active_effect;
    void velocityLimitProtection();
    bool transferData(char identifier, char* user_command);

    void BLDC_Config(char* cmd);
    void BLDC_Data(char* cmd);
    void Effect_Data(char* cmd);
    void Data_Points(char* cmd);
    void Return_Data(char* cmd);

    void Cogging_Test();
    void testAlignmentAndCogging(int direction, int sample_count, int shaft_rotation);

};


#endif
