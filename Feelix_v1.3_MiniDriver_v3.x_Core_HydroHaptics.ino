#include "FeelixCore.h"

/*
 * This is a library written to be used to play exported effects from Feelix.
 * The library is based on the SimpleFOC library (Copyright (c) 2020 Antun Skuric)
 */

/* initialize commander for usage with SimpleFOC library */
Commander command = Commander(Serial, '&', false);



/* initialize Feelix */
FeelixM feelix = FeelixM('A');

uint16_t loop_count = 0;
bool endOfCommand = false;

void process_data(char *cmd)
{
  // Serial.println((String) "&p " + cmd);
  feelix.process_data(cmd);
};

void setup()
{
  delay(1000);

  Serial.begin(115200);

  feelix.init();

  delay(1000);

  command.add('F', process_data);

  delay(100);

}

void loop()
{

  feelix.run(loop_count);

  if (loop_count++ > feelix.communication_speed) {

    feelix.blinkStatusLED();
    feelix.readTemperature();

    if (feelix.INITIALIZED)
    { 
      feelix.send_data();
    }

    loop_count = 0;
  }

  command.run();
}

