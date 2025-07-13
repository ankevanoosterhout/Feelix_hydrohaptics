#include <LM75.h>


LM75::LM75(TwoWire* _Wire2_LM75) {
    address = LM75_ADDRESS;
    Wire2_LM75 = _Wire2_LM75;
}


float LM75::regdata2float(word regdata)
{
  return ((float)(int) regdata / 32) / 8;
}


float LM75::getTemperature(void) {
  return regdata2float(_register16(LM75_TEMP_REGISTER));
}


word LM75::_register16 (byte reg) {
  Wire2_LM75->beginTransmission(address);
  Wire2_LM75->write(reg);	
  Wire2_LM75->endTransmission();
  
  Wire2_LM75->requestFrom(address, 2);
  word regdata = (Wire2_LM75->read() << 8) | Wire2_LM75->read();
  return regdata;
}