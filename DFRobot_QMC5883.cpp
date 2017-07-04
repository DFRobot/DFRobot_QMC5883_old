/*!
 * @file DFRobot_QMC5883.cpp
 * @brief Compatible with QMC5883 and QMC5883
 * @n 3-Axis Digital Compass IC
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2017
 * @copyright	GNU Lesser General Public License
 *
 * @author [dexian.huang](952838602@qq.com)
 * @version  V1.0
 * @date  2017-7-3
 */

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>
#include "DFRobot_QMC5883.h"



bool DFRobot_QMC5883::begin()
{
  int retry;
  retry = 3;
  while(retry--){
    Wire.begin();
    Wire.beginTransmission(HMC5883L_ADDRESS);
    isHMC = (0 == Wire.endTransmission());
    if(isHMC){
      break;
    }
    delay(50);
  }
  Serial.print("isHMC= ");
  Serial.println(isHMC);

  if(isHMC){
    if ((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48)
    || (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34)
    || (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33)){
      return false;
    }

    setRange(HMC5883L_RANGE_1_3GA);
    setMeasurementMode(HMC5883L_CONTINOUS);
    setDataRate(HMC5883L_DATARATE_15HZ);
    setSamples(HMC5883L_SAMPLES_1);
    mgPerDigit = 0.92f;
    return true;
  }else{
    retry = 3;
    while(retry--){
      Wire.begin();
      Wire.beginTransmission(QMC5883_ADDRESS);
      isQMC = (0 == Wire.endTransmission());
      if(isHMC){
        break;
      }
      delay(50);
    }
    Serial.print("isQMC= ");
    Serial.println(isQMC);
    if(isQMC){
      if ((fastRegister8(QMC5883_REG_IDENT_B) != 0x00)
        || (fastRegister8(QMC5883_REG_IDENT_C) != 0x01)
        || (fastRegister8(QMC5883_REG_IDENT_D) != 0xFF)){
        return false;
      }
    
      setRange(QMC5883_RANGE_8GA);
      setMeasurementMode(QMC5883_CONTINOUS);
      setDataRate(QMC5883_DATARATE_50HZ);
      setSamples(QMC5883_SAMPLES_8);
      mgPerDigit = 4.25f;
      return true;
    }
  }
  return false;
}

Vector DFRobot_QMC5883::readRaw(void)
{
  if(isHMC){
    v.XAxis = readRegister16(HMC5883L_REG_OUT_X_M) - xOffset;
    v.YAxis = readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset;
    v.ZAxis = readRegister16(HMC5883L_REG_OUT_Z_M);
  }else if(isQMC){
    v.XAxis = readRegister16(QMC5883_REG_OUT_X_M) - xOffset;
    v.YAxis = readRegister16(QMC5883_REG_OUT_Y_M) - yOffset;
    v.ZAxis = readRegister16(QMC5883_REG_OUT_Z_M);
  }
  return v;
}

Vector DFRobot_QMC5883::readNormalize(void)
{
  if(isHMC){
    v.XAxis = ((float)readRegister16(HMC5883L_REG_OUT_X_M) - xOffset) * mgPerDigit;
    v.YAxis = ((float)readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset) * mgPerDigit;
    v.ZAxis = (float)readRegister16(HMC5883L_REG_OUT_Z_M) * mgPerDigit;
    return v;
  }else if(isQMC){
    v.XAxis = ((float)readRegister16(QMC5883_REG_OUT_X_M) - xOffset) * mgPerDigit;
    v.YAxis = ((float)readRegister16(QMC5883_REG_OUT_Y_M) - yOffset) * mgPerDigit;
    v.ZAxis = (float)readRegister16(QMC5883_REG_OUT_Z_M) * mgPerDigit;
  }
  return v;
}

void DFRobot_QMC5883::setOffset(int xo, int yo)
{
  xOffset = xo;
  yOffset = yo;
}

void DFRobot_QMC5883::setRange(QMC5883_range_t range)
{
  if(isHMC){
    switch(range){
    case HMC5883L_RANGE_0_88GA:
      mgPerDigit = 0.073f;
      break;

    case HMC5883L_RANGE_1_3GA:
      mgPerDigit = 0.92f;
      break;

    case HMC5883L_RANGE_1_9GA:
      mgPerDigit = 1.22f;
      break;

    case HMC5883L_RANGE_2_5GA:
      mgPerDigit = 1.52f;
      break;

    case HMC5883L_RANGE_4GA:
      mgPerDigit = 2.27f;
      break;

    case HMC5883L_RANGE_4_7GA:
      mgPerDigit = 2.56f;
      break;

    case HMC5883L_RANGE_5_6GA:
      mgPerDigit = 3.03f;
      break;

    case HMC5883L_RANGE_8_1GA:
      mgPerDigit = 4.35f;
      break;

    default:
      break;
    }

    writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
  }else if(isQMC){
    switch(range)
    {
    case QMC5883_RANGE_2GA:
      mgPerDigit = 1.22f;
      break;
    case QMC5883_RANGE_8GA:
      mgPerDigit = 4.35f;
      break;
    default:
      break;
    }

    writeRegister8(QMC5883_REG_CONFIG_2, range << 4);
  }
}

QMC5883_range_t DFRobot_QMC5883::getRange(void)
{
  if(isHMC){
    return (QMC5883_range_t)((readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
  }else if(isQMC){
    return (QMC5883_range_t)((readRegister8(QMC5883_REG_CONFIG_2) >> 3));
  }
  return QMC5883_RANGE_8GA;
}

void DFRobot_QMC5883::setMeasurementMode(QMC5883_mode_t mode)
{
  uint8_t value;
  if(isHMC){
    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    writeRegister8(HMC5883L_REG_MODE, value);
  }else if(isQMC){
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0xfc;
    value |= mode;

    writeRegister8(QMC5883_REG_CONFIG_1, value);
  }
}

QMC5883_mode_t DFRobot_QMC5883::getMeasurementMode(void)
{
  uint8_t value=0;
  if(isHMC){
    value = readRegister8(HMC5883L_REG_MODE);
  }else if(isQMC){
    value = readRegister8(QMC5883_REG_CONFIG_1); 
  }
  value &= 0b00000011;  
  return (QMC5883_mode_t)value;
}

void DFRobot_QMC5883::setDataRate(QMC5883_dataRate_t dataRate)
{
  uint8_t value;
  if(isHMC){
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    writeRegister8(HMC5883L_REG_CONFIG_A, value);
  }else if(isQMC){
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0xf3;
    value |= (dataRate << 2);

    writeRegister8(QMC5883_REG_CONFIG_1, value);
  }
}

QMC5883_dataRate_t DFRobot_QMC5883::getDataRate(void)
{
  uint8_t value=0;
  if(isHMC){
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;
  }else if(isQMC){
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0b00001100;
    value >>= 2;
  }
  return (QMC5883_dataRate_t)value;
}

void DFRobot_QMC5883::setSamples(QMC5883_samples_t samples)
{
  uint8_t value;
  if(isHMC){
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);
    writeRegister8(HMC5883L_REG_CONFIG_A, value);
  }else if(isQMC){
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0x3f;
    value |= (samples << 6);
    writeRegister8(QMC5883_REG_CONFIG_1, value);
  }
}

QMC5883_samples_t DFRobot_QMC5883::getSamples(void)
{
  uint8_t value=0;
  if(isHMC){
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;
  }else if(isQMC){
    value = readRegister8(QMC5883_REG_CONFIG_1);
    value &= 0x3f;
    value >>= 6;
  }
  return (QMC5883_samples_t)value;
}

// Write byte to register
void DFRobot_QMC5883::writeRegister8(uint8_t reg, uint8_t value)
{
  if(isHMC){
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
        Wire.write(value);
    #else
        Wire.send(reg);
        Wire.send(value);
    #endif
    Wire.endTransmission();
  }else if(isQMC){
    Wire.beginTransmission(QMC5883_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
        Wire.write(value);
    #else
        Wire.send(reg);
        Wire.send(value);
    #endif
    Wire.endTransmission();
  }
}
// Read byte to register
uint8_t DFRobot_QMC5883::fastRegister8(uint8_t reg)
{
  uint8_t value=0;
  if(isHMC){
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif
    Wire.endTransmission();
  }else if(isQMC){
    Wire.beginTransmission(QMC5883_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
    Wire.requestFrom(QMC5883_ADDRESS, 1);
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif
    Wire.endTransmission();
  }
  return value;
}

// Read byte from register
uint8_t DFRobot_QMC5883::readRegister8(uint8_t reg)
{
  uint8_t value=0;
  if(isHMC){
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();

    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, 1);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif
    Wire.endTransmission();
  }else if(isQMC){
    Wire.beginTransmission(QMC5883_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
    Wire.beginTransmission(QMC5883_ADDRESS);
    Wire.requestFrom(QMC5883_ADDRESS, 1);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif
    Wire.endTransmission();
  }
  return value;
}
// Read word from register
int16_t DFRobot_QMC5883::readRegister16(uint8_t reg)
{
  int16_t value=0;
  if(isHMC){
    Wire.beginTransmission(HMC5883L_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
    Wire.beginTransmission(HMC5883L_ADDRESS);
    Wire.requestFrom(HMC5883L_ADDRESS, 2);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif
    Wire.endTransmission();
    value = vha << 8 | vla;
  }else if(isQMC){
    Wire.beginTransmission(QMC5883_ADDRESS);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
    Wire.beginTransmission(QMC5883_ADDRESS);
    Wire.requestFrom(QMC5883_ADDRESS, 2);
    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vha = Wire.read();
        uint8_t vla = Wire.read();
    #else
        uint8_t vha = Wire.receive();
        uint8_t vla = Wire.receive();
    #endif
    Wire.endTransmission();
    value = vha << 8 | vla;
  }
  return value;
}

void DFRobot_QMC5883::calibrate(int* offX, int* offY)
{
  static int minX = 0;
  static int maxX = 0;
  static int minY = 0;
  static int maxY = 0;
  Vector mag = readRaw();

  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;  
  
  *offX = (maxX + minX)/2;
  *offY = (maxY + minY)/2;  
  // delay(10000);
}

int DFRobot_QMC5883::getICType(void)
{
  if(isHMC){
    return IC_HMC5883L;
  }else if(isQMC){
    return IC_QMC5883;
  }else{
    return IC_NONE;
  }
}