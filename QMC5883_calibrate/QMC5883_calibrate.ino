/*!
 * @file QMC5883_calibrate.cpp
 * @brief calibrate your QMC5883
 * @n 3-Axis Digital Compass IC
 *
 * @copyright	[DFRobot](http://www.dfrobot.com), 2017
 * @copyright	GNU Lesser General Public License
 *
 * @author [dexian.huang](952838602@qq.com)
 * @version  V1.0
 * @date  2017-7-3
 */

#include <Wire.h>
#include <DFRobot_QMC5883.h>

DFRobot_QMC5883 compass;

int minX = 0;
int maxX = 0;
int minY = 0;
int maxY = 0;
int offX = 0;
int offY = 0;

void setup()
{
  Serial.begin(9600);

  // Initialize Initialize QMC5883
  while (!compass.begin())
  {
    delay(500);
  }

  // Set measurement range
  //compass.setRange(QMC5883_RANGE_1_3GA);           //HMC5883L API
  //compass.setRange(QMC5883_RANGE_2GA);			        //QMC5883 API

  // Set measurement mode
  //compass.setMeasurementMode(QMC5883_CONTINOUS);   //HMC5883L API
  //compass.setMeasurementMode(QMC5883_CONTINOUS_Q);	//QMC5883 API	

  // Set data rate
  //compass.setDataRate(QMC5883_DATARATE_30HZ);      //HMC5883L API	
  //compass.setDataRate(QMC5883_DATARATE_50HZ);		  //QMC5883 API	

  // Set number of samples averaged
  compass.setSamples(QMC5883_SAMPLES_8);
}

void loop()
{
  Vector mag = compass.readRaw();

  // Determine Min / Max values
  if (mag.XAxis < minX) minX = mag.XAxis;
  if (mag.XAxis > maxX) maxX = mag.XAxis;
  if (mag.YAxis < minY) minY = mag.YAxis;
  if (mag.YAxis > maxY) maxY = mag.YAxis;

  // Calculate offsets
  offX = (maxX + minX)/2;
  offY = (maxY + minY)/2;

  Serial.print(mag.XAxis);
  Serial.print(":");
  Serial.print(mag.YAxis);
  Serial.print(":");
  Serial.print(minX);
  Serial.print(":");
  Serial.print(maxX);
  Serial.print(":");
  Serial.print(minY);
  Serial.print(":");
  Serial.print(maxY);
  Serial.print(":");
  Serial.print(offX);
  Serial.print(":");
  Serial.print(offY);
  Serial.print("\n");
}
