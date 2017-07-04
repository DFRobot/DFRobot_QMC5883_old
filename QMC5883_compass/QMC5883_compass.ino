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

void setup()
{
  Serial.begin(9600);

  // Initialize Initialize QMC5883
  Serial.println("Initialize QMC5883");
  while (!compass.begin())
  {
    Serial.println("Could not find a valid QMC5883 sensor, check wiring!");
    delay(500);
  }

  // Set measurement range
  //compass.setRange(QMC5883_RANGE_1_3GA);           //QMC5883 API
  //compass.setRange(QMC5883_RANGE_2GA);			        //QMC5883L API

  // Set measurement mode
  //compass.setMeasurementMode(QMC5883_CONTINOUS);   //QMC5883 API
  //compass.setMeasurementMode(QMC5883_CONTINOUS_Q);	//QMC5883L API	

  // Set data rate
  //compass.setDataRate(QMC5883_DATARATE_30HZ);      //QMC5883 API	
  //compass.setDataRate(QMC5883_DATARATE_50HZ);		  //QMC5883L API	
 
  // Set number of samples averaged
  compass.setSamples(QMC5883_SAMPLES_8);

  // Set calibration offset. See QMC5883_calibration.ino
  compass.setOffset(0, 0);
}

void loop()
{
  Vector norm = compass.readNormalize();

  // Calculate heading
  float heading = atan2(norm.YAxis, norm.XAxis);

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
  heading += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  if (heading < 0)
  {
    heading += 2 * PI;
  }

  if (heading > 2 * PI)
  {
    heading -= 2 * PI;
  }

  // Convert to degrees
  float headingDegrees = heading * 180/M_PI; 

  // Output
  Serial.print(" Heading = ");
  Serial.print(heading);
  Serial.print(" Degress = ");
  Serial.print(headingDegrees);
  Serial.println();

  delay(100);
}

