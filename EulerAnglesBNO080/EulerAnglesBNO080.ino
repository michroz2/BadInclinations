/*
  Using the BNO080 IMU

  Example : Euler Angles
  By: Paul Clark
  Date: April 28th, 2020

  Based on: Example1-RotationVector
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the Euler angles: roll, pitch and yaw.
  The yaw (compass heading) is tilt-compensated, which is nice.
  https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

//= MILLIS TIMER MACRO =
// performs the {subsequent} code once and then again after each x ms
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flg = millis() - tmr >= (x);\
  if (flg) tmr = millis();\
  if (flg)
//=====
void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();

  //Are you using a ESP? Check this issue for more information: https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library/issues/16
  //  //=================================
  //  delay(100); //  Wait for BNO to boot
  //  // Start i2c and BNO080
  //  Wire.flush();   // Reset I2C
  //  IMU.begin(BNO080_DEFAULT_ADDRESS, Wire);
  //  Wire.begin(4, 5);
  //  Wire.setClockStretchLimit(4000);
  //  //=================================

  if (myIMU.begin() == false)
  {
    Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
    while (1) ;
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableGameRotationVector(10); //Send data update every 50ms
//  myIMU.enableStepCounter(10); //Send data update every 50ms
//  myIMU.enableStabilityClassifier(10); //Send data update every 50ms

  Serial.println(F("Game Rotation vector enabled"));
  Serial.println(F("Output in form roll, pitch, yaw"));
}

void loop()
{
  EVERY_MS(20) {
    Serial.print(millis());
    Serial.print("\t===\t");
    //Look for reports from the IMU
    if (myIMU.dataAvailable() == true)
    {
      float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
      float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
      float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees
//      long steps = (myIMU.getStepCount()); // Internal Steps counter?
//      byte classification = myIMU.getStabilityClassifier();

      Serial.print(F("roll:\t"));
      Serial.print(roll, 3);
      Serial.print(F("\tpitch:\t"));
      Serial.print(pitch, 3);
      //    Serial.print(F("\tyaw:\t"));
      //    Serial.print(yaw, 3);
//      Serial.print(F("\tsteps:\t"));
//      Serial.print(steps);
//      if (classification == 0) Serial.print(F("\tUnknown motion"));
//      else if (classification == 1) Serial.print(F("\tOn table"));
//      else if (classification == 2) Serial.print(F("\tStationary"));
//      else if (classification == 3) Serial.print(F("\tStable"));
//      else if (classification == 4) Serial.print(F("\tMotion"));
//      else if (classification == 5) Serial.print(F("\t[Reserved]"));

      Serial.println();
    }
  }
  //  Serial.println("===");
}
