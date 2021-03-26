/*
  Using the BNO080 IMU
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
void setup()//===================SETUP==============================
{
  Serial.begin(9600);

  Wire.begin();

  if (myIMU.begin() == false)
  {
    Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
    while (1) ;
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableGameRotationVector(50); //Send data update every 50ms
  delay(1000);
  myIMU.sendTareGameXYZCommand();
  delay(100);

}

void loop()//===================LOOP==============================
{
//  EVERY_MS(10000) {
//    myIMU.sendTareGameXYZCommand();
//    delay(100);
//  }
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees

    Serial.print(F("pitch="));
    Serial.print(pitch, 3);
    Serial.print(":");
    Serial.print(constrain(-pitch, -2, 2), 3);
    Serial.print(",");

    Serial.print(F("roll="));
    Serial.print(roll, 3);
    Serial.print(":");
    Serial.print(constrain(-roll, -2, 2), 3);
    Serial.println();

  }
}
