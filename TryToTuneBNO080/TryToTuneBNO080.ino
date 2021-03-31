/*
  Using the BNO080 IMU
*/

#include <Wire.h>
#include <OneButton.h> //для кнопки
// PIN для кнопки:
#define PIN_CONTROL_BUTTON 5 //Button PIN для обнуления уровня (второй вывод NO кнопки ->GND)
//Define Control Button.
OneButton buttonControl(PIN_CONTROL_BUTTON, true);

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

uint32_t prevTime, curTime, DTime;

float accPitch;
float Pitch0; // Обнуление корректированного угла - выставление прибора в ноль
float avrY, DeltaPitch, CorPitch;

void setup()//===================SETUP==============================
{
  Serial.begin(19200);
  delay(100);
  
  initButtons();

  Wire.begin();

  if (myIMU.begin() == false)
  {
    Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
    while (1) ;
  }

  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableGameRotationVector(25); //Send data update every 50ms
  delay(1000);
  myIMU.enableAccelerometer(25); //Send data update every 50ms
  delay(1000);
  myIMU.calibrateAccelerometer();
  delay(1000);
  myIMU.calibrateGyro();
  delay(1000);
  myIMU.sendTareGameXYZCommand();
  delay(100);

}

void loop()//===================LOOP==============================
{
  buttonControl.tick();   // keep watching the push button

  curTime = millis();
  DTime = (curTime - prevTime);
  prevTime = curTime;


  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    float roll = (myIMU.getRoll()) * 180.0 / PI; // Convert roll to degrees
    float pitch = (myIMU.getPitch()) * 180.0 / PI; // Convert pitch to degrees
    //    float yaw = (myIMU.getYaw()) * 180.0 / PI; // Convert yaw / heading to degrees

    float accx = myIMU.getAccelX();
    float accy = myIMU.getAccelY();
    float accz = myIMU.getAccelZ();

    accPitch = atan2(accx, accz) * 57.29578;     //  /PI * 180;
    //    float acc = (sq(accx) + sq(accy) + sq(accz)) - sq(9.8);

    Serial.print(F("DTime="));
    Serial.print(DTime);
    Serial.print(":");
    //  Serial.print(DTime);
    Serial.print(",");

    //    Serial.print(F("acc="));
    //    Serial.print(acc, 2 );
    //    Serial.print(":");
    //    Serial.print(constrain(acc, -2, 2));
    //    Serial.print(",");

    Serial.print(F("accPitch="));
    Serial.print(accPitch, 2 );
    Serial.print(":");
    Serial.print(constrain(accPitch, -2, 2));
    Serial.print(",");

    Serial.print(F("pitch="));
    Serial.print(pitch, 3);
    Serial.print(":");
    Serial.print(constrain(-pitch, -2, 2), 3);
    //    Serial.print(",");

    //    Serial.print(F("roll="));
    //    Serial.print(roll, 3);
    //    Serial.print(":");
    //    Serial.print(constrain(-roll, -2, 2), 3);
    Serial.println();

  }
}////============================/LOOP==================================

void initButtons() {
  //  PROCln(F("initButtons()"));
  // link the CONTROL button functions.
  buttonControl.attachClick(clickControl);
  buttonControl.attachDoubleClick(doubleclickControl);
  //  buttonControl.attachLongPressStart(longPressStartControl);
  //  buttonControl.attachLongPressStop(longPressStopControl);
  //  buttonControl.attachDuringLongPress(longPressControl);
}////initButtons()

void clickControl() {
  //  PROCln(F("Control Button clicked"));
  //  DEBUGln(F("Fade Function"));
  //Pitch0 = CorPitch;
  delay(1000);
  myIMU.sendTareGameXYZCommand();
  delay(100);

}////clickControl()

void doubleclickControl() {
  delay(1000);
  myIMU.sendTareGameXYZCommand();
  delay(100);
  Pitch0 = 0;
}////doubleclickControl()
