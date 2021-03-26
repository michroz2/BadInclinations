// Connect GND, S1 and SR pins together.
/*
  Задача: исправить сбивающийся НОЛЬ выходного фьюжн сигнала с помощью чистого акселерометра.
  1) Из чисто акселерометра находим чисто угол.
  2) Усредняем его методом running average по NUM_AVERAGE = 10 показаниям
  3) Определяем в покое ли прибор: последующие NUM_OK = 2 показания
  должны не отличаться от running average ,более чем на 0.01
  4) При условии покоя корректируем показания фьюжн-угла на нужную дельту.
  5) Пользуемся на выходе корректированными показаниями фьюжн угла
*/

#include <Wire.h>
float Yaw, Roll, Pitch, magx, magy, magz, accx, accy, accz, gyrox, gyroy, gyroz, q0, q1, q2, q3, Roll2, Pitch2, Yaw2, LIAx, LIAy, LIAz, GRVx, GRVy, GRVz, GRV, acc;
float accPitch;
float Pitch0; // Обнуление корректированного угла - выставление прибора в ноль
const int GY_955 = 0x29;

#include <OneButton.h> //для кнопки
// PIN для кнопки:
#define PIN_CONTROL_BUTTON 5 //Button PIN для обнуления уровня (второй вывод NO кнопки ->GND)
//Define Control Button.
OneButton buttonControl(PIN_CONTROL_BUTTON, true);

//==== MILLIS TIMER MACRO ====
// performs the {subsequent} code once and then again after each x ms
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flg = millis() - tmr >= (x);\
  if (flg) tmr = millis();\
  if (flg)
//===========================

uint32_t prevTime, curTime, DTime;

float avrY, DeltaPitch, CorPitch;
float avrY2; //test


#define NUM_AVERAGE 10
#define NUM_OK 2

float avrData[NUM_AVERAGE];
byte curNum;

float accDelta;
byte numOK;
bool accOK, noAcc;

float runAverage() {
  float delta = (accPitch - avrData[curNum]) / NUM_AVERAGE; //we exclude old value and replace with new
  avrData[curNum++] = accPitch;
  curNum %= NUM_AVERAGE; //set curNum for the next time
  return avrY + delta;
}////runAverage()

void setup()//===================SETUP=================================================
{
  Serial.begin(19200);  //Setting the baudrate
  delay(100);

  initButtons();

  Wire.begin();
  Wire.setClock(400000); // I2C clock rate ,You can delete it but it helps the speed of I2C (default rate is 100000 Hz)
  delay(100);

  //Normal Power mode:
  Wire.beginTransmission(GY_955);
  Wire.write(0x3E); // Power Mode
  Wire.write(0x00); // Normal:0X00 (or B00), Low Power: 0X01 (or B01) , Suspend Mode: 0X02 (orB10)
  Wire.endTransmission();
  delay(50);
  //CONFIG mode:
  Wire.beginTransmission(GY_955);
  Wire.write(0x3D); // Operation Mode
  Wire.write(0x00); //0=CONFIG Mode; NDOF:0X0C (or B1100) , IMU:0x08 (or B1000) , NDOF_FMC_OFF: 0x0B (or B1011)
  Wire.endTransmission();
  delay(50);
  //UNIT_SEL (units selection):
  Wire.beginTransmission(GY_955);
  Wire.write(0x3B); // UNIT_SEL
  Wire.write(0b00000000); //m/s^2; Deg; °C; like Windows
  Wire.endTransmission();
  delay(50);
  //IMU Mode:
  Wire.beginTransmission(GY_955);
  Wire.write(0x3D); // Operation Mode
  Wire.write(0x08); //NDOF:0X0C (or B1100) , IMU:0x08 (or B1000) , NDOF_FMC_OFF: 0x0B (or B1011)
  Wire.endTransmission();
  delay(50);

  // Serial.println();
}
void loop() //====================LOOP==================================================
{
  buttonControl.tick();   // keep watching the push button

  curTime = millis();
  DTime = (curTime - prevTime);
  prevTime = curTime;

  Wire.beginTransmission(GY_955);
  Wire.write(0x08);
  Wire.endTransmission(false);
  Wire.requestFrom(GY_955, 24, true);
  // Accelerometer
  accx = (int16_t)(Wire.read() | Wire.read() << 8 );// / 100.00; // m/s^2
  accy = (int16_t)(Wire.read() | Wire.read() << 8 );// / 100.00; // m/s^2
  accz = (int16_t)(Wire.read() | Wire.read() << 8 );// / 100.00; // m/s^2
  // Magnetometer
  magx = (int16_t)(Wire.read() | Wire.read() << 8 );// / 16.00; // mT
  magy = (int16_t)(Wire.read() | Wire.read() << 8 );// / 16.00; // mT
  magz = (int16_t)(Wire.read() | Wire.read() << 8 );// / 16.00; // mT
  // Gyroscope
  gyrox = (int16_t)(Wire.read() | Wire.read() << 8 );// / 16.00; // Dps
  gyroy = (int16_t)(Wire.read() | Wire.read() << 8 );// / 16.00; // Dps
  gyroz = (int16_t)(Wire.read() | Wire.read() << 8 );// / 16.00; // Dps
  // Euler Angles
  Yaw = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; //in Degrees unit
  Roll = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; //in Degrees unit
  Pitch = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; //in Degrees unit

  // Quaternions
  //  q0 = (int16_t)(Wire.read() | Wire.read() << 8 ) / (pow(2, 14)); //unit less
  //  q1 = (int16_t)(Wire.read() | Wire.read() << 8 ) / (pow(2, 14)); //unit less
  //  q2 = (int16_t)(Wire.read() | Wire.read() << 8 ) / (pow(2, 14)); //unit less
  //  q3 = (int16_t)(Wire.read() | Wire.read() << 8 ) / (pow(2, 14)); //unit less

  //Convert Quaternions to Euler Angles
  //MR> These calculated values are negative to direct chip values!
  //  Yaw2 = (atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (pow(q2, 2) + pow(q3, 2)))) * 180 / PI;
  //  Roll2 = (asin(2 * (q0 * q2 - q3 * q1))) * 180 / PI;
  //  Pitch2 = (atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (pow(q1, 2) + pow(q2, 2)))) * 180 / PI;
  //Linear (Dynamic) & Gravitational (static) Acceleration

  //  Wire.beginTransmission(0x29);
  //  Wire.write(0x28);
  //  Wire.endTransmission(false);
  //  Wire.requestFrom(0x29, 12, true);
  //  LIAx = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00; // m/s^2
  //  LIAy = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00; // m/s^2
  //  LIAz = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00; // m/s^2
  //  GRVx = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00; // m/s^2
  //  GRVy = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00; // m/s^2
  //  GRVz = (int16_t)(Wire.read() | Wire.read() << 8) / 100.00; // m/s^2

  //  GRV = sqrt(sq(GRVx) + sq(GRVy) + sq(GRVz));
  //  Dacc += ((sq(accx) + sq(accy) + sq(accz)) - 90 - Dacc) / 10;

  accPitch = atan2(accy, accz) * 57.29578;     //  /PI * 180;
  accDelta = (accPitch - avrY) / NUM_AVERAGE;
  avrY += accDelta; //Попытка раннинг эверадж простым путём
  //  avrY2 = runAverage(); //Более точное усреднение (не заработало)
  if (abs(accDelta) < 0.01) {
    accOK = (numOK++ >= NUM_OK);
  }
  else {
    accOK = false;
    numOK = 0;
  }
  if (accOK)     DeltaPitch = (avrY - Pitch);

  CorPitch = Pitch + DeltaPitch;

  // Print data
  //  Serial.print(readPowerMode());
  //  Serial.print(",");

  Serial.print(F("DTime="));
  Serial.print(DTime);
  Serial.print(":");
  //  Serial.print(DTime);
  Serial.print(",");

  //  Serial.print(F("accPitch="));
  //  Serial.print(accPitch, 2 );
  //  Serial.print(":");
  //  Serial.print(constrain(accPitch, -2, 2));
  //  Serial.print(",");

  Serial.print(F("avrY="));
  Serial.print(avrY, 2 );
  Serial.print(":");
  Serial.print(constrain(avrY, -2, 2));
  Serial.print(",");

  Serial.print(F("accOK="));
  Serial.print(accOK);
  Serial.print(":");
  Serial.print(accOK);
  Serial.print(",");

  Serial.print(F("DeltaPitch="));
  Serial.print(DeltaPitch, 2 );
  Serial.print(":");
  //  Serial.print(constrain(DeltaPitch, -2, 2));
  Serial.print(",");

  Serial.print(F("CorPitch="));
  Serial.print(CorPitch);
  Serial.print(":");
  Serial.print(constrain(CorPitch - Pitch0, -2, 2));
  Serial.print(",");

  Serial.print(F("Pitch="));
  Serial.print(Pitch);
  Serial.print("/");
  Serial.print(Pitch0);
  Serial.print(":");
  Serial.print(constrain(Pitch - Pitch0, -2, 2));

  Serial.println("");
  //  setPowerModeNormal();
  //Serial.println();
}////loop() //====================////LOOP===============================

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
  Pitch0 = CorPitch;

}////clickControl()

void doubleclickControl() {
  Pitch0 = 0;
}////doubleclickControl()

byte readPowerMode() {
  Wire.beginTransmission(GY_955);
  Wire.write(0x3E); // Power Mode
  Wire.endTransmission(false);
  Wire.requestFrom(GY_955, 1, true);
  return Wire.read();
}////readPowerMode()

void setPowerModeNormal() {//set Normal Power Mode at least every 4 sec, so that chip is not switched to low power
  EVERY_MS(40000) {
    Wire.beginTransmission(GY_955);
    Wire.write(0x3E); // Power Mode
    Wire.write(0x00); // Normal:0X00 (or B00), Low Power: 0X01 (or B01) , Suspend Mode: 0X02 (orB10)
    Wire.endTransmission();
    delay(100);
  }
}////setPowerModeNormal()
