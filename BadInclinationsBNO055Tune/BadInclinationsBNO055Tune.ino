//Version 1
// Connect GND, S1 and SR pins together.

// Дебагирование: раскомментить для использования 1 строчку:
#define DEBUG_ENABLE
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.print(String(millis())+" "+x)
#define DEBUGln(x) Serial.println(String(millis())+" "+x)
#else
#define DEBUG(x)
#define DEBUGln(x)
#endif

//==== MILLIS TIMER MACRO ====
// performs the {subsequent} code once and then again after each x ms
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flg = millis() - tmr >= (x);\
  if (flg) tmr = millis();\
  if (flg)
//===========================


#include <Wire.h>
float Yaw, Roll, Pitch, magx, magy, magz, accx, accy, accz, gyrox, gyroy, gyroz, q0, q1, q2, q3, GRVx, GRVy, GRVz;

#define GY_955    0x29
#define OPR_MODE  0x3D
#define PWR_MODE  0x3E
#define ACC_Config  0x00 //fake 0 until I know
#define GYR_Config  0x00 //fake 0 until I know
#define UNIT_SEL  0x00 //fake until I know. Value is 0



void setup() {

  // initialize serial port to output the debug information (if necessary)
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
  DEBUGln(F("===========  SETUP ============="));
#endif

  
  Wire.begin();
  Wire.setClock(400000); // I2C clock rate ,You can delete it but it helps the speed of I2C (default rate is 100000 Hz)
  delay(100);

  Wire.beginTransmission(GY_955);
  Wire.write(PWR_MODE); // Power Mode
  Wire.write(0x00); // Normal:0X00 (or B00), Low Power: 0X01 (or B01) , Suspend Mode: 0X02 (orB10)
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(GY_955);
  Wire.write(OPR_MODE); // Operation Mode
  Wire.write(0x08); //NDOF:0X0C (or B1100) , IMU:0x08 (or B1000) , NDOF_FMC_OFF: 0x0B (or B1011)
  Wire.endTransmission();
  delay(100);

//  Serial.begin(115200);  //Setting the baudrate
//  delay(100);
}
void loop()
{
  Wire.beginTransmission(GY_955);
//  Wire.write(0x08); //ACC_DATA_X register
  Wire.write(0x1a); //EUL_DATA_X register
  Wire.endTransmission(false);
//  Wire.requestFrom(GY_955, 32, true);  //Это стояло для чтоения ВСЕХ данных
  Wire.requestFrom(GY_955, 6, true);    //Этого достаточно для чтения ТОЛЬКО углов (на самом деле достаточно будет даже 1 угла)

//  // Accelerometer
//  accx = (int16_t)(Wire.read() | Wire.read() << 8 ) / 100.00; // m/s^2
//  accy = (int16_t)(Wire.read() | Wire.read() << 8 ) / 100.00; // m/s^2
//  accz = (int16_t)(Wire.read() | Wire.read() << 8 ) / 100.00; // m/s^2
//  // Magnetometer
//  magx = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; // mT
//  magy = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; // mT
//  magz = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; // mT
//  // Gyroscope
//  gyrox = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; // Dps
//  gyroy = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; // Dps
//  gyroz = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; // Dps

  // Euler Angles
  Yaw = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; //in Degrees unit
  Roll = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; //in Degrees unit
  Pitch = (int16_t)(Wire.read() | Wire.read() << 8 ) / 16.00; //in Degrees unit

//  // Quaternions
//  q0 = (int16_t)(Wire.read() | Wire.read() << 8 ) / (pow(2, 14)); //unit less
//  q1 = (int16_t)(Wire.read() | Wire.read() << 8 ) / (pow(2, 14)); //unit less
//  q2 = (int16_t)(Wire.read() | Wire.read() << 8 ) / (pow(2, 14)); //unit less
//  q3 = (int16_t)(Wire.read() | Wire.read() << 8 ) / (pow(2, 14)); //unit less

  // Print data
  DEBUG("Yaw=");
  DEBUG(Yaw);
  DEBUG("\tRoll=");
  DEBUG(Roll);
  DEBUG("\tPitch=");
  DEBUGln(Pitch);
  delay(100);
}
