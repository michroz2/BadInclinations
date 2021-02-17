/*
   EEPROM Read
*/

#include <EEPROM.h>

#define OLD_CODE B01010101  //85


struct SaveData {
  byte code;
  int delta;
  byte state; //по битам: FFFF|SSS|R = Fade|Sensitivity|Reverse
};

byte value = 0;

SaveData writeData;
SaveData readData;

float deltaZero = 2.0;
byte curFade = 3;
bool reverse = true;
byte curSensitivity = 3;

void   codeSaveData() { //Кодирует записываемые данные во writeData
  float deltaLSB = (deltaZero * 16);
  Serial.print("deltaLSB: ");
  Serial.println(deltaLSB);
  writeData.code = OLD_CODE;
  writeData.delta = (int)deltaLSB;
  writeData.state = byte(reverse) | curSensitivity << 1 | curFade << 4;
}/////codeSaveData()

void   uncodeSaveData() { //ДЕ-кодирует записываемые данные из readData
  deltaZero = (float)readData.delta / 16;
  curFade = (writeData.state & B01110000) >> 4;
  reverse = (writeData.state & B00000001);
  curSensitivity = (writeData.state & B00001110) >> 1;;
}/////uncodeSaveData()

void printData() {
  Serial.println("----------");
  Serial.println(  deltaZero);
  Serial.println(  curFade);
  Serial.println(  reverse);
  Serial.println(  curSensitivity);
  Serial.println("----------");
}

void setup() {
  // initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {  }


  deltaZero = 85.0 / 16;
  curFade = 3;
  reverse = true;
  curSensitivity = 3;

  printData();

  codeSaveData();

  deltaZero = 2;
  curFade = 2;
  reverse = false;
  curSensitivity = 2;

  printData();

  //    readData = writeData;


  //Читаем - это нужно для определения физического порядка в ЕЕПРОМ
  Serial.print(F("RAW EEPROM memory, bytes: "));
  Serial.println(sizeof(SaveData) + 20);
  for (int i = 0; i < sizeof(SaveData) + 20 ; i++) {
    value = EEPROM.read(i);

    Serial.print(i);
    Serial.print(":\t");
    Serial.println(value, DEC);
  }


  int lastAddress = 0;
  //Находим положение последней записи:
  for (int i = 0; i < (EEPROM.length() - sizeof(SaveData)); i++) {
    value = EEPROM.read(i);
    if (value != OLD_CODE ) {
      lastAddress = i;
      break;
    }
  }
  Serial.print(F("Last Address:\t"));
  Serial.println(lastAddress);

//  //Записываем структуру в ЕЕПРОМ на новое место:
//  EEPROM.put(lastAddress, writeData);
//  delay(100);

  lastAddress = 0;
  //Находим положение последней записи:
  for (int i = 0; i < (EEPROM.length() - sizeof(SaveData)); i++) {
    value = EEPROM.read(i);
    if (value != OLD_CODE ) {
      lastAddress = i;
      break;
    }
  }
  Serial.print(F("Last Address:\t"));
  Serial.println(lastAddress);


  EEPROM.get(lastAddress - 1, readData);

  uncodeSaveData();
  printData();

}/////setup

void loop() {

}/////loop()

/***
  Advance to the next address, when at the end restart at the beginning.

  Larger AVR processors have larger EEPROM sizes, E.g:
  - Arduno Duemilanove: 512b EEPROM storage.
  - Arduino Uno:        1kb EEPROM storage.
  - Arduino Mega:       4kb EEPROM storage.

  Rather than hard-coding the length, you should use the pre-provided length function.
  This will make your code portable to all AVR processors.
***/
//    if (address == EEPROM.length()) {

/***
  As the EEPROM sizes are powers of two, wrapping (preventing overflow) of an
  EEPROM address is also doable by a bitwise and of the length - 1.

  ++address &= EEPROM.length() - 1;
***/
