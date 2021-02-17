/*
  Соединяем IMU BNO055
  и светодиодную ленту WS2812B
  в одном приборе!
  Получаем инерционно-независимый уровень.
  Соединения:
    BNO055 (модуль GY_955):
      Connect GND, S1 and SR pins together. ->GND
      VCC -> 5v Arduino
      SCL -> A5 Arduino (возможно, настраивается библиотекой Wire)
      SDA -> A4 Arduino (возможно, настраивается библиотекой Wire)
    Arduino:
      Control button -> "NO" контакты: GND и D5 Arduino (настраивается в коде внизу)
    Лента WS2812B - 13 светодиодов (настраивается в коде):
      +5v - 5v Arduino
      GND -> GND Arduino
      DIN -> D7 Arduino (настраивается в коде)
*/
//Поиск настроек по ключевому слову: НАСТРОЙКА

//Для проверки без BNO055 РАСкомментировать следующую строчку:
//#define FAKE_BNO055_RANDOM

#include <FastLED.h>
#include <OneButton.h>
#include <Wire.h>
#include <EEPROM.h>


//НАСТРОЙКА: Дебагирование (вывод текстов на терминал): раскомментить для использования 1 строчку:
#define DEBUG_ENABLE  //ЗАКОММЕНТИРОВАТЬ, когда всё отработано!!!
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.print(x)
#define DEBUGln(x) Serial.println(x)
#else
#define DEBUG(x)
#define DEBUGln(x)
#endif

//= MILLIS TIMER MACRO =
// performs the {subsequent} code once and then again after each x ms
#define EVERY_MS(x) \
  static uint32_t tmr;\
  bool flg = millis() - tmr >= (x);\
  if (flg) tmr = millis();\
  if (flg)
//=====

//Настройки для ленты:
#define PIN_LEDS 7    //К какому дигитальному пину подключено управление ЛЕДами
#define NUM_LEDS 13   //количество ЛЕДов в ленте
#define NUM_MODES 13  //Количество вариантов свечения ленты

// PIN для кнопки:
#define PIN_CONTROL_BUTTON 5 //Button PIN для обнуления уровня (второй вывод NO кнопки ->GND)

// Настройки IMU BNO055:
#define GY_955    0x29  //Дефолтовый I2C адрес GY_955 
#define OPR_MODE  0x3D  //Регистр режима работы
#define PWR_MODE  0x3E  //Регистр режима питания
#define EUL_DATA_Y  0x1C  //Регистр угла крена (LSD)

//Это «рабочий» массив для ленты
CRGB leds [NUM_LEDS];

//НАСТРОЙКА: Здесь можно исправлять вручную паттерны ЛЕДов для отколонений влево.
// (паттерны для уклонов вправо задавать не надо - они получатся симметрично автоматически)
CRGB modes[NUM_MODES][NUM_LEDS] =
  //LED_FULL_BRIGHTNESS
{
  {0xff0000, 0x780000, 0x641000, 0x461400, 0x644600, 0x3c3c00, 0x000600,        0, 0, 0, 0, 0, 0},  //L6
  {0x0a0000, 0x780000, 0x641000, 0x461400, 0x644600, 0x3c3c00, 0x000a00,        0, 0, 0, 0, 0, 0},  //L5
  {0x040000, 0x080000, 0x641000, 0x461400, 0x644600, 0x3c3c00, 0x001400,        0, 0, 0, 0, 0, 0},  //L4
  {0,        0x040000, 0x080000, 0x461400, 0x644600, 0x3c3c00, 0x002800,        0, 0, 0, 0, 0, 0},  //L3
  {0,        0,        0x040000, 0x080000, 0x644600, 0x3c3c00, 0x003c00,        0, 0, 0, 0, 0, 0},  //L2
  {0,        0,        0,        0,        0x00000c, 0x000032, 0x00c800,        0, 0, 0, 0, 0, 0},  //L1
  {0,        0,        0,        0,        0,        0x00000c, 0x00ff00, 0x00000c, 0, 0, 0, 0, 0}   //!LEVEL!
};

//НАСТРОЙКА: здесь можно задавать яркости:
#define NUM_FADES 4   //Количество вариантов яркости ленты
uint8_t fades [NUM_FADES] = {255, 128, 64, 32}; //максимальное значение яркости (каждого цвета)

//Следующий паттерн («двойная радуга») загорится при длинном нажатии кнопки (обнуление).
//При отпускании, загорится «негативный» к этому паттерн и пойдёт обнуление.
CRGB modeLongPressStart [NUM_LEDS] =
{
  CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Indigo, CRGB::Violet,
  CRGB::Indigo, CRGB::Blue, CRGB::Green, CRGB::Yellow, CRGB::Orange, CRGB::Red
};

//Это паттерн  приветствия
//Последовательно пробежит, заполняясь слева направо от края до края,
//вот такое количество таких цветов:
#define NUM_TEST_COLORS 3
CRGB testColors [NUM_TEST_COLORS] =
{  CRGB::Red, CRGB::Green, CRGB::Blue };

//Define Control Button.
OneButton buttonControl(PIN_CONTROL_BUTTON, true);

byte curFade; //Текущее значение яркости
byte curMode;   //Новое значение режима (зависит от угла наклона)
byte prevMode = -1; //Предыдущее значение режима
float Roll;   //Крен, который и надо показать светодиодами
float deltaZero; //Поправка на неровность установки
boolean revers;   //Переключалка для сторон уровня

#define NUM_SENSITIVITIES 4                         //НАСТРОЙКА количества вариантов чувствительности
float modeRange[NUM_SENSITIVITIES][NUM_MODES - 1] = //НАСТРОЙКА границ диапазонов крена - в градусах - «0» не включать!
  //!Можно задать только первую половину значений каждой линейки - остальные будут вычислены симметрично!
{
  /*0   1   2   3     4     5     6    - соответствующие «моды» */
  {   5,  3,  2,  1.2,  0.8,  0.3,   }, //во вторую половину можно записать нули или вообще убрать
  {   5,  4,  3,  2,    1,    0.5,   }, //во вторую половину можно записать нули или вообще убрать
  {   10, 5,  3,  1,    0.5,  0.2,   }, //во вторую половину можно записать нули или вообще убрать
  {   10, 5,  2,  1,    0.5,  0.1,   }, //во вторую половину можно записать нули или вообще убрать
};

byte curSensitivity = 0;    //Текущее значение чувствительности


#define VERY_LONG_PRESS_MS  3000  //НАСТРОЙКА: длительность длинного нажатия, при котором
//вместо калибровки пойдёт инверсия сторон индикатора
boolean startCalibrationMode = false;
uint32_t verylongPressTimer = 0;


//EEPROM things
#define WRITE_EEPROM_DELAY_MS   15000   //НАСТРОЙКА: 15 sec - задержка между последним изменением параметров и 
//сохранением их в ЕЕПРОМ
#define EEPROM_OLD_CODE 254  // - это код для распознавания нужного места для чтения/записи ЕЕПРОМ

struct EEPROMData {
  byte code;  // = 254
  byte state; // = по битам: 0|FFF|SSS|R = Fade|Sensitivity|Reverse (никогда не равно 254)
  int deltaLSD;  // = deltaZero * 16 - Это по спеку датчика должно быть целое число в единицах LSD
};

EEPROMData readEEPROMData;
EEPROMData writeEEPROMData;
boolean    writeEEPROM = false;
uint32_t    writeEEPROMtimer = 0;

//НАСТРОЙКА: Setting defaults for EEPROM values:
byte defaultFade = 0;
byte defaultSensitivity = 0;
boolean defaultrevers = false;
int defaultdeltaLSD = 0;     //DeltaZero in degrees * 16 (in "LSD" units)

//Переменные для хранения последних прочитанных данных
int lastEEPROMAddress = 0;
byte readFade;
byte readSensitivity;
boolean readrevers;
int readdeltaLSD;
int deltaLSD;
int writesEEPROM = 0;   //Number of EEPROM writes in this session
int static maxWrites = 10;  //After this number of writes, we shift the address

//****************************************************************************************************
void setup() { //===========  SETUP =============

  // initialize serial port to output the debug information (if necessary)
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  // Init Wire library for I2C:
  Wire.begin();
  Wire.setClock(400000); // I2C clock rate ,You can delete it but it helps the speed of I2C (default rate is 100000 Hz)
  delay(100);

  pinMode(PIN_LEDS, OUTPUT);
  curMode = NUM_MODES / 2;  //в надежде, что это будет = 6, то есть «LEVEL» и с него мы начнём работу
  prevMode = -1;

  initButtons();
  initIMU();
  initMODS();
  initModeRanges();
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  readEEPROM();
  FastLED.setBrightness(fades[curFade]);
  playGreeting();
  //  copyMode();
  //  FastLED.show();

}

void loop() {  //===========  LOOP =============
  buttonControl.tick();   // keep watching the push button
  getNextRoll();          //получить новое значение крена
  curMode = getMode();    //узнаём в какой диапазон это попадает
  processLEDS();          //Обновляем (если надо) паттерн свечения светодиодов
  processEEPROM();          //Проверяем надо ли писать в ЕЕПРОМ - и пишем, если надо.
}              //=========== /LOOP =============

void initButtons() {
  // link the CONTROL button functions.
  buttonControl.attachClick(clickControl);
  buttonControl.attachDoubleClick(doubleclickControl);
  buttonControl.attachLongPressStart(longPressStartControl);
  buttonControl.attachLongPressStop(longPressStopControl);
  buttonControl.attachDuringLongPress(longPressControl);
}////initButtons()

void clickControl() {
  DEBUGln(F("Control Button clicked"));
  DEBUGln(F("Fade Function"));
  curFade = (curFade + 1) % NUM_FADES;
  // set master brightness control
  FastLED.setBrightness(fades[curFade]);
  prevMode = -1;    //Делаем так, чтобы индикатор обновился, даже если не изменился угол
  prepareEEPROMWrite();
  DEBUG(F("Current Brightness Number: "));
  DEBUG(curFade);
  DEBUG(F(",\tCurrent Brightness: "));
  DEBUGln(fades[curFade]);
}////clickControl()

void doubleclickControl() {
  DEBUGln(F("Control Button double-clicked"));
  curSensitivity = (curSensitivity + 1) % NUM_SENSITIVITIES;
  showSensitivity();
  prepareEEPROMWrite();
}////doubleclickControl()

void showSensitivity() {
  DEBUGln(F("showSensitivity()"));
  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = 0;
  }
  leds[curSensitivity] = modeLongPressStart[curSensitivity];
  leds[NUM_LEDS-curSensitivity-1] = modeLongPressStart[curSensitivity];
  FastLED.show();
  delay(1000);
  DEBUG(F("Sensitivity: "));
  DEBUGln(curSensitivity);

}////showSensitivity()

void prepareEEPROMWrite() {
  DEBUGln(F("prepareEEPROMWrite()"));
  writeEEPROM = true;
  writeEEPROMtimer = millis() + WRITE_EEPROM_DELAY_MS;
}////prepareEEPROMWrite()

void switchSides() {
  DEBUG(F("Switching the sides: "));
  revers = !revers;
  DEBUGln(revers);
  showSwitchSides();
  copyMode();
  prevMode = -1;
}////switchSides()

void showSwitchSides() {
  for (byte i = 0; i < NUM_LEDS / 2; i++) {
    leds[i] = CRGB::Blue;
    leds[NUM_LEDS - i - 1] = CRGB::Red;
    FastLED.show();
  }
  delay(1000);
  for (byte i = 0; i < NUM_LEDS / 2; i++) {
    leds[i] = CRGB::Red;
    leds[NUM_LEDS - i - 1] = CRGB::Blue;
  }
  FastLED.show();
  delay(1000);

}////showSwitchSides()


void longPressStartControl() {
  DEBUGln(F("Control Button long-press started"));
  startCalibrationMode = true;  //if release button soon, then start calibration

  verylongPressTimer = millis() + VERY_LONG_PRESS_MS;  //start timer - для определения длины нажатия

  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = modeLongPressStart[i];
  }
  FastLED.show();
  delay(500);
}////longPressStartControl()

void longPressControl() {
  //  DEBUGln(F("Long Press Control Button ..."));
  if (millis() > verylongPressTimer) {
    DEBUGln(F("VERY Long Press detected..."));
    verylongPressTimer = millis() + VERY_LONG_PRESS_MS;  //start new timer - для следующего reverse
    startCalibrationMode = false;  //too long pressed for calibration
    switchSides();
    prepareEEPROMWrite();
  }
}////longPressControl()


void longPressStopControl() {
  DEBUGln(F("Control Button long-press stopped"));
  if (startCalibrationMode) {     //only if NOT VERY long pressed
    DEBUGln(F("Starting Calibration!"));
    startCalibrationMode = false;  //just in case, seems not necessary, but...
    for (byte i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Blue;  //-leds[i];  //set negative lights
    }
    FastLED.show();
    delay(500);
    prepareEEPROMWrite();
    setupDelta();
  }
}////longPressStopControl()

void setupDelta() { //calculate the average roll - i.e. "calibration"
  DEBUGln(F("setupDelta()"));
  float delta0 = 0;
  for (int i = 0; i < 1000; i++) {  //read and sum 1000 values
    Wire.beginTransmission(GY_955);
    Wire.write(EUL_DATA_Y); //EUL_DATA_Y_LSD register
    Wire.endTransmission(false);
    Wire.requestFrom(GY_955, 2, true);    //для чтения ТОЛЬКО крена
#ifdef FAKE_BNO055_RANDOM
    delta0 = delta0 + (int16_t)random(-10, 20); //DEBUG MODE!
#else
    delta0 = delta0 + (int16_t)(Wire.read() | Wire.read() << 8 ); //LSD units (16*Degrees)
#endif
  }
  delta0 = delta0 / 1000 / 16; //average 0 delta in Degrees
  if (delta0 != deltaZero) { //if the calibration gives a new (different) delta value!
    deltaZero = delta0;
  }

  DEBUG(F("deltaZero: "));
  DEBUGln(deltaZero);

}////setupDelta()

void initIMU() {
  Wire.beginTransmission(GY_955);
  Wire.write(PWR_MODE); // Power Mode
  Wire.write(0x00); // Normal:0X00 (or B00), Low Power: 0X01 (or B01) , Suspend Mode: 0X02 (orB10)
  Wire.endTransmission();
  delay(100);

  Wire.beginTransmission(GY_955);
  Wire.write(OPR_MODE); // Operation Mode - Use IMU mode(нам не нужен магнетометр):
  Wire.write(0x08); //NDOF:0X0C (or B1100) , IMU:0x08 (or B1000) , NDOF_FMC_OFF: 0x0B (or B1011)
  Wire.endTransmission();
  delay(100);
}////initIMU()


void initMODS() { //Симетрично инициализируем значения массивов ледов для отклонения вправо

  for (byte i = 0; i < NUM_MODES / 2; i++) {
    for (byte j = 0; j < NUM_LEDS; j++) {
      modes[NUM_MODES - i - 1][NUM_LEDS - j - 1] = modes[i][j];
    }
  }
  //Контроль правильности присвоения всем модам и всем яркостям:
#ifdef DEBUG_ENABLE
  Serial.println(F("Modes:")); //заголовок
  for (byte i = 0; i < NUM_MODES; i++) {
    for (byte j = 0; j < NUM_LEDS; j++) {
      for (byte k = 0; k < 2; k++) {
        Serial.print(String(modes[i][j][k]) + ("/")); //значения R и G единичных ледов
      }
      Serial.print(String(modes[i][j][2])); //значениe Blue единичного леда
      Serial.print(("\t")); //разделитель отдельных значений одной моды
    }
    Serial.println();   //перевод строки - разделитель мод
  }
  Serial.println(F("------------------"));   //конец вывода мод
#endif

}////initMODS()

void initModeRanges()  { //Симметрично добавляем границы диапазонов чувствителностей в массив
  DEBUGln(F("<<<Mode Ranges>>>"));
  for (byte s = 0; s < NUM_SENSITIVITIES; s++) {
    DEBUG(F("Sensitivity: "));
    DEBUGln(s);
    for (byte i = 0; i < NUM_MODES - 1 ; i++) {
      modeRange[s][NUM_MODES - 2 - i] = 0 - modeRange[s][i];
      DEBUG(modeRange[s][i]);
      DEBUG(F(",\t"));
    }
    DEBUGln();
  }
}////initModeRanges()

void copyMode() {
  byte k;
  for (byte i = 0; i < NUM_MODES; i++) {
    k = revers ? (NUM_MODES - 1 - i) : i;
    leds[i] = modes[curMode][k] ;
  }

}////copyMode()

void processLEDS()  {
  if (curMode != prevMode) //
  {
    copyMode();
    FastLED.show();
  }
  prevMode = curMode;
}  ////processLEDS()

void getNextRoll() {  //читает значение крена и записывает в циклический массив
  Wire.beginTransmission(GY_955);
  Wire.write(EUL_DATA_Y); //EUL_DATA_Y_LSD register
  Wire.endTransmission(false);
  Wire.requestFrom(GY_955, 2, true);    //достаточно для чтения ТОЛЬКО крена

#ifdef FAKE_BNO055_RANDOM
  Roll = random(-120, 120); //DEBUG MODE!
#else
  Roll = (Wire.read() | Wire.read() << 8 );      //LSD units (16*Degrees)
#endif
  //  DEBUG(F("Current Roll= "));
  //  DEBUGln(Roll);
}////getNextRoll()

byte getMode() {
  Roll = Roll / 16 - deltaZero; //in Degrees, corrected
  //  DEBUG(F("Corrected Incline = "));
  //  DEBUGln(Roll);
  //  DEBUG(F("Mode: "));
  for (byte i = 0; i < (NUM_MODES - 1); i++) {
    if (Roll > modeRange[curSensitivity][i]) { //по порядку проверяем диапазоны крена...
      //      DEBUGln(i);
      return i;                     //... и возвращаем номер первого диапазона,
    }
  }                                 //в который вписывается текущее среднее значение.
  //  DEBUGln(NUM_MODES);
  return (NUM_MODES - 1);               //значит, очень много!
}////getMode()

void playGreeting() {
  DEBUGln(F("«««««playGreeting()»»»»»"));
  for (byte j = 0; j < 3; j++) {
    for (byte i = 0; i < NUM_LEDS; i++) {
      leds[i] = testColors[j];
      FastLED.show();
      delay(60);
    }
    delay(140);
  }
}////playGreeting();

void readEEPROM() {
  DEBUGln(F("Reading from EEPROM"));
  lastEEPROMAddress = 0;
  byte value;
  //Находим по коду положение предыдущей, последней записи в ЕЕПРОМ
  //(Это там, где обрываются записи 254,254,254...)
  for (int i = 0; i < (EEPROM.length() - sizeof(EEPROMData)); i++) {
    value = EEPROM.read(i);
    if (value != EEPROM_OLD_CODE ) {
      lastEEPROMAddress = i;
      break;
    }
  }
  //Если здесь будет lastEEPROMAddress == 0, то в ЕЕПРОМе не найдена запись предыдущих параметров
  if (lastEEPROMAddress == 0) {
    DEBUGln(F("Setting defaults!!!"));
    curFade = defaultFade;
    curSensitivity = defaultSensitivity;
    revers = defaultrevers;
    deltaZero = (float)defaultdeltaLSD / 16;
    readFade = defaultFade;
    readSensitivity = defaultSensitivity;
    readrevers = defaultrevers;
    readdeltaLSD = defaultdeltaLSD;

    return;   //exit from function
  };
  //Here the address is > 0!
  lastEEPROMAddress = lastEEPROMAddress - 1;
  DEBUG(F("EEPROM Data found at position: "));
  DEBUGln(lastEEPROMAddress);

  EEPROM.get(lastEEPROMAddress, readEEPROMData);
  uncodeEEPROMData();

  curFade = readFade;
  curSensitivity = readSensitivity;
  revers = readrevers;
  deltaLSD = readdeltaLSD;
  deltaZero = (float)deltaLSD / 16;

#ifdef DEBUG_ENABLE
  printEEPROMData();
#endif
}////readEEPROM()

void   codeEEPROMData() { //Кодирует записываемые данные во writeEEPROMData
  float deltaLSD = (deltaZero * 16);
  DEBUG("deltaLSD: ");
  DEBUGln(deltaLSD);
  writeEEPROMData.code = EEPROM_OLD_CODE;
  writeEEPROMData.state = byte(revers) | curSensitivity << 1 | curFade << 4;
  writeEEPROMData.deltaLSD = (int)deltaLSD;
}/////codeEEPROMData()

void uncodeEEPROMData() {   //ДЕ-кодирует записываемые данные из readEEPROMData
  readFade = (readEEPROMData.state & B01110000) >> 4;
  readrevers = (readEEPROMData.state & B00000001);
  readSensitivity = (readEEPROMData.state & B00001110) >> 1;
  readdeltaLSD = readEEPROMData.deltaLSD;
}////uncodeEEPROMData()

#ifdef DEBUG_ENABLE
void printEEPROMData() {
  Serial.println("----------");
  Serial.print(F("deltaZero:\t"));
  Serial.println(  deltaZero);
  Serial.print(F("Fade:\t"));
  Serial.println(  curFade);
  Serial.print(F("revers:\t"));
  Serial.println(  revers);
  Serial.print(F("Sensitivity:\t"));
  Serial.println(  curSensitivity);
  Serial.println("----------");
}
#endif

void processEEPROM() {    //Проверяем надо ли писать в ЕЕПРОМ и пишем если надо
  if (writeEEPROM) { //Надо ли вообще писать? (изменЯлось ли что-то?)
    if (millis() > writeEEPROMtimer) {  //Выждано ли достаточное время?
      DEBUGln(F("TIME to write EEPROM!"));
      float delta0 = deltaZero * 16 + 0.1;
      int deltaLSD = delta0;
      writeEEPROM = false;
      writeEEPROMtimer = 0;
      DEBUGln(F("Will try to write to EEPROM!"));
      codeEEPROMData();
      writesEEPROM = writesEEPROM + 1;
      DEBUG(F("Number of writes in this session: "));
      DEBUGln(writesEEPROM);
      if ((writesEEPROM % maxWrites) == 0) {
        DEBUGln(F("Shifting to the NEXT address!"));
        lastEEPROMAddress = (lastEEPROMAddress + 1) % (EEPROM.length() - sizeof(EEPROMData) );
        DEBUG(F("Writing to EEPROM at position:"));
        DEBUGln(lastEEPROMAddress);
#ifdef DEBUG_ENABLE
        printEEPROMData();
#endif
        EEPROM.put(lastEEPROMAddress, writeEEPROMData);   //Actual update of EEPROM
        readEEPROM(); //read the written values back for control
      }
    }
  }
}////processEEPROM()
