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

//Для проверки без BNO055 РАСкомментировать следующую строчку:
//#define FAKE_BNO055_RANDOM

#include <FastLED.h>
#include <OneButton.h>
#include <Wire.h>

// Дебагирование (вывод текстов на терминал): раскомментить для использования 1 строчку:
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

// Настройки для ленты:
#define PIN_LEDS 7    //К какому пину подключено управление
#define NUM_LEDS 13   //количество ЛЕДов в ленте
#define NUM_MODES 13  //Количество вариантов свечения ленты

// PIN для кнопки:
#define PIN_CONTROL_BUTTON 5 //Button PIN для обнуления уровня (второй вывод NO кнопки ->GND)

// Настройки IMU BNO055:
#define GY_955    0x29  //Дефолтовый I2C адрес GY_955 
#define OPR_MODE  0x3D  //Регистр режима работы
#define PWR_MODE  0x3E  //Регистр режима питания
#define EUL_DATA_Y  0x1C  //Регистр угла крена (LSB)

//Это «рабочий» массив для ленты
CRGB leds [NUM_LEDS];

// Здесь задаются вручную паттерны ЛЕДов для отколонений влево.
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

//здесь можно задавать яркости:
#define NUM_FADES 4   //Количество вариантов яркости ленты
uint8_t fades [NUM_FADES] = {255, 128, 64, 32}; //максимальное значение яркости (каждого цвета)

//Следующий паттерн («двойная радуга») загорится при длинном нажатии кнопки (обнуление).
//При отпускании, загорится «негативный» к этому паттерн и пойдёт обнуление.
CRGB modeZero [NUM_LEDS] =
{
  CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Indigo, CRGB::Violet,
  CRGB::Indigo, CRGB::Blue, CRGB::Green, CRGB::Yellow, CRGB::Orange, CRGB::Red
};

//Это паттерн  приветствия (хотя возможно лучше зашитый в код?)
//Последовательно пробежит, заполняясь слева направо вот такое количество таких цветов:
#define NUM_TEST_COLORS 3
CRGB testColors [NUM_TEST_COLORS] =
{  CRGB::Red, CRGB::Green, CRGB::Blue };

//Define Control Button.
OneButton buttonControl(PIN_CONTROL_BUTTON, true);

byte curFade; //Текущее значение яркости
byte curMode;   //Новое значение режима (зависит от угла наклона)
byte prevMode; //Предыдущее значение режима

float Roll;   //Крен, который и надо показать светодиодами
float deltaZero = 0; //Поправка на неровность установки

float modeRange[NUM_MODES - 1] =  // границы диапазонов крена - в градусах - «0» не включать!
  //!Можно задать только первую половину значений - остальные будут вычислены симметрично!
{ -5,  -3,  -2,  -1.2,  -0.8,  -0.3,
  0.3, 0.8, 1.2, 2, 3, 5  //во вторую половину можно записать нули или вообще убрать эту строчку
};

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
  curFade = 0;    //полная яркость
  curMode = NUM_MODES / 2;  //в надежде, что это будет = 6, то есть «LEVEL» и с него мы начнём работу
  prevMode = curMode;

  initButtons();
  initIMU();
  initMODS();
  initModeRanges();
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  // set master brightness control
  FastLED.setBrightness(fades[curFade]);
  playGreeting();
  //  setupDelta();   //Это калибровка уровня при старте
  copyMode();
  FastLED.show();

}

void loop() {  //===========  LOOP =============
  // keep watching the push buttons:
  buttonControl.tick();
  getNextRoll();
  curMode = getMode();
  EVERY_MS(10) {
    processLEDS();
  }
}              //=========== /LOOP =============

void initButtons() {
  // link the CONTROL button functions.
  buttonControl.attachClick(clickControl);
  buttonControl.attachDoubleClick(doubleclickControl);
  buttonControl.attachLongPressStart(longPressStartControl);
  buttonControl.attachLongPressStop(longPressStopControl);
  //  buttonControl.attachDuringLongPress(longPressControl);
}/////initButtons()

void clickControl() {
  DEBUGln(F("Control Button clicked"));
  DEBUGln(F("Fade Function"));
  curFade = (curFade + 1) % NUM_FADES;
  // set master brightness control
  FastLED.setBrightness(fades[curFade]);
  prevMode = -1;
  DEBUG(F("Current Brightness Number: "));
  DEBUG(curFade);
  DEBUG(F(",\tCurrent Brightness: "));
  DEBUGln(fades[curFade]);
}/////clickControl()

void doubleclickControl() {
  DEBUGln(F("Control Button double-clicked"));
  DEBUGln(F("UnFade Function"));
  curFade = (curFade - 1) % NUM_FADES;
  // set master brightness control
  FastLED.setBrightness(fades[curFade]);
  prevMode = -1;
  DEBUG(F("Current Brightness Number: "));
  DEBUG(curFade);
  DEBUG(F(",\tCurrent Brightness: "));
  DEBUGln(fades[curFade]);
}/////doubleclickControl()

void longPressStartControl() {
  DEBUGln(F("Control Button long-press started"));
  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = modeZero[i];
  }
  FastLED.setBrightness(fades[0]);
  FastLED.show();
  deltaZero = 0;
  delay(100);
}/////longPressStartControl()

void longPressStopControl() {
  DEBUGln(F("Control Button long-press stopped"));
  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = -leds[i];  //set negative lights
  }
  FastLED.setBrightness(fades[0]);
  FastLED.show();
  delay(500);
  setupDelta();
  FastLED.setBrightness(fades[curFade]);

}/////longPressStopControl()

void setupDelta() { //calculate the average roll - i.e. "calibration"
  for (int i = 0; i < 1000; i++) {  //read and sum 1000 values
    Wire.beginTransmission(GY_955);
    Wire.write(EUL_DATA_Y); //EUL_DATA_Y_LSB register
    Wire.endTransmission(false);
    Wire.requestFrom(GY_955, 2, true);    //для чтения ТОЛЬКО крена
#ifdef FAKE_BNO055_RANDOM
    deltaZero = deltaZero + (int16_t)random(-10, 20); //DEBUG MODE!
#else
    deltaZero = deltaZero + (int16_t)(Wire.read() | Wire.read() << 8 ); //LSD units (16*Degrees)
#endif
  }
  deltaZero = deltaZero / 1000 / 16; //average 0 delta in Degrees

  DEBUG(F("deltaZero: "));
  DEBUGln(deltaZero);

}/////setupDelta()

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
}/////initIMU()


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

}/////initMODS()

void initModeRanges()  { //Симметрично добавляем границы диапазонов в массив
  DEBUGln(F("Mode Ranges:"));
  for (byte i = 0; i < NUM_MODES - 1 ; i++) {
    modeRange[NUM_MODES - 2 - i] = 0 - modeRange[i];
    DEBUG(modeRange[i]);
    DEBUG(F(",\t"));
  }
  DEBUGln();
}/////initModeRanges()

void copyMode() {
  for (byte i = 0; i < NUM_MODES; i++) {
    leds[i] = modes[curMode][i];
  }

}/////copyMode()

void processLEDS()  {
  if (curMode != prevMode) //
  {
    copyMode();
    FastLED.show();
  }
  prevMode = curMode;
}  /////processLEDS()

void getNextRoll() {  //читает значение крена и записывает в циклический массив
  Wire.beginTransmission(GY_955);
  Wire.write(EUL_DATA_Y); //EUL_DATA_Y_LSB register
  Wire.endTransmission(false);
  Wire.requestFrom(GY_955, 2, true);    //достаточно для чтения ТОЛЬКО крена

#ifdef FAKE_BNO055_RANDOM
  Roll = random(-120, 120); //DEBUG MODE!
#else
  Roll = (Wire.read() | Wire.read() << 8 );      //LSD units (16*Degrees)
#endif
  //  DEBUG(F("Current Roll= "));
  //  DEBUGln(Roll);
}/////getNextRoll()

byte getMode() {
  Roll = Roll / 16 - deltaZero; //in Degrees, corrected
  DEBUG(F("Corrected Incline = "));
  DEBUGln(Roll);
  //  DEBUG(F("Mode: "));
  for (byte i = 0; i < (NUM_MODES - 1); i++) {
    if (Roll < modeRange[i]) { //по порядку проверяем диапазоны крена...
      //      DEBUGln(i);
      return i;                     //... и возвращаем номер первого диапазона,
    }
  }                                 //в который вписывается текущее среднее значение.
  //  DEBUGln(NUM_MODES);
  return (NUM_MODES - 1);               //значит, очень много!
}/////getMode()

void   playGreeting() {
  DEBUGln(F("«««««playGreeting()»»»»»"));
  for (byte j = 0; j < 3; j++) {
    for (byte i = 0; i < NUM_LEDS; i++) {
      leds[i] = testColors[j];
      FastLED.show();
      delay(60);
    }
    delay(140);
  }
}/////playGreeting();
