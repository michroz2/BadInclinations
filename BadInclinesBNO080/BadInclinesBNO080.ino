/*
  Цель: инерционно-независимый уровень.
  Соединяем IMU BNO080 и светодиодную ленту WS2812B в одном приборе!
  Цель этой итерации - разделить код для разных IMU:
  Соединения:
    BNO080 (модуль GY-BNO08X):
      GND -> GND
      VCC -> 5v OR 3.3v Arduino (Надо смотреть что требует конкретная плата датчика для питания или есть ли на ней преобразователь 5в->3.3в)
      SCL -> A5 Arduino (возможно, настраивается библиотекой Wire)
      SDA -> A4 Arduino (возможно, настраивается библиотекой Wire)
    Arduino:
      Control button -> "NO" контакты: GND и D5 Arduino (задаётся в коде)
    Лента WS2812B - 13 светодиодов (задаётся в коде):
      +5v (красный) -> 5v Arduino
      GND (белый) -> GND Arduino
      DIN (зелёный) -> D7 Arduino (настраивается в коде)
  Функции кнопки:
  Короткое нажатие: Яркость (следующий шаг яркости индикатора)
  Двойное нажатие: Чувствительность (следующая настройка чувствительности)
  Длинное нажатие (короткое): Обнуление датчика;
  Длинное нажатие (длинное): Инверсия сторон индикатора
*/
//Поиск мест, где можно что-то менять - по ключевому слову: ПОДСТРОЙКА

#include <FastLED.h> //для ленты WS2812B
#include <OneButton.h> //для кнопки
#include <Wire.h> //для I2C
#include <EEPROM.h> //для сохранения настроек прибора

#define USE_X_AXIS 1  //true
#define USE_Y_AXIS 0  //false

//ПОДСТРОЙКА - выбор используемой оси:
bool usedAxis = USE_X_AXIS;    //true = X; false=Y;

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

//ПОДСТРОЙКА: Дебагирование (вывод текстов на терминал): раскомментить для использования 1 строчку:
//#define DEBUG_ENABLE  //ЗАКОММЕНТИРОВАТЬ, когда всё отработано, например, перед загрузкой на Pro-mini.
#ifdef DEBUG_ENABLE
#define DEBUG(x) Serial.print(x)
#define DEBUGln(x) Serial.println(x)
#else
#define DEBUG(x)
#define DEBUGln(x)
#endif

#ifdef DEBUG_ENABLE
#define TODO(x) Serial.print(F("TODO:\t")); Serial.println(F("x"))
#else
#define TODO(x)
#endif

//#define PROC_ENABLE  //ЗАКОММЕНТИРОВАТЬ, когда отработаны последовательности переходов процедур
#ifdef PROC_ENABLE
#define PROC(x) Serial.print(x)
#define PROCln(x) Serial.println(x)
#else
#define PROC(x)
#define PROCln(x)
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

//Это «рабочий» массив для ленты
CRGB leds [NUM_LEDS];

//ПОДСТРОЙКА: Здесь можно исправлять вручную паттерны ЛЕДов для отколонений влево.
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

//ПОДСТРОЙКА: здесь можно задавать яркости:
#define NUM_FADES 4   //Количество вариантов яркости ленты
uint8_t fades [NUM_FADES] = {255, 128, 64, 32}; //максимальное значение яркости (каждого цвета)

//Следующий паттерн («двойная радуга») загорится при длинном нажатии кнопки (обнуление).
//При отпускании вся лента загорится синим и пойдёт процесс обнуления.
CRGB modeLongPressStart [NUM_LEDS] =
{
  CRGB::Red, CRGB::Orange, CRGB::Yellow, CRGB::Green, CRGB::Blue, CRGB::Indigo, CRGB::Violet,
  CRGB::Indigo, CRGB::Blue, CRGB::Green, CRGB::Yellow, CRGB::Orange, CRGB::Red
};

//(Возможна ПОДСТРОЙКА :)
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

#define NUM_SENSITIVITIES 4                         //ПОДСТРОЙКА количества вариантов чувствительности
float modeRange[NUM_SENSITIVITIES][NUM_MODES - 1] = //ПОДСТРОЙКА границ диапазонов крена - в градусах - «0» не включать!
  //!Нужно задать только первую (левую) половину граничных значений углов для каждой чувствительности - (правые) будут вычислены симметрично!
{
  /*0    1     2     3     4     5     6    - соответствующие «моды» */
  {   4,   2.4,  1.2,  0.6,  0.3,  0.1,   }, //во вторую половину можно записать нули или вообще убрать
  {   5,   3.2,   2,    1,   0.4,  0.2,   }, //во вторую половину можно записать нули или вообще убрать
  {   6,   3.8,  2.6,  1.2,  0.5,  0.3,   }, //во вторую половину можно записать нули или вообще убрать
  {   8,    5,    3,   1.8,  0.8,  0.4,   }, //во вторую половину можно записать нули или вообще убрать
};

byte curSensitivity = 0;    //Текущее значение чувствительности, для начала 0


#define VERY_LONG_PRESS_MS  3000  //это ПОДСТРОЙКА: длительность очень длинного нажатия (мс), при котором
//вместо калибровки произойдёт инверсия сторон индикатора

boolean startCalibrationMode = false;
uint32_t verylongPressTimer = 0;

//EEPROM things
#define WRITE_EEPROM_DELAY_MS   30000   //ПОДСТРОЙКА: задержка между последним изменением параметров и 
//сохранением их в ЕЕПРОМ. (Если за это время произвести новое изменение параметров, то сохранение отложится ещё на такое же время. )
#define EEPROM_OLD_CODE 254  // - специальный код для распознавания нужного места для чтения/записи ЕЕПРОМ

struct EEPROMData { //Структура для чтения и записи данных EEPROM
  byte code;  // = 254
  byte state; // = по битам: 0|FFF|SSS|R = Fade|Sensitivity|Reverse (это никогда не равно 254, то есть не может быть спутано с кодом при чтении)
  int deltaLSD;  // = deltaZero * 16 - Это по спеку датчика BNO055 должно быть целое число в единицах LSD
  //Для экономии места EEPROM, угол поправки записывается в виде целого числа = угол * 16 (округлённо)
};

EEPROMData readEEPROMData;
EEPROMData writeEEPROMData;
boolean    writeEEPROM = false;
uint32_t    writeEEPROMtimer = 0;

//ПОДСТРОЙКА: Setting defaults for EEPROM values:
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
int static maxWrites = 10;  //After this number of writes in one session, we shift the EEPROM address by 1 to prevent wear (?)

//Моргание встроенным светодиодом - переключение каждые N циклов.
#define BLINK_EVERY_N 50
int blinkAlive;
bool  blinkLED;

#ifdef DEBUG_ENABLE
void scanI2C() {
  PROCln(F("Scanning I2C"));
  for (byte i = 8; i < 120; i++)              //I2C Scanner for debug purpose
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print (F("Found address: "));
      Serial.print (i, DEC);
      Serial.print (F(" (0x"));
      Serial.print (i, HEX);
      Serial.println (F(")"));
    }
  }
}
#endif

//****************************************************************************************************
void setup() { //===========  SETUP =============

  // initialize serial port to output the debug information (if necessary)
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
#endif

  DEBUGln(F("Bad Inclines Program ==== Setup ===="));
  DEBUGln(F("Init Wire library for I2C:"));
  Wire.begin();
  Wire.setClock(400000); // I2C clock rate ,You can delete it but it helps the speed of I2C (default rate is 100000 Hz)
  delay(100);

  pinMode(PIN_LEDS, OUTPUT);
  curMode = NUM_MODES / 2;  //в надежде, что это будет = 6, то есть «LEVEL» и с него мы начнём работу
  prevMode = -1;

  initButtons();

#ifdef DEBUG_ENABLE
  scanI2C();
#endif

  initIMU();  //Инициализация модуля IMU
  initMODS();
  initModeRanges();
  readEEPROM();
  initLEDs();
  playGreeting();

}

void loop() {  //===========  LOOP =============
  PROCln(F("tick()"));
  buttonControl.tick();   // keep watching the push button
  PROCln(F("/tick()"));
  getNextRoll();          //получить новое значение крена - код зависит от датчика
  curMode = getMode();    //узнаём в какой диапазон это попадает
  processLEDS();          //Обновляем (если надо) паттерн свечения светодиодов
  processEEPROM();          //Проверяем надо ли писать в ЕЕПРОМ - и пишем, если надо.
  processBlink(BLINK_EVERY_N);  //Blink on-board LED
}              //=========== /LOOP =============

void initButtons() {
  PROCln(F("initButtons()"));
  // link the CONTROL button functions.
  buttonControl.attachClick(clickControl);
  buttonControl.attachDoubleClick(doubleclickControl);
  buttonControl.attachLongPressStart(longPressStartControl);
  buttonControl.attachLongPressStop(longPressStopControl);
  buttonControl.attachDuringLongPress(longPressControl);
}////initButtons()

void clickControl() {
  PROCln(F("Control Button clicked"));
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
  PROCln(F("Control Button double-clicked"));
  curSensitivity = (curSensitivity + 1) % NUM_SENSITIVITIES;
  showSensitivity();
  prepareEEPROMWrite();
}////doubleclickControl()

void showSensitivity() {
  PROCln(F("showSensitivity()"));
  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = 0;
  }
  leds[curSensitivity] = modeLongPressStart[curSensitivity];
  leds[NUM_LEDS - curSensitivity - 1] = modeLongPressStart[curSensitivity];
  FastLED.show();
  delay(1000);
  prevMode = -1;
  DEBUG(F("Sensitivity: "));
  DEBUGln(curSensitivity);

}////showSensitivity()

void prepareEEPROMWrite() {
  PROCln(F("prepareEEPROMWrite()"));
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
  PROCln(F("Control Button long-press started"));
  startCalibrationMode = true;  //if release button soon, then start calibration

  verylongPressTimer = millis() + VERY_LONG_PRESS_MS;  //start timer - для определения длины нажатия

  for (byte i = 0; i < NUM_LEDS; i++) {
    leds[i] = modeLongPressStart[i];
  }
  FastLED.show();
  delay(500);
}////longPressStartControl()

void longPressControl() {
  PROCln(F("Long Press Control Button ..."));
  if (millis() > verylongPressTimer) {
    DEBUGln(F("VERY Long Press detected..."));
    verylongPressTimer = millis() + VERY_LONG_PRESS_MS;  //start new timer - для следующего reverse
    startCalibrationMode = false;  //too long pressed for calibration
    switchSides();
    prepareEEPROMWrite();
  }
}////longPressControl()


void longPressStopControl() {
  PROCln(F("Control Button long-press stopped"));
  if (startCalibrationMode) {     //only if NOT VERY long pressed
    DEBUGln(F("Starting Calibration!"));
    startCalibrationMode = false;  //just in case, seems not necessary, but...
    for (byte i = 0; i < NUM_LEDS; i++) {
      leds[i] = CRGB::Blue;  //set Blue lights to indicate calibration
    }
    FastLED.show();
    delay(500);
    prevMode = -1;
    prepareEEPROMWrite();
    setZERO();
  }
}////longPressStopControl()

void setZERO() { //calculate the average roll - i.e. "calibration"
  PROCln(F("setZERO()"));
  DEBUG(F("Old deltaZero: "));
  DEBUGln(deltaZero);

  //For this chip we first check if it is not moving:
  byte moving_status = 5;
  do {
    delay(200);
    if (myIMU.dataAvailable())
      moving_status = myIMU.getStabilityClassifier();
    if (moving_status == 0) DEBUGln(F("\tUnknown motion"));
    else if (moving_status == 1) DEBUGln(F("\tOn table"));
    else if (moving_status == 2) DEBUGln(F("\tStationary"));
    else if (moving_status == 3) DEBUGln(F("\tStable"));
    else if (moving_status == 4) DEBUGln(F("\tMotion"));
    else if (moving_status == 5) DEBUGln(F("\t[Reserved]"));
  } while (moving_status > 3); //будет висеть тут, пока датчик находится в движении

  //Then for this chip we use the internal Tare function (it was not in the original library):
  myIMU.sendTareGameXYZCommand();

  //...and now we just average the current inclination (should be zero)
  float delta0 = 0;
  for (int i = 0; i < 100; i++) {  //read and sum 100 values
    delta0 = delta0 + (usedAxis ? myIMU.getPitch() : myIMU.getRoll()); // (RAD)
    //(Due to GY-BNO08X board axes layout we use Pitch = y axis, actually...)
    delay(50);
  }
  delta0 = delta0 * 180.0 / PI / 100; //average delta0 in Degrees
  deltaZero = delta0;

  DEBUG(F("New deltaZero: "));
  DEBUGln(deltaZero);
}////setZERO()

void initIMU() {
  PROCln(F("initIMU()"));

  DEBUG(F("Contacting BNO080 on address:\t"));
  DEBUGln((int)BNO080_DEFAULT_ADDRESS);
  if (myIMU.begin() == false)
  {
    DEBUGln(F("BNO080 not detected at default I2C address. Check your connections. Freezing..."));
    while (1) ;
  }
  myIMU.enableGameRotationVector(25); //Send data update every 25ms
  delay(100);
  myIMU.enableStabilityClassifier(25); //This is used for setZERO function -
  //only zero when the chip is at rest!
  delay(100);

}////initIMU()


void initMODS() { //Симетрично инициализируем значения массивов ледов для отклонения вправо
  PROCln(F("initMODS()"));

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

void initModeRanges()  { //Симметрично заполняем правую половину массива границ диапазонов чувствительностей по заданной левой
  PROCln(F("initModeRanges()>>>>>"));
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
  PROCln(F("processLEDS()"));
  if (curMode != prevMode) //
  {
    copyMode();
    FastLED.show();
  }
  prevMode = curMode;
  PROCln(F("/processLEDS()"));
}  ////processLEDS()

void getNextRoll() {  //читает с датчика значение крена в переменную Roll (в град.)
  PROCln(F("getNextRoll()"));

  if (myIMU.dataAvailable()) {
    Roll = (usedAxis ? myIMU.getPitch() : myIMU.getRoll());
    // Return the roll (rotation around the x-axis) in Radians
    //На плате GY-BMO08X ось Х датчика расположена вдоль длинной стороны платы,
    //поэтому из чисто механических соображений мы берём не крен, а тонгаж (или наоборот).

    //  delay(50);
    DEBUG(F("Roll BNO080 (RAD)=\t"));
    DEBUG(Roll);
    Roll *= 57.29578;  // * 180.0 / PI - deltaZero; //in Degrees, corrected

#ifdef DEBUG_ENABLE
    byte moving_status = myIMU.getStabilityClassifier();
    DEBUG(F("\tIncline (deg)=\t"));
    DEBUG(Roll);
    DEBUG(F("\tStatus:"));
    if (moving_status == 0) DEBUGln(F("\tUnknown motion"));
    else if (moving_status == 1) DEBUGln(F("\tOn table"));
    else if (moving_status == 2) DEBUGln(F("\tStationary"));
    else if (moving_status == 3) DEBUGln(F("\tStable"));
    else if (moving_status == 4) DEBUGln(F("\tMotion"));
    else if (moving_status == 5) DEBUGln(F("\t[Reserved]"));
#endif
  }
  PROCln(F("/getNextRoll()"));
}////getNextRoll()

byte getMode() {
  PROCln(F("getMode: "));
  for (byte i = 0; i < (NUM_MODES - 1); i++) {
    if (Roll > modeRange[curSensitivity][i]) { //по порядку проверяем диапазоны крена...
      //      DEBUGln(i);
      return i;                     //... и возвращаем номер первого диапазона,
    }
  }                                 //в который вписывается текущее среднее значение.
  //  DEBUGln(NUM_MODES);
  return (NUM_MODES - 1);               //значит, очень много!
}////getMode()

void  initLEDs()  { //Инициализация индикаторной ленты
  PROCln(F("initLEDs()"));
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(fades[curFade]);
}////initLEDs()

void playGreeting() {
  PROCln(F("«««««playGreeting()»»»»»"));
  for (byte j = 0; j < 3; j++) {
    for (byte i = 0; i < NUM_LEDS; i++) {
      leds[i] = testColors[j];
      FastLED.show();
      delay(60);   //ПОДСТРОЙКА
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
}////printEEPROMData()
#endif

void processEEPROM() {    //Проверяем (в loop) надо ли писать в ЕЕПРОМ и пишем если надо
  PROCln(F("processEEPROM()"));
  if (writeEEPROM) { //Надо ли вообще писать? (изменЯлось ли что-то?)
    if (millis() > writeEEPROMtimer) {  //Выждано ли достаточное время?
      DEBUGln(F("TIME to write EEPROM!"));
      float delta0 = deltaZero * 16 + 0.1;
      int deltaLSD = delta0;
      writeEEPROM = false;
      writeEEPROMtimer = 0;
      //            DEBUGln(F("Will try to write to EEPROM!"));
      codeEEPROMData();
      writesEEPROM = writesEEPROM + 1;
      DEBUG(F("Number of writes in this session: "));
      DEBUGln(writesEEPROM);
      if ((writesEEPROM % maxWrites) == 0) {
        DEBUGln(F("Shifting to the NEXT address!"));
        lastEEPROMAddress = (lastEEPROMAddress + 1) % (EEPROM.length() - sizeof(EEPROMData) );
      }
      DEBUG(F("Writing to EEPROM at position:"));
      DEBUGln(lastEEPROMAddress);
#ifdef DEBUG_ENABLE
      printEEPROMData();
#endif
      EEPROM.put(lastEEPROMAddress, writeEEPROMData);   //Actual update of EEPROM
      readEEPROM(); //read the written values back for control
    }
  }
  PROCln(F("/processEEPROM()"));
}////processEEPROM()

void processBlink(int everyN) {
  blinkAlive = ++blinkAlive % everyN;
  if (blinkAlive == 0) {
    blinkLED = !blinkLED;
    digitalWrite(LED_BUILTIN, blinkLED);
  }
}////processBlink(int everyN)
