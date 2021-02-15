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
      Zero button -> NO контакты: GND и D5 Arduino (настраивается в коде внизу)
      Fade button -> NO контакты: GND и D6 Arduino (настраивается в коде внизу)
    Лента WS2812B - 13 светодиодов (настраивается в коде):
      +5v - 5v Arduino
      GND -> GND Arduino
      DIN -> D7 Arduino (настраивается в коде)
*/

//Для проверки без BNO055 РАСкомментировать следующую строчку:
#define BNO055_RANDOM

#include <FastLED.h>
#include <OneButton.h>
#include <Wire.h>

// Дебагирование: раскомментить для использования 1 строчку:
#define DEBUG_ENABLE
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

// Настройки для кнопок:
#define PIN_ZERO_BUTTON 5 //Button PIN для обнуления уровня (второй вывод NO кнопки ->GND)
#define PIN_FADE_BUTTON 6 //Button PIN для регулировки яркости (второй вывод NO кнопки ->GND)


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
  {0xff0000, 0xc80000, 0xaa3200, 0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L6
  {0,        0xc80000, 0xaa3200, 0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L5
  {0,        0,        0xaa3200, 0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L4
  {0,        0,        0,        0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L3
  {0,        0,        0,        0,        0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L2
  {0,        0,        0,        0,        0,        0x3c9600, 0x00ff00,        0, 0, 0, 0, 0, 0},  //L1
  {0,        0,        0,        0,        0,        0x3c9600, 0x00ff00, 0x3c9600, 0, 0, 0, 0, 0}   //<LEVEL>
};

//здесь можно задавать яркости:
#define NUM_FADES 4   //Количество вариантов яркости ленты
uint8_t fades [NUM_FADES] = {255, 128, 64, 32}; //максимальное значение (каждого цвета)

// Setup a Button.
OneButton buttonZero(PIN_ZERO_BUTTON, true);
// Setup another Button.
OneButton buttonFade(PIN_FADE_BUTTON, true);

byte curFade; //Новое значение яркости
byte curMode;   //Новое значение режима
byte prevMode; //Предыдущее значение режима

#define NUM_ROLLS 10  //Число последовательных измерений крена, которые усредняются
int16_t Rolls[NUM_ROLLS]; //набор измерений крена (в целочисленном виде)
byte curRollPos; //Положение самого старого измерения крена в массиве (сюда надо писать новое значение)
float Roll;   //Усреднённый крен, который и надо показать светодиоами
float modeRange[NUM_MODES - 1] =  // границы диапазона крена - «0» не включать!
{ -5,  -3,  -2,  -1.2,  -0.8,  -0.3,  0.3, 0.8, 1.2, 2, 3, 5  };

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

  copyMode();

  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  // set master brightness control
  FastLED.setBrightness(fades[curFade]);
  FastLED.show();

}

void loop() {  //===========  LOOP =============
  // keep watching the push buttons:
  buttonZero.tick();
  buttonFade.tick();
  getNextRoll();
  curMode = getMode();
  EVERY_MS(100) {
    processLEDS();
  }

}              //=========== /LOOP =============

void initButtons() {
  // link the ZERO button functions.
  buttonZero.attachClick(clickZero);
  //  buttonZero.attachDoubleClick(doubleclickZero);
  //  buttonZero.attachLongPressStart(longPressStartZero);
  //  buttonZero.attachLongPressStop(longPressStopZero);
  //  buttonZero.attachDuringLongPress(longPressZero);

  // link the FADE button functions.
  buttonFade.attachClick(clickFade);
  buttonFade.attachDoubleClick(doubleclickFade);
  //  buttonFade.attachLongPressStart(longPressStartFade);
  //  buttonFade.attachLongPressStop(longPressStopFade);
  //  buttonFade.attachDuringLongPress(longPressFade);
} /////initButtons()

void clickZero() {
  DEBUGln(F("Zero Button clicked"));
} /////clickZero()

void clickFade() {
  DEBUGln(F("Fade Button clicked"));
  curFade = (curFade+1) % NUM_FADES;
  // set master brightness control
  FastLED.setBrightness(fades[curFade]);
  DEBUG(F("Current Brightness Number: "));
  DEBUG(curFade);
  DEBUG(F(",\tCurrent Brightness: "));
  DEBUGln(fades[curFade]);
} /////clickFade()

void doubleclickFade() {
  DEBUGln(F("Fade Button double-clicked"));
  curFade = (curFade-1) % NUM_FADES;
  // set master brightness control
  FastLED.setBrightness(fades[curFade]);
  DEBUG(F("Current Brightness Number: "));
  DEBUG(curFade);
  DEBUG(F(",\tCurrent Brightness: "));
  DEBUGln(fades[curFade]);
} /////doubleclickFade()

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
} /////initIMU()


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

} /////initMODS()

void copyMode() {
  for (byte i = 0; i < NUM_MODES; i++) {
    leds[i] = modes[curMode][i];
  }

} /////copyMode()

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

#ifdef BNO055_RANDOM
  Rolls[curRollPos] = (int16_t)random(-120, 120); //DEBUG MODE!
#else
  Rolls[curRollPos] = (int16_t)(Wire.read() | Wire.read() << 8 ); //LSD units (16*Degrees)
#endif

//  DEBUG(F("Current Roll= "));
//  DEBUGln(Rolls[curRollPos]);
  curRollPos = (curRollPos + 1) % NUM_ROLLS; //Циклическое изменение положения в массиве
} /////getNextRoll()

byte getMode() {
  float averageRoll = 0;
//  DEBUG(F("Rolls: ("));
  for (byte i = 0; i < NUM_ROLLS; i++) {
    averageRoll = averageRoll + Rolls[i];
//    DEBUG(Rolls[i]);
//    DEBUG(F(",\t"));
  }
//  DEBUGln(")");
  averageRoll = averageRoll / NUM_ROLLS / 16; //Now it is in Degrees!
  DEBUG(F("Averaged Incline = "));
  DEBUGln(averageRoll);
//  DEBUG(F("Mode: "));
  for (byte i = 0; i < NUM_MODES; i++) {
    if (averageRoll < modeRange[i]) { //по порядку проверяем диапазоны крена...
//      DEBUGln(i);
      return i;                     //... и возвращаем номер первого диапазона,
    }
  }                                 //в который вписывается текущее среднее значение.
//  DEBUGln(NUM_MODES);
  return NUM_MODES;                 //значит, очень много!
} /////getMode()
