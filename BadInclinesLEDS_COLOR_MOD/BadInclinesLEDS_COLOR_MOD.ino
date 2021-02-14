/*
  VERSION 4
  тестовая программа для вывода иммитируемых уровней наклона
  на светодионую ленту с управляемыми светодиодами
  сначала выводится «0», потом с каждым нажатием растущий «уклон» влево,
  потом снова «0» и растущий уклон вправо.
  Затем всё сначала.
*/

#include <FastLED.h>

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


// How many leds in your strip?
#define NUM_LEDS 13

// For led chips like WS2812, which have a data line, ground, and power, you just
// need to define PIN_LEDS.
#define PIN_LEDS 7
#define PIN_BUTTON 5 //Test Button PIN
#define NUM_MODS 13  //Количество вариантов свечения ленты

// Здесь задаются вручную паттерны для уклонов влево.
// Паттерны для уклонов вправо получатся симметрично
CRGB leds [NUM_LEDS];
CRGB mods[NUM_MODS][NUM_LEDS] =
{
  {0x800000, 0x3c0000, 0x320800, 0x230a00, 0x322300, 0x1e1e00, 0x000400,        0, 0, 0, 0, 0, 0},  //L6
  {0x060000, 0x3c0000, 0x320800, 0x230a00, 0x322300, 0x1e1e00, 0x000a00,        0, 0, 0, 0, 0, 0},  //L5
  {0x020000, 0x040000, 0x320800, 0x230a00, 0x322300, 0x1e1e00, 0x001400,        0, 0, 0, 0, 0, 0},
  {0,        0x020000, 0x040000, 0x230a00, 0x322300, 0x1e1e00, 0x001e00,        0, 0, 0, 0, 0, 0},
  {0,        0,        0x020000, 0x040000, 0x322300, 0x1e1e00, 0x002800,        0, 0, 0, 0, 0, 0},
  {0,        0,        0,        0,        0x000006, 0x000019, 0x005000,        0, 0, 0, 0, 0, 0},  //L1
  {0,        0,        0,        0,        0,        0x000006, 0x008000, 0x000006, 0, 0, 0, 0, 0}   //!LEVEL!
};


int curMod;
int prevMode;
int prevButtonState;
int curButtonState;

void setup() { //===========  SETUP =============

  // initialize serial port to output the debug information (if necessary)
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
  DEBUGln(F("===========  SETUP ============="));
#endif

  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LEDS, OUTPUT);
  curButtonState = digitalRead(PIN_BUTTON);
  prevButtonState = curButtonState;
  curMod = NUM_MODS / 2;  //в надежде, что это будет = 6, то есть «LEVEL» и с него мы начнём тест
  prevMode = curMod;

  initLEDS();
  copyMod(curMod);
  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.show();
}

void loop() {  //===========  LOOP =============
  EVERY_MS(100) {
    processButton();
    processLEDS();
  }

}              //=========== /LOOP =============

void initLEDS() { //Симетрично инициализируем значения массива ледов для отклонения вправо
  for (byte i = 0; i < NUM_MODS / 2; i++) {
    for (byte j = 0; j < NUM_LEDS; j++) {
      mods[NUM_MODS - i - 1][NUM_LEDS - j - 1] = mods[i][j];
    }
  }

#ifdef DEBUG_ENABLE
  for (byte i = 0; i < NUM_MODS; i++) {
    for (byte j = 0; j < NUM_LEDS; j++) {
      for (byte k = 0; k < 2; k++) {
        Serial.print(String(mods[i][j][k]) + ("/")); //значения R и G единичных ледов
      }
      Serial.print(String(mods[i][j][2])); //значениe Blue единичного леда
      Serial.print(("\t")); //значения массивов
    }
    Serial.println();   //перевод строки
  }
#endif

} /////initLEDS()

void processButton()
{
  curButtonState = digitalRead(PIN_BUTTON); //читаем значение кнопки: 1 = ненажата, 0 = нажата
  if (curButtonState < prevButtonState)   //то есть, только в случае нажатия
  {
    curMod = (curMod + 1) % NUM_MODS;  //по кругу увеличивает на 1: 0,1,2...11,12,0,1...
    copyMod(curMod);
    DEBUGln(F("Switch to mode: ") + (curMod - (NUM_MODS / 2)));
    DEBUGln(F("\tLED Values: ") + (curMod - (NUM_MODS / 2)));
#ifdef DEBUG_ENABLE
    Serial.print("\t");   //перевод строки
    for (byte j = 0; j < NUM_LEDS; j++) {
      for (byte k = 0; k < 2; k++) {
        Serial.print(String(mods[curMod][j][k]) + ("/")); //значения единичных ледов
      }
      Serial.print(String(mods[curMod][j][2])); //значения «B» единичных ледов
      Serial.print((" ")); //значения массивов
    }
    Serial.println();   //перевод строки
#endif
  }
  prevButtonState = curButtonState; //запоминаем значение кнопки

}  //////processButton()

void copyMod(byte mod) {
  for (byte i = 0; i < NUM_MODS; i++) {
    leds[i] = mods[mod][i];
  }

} /////copyMod(byte mod)


void processLEDS()  {
  if (curMod != prevMode) //то есть, была нажата кнопка
  {
    FastLED.show();
  }
  prevMode = curMod;

}  /////processLEDS()

//LED_FULL_BRIGHTNESS
//  {0xff0000, 0x780000, 0x641000, 0x461400, 0x644600, 0x3c3c00, 0x000600,        0, 0, 0, 0, 0, 0},  //L6
//  {0x0a0000, 0x780000, 0x641000, 0x461400, 0x644600, 0x3c3c00, 0x000a00,        0, 0, 0, 0, 0, 0},  //L5
//  {0x040000, 0x080000, 0x641000, 0x461400, 0x644600, 0x3c3c00, 0x001400,        0, 0, 0, 0, 0, 0},
//  {0,        0x040000, 0x080000, 0x461400, 0x644600, 0x3c3c00, 0x002800,        0, 0, 0, 0, 0, 0},
//  {0,        0,        0x040000, 0x080000, 0x644600, 0x3c3c00, 0x003c00,        0, 0, 0, 0, 0, 0},
//  {0,        0,        0,        0,        0x00000c, 0x000032, 0x00c800,        0, 0, 0, 0, 0, 0},  //L1
//  {0,        0,        0,        0,        0,        0x00000c, 0x00ff00, 0x00000c, 0, 0, 0, 0, 0}   //!LEVEL!
//
//LED_HALF_BRIGHTNESS
//  {0x800000, 0x3c0000, 0x320800, 0x230a00, 0x322300, 0x1e1e00, 0x000400,        0, 0, 0, 0, 0, 0},  //L6
//  {0x060000, 0x3c0000, 0x320800, 0x230a00, 0x322300, 0x1e1e00, 0x000a00,        0, 0, 0, 0, 0, 0},  //L5
//  {0x020000, 0x040000, 0x320800, 0x230a00, 0x322300, 0x1e1e00, 0x001400,        0, 0, 0, 0, 0, 0},
//  {0,        0x020000, 0x040000, 0x230a00, 0x322300, 0x1e1e00, 0x001e00,        0, 0, 0, 0, 0, 0},
//  {0,        0,        0x020000, 0x040000, 0x322300, 0x1e1e00, 0x002800,        0, 0, 0, 0, 0, 0},
//  {0,        0,        0,        0,        0x000006, 0x000019, 0x005000,        0, 0, 0, 0, 0, 0},  //L1
//  {0,        0,        0,        0,        0,        0x000006, 0x008000, 0x000006, 0, 0, 0, 0, 0}   //!LEVEL!
//
//LED_MINIMUM
//  {0x200000, 0x0f0000, 0x0c0200, 0x090300, 0x0c0900, 0x070700, 0x000100,        0, 0, 0, 0, 0, 0},  //L6
//  {0x010000, 0x0f0000, 0x0c0200, 0x090300, 0x0c0900, 0x070700, 0x000200,        0, 0, 0, 0, 0, 0},  //L5
//  {0x000000, 0x010000, 0x0c0200, 0x090300, 0x0c0900, 0x070700, 0x000500,        0, 0, 0, 0, 0, 0},
//  {0,        0x000000, 0x010000, 0x090300, 0x0c0900, 0x070700, 0x000800,        0, 0, 0, 0, 0, 0},
//  {0,        0,        0x000000, 0x010000, 0x0c0900, 0x070700, 0x000a00,        0, 0, 0, 0, 0, 0},
//  {0,        0,        0,        0,        0x000001, 0x000006, 0x001400,        0, 0, 0, 0, 0, 0},  //L1
//  {0,        0,        0,        0,        0,        0x000001, 0x001a00, 0x000001, 0, 0, 0, 0, 0}   //!LEVEL!
