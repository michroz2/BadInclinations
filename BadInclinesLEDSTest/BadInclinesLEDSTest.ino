/*
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
CRGB leds[NUM_MODS][NUM_LEDS] =
{
  {0xff0000, 0xc80000, 0xaa3200, 0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L6
  {0,        0xc80000, 0xaa3200, 0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L5
  {0,        0,        0xaa3200, 0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},
  {0,        0,        0,        0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},
  {0,        0,        0,        0,        0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},
  {0,        0,        0,        0,        0,        0x3c9600, 0x00ff00,        0, 0, 0, 0, 0, 0},  //L1
  {0,        0,        0,        0,        0,        0x3c9600, 0x00ff00, 0x3c9600, 0, 0, 0, 0, 0}   //!LEVEL!
};


int curMode;
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
  curMode = NUM_MODS / 2;  //в надежде, что это будет = 6, то есть «LEVEL» и с него мы начнём тест
  prevMode = curMode;

  initLEDS();

  FastLED.addLeds<WS2812B, PIN_LEDS, RGB>(leds[curMode], NUM_LEDS);
  FastLED.show();
}

void loop() {  //===========  LOOP =============
  processButton();
  EVERY_MS(200) {
    processLEDS();
  }

}              //=========== /LOOP =============

void initLEDS() { //Симетрично инициализируем значения массива ледов для отклонения вправо
  for (byte i = 0; i < NUM_MODS / 2; i++) {
    for (byte j = 0; j < NUM_LEDS; j++) {
      leds[NUM_MODS - i - 1][NUM_LEDS - j - 1] = leds[i][j];
    }
  }
#ifdef DEBUG_ENABLE
  for (byte i = 0; i < NUM_MODS; i++) {
    for (byte j = 0; j < NUM_LEDS; j++) {
      for (byte k = 0; k < 3; k++) {
        Serial.print(String(leds[i][j][k]) + ("/")); //значения единичных ледов
      }
      Serial.print((" ")); //значения массивов
    }
    Serial.println();   //перевод строки
  }
#endif

}

void processButton()
{
  curButtonState = digitalRead(PIN_BUTTON); //читаем значение кнопки: 1 = ненажата, 0 = нажата
  if (curButtonState < prevButtonState)   //то есть, только в случае нажатия
  {
    curMode = (curMode + 1) % NUM_MODS;  //по кругу увеличивает на 1: 0,1,2...11,12,0,1...
DEBUGln(F("Switch to mode: ")+(curMode - (NUM_MODS/2)));
  }
  prevButtonState = curButtonState; //запоминаем значение кнопки

}  //processButton()


void processLEDS()
{
  if (curMode != prevMode) //то есть, была нажата кнопка
  {
    FastLED.addLeds<WS2812B, PIN_LEDS, RGB>(leds[curMode], NUM_LEDS);
  }
  prevButtonState = curButtonState; //запоминаем значение кнопки

  FastLED.show();

}  //processLEDS()
