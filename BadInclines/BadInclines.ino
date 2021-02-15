/*
  Соединяем IMU BNO055
  и светодиодную ленту WS2812B
  в одном приборе!
  Получаем инерционно-независимый уровень.
*/

#include <FastLED.h>
#include "OneButton.h"

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

// need to define output PIN_LED.
#define PIN_LEDS 7
#define PIN_ZERO_BUTTON 5 //Button PIN для обнуления уровня
#define PIN_FADE_BUTTON 6 //Button PIN для регулировки яркости

#define NUM_MODES 13  //Количество вариантов свечения ленты
#define NUM_FADES 3   //Количество вариантов яркости ленты

//Это «рабочий» массив для ленты
CRGB leds [NUM_LEDS];

// А здесь задаются вручную паттерны для отколонений влево.
// (паттерны для уклонов вправо получатся симметрично автоматически)
CRGB mode [NUM_FADES][NUM_MODES][NUM_LEDS] =
{
  //LED_FULL_BRIGHTNESS
  {
    {0xff0000, 0xc80000, 0xaa3200, 0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L6
    {0,        0xc80000, 0xaa3200, 0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L5
    {0,        0,        0xaa3200, 0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L4
    {0,        0,        0,        0x8c5a00, 0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L3
    {0,        0,        0,        0,        0x647800, 0x3c9600,        0,        0, 0, 0, 0, 0, 0},  //L2
    {0,        0,        0,        0,        0,        0x3c9600, 0x00ff00,        0, 0, 0, 0, 0, 0},  //L1
    {0,        0,        0,        0,        0,        0x3c9600, 0x00ff00, 0x3c9600, 0, 0, 0, 0, 0}   //<LEVEL>
  },
  //LED_HALF_BRIGHTNESS
  {
    {0x800000, 0x3c0000, 0x320800, 0x230a00, 0x322300, 0x1e1e00, 0x000400,        0, 0, 0, 0, 0, 0},  //L6
    {0x060000, 0x3c0000, 0x320800, 0x230a00, 0x322300, 0x1e1e00, 0x000a00,        0, 0, 0, 0, 0, 0},  //L5
    {0x020000, 0x040000, 0x320800, 0x230a00, 0x322300, 0x1e1e00, 0x001400,        0, 0, 0, 0, 0, 0},
    {0,        0x020000, 0x040000, 0x230a00, 0x322300, 0x1e1e00, 0x001e00,        0, 0, 0, 0, 0, 0},
    {0,        0,        0x020000, 0x040000, 0x322300, 0x1e1e00, 0x002800,        0, 0, 0, 0, 0, 0},
    {0,        0,        0,        0,        0x000006, 0x000019, 0x005000,        0, 0, 0, 0, 0, 0},  //L1
    {0,        0,        0,        0,        0,        0x000006, 0x008000, 0x000006, 0, 0, 0, 0, 0}   //<LEVEL>
  },
  //LED_MINIMUM
  {
    {0x200000, 0x0f0000, 0x0c0200, 0x090300, 0x0c0900, 0x070700, 0x000100,        0, 0, 0, 0, 0, 0},  //L6
    {0x010000, 0x0f0000, 0x0c0200, 0x090300, 0x0c0900, 0x070700, 0x000200,        0, 0, 0, 0, 0, 0},  //L5
    {0x000000, 0x010000, 0x0c0200, 0x090300, 0x0c0900, 0x070700, 0x000500,        0, 0, 0, 0, 0, 0},
    {0,        0x000000, 0x010000, 0x090300, 0x0c0900, 0x070700, 0x000800,        0, 0, 0, 0, 0, 0},
    {0,        0,        0x000000, 0x010000, 0x0c0900, 0x070700, 0x000a00,        0, 0, 0, 0, 0, 0},
    {0,        0,        0,        0,        0x000001, 0x000006, 0x001400,        0, 0, 0, 0, 0, 0},  //L1
    {0,        0,        0,        0,        0,        0x000001, 0x001a00, 0x000001, 0, 0, 0, 0, 0}   //<LEVEL>
  },
};

// Setup a new OneButton on pin A1.
OneButton buttonZero(PIN_ZERO_BUTTON, true);
// Setup a new OneButton on pin A2.
OneButton buttonFade(PIN_FADE_BUTTON, true);

int curFade;
int prevFade;
int curMod;
int prevMode;
int prevZeroButtonState;
int curZeroButtonState;
int prevFadeButtonState;
int curFadeButtonState;

void setup() { //===========  SETUP =============

  // initialize serial port to output the debug information (if necessary)
#ifdef DEBUG_ENABLE
  Serial.begin(9600);
  while (!Serial);
  DEBUGln(F("===========  SETUP ============="));
#endif

  pinMode(PIN_LEDS, OUTPUT);
  curFade = 0;    //полная яркость
  curMod = NUM_MODES / 2;  //в надежде, что это будет = 6, то есть «LEVEL» и с него мы начнём работу
  prevMode = curMod;
  prevFade = curFade;

  initButtons();

  initLEDS();

  copyFadeMode();

  FastLED.addLeds<WS2812B, PIN_LEDS, GRB>(leds, NUM_LEDS);
  FastLED.show();
}

void loop() {  //===========  LOOP =============
  // keep watching the push buttons:
  buttonZero.tick();
  buttonFade.tick();

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
  //  buttonFade.attachDoubleClick(doubleclickFade);
  //  buttonFade.attachLongPressStart(longPressStartFade);
  //  buttonFade.attachLongPressStop(longPressStopFade);
  //  buttonFade.attachDuringLongPress(longPressFade);
} /////initButtons()

void clickZero() {
  DEBUGln(F("Zero Button clicked"));
} /////clickZero()

void clickFade() {
  DEBUGln(F("Fade Button clicked"));
} /////clickFade()


void initLEDS() { //Симетрично инициализируем значения массивов ледов для отклонения вправо

  for (byte f = 0; f < NUM_FADES; f++) {
    for (byte i = 0; i < NUM_MODES / 2; i++) {
      for (byte j = 0; j < NUM_LEDS; j++) {
        mode[f][NUM_MODES - i - 1][NUM_LEDS - j - 1] = mode[f][i][j];
      }
    }
  }

  //Контроль правильности присвоения всем модам и всем яркостям:
#ifdef DEBUG_ENABLE
  for (byte f = 0; f < NUM_FADES; f++) {
    Serial.println("BRIGHTNESS: " + f); //начало блока одной яркости
    for (byte i = 0; i < NUM_MODES; i++) {
      for (byte j = 0; j < NUM_LEDS; j++) {
        for (byte k = 0; k < 2; k++) {
          Serial.print(String(mode[f][i][j][k]) + ("/")); //значения R и G единичных ледов
        }
        Serial.print(String(mode[f][i][j][2])); //значениe Blue единичного леда
        Serial.print(("\t")); //разделитель значений массивов одной моды
      }
      Serial.println();   //перевод строки - разделитель мод
    }
  }
  Serial.println(F("------------------"));   //конец вывода блоков яркости
#endif

} /////initLEDS()

void copyFadeMode() {
  for (byte i = 0; i < NUM_MODES; i++) {
    leds[i] = mode[curFade][curMod][i];
  }

} /////copyFadeMode()

void processLEDS()  {
  if (curMod != prevMode) //то есть, была нажата кнопка
  {
    copyFadeMode();
    FastLED.show();
  }
  prevMode = curMod;
  prevFade = curFade;

}  /////processLEDS()
