#include "config.h"

// Define GPIO Pins

#define LED_1_GATE      GPIO_NUM_35 // Gate for LED 1
#define LED_2_GATE      GPIO_NUM_36 // Gate for LED 2
#define LED_3_GATE      GPIO_NUM_37 // Gate for LED 3
#define LED_4_GATE      GPIO_NUM_38 // Gate for LED 4

#define DOWN_ARROW_LED  LED_1_GATE // DOWN ARROW is LED/HX711 1
#define RIGHT_ARROW     LED_2_GATE // RIGHT ARROW is LED/HX711 2
#define UP_ARROW        LED_3_GATE // UP ARROW is LED/HX711 3
#define LEFT_ARROW      LED_4_GATE // LEFT ARROW is LED/HX711 4

#if PADS_USE_LOAD_CELLS

#define HX711_SCK       GPIO_NUM_8 // Common clock pin for all HX711s
#define HX711_1_DT      GPIO_NUM_4 // Data pin for HX711 1
#define HX711_2_DT      GPIO_NUM_5 // Data pin for HX711 2
#define HX711_3_DT      GPIO_NUM_6 // Data pin for HX711 3
#define HX711_4_DT      GPIO_NUM_7 // Data pin for HX711 4
#define DOWN_ARROW_HX711  HX711_1_DT // DOWN ARROW is LED/HX711 1
#define RIGHT_ARROW_HX711 HX711_2_DT // RIGHT ARROW is LED/HX711 2
#define UP_ARROW_HX711    HX711_3_DT // UP ARROW is LED/HX711 3
#define LEFT_ARROW_HX711  HX711_4_DT // LEFT ARROW is LED/HX711 4

#endif

#if PADS_USE_TOUCHPADS

#define UP_ARROW_TOUCH  TOUCH_PAD_NUM4 // UP ARROW is TOUCH PAD 4
#define RIGHT_ARROW_TOUCH TOUCH_PAD_NUM5 // RIGHT ARROW is TOUCH PAD 6
#define DOWN_ARROW_TOUCH    TOUCH_PAD_NUM7 // DOWN ARROW is TOUCH PAD 7
#define LEFT_ARROW_TOUCH  TOUCH_PAD_NUM6 // LEFT ARROW is TOUCH PAD 5

#endif

