#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define BTN_UP   2
#define BTN_DOWN 3
#define PWM_PIN  9

#define STEP 15

uint8_t pwmValue = 0;
uint8_t lastShown = 255;   // force first redraw

void setup() {

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(PWM_PIN, OUTPUT);

  analogWrite(PWM_PIN, pwmValue);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while(1); // stop if OLED fails
  }

  display.setRotation(2);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
}

void loop() {

  if (digitalRead(BTN_UP) == LOW) {
    delay(20); // debounce
    if (digitalRead(BTN_UP) == LOW) {

      if (pwmValue <= 255 - STEP) pwmValue += STEP;

      while (digitalRead(BTN_UP) == LOW); // wait until release
      delay(50);
    }
  }

  if (digitalRead(BTN_DOWN) == LOW) {
    delay(20); // debounce
    if (digitalRead(BTN_DOWN) == LOW) {

      if (pwmValue >= STEP) pwmValue -= STEP;

      while (digitalRead(BTN_DOWN) == LOW); // wait until release
      delay(50);
    }
  }

  analogWrite(PWM_PIN, pwmValue);

  if (pwmValue != lastShown) {
    drawNumber();
    lastShown = pwmValue;
  }
}

void drawNumber() {

  display.clearDisplay();

  display.setTextSize(6);
  display.setCursor(10,10);
  display.print(pwmValue);

  display.display();
}