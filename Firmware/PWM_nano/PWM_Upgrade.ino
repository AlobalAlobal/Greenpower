#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Buttons
#define BTN_UP   2
#define BTN_DOWN 3

// PWM + control pins
#define PWM_PIN        9
#define PWM_SD_PIN     10
#define TURBO_PIN      11
#define TURBO_SD_PIN   12

#define STEP 15

uint8_t pwmValue = 0;
uint8_t lastShown = 255;

String inputBuffer = "";

void setup() {

  Serial.begin(115200);
  Serial.println("System start");

  // Buttons
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  // Outputs
  pinMode(PWM_PIN, OUTPUT);
  pinMode(PWM_SD_PIN, OUTPUT);
  pinMode(TURBO_PIN, OUTPUT);
  pinMode(TURBO_SD_PIN, OUTPUT);

  // SD pins LOW
  digitalWrite(PWM_SD_PIN, LOW);
  digitalWrite(TURBO_SD_PIN, LOW);

  // Initial PWM
  analogWrite(PWM_PIN, pwmValue);
  analogWrite(TURBO_PIN, 25); // ~10%

  Serial.println("PWM initialized to 0");
  Serial.println("Type 0-255 and press ENTER to set PWM");

  // OLED init
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED failed");
    while (1);
  }

  display.setRotation(2);
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  drawNumber();
}

void loop() {

  handleSerial();

  // UP button
  if (digitalRead(BTN_UP) == LOW) {
    delay(20);
    if (digitalRead(BTN_UP) == LOW) {

      if (pwmValue <= 255 - STEP)
        pwmValue += STEP;

      Serial.print("BTN UP -> PWM: ");
      Serial.println(pwmValue);

      while (digitalRead(BTN_UP) == LOW);
      delay(50);
    }
  }

  // DOWN button
  if (digitalRead(BTN_DOWN) == LOW) {
    delay(20);
    if (digitalRead(BTN_DOWN) == LOW) {

      if (pwmValue >= STEP)
        pwmValue -= STEP;

      Serial.print("BTN DOWN -> PWM: ");
      Serial.println(pwmValue);

      while (digitalRead(BTN_DOWN) == LOW);
      delay(50);
    }
  }

  // Apply PWM
  analogWrite(PWM_PIN, pwmValue);

  // Update display
  if (pwmValue != lastShown) {
    drawNumber();
    lastShown = pwmValue;
  }
}

void handleSerial() {

  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {

        int value = inputBuffer.toInt();

        if (value >= 0 && value <= 255) {
          pwmValue = value;

          Serial.print("Serial set PWM to: ");
          Serial.println(pwmValue);
        } else {
          Serial.println("Invalid value (0-255 only)");
        }

        inputBuffer = "";
      }
    } else {
      inputBuffer += c;
    }
  }
}

void drawNumber() {

  display.clearDisplay();

  display.setTextSize(6);
  display.setCursor(10, 10);
  display.print(pwmValue);

  display.display();
}