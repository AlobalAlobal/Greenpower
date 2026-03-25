#define PIN_BAT_TOTAL   A0 // Bat total, also provides power for PCB with built in regulator
#define PIN_BAT_1       A7 // Bat 1, Bat 2 is calculated by BAT_TOTAL - BAT_1
#define PIN_CURRENT     A2 // Not used, better solution found
#define PIN_THROTTLE    A3 // Not used, PWM is not controlled by potentiometer byt digitaly
#define PIN_TEMP_1      A5 // 1.5kOHM thermistor connected
#define PIN_TEMP_2      A4 // 1.5kOHM thermistor connected

#define PIN_BRAKE_IN    7  // Wired along with other datalogger cables (UBTN1, UBTN2 and GND) but not connected ATM to anything, if done it will transmit brake state, Green Cable
#define PIN_BUTTON_1    12 // Also sometimes labeled as user button 1, UBTN1 (Upper one in 3rd column on the steering wheel), Brown cable
#define PIN_BUTTON_2    8  // Also sometimes labeled as user button 2, UBTN2 (Lower one in 3rd column on the steering wheel), Orange cable

#define PIN_RPM_1       3 // IR Light barier sensor connected to it, counts both teeth --> gap and ap --> teeth as a trigger so generates twice per teeth
#define PIN_RPM_2       2 // Not connected on PCB, to enable solder a wire from leg 1 of R18 to Arduino pin D2
#define PIN_BT_EN       4 // Not connected on PCB, to enable solder a wire from BT connector EN pin to Arduino pin D4

// ================= CONFIG =================
const float ADC_REF_VOLTAGE = 5.0;

// Resistor values for BAT voltage dividers, they may be changed with new board, the original 17kOHM/82kOHM used poor quality components so was not precise
const float DIVIDER_R1 = 68500.0;
const float DIVIDER_R2 = 9700.0;
const float DIVIDER_R3 = 68500.0;
const float DIVIDER_R4 = 9700.0;

// Thermistor
const float NTC_NOMINAL     = 1500.0;
const float TEMP_NOMINAL    = 25.0;
  float r2 = NTC_TOP_RES * (v2 / (ADC_REF_VOLTAGE - v2));

  float t2 = log(r2 / NTC_NOMINAL) / B_COEFFICIENT;
  t2 += 1.0 / (TEMP_NOMINAL + 273.15);
const float B_COEFFICIENT   = 3950.0;
const float NTC_TOP_RES     = 10000.0;

// Wheel + RPM
const int   PULSES_PER_REV = 46;
const float WHEEL_RADIUS_M = 0.225;   // Radius in meters under nominal weight
const float MAX_SPEED_KMH  = 45.0; // Simple filtration of noise

// ================= STATE =================
float bat1Voltage = 0, bat2Voltage = 0, batTotal = 0;
float current = 0, throttle = 0;
float temp1C = 0, temp2C = 0;

byte btn1State = 0, btn2State = 0, brakeState = 0;

// RPM measurement
volatile unsigned int pulseCount = 0;
unsigned long lastSampleTime = 0;

// Output values
float RPM_wheel = 0;
float speed_km_h = 0;

// Averaging buffer
#define AVG_SAMPLES 5
float speedBuffer[AVG_SAMPLES] = {0};
int speedIndex = 0;

// ================= ISR =================
void rpmISR() {
  static unsigned long lastEdge = 0;
  unsigned long now = micros();

  // simple noise filter (ignore <1ms pulses)
  if (now - lastEdge < 1000) return;

  pulseCount++;
  lastEdge = now;
}

// ================= SETUP =================
void setup() {
  Serial.begin(9600);

  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_BRAKE_IN, INPUT_PULLUP);
  pinMode(PIN_RPM_1, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_RPM_1), rpmISR, RISING);

  pinMode(PIN_BT_EN, OUTPUT);
  digitalWrite(PIN_BT_EN, LOW);
}

// ================= INPUT =================
void readButtons() {
  btn1State = !digitalRead(PIN_BUTTON_1);
  btn2State = !digitalRead(PIN_BUTTON_2);
  brakeState = !digitalRead(PIN_BRAKE_IN);
}

void readBatteries() {
  int rawTotal = analogRead(PIN_BAT_TOTAL);
  float vTotal = (rawTotal * ADC_REF_VOLTAGE) / 1023.0;
  batTotal = vTotal / (DIVIDER_R2 / (DIVIDER_R1 + DIVIDER_R2));

  int rawBat1 = analogRead(PIN_BAT_1);
  float vBat1 = (rawBat1 * ADC_REF_VOLTAGE) / 1023.0;
  bat1Voltage = vBat1 / (DIVIDER_R4 / (DIVIDER_R3 + DIVIDER_R4));

  bat2Voltage = batTotal - bat1Voltage;

  int rawCurrent = analogRead(PIN_CURRENT);
  float voltage = (rawCurrent * ADC_REF_VOLTAGE) / 1023.0;
  current = (voltage - 2.5) * 10.0;

  int rawThrottle = analogRead(PIN_THROTTLE);
  throttle = (rawThrottle / 1023.0) * 100.0;
}

void readTemperatures() {
  int raw1 = analogRead(PIN_TEMP_1);
  float v1 = (raw1 * ADC_REF_VOLTAGE) / 1023.0;
  float r1 = NTC_TOP_RES * (v1 / (ADC_REF_VOLTAGE - v1));

  float t1 = log(r1 / NTC_NOMINAL) / B_COEFFICIENT;
  t1 += 1.0 / (TEMP_NOMINAL + 273.15);
  temp1C = (1.0 / t1) - 273.15;

  int raw2 = analogRead(PIN_TEMP_2);
  float v2 = (raw2 * ADC_REF_VOLTAGE) / 1023.0;
  float r2 = NTC_TOP_RES * (v2 / (ADC_REF_VOLTAGE - v2));

  float t2 = log(r2 / NTC_NOMINAL) / B_COEFFICIENT;
  t2 += 1.0 / (TEMP_NOMINAL + 273.15);
  temp2C = (1.0 / t2) - 273.15;
}

// ================= RPM + SPEED =================
void updateSpeed() {
  unsigned long now = millis();
  float dt = (now - lastSampleTime) / 1000.0;

  if (dt <= 0) return;

  noInterrupts();
  unsigned int pulses = pulseCount;
  pulseCount = 0;
  interrupts();

  lastSampleTime = now;

  float revs = (float)pulses / PULSES_PER_REV;
  float rps = revs / dt;
  float rpm = rps * 60.0;

  float circumference = 2.0 * 3.14159265359 * WHEEL_RADIUS_M;
  float speed = rps * circumference * 3.6;

  // clamp invalid speeds
  if (speed > MAX_SPEED_KMH) speed = 0;

  // averaging buffer
  speedBuffer[speedIndex] = speed;
  speedIndex = (speedIndex + 1) % AVG_SAMPLES;

  float sum = 0;
  for (int i = 0; i < AVG_SAMPLES; i++) sum += speedBuffer[i];

  speed_km_h = sum / AVG_SAMPLES;
  RPM_wheel = rpm;
}

// ================= OUTPUT =================
void sendToESP() {
  String data =
    String(btn1State) + "," +
    String(btn2State) + "," +
    String(brakeState) + "," +
    String(temp1C, 1) + "," +
    String(temp2C, 1) + "," +
    String(bat1Voltage, 2) + "," +
    String(bat2Voltage, 2) + "," +
    String(batTotal, 2) + "," +
    String(current, 1) + "," +
    String(throttle, 0) + "," +
    String(RPM_wheel, 0) + "," +
    String(speed_km_h, 1) + "," +
    String(millis());

  Serial.println(data);
}

// ================= LOOP =================
void loop() {
  readButtons();
  readBatteries();
  readTemperatures();

  if (millis() - lastSampleTime >= 100) {
    updateSpeed();
    sendToESP();
  }
}