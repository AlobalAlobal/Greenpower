// ================= SOFTWARE I2C (SDA = D9, SCL = D1,
) =================
#include <Arduino.h>

#define SDA_PIN 9
#define SCL_PIN 10
#define PM_ADDR 0x45

// I2C timing (100 kHz)
#define I2C_DELAY_US 5

void i2c_init() {
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
}

void i2c_start() {
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, LOW);
  delayMicroseconds(I2C_DELAY_US);
}

void i2c_stop() {
  digitalWrite(SDA_PIN, LOW);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SDA_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
}

void i2c_write_bit(uint8_t bit) {
  digitalWrite(SDA_PIN, bit ? HIGH : LOW);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, LOW);
  delayMicroseconds(I2C_DELAY_US);
}

uint8_t i2c_read_bit() {
  pinMode(SDA_PIN, INPUT_PULLUP);
  delayMicroseconds(I2C_DELAY_US);
  digitalWrite(SCL_PIN, HIGH);
  delayMicroseconds(I2C_DELAY_US);
  uint8_t bit = digitalRead(SDA_PIN);
  digitalWrite(SCL_PIN, LOW);
  delayMicroseconds(I2C_DELAY_US);
  pinMode(SDA_PIN, OUTPUT);
  return bit;
}

uint8_t i2c_write_byte(uint8_t data) {
  for (uint8_t i = 0; i < 8; i++) {
    i2c_write_bit((data >> (7 - i)) & 0x01);
  }
  // read ACK
  uint8_t ack = i2c_read_bit();
  return ack; // 0 = ACK, 1 = NACK
}

uint8_t i2c_read_byte(uint8_t ack) {
  uint8_t byte = 0;
  for (uint8_t i = 0; i < 8; i++) {
    byte = (byte << 1) | i2c_read_bit();
  }
  // send ACK/NACK
  i2c_write_bit(ack ? 0 : 1); // 0 = ACK, 1 = NACK
  return byte;
}

// Read 24-bit unsigned from a register
uint32_t read24_sw(uint8_t reg) {
  i2c_start();
  // send address + write
  i2c_write_byte((PM_ADDR << 1) | 0);
  i2c_write_byte(reg);
  // repeated start
  i2c_start();
  i2c_write_byte((PM_ADDR << 1) | 1);
  // read 3 bytes
  uint32_t val = 0;
  val |= ((uint32_t)i2c_read_byte(1) << 16);
  val |= ((uint32_t)i2c_read_byte(1) << 8);
  val |= i2c_read_byte(0);
  i2c_stop();
  return val;
}

// Read 24-bit signed from a register
int32_t readSigned24_sw(uint8_t reg) {
  uint32_t val = read24_sw(reg);
  if (val & 0x800000) val |= 0xFF000000;
  return (int32_t)val;
}

// Read current from the I2C sensor (address 0x45, register 0x07)
float readI2CCurrent() {
  int32_t rawI = readSigned24_sw(0x07);
  float rawAmps = rawI * 0.00003665;
  float amps = rawAmps * 1.0178 - 0.0076;
  return amps;
}

// ================= ORIGINAL ECHOOK CODE (with current replaced) =================
#define PIN_BAT_TOTAL   A0 // Bat total, also provides power for PCB with built in regulator
#define PIN_BAT_1       A7 // Bat 1, Bat 2 is calculated by BAT_TOTAL - BAT_1
#define PIN_CURRENT     A2 // Not used, better solution found
#define PIN_THROTTLE    A3 // Not used, PWM is not controlled by potentiometer but digitaly
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
const float B_COEFFICIENT   = 3950.0;
const float NTC_TOP_RES     = 10000.0;

// Wheel + RPM
const int   PULSES_PER_REV = 46; // IR Light barier sensor connected to it, counts both teeth --> gap and ap --> teeth as a trigger so generates twice per teeth
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
  i2c_init();   // initialize software I2C pins

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

  // Replace analog current reading with I2C sensor
  current = readI2CCurrent();

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