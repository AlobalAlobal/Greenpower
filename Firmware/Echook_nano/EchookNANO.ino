#define PIN_BAT_TOTAL   A0
#define PIN_BAT_1       A7
#define PIN_CURRENT     A2
#define PIN_THROTTLE    A3
#define PIN_TEMP_1      A5
#define PIN_TEMP_2      A4

#define PIN_BRAKE_IN    7
#define PIN_BUTTON_1    12
#define PIN_BUTTON_2    8

#define PIN_RPM_1       3
#define PIN_RPM_2       2 //Not connected on PC, to enable solder wire from leg 1 of R18 to Arduino pin D2
#define PIN_BT_EN       4 //NC

//Battery buzmac
const float ADC_REF_VOLTAGE = 5.0;
const float DIVIDER_R1 = 72000.0;
const float DIVIDER_R2 = 10000.0;
const float DIVIDER_R3 = 72000.0;
const float DIVIDER_R4 = 10000.0;

float bat1Voltage   = 0.0;
float bat2Voltage   = 0.0;
float batTotal      = 0.0;
float current       = 0.0;
float throttle      = 0.0;

// Termistor buzmac
const float NTC_NOMINAL     = 1500.0;
const float TEMP_NOMINAL    = 25.0;
const float B_COEFFICIENT   = 3950.0;
const float NTC_TOP_RES     = 10000.0;

float temp1C = 0.0;
float temp2C = 0.0;

//Buttons
byte btn1State = 0;
byte btn2State = 0;
byte brakeState = 0;

//Delay and pulse calculation
unsigned long lastTime = 0;
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulsePeriod = 0;
//amount of teeth on the gear on the rpm measuring device(opticka zavora)       
const int Gear_optical = 23;
float RPM_wheel = 0.0;
// Wheel + speed calculation
const float wheelRadius_cm = 22.5;   // wheel radius in cm
const float PI_VAL = 3.14159265359;

float speed_cm_s = 0.0;
float speed_m_s  = 0.0;
float speed_km_h = 0.0;

void rpmISR() {
  unsigned long now = micros();

  if (lastPulseTime != 0) {
    pulsePeriod = now - lastPulseTime;
  }

  lastPulseTime = now;
}

void setup()
{
  // Start Serial communication with ESP32-C3
  Serial.begin(9600); // Serial to ESP32-C3 (RX/TX)
  
  pinMode(PIN_BUTTON_1, INPUT_PULLUP);
  pinMode(PIN_BUTTON_2, INPUT_PULLUP);
  pinMode(PIN_BRAKE_IN, INPUT_PULLUP);
  pinMode(PIN_RPM_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_RPM_1), rpmISR, FALLING);
  pinMode(PIN_BT_EN, OUTPUT);
  digitalWrite(PIN_BT_EN, LOW);
  
  delay(1000);
}

void readButtons()
{
  btn1State   = (digitalRead(PIN_BUTTON_1) == LOW) ? 1 : 0;
  btn2State   = (digitalRead(PIN_BUTTON_2) == LOW) ? 1 : 0;
  brakeState  = (digitalRead(PIN_BRAKE_IN) == LOW) ? 1 : 0;
}

void readBatteries()
{
  // BAT Total (24V)
  int rawTotal = analogRead(PIN_BAT_TOTAL);
  float vTotal = (rawTotal * ADC_REF_VOLTAGE) / 1023.0;
  batTotal = vTotal / (DIVIDER_R2 / (DIVIDER_R1 + DIVIDER_R2));
  if (batTotal < 0.1) batTotal = 0.0;

  // BAT1 (12V)
  int rawBat1 = analogRead(PIN_BAT_1);
  float vBat1 = (rawBat1 * ADC_REF_VOLTAGE) / 1023.0;
  bat1Voltage = vBat1 / (DIVIDER_R4/ (DIVIDER_R3 + DIVIDER_R4));
  if (bat1Voltage < 0.1) bat1Voltage = 0.0;

  // BAT2 (12V)
  bat2Voltage = batTotal - bat1Voltage;
  
  // Read current (if connected)
  int rawCurrent = analogRead(PIN_CURRENT);
  // Convert to amps - you'll need to calibrate this based on your sensor
  // Example: ACS712 20A module: 2.5V = 0A, sensitivity = 100mV/A
  float voltage = (rawCurrent * ADC_REF_VOLTAGE) / 1023.0;
  current = (voltage - 2.5) * 10.0; // For ACS712 20A
  
  // Read throttle
  int rawThrottle = analogRead(PIN_THROTTLE);
  throttle = (rawThrottle / 1023.0) * 100.0; // 0-100%
}

void readTemperatures()
{
  // TEMP1
  int raw1 = analogRead(PIN_TEMP_1);
  float v1 = (raw1 * ADC_REF_VOLTAGE) / 1023.0;

  float rNTC1 = NTC_TOP_RES * (v1 / (ADC_REF_VOLTAGE - v1));

  float t1 = rNTC1 / NTC_NOMINAL;
  t1 = log(t1);
  t1 /= B_COEFFICIENT;
  t1 += 1.0 / (TEMP_NOMINAL + 273.15);
  t1 = 1.0 / t1;
  temp1C = t1 - 273.15;

  // TEMP2 (Same shit)
  int raw2 = analogRead(PIN_TEMP_2);
  float v2 = (raw2 * ADC_REF_VOLTAGE) / 1023.0;

  float rNTC2 = NTC_TOP_RES * (v2 / (ADC_REF_VOLTAGE - v2));

  float t2 = rNTC2 / NTC_NOMINAL;
  t2 = log(t2);
  t2 /= B_COEFFICIENT;
  t2 += 1.0 / (TEMP_NOMINAL + 273.15);
  t2 = 1.0 / t2;
  temp2C = t2 - 273.15;
}

void RPM_calculation()
{
  noInterrupts();
  unsigned long period = pulsePeriod;
  unsigned long lastPulse = lastPulseTime;
  interrupts();

  // Timeout: wheel stopped
  if (period > 0 && (micros() - lastPulse) < 500000UL) {
    float periodSec = period / 1e6;               // µs → s
    RPM_wheel = 60.0 / (periodSec * Gear_optical);
  } else {
    RPM_wheel = 0.0;
  }

  // Speed calculation
  float rps = RPM_wheel / 60.0;
  speed_cm_s = 2.0 * PI_VAL * wheelRadius_cm * rps;
  speed_m_s  = speed_cm_s / 100.0;
  speed_km_h = speed_m_s * 3.6;
}

// Send ALL data to ESP32-C3 in CSV format
void sendToESP() {
  // Format: 
  // B1,B2,BRK,TEMP1,TEMP2,BAT1,BAT2,BAT_TOTAL,CURRENT,THROTTLE,RPM,SPEED_KMH,TIMESTAMP
  String data = 
    String(btn1State) + "," +
    String(btn2State) + "," +
    String(brakeState) + "," +
    String(temp1C, 1) + "," +      // TEMP1
    String(temp2C, 1) + "," +      // TEMP2
    String(bat1Voltage, 2) + "," + // BAT1
    String(bat2Voltage, 2) + "," + // BAT2
    String(batTotal, 2) + "," +    // BAT_TOTAL
    String(current, 1) + "," +     // CURRENT
    String(throttle, 0) + "," +    // THROTTLE
    String(RPM_wheel, 0) + "," +   // RPM
    String(speed_km_h, 1) + "," +  // SPEED km/h
    String(millis());              // TIMESTAMP
  
  Serial.println(data); // Send to ESP32-C3 via TX
}

void loop()
{
  readButtons();
  readBatteries();
  readTemperatures();
  RPM_calculation();
  
  if (millis() - lastTime >= 100) { // Update every 100ms
    lastTime = millis();
    
    // Send ALL data to ESP32-C3
    sendToESP();
  }
}