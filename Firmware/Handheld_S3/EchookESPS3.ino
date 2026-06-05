#include <Arduino_GFX_Library.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

/* ===================== ESP-NOW DEFINITIONS ===================== */
typedef struct sensor_data_full {
    uint8_t btn1;
    uint8_t btn2;
    uint8_t brake;
    float temp1;
    float temp2;
    float bat1;
    float bat2;
    float batTotal;
    float current;
    float throttle;
    int rpm;
    float speedKmh;
    uint32_t timestamp;
    float vsens;
    float power;
    float wh;
} sensor_data_full_t;

sensor_data_full_t receivedData;
bool newDataAvailable = false;
unsigned long lastReceivedTime = 0;
unsigned long lastPacketLocalTime = 0;
int receivedPacketCount = 0;
int errorPacketCount = 0;
int currentRSSI = 0;

/* ===================== DISPLAY PINS ===================== */
#define TFT_CS   10
#define TFT_DC   13
#define TFT_RST  -1
#define TFT_SCK  12
#define TFT_MOSI 11
#define TFT_BL   14

/* ===================== JOYSTICKS ===================== */
#define JOY1_X   1
#define JOY1_Y   2
#define JOY1_BTN 42
#define JOY2_X   4
#define JOY2_Y   5
#define JOY2_BTN 7

/* ===================== BATTERY ===================== */
#define BAT_ADC  6

/* ===================== SWITCH GPIO GROUPS ===================== */
const int sw3pos[4][2] = {
  {15, 16},
  {18, 17},
  {39, 38},
  {41, 40}
};

const int sw2pos[4][2] = {
  {8, 3},
  {46, 9},
  {35, 45},
  {36, 37}
};

/* ===================== DISPLAY OBJECT ===================== */
Arduino_DataBus *bus = new Arduino_ESP32SPI(
  TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, -1, SPI2_HOST
);
Arduino_GFX *gfx = new Arduino_ST7796(bus, TFT_RST, 1, true);

/* ===================== SCREEN DEFINITIONS ===================== */
enum ScreenMode {
  SCREEN_CONTROLS = 0,
  SCREEN_TELEMETRY,
  SCREEN_GRAPHS,
  SCREEN_DASHBOARD,
  SCREEN_LOG,
  SCREEN_STATUS,
  SCREEN_COUNT
};

ScreenMode currentScreen = SCREEN_CONTROLS;
bool screenNeedsRedraw = true;

const unsigned long debounceDelay = 200;

/* ===================== LAYOUT CONSTANTS ===================== */
#define J1_CX 120
#define J1_CY 150
#define J2_CX 360
#define J2_CY 150
#define RADIUS 60

#define SW_X_START 40
#define SW_GAP     25
#define SW3_W 70
#define SW2_W 50
#define SW_H  20
#define SW3_Y 260
#define SW2_Y 295

#define TELEM_X 20
#define TELEM_Y 40
#define TELEM_WIDTH 440
#define TELEM_HEIGHT 240
#define LABEL_X 30
#define VALUE_X 250
#define LINE_HEIGHT 28

#define GRAPH_X 20
#define GRAPH_Y 60
#define GRAPH_WIDTH 440
#define GRAPH_HEIGHT 200
#define MAX_GRAPH_POINTS 60

#define STATUS_X 20
#define STATUS_Y 50
#define STATUS_WIDTH 440
#define STATUS_HEIGHT 240

/* ===================== GRAPH HISTORY ===================== */
float temp1History[MAX_GRAPH_POINTS];
float temp2History[MAX_GRAPH_POINTS];
float voltHistory[MAX_GRAPH_POINTS];
float speedHistory[MAX_GRAPH_POINTS];
int historyIndex = 0;
bool graphsDrawn = false;

/* ===================== LOG DATA ===================== */
#define LOG_LINES 8
String logBuffer[LOG_LINES];
int logIndex = 0;

/* ===================== STATE ===================== */
int prevX1 = 0, prevY1 = 0;
int prevX2 = 0, prevY2 = 0;

/* ===================== COLORS ===================== */
uint16_t BG, GREEN, RED, BLUE, YELLOW, WHITE, CYAN, MAGENTA;
uint16_t DATA_BG, VALUE_COLOR, LABEL_COLOR, ALARM_COLOR, GOOD_COLOR, WARNING_COLOR;

/* ===================== ESP-NOW CALLBACK ===================== */
void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
    if (len == sizeof(receivedData)) {
        memcpy(&receivedData, incomingData, sizeof(receivedData));
        newDataAvailable = true;
        receivedPacketCount++;
        lastReceivedTime = millis();
        lastPacketLocalTime = millis();

        currentRSSI = recv_info->rx_ctrl->rssi;

        // Shift history
        for (int i = 0; i < MAX_GRAPH_POINTS - 1; i++) {
            temp1History[i] = temp1History[i + 1];
            temp2History[i] = temp2History[i + 1];
            voltHistory[i] = voltHistory[i + 1];
            speedHistory[i] = speedHistory[i + 1];
        }
        temp1History[MAX_GRAPH_POINTS - 1] = receivedData.temp1;
        temp2History[MAX_GRAPH_POINTS - 1] = receivedData.temp2;
        voltHistory[MAX_GRAPH_POINTS - 1] = receivedData.batTotal;
        speedHistory[MAX_GRAPH_POINTS - 1] = receivedData.speedKmh;
        historyIndex = min(historyIndex + 1, MAX_GRAPH_POINTS);

        // Add to on-screen log
        char entry[80];
        unsigned long seconds = lastPacketLocalTime / 1000;
        snprintf(entry, sizeof(entry), "%02d:%02d T:%.1f/%.1f V:%.1f P:%.0fW Wh:%.2f",
                 (int)(seconds / 60) % 60, (int)(seconds % 60),
                 receivedData.temp1, receivedData.temp2, receivedData.batTotal, 
                 receivedData.power, receivedData.wh);
        logBuffer[logIndex] = String(entry);
        logIndex = (logIndex + 1) % LOG_LINES;

        // CSV OUTPUT OVER SERIAL (16 fields)
        Serial.printf("%lu,%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.3f\n",
                      receivedData.timestamp,
                      receivedData.btn1,
                      receivedData.btn2,
                      receivedData.brake,
                      receivedData.temp1,
                      receivedData.temp2,
                      receivedData.bat1,
                      receivedData.bat2,
                      receivedData.batTotal,
                      receivedData.current,
                      receivedData.throttle,
                      receivedData.rpm,
                      receivedData.speedKmh,
                      receivedData.vsens,
                      receivedData.power,
                      receivedData.wh);
    } else {
        errorPacketCount++;
        Serial.printf("Packet size mismatch: expected %d, got %d\n", sizeof(receivedData), len);
    }
}

/* ===================== SCREEN SWITCHING ===================== */
void checkScreenSwitch() {
    static bool btn1State = HIGH;
    static bool btn2State = HIGH;
    static bool lastReading1 = HIGH;
    static bool lastReading2 = HIGH;
    static unsigned long debounceTimer1 = 0;
    static unsigned long debounceTimer2 = 0;

    bool reading1 = digitalRead(JOY1_BTN);
    bool reading2 = digitalRead(JOY2_BTN);

    if (reading1 != lastReading1) {
        debounceTimer1 = millis();
    }
    if ((millis() - debounceTimer1) > debounceDelay) {
        if (reading1 != btn1State) {
            btn1State = reading1;
            if (btn1State == LOW) {
                currentScreen = (ScreenMode)((currentScreen + 1) % SCREEN_COUNT);
                screenNeedsRedraw = true;
                graphsDrawn = false;
            }
        }
    }
    lastReading1 = reading1;

    if (reading2 != lastReading2) {
        debounceTimer2 = millis();
    }
    if ((millis() - debounceTimer2) > debounceDelay) {
        if (reading2 != btn2State) {
            btn2State = reading2;
            if (btn2State == LOW) {
                currentScreen = (ScreenMode)((currentScreen - 1 + SCREEN_COUNT) % SCREEN_COUNT);
                screenNeedsRedraw = true;
                graphsDrawn = false;
            }
        }
    }
    lastReading2 = reading2;
}

/* ===================== HELPER FUNCTIONS ===================== */
int joyToDelta(int v) {
    return map(v, 0, 4095, -RADIUS, RADIUS);
}

float readBatteryVoltage() {
    int raw = analogRead(BAT_ADC);
    return (raw / 4095.0) * 3.55 * 4.0;
}

float constrainFloat(float x, float min, float max) {
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

void drawHeader() {
    static unsigned long lastHeaderUpdate = 0;
    if (millis() - lastHeaderUpdate < 200) return;
    lastHeaderUpdate = millis();

    gfx->fillRect(0, 0, 480, 40, BLUE);
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);

    const char* screenNames[] = {"CONTROLS", "TELEMETRY", "GRAPHS", "DASHBOARD", "DATA LOG", "STATUS"};
    gfx->setCursor(10, 12);
    gfx->print(screenNames[currentScreen]);

    gfx->setTextSize(1);
    gfx->setCursor(380, 15);
    gfx->printf("<%d/%d>", currentScreen + 1, SCREEN_COUNT);

    float v = readBatteryVoltage();
    gfx->fillRect(280, 10, 80, 20, BLUE);
    gfx->setCursor(280, 15);
    gfx->print("BAT: ");
    gfx->print(v, 2);
    gfx->print("V");
}

/* ===================== SCREEN 1: CONTROLS ===================== */
void drawControlsScreen() {
    gfx->fillRect(0, 40, 480, 280, BG);

    gfx->drawCircle(J1_CX, J1_CY, RADIUS, WHITE);
    gfx->drawCircle(J2_CX, J2_CY, RADIUS, WHITE);

    gfx->setTextColor(CYAN);
    gfx->setTextSize(2);
    gfx->setCursor(90, 230);
    gfx->print("JOY 1");
    gfx->setCursor(330, 230);
    gfx->print("JOY 2");

    gfx->setTextSize(1);
    gfx->setTextColor(MAGENTA);

    int x = SW_X_START;
    for (int i = 0; i < 4; i++) {
        gfx->fillRoundRect(x, SW3_Y, SW3_W, SW_H, 4, DATA_BG);
        gfx->drawRoundRect(x, SW3_Y, SW3_W, SW_H, 4, WHITE);
        gfx->setCursor(x + 25, SW3_Y - 15);
        gfx->print("T");
        gfx->print(i + 1);
        x += SW3_W + SW_GAP;
    }

    x = SW_X_START + 10;
    for (int i = 0; i < 4; i++) {
        gfx->fillRoundRect(x, SW2_Y, SW2_W, SW_H, 4, DATA_BG);
        gfx->drawRoundRect(x, SW2_Y, SW2_W, SW_H, 4, WHITE);
        gfx->setCursor(x + 18, SW2_Y - 15);
        gfx->print("S");
        gfx->print(i + 1);
        x += SW2_W + SW_GAP;
    }

    gfx->fillCircle(J1_CX, J1_CY, 10, GREEN);
    gfx->fillCircle(J2_CX, J2_CY, 10, GREEN);

    prevX1 = 0; prevY1 = 0;
    prevX2 = 0; prevY2 = 0;
}

void updateControlsScreen() {
    int joy1RawX = analogRead(JOY1_X);
    int joy1RawY = analogRead(JOY1_Y);
    int dx1 = -joyToDelta(joy1RawY);
    int dy1 = joyToDelta(joy1RawX);
    bool btn1 = digitalRead(JOY1_BTN) == LOW;

    int joy2RawX = analogRead(JOY2_X);
    int joy2RawY = analogRead(JOY2_Y);
    int dx2 = -joyToDelta(joy2RawY);
    int dy2 = joyToDelta(joy2RawX);
    bool btn2 = digitalRead(JOY2_BTN) == LOW;

    gfx->fillCircle(J1_CX + prevX1, J1_CY + prevY1, 15, BG);
    gfx->fillCircle(J2_CX + prevX2, J2_CY + prevY2, 15, BG);

    gfx->drawCircle(J1_CX, J1_CY, RADIUS, WHITE);
    gfx->drawCircle(J2_CX, J2_CY, RADIUS, WHITE);

    gfx->fillCircle(J1_CX + dx2, J1_CY + dy2, 10, btn2 ? RED : GREEN);
    gfx->fillCircle(J2_CX + dx1, J2_CY + dy1, 10, btn1 ? RED : GREEN);

    prevX1 = dx2; prevY1 = dy2;
    prevX2 = dx1; prevY2 = dy1;

    int x = SW_X_START;
    for (int i = 0; i < 4; i++) {
        bool swA = digitalRead(sw3pos[i][0]) == LOW;
        bool swB = digitalRead(sw3pos[i][1]) == LOW;
        int pos = 0;
        if (swA && !swB) pos = -1;
        else if (!swA && swB) pos = 1;
        int cx = x + SW3_W / 2 + pos * (SW3_W / 3);
        gfx->fillRoundRect(x, SW3_Y, SW3_W, SW_H, 4, DATA_BG);
        gfx->drawRoundRect(x, SW3_Y, SW3_W, SW_H, 4, WHITE);
        gfx->fillCircle(cx, SW3_Y + SW_H / 2, 6, CYAN);
        x += SW3_W + SW_GAP;
    }

    x = SW_X_START + 10;
    for (int i = 0; i < 4; i++) {
        bool swA = digitalRead(sw2pos[i][0]) == LOW;
        int pos = swA ? -1 : 1;
        int cx = x + SW2_W / 2 + pos * (SW2_W / 4);
        gfx->fillRoundRect(x, SW2_Y, SW2_W, SW_H, 4, DATA_BG);
        gfx->drawRoundRect(x, SW2_Y, SW2_W, SW_H, 4, WHITE);
        gfx->fillCircle(cx, SW2_Y + SW_H / 2, 6, CYAN);
        x += SW2_W + SW_GAP;
    }
}

/* ===================== SCREEN 2: TELEMETRY (COMPLETELY REWRITTEN) ===================== */
uint8_t prevBtn1, prevBtn2, prevBrake;
float prevTemp1, prevTemp2, prevBat1, prevBat2, prevBatTotal, prevSpeed, prevCurrent, prevVsens, prevPower, prevWh;

// Declare updateTelemetryScreen first, then drawTelemetryScreen
void updateTelemetryScreen(bool forceFullUpdate = false) {
    if (!newDataAvailable && !forceFullUpdate) return;
    
    gfx->setTextSize(2);
    int y = TELEM_Y + 30;
    
    // ============ LEFT COLUMN ============
    
    // BTN1
    if (forceFullUpdate || receivedData.btn1 != prevBtn1) {
        gfx->fillRect(100, y, 120, LINE_HEIGHT, BG);
        gfx->setTextColor(receivedData.btn1 ? GREEN : VALUE_COLOR);
        gfx->setCursor(100, y);
        gfx->print(receivedData.btn1 ? "ON" : "OFF");
        prevBtn1 = receivedData.btn1;
    }
    
    // BTN2
    if (forceFullUpdate || receivedData.btn2 != prevBtn2) {
        gfx->fillRect(100, y + LINE_HEIGHT, 120, LINE_HEIGHT, BG);
        gfx->setTextColor(receivedData.btn2 ? GREEN : VALUE_COLOR);
        gfx->setCursor(100, y + LINE_HEIGHT);
        gfx->print(receivedData.btn2 ? "ON" : "OFF");
        prevBtn2 = receivedData.btn2;
    }
    
    // BRAKE
    if (forceFullUpdate || receivedData.brake != prevBrake) {
        gfx->fillRect(100, y + LINE_HEIGHT*2, 120, LINE_HEIGHT, BG);
        gfx->setTextColor(receivedData.brake ? RED : VALUE_COLOR);
        gfx->setCursor(100, y + LINE_HEIGHT*2);
        gfx->print(receivedData.brake ? "ON" : "OFF");
        prevBrake = receivedData.brake;
    }
    
    // TEMP1
    auto tempColor = [](float t) {
        return t > 35 ? ALARM_COLOR : (t > 30 ? WARNING_COLOR : VALUE_COLOR);
    };
    if (forceFullUpdate || abs(receivedData.temp1 - prevTemp1) >= 0.1) {
        gfx->fillRect(100, y + LINE_HEIGHT*3, 140, LINE_HEIGHT, BG);
        gfx->setTextColor(tempColor(receivedData.temp1));
        gfx->setCursor(100, y + LINE_HEIGHT*3);
        gfx->printf("%.1fC", receivedData.temp1);
        prevTemp1 = receivedData.temp1;
    }
    
    // TEMP2
    if (forceFullUpdate || abs(receivedData.temp2 - prevTemp2) >= 0.1) {
        gfx->fillRect(100, y + LINE_HEIGHT*4, 140, LINE_HEIGHT, BG);
        gfx->setTextColor(tempColor(receivedData.temp2));
        gfx->setCursor(100, y + LINE_HEIGHT*4);
        gfx->printf("%.1fC", receivedData.temp2);
        prevTemp2 = receivedData.temp2;
    }
    
    // ***** WH - CRITICAL FIX *****
    // Always update Wh on every packet to ensure it shows
    gfx->fillRect(100, y + LINE_HEIGHT*5, 150, LINE_HEIGHT, BG);
    gfx->setTextColor(CYAN);
    gfx->setCursor(100, y + LINE_HEIGHT*5);
    gfx->printf("%.3f Wh", receivedData.wh);
    prevWh = receivedData.wh;
    
    // VSENS
    if (forceFullUpdate || abs(receivedData.vsens - prevVsens) >= 0.05) {
        gfx->fillRect(100, y + LINE_HEIGHT*6, 140, LINE_HEIGHT, BG);
        gfx->setTextColor(VALUE_COLOR);
        gfx->setCursor(100, y + LINE_HEIGHT*6);
        gfx->printf("%.2fV", receivedData.vsens);
        prevVsens = receivedData.vsens;
    }
    
    // ============ RIGHT COLUMN ============
    
    auto batColor = [](float v) {
        return (v < 12.0 || v > 14.5) ? ALARM_COLOR : ((v < 12.5 || v > 14.0) ? WARNING_COLOR : VALUE_COLOR);
    };
    
    // BAT1
    if (forceFullUpdate || abs(receivedData.bat1 - prevBat1) >= 0.05) {
        gfx->fillRect(350, y, 120, LINE_HEIGHT, BG);
        gfx->setTextColor(batColor(receivedData.bat1));
        gfx->setCursor(350, y);
        gfx->printf("%.1fV", receivedData.bat1);
        prevBat1 = receivedData.bat1;
    }
    
    // BAT2
    if (forceFullUpdate || abs(receivedData.bat2 - prevBat2) >= 0.05) {
        gfx->fillRect(350, y + LINE_HEIGHT, 120, LINE_HEIGHT, BG);
        gfx->setTextColor(batColor(receivedData.bat2));
        gfx->setCursor(350, y + LINE_HEIGHT);
        gfx->printf("%.1fV", receivedData.bat2);
        prevBat2 = receivedData.bat2;
    }
    
    // BAT TOTAL
    if (forceFullUpdate || abs(receivedData.batTotal - prevBatTotal) >= 0.05) {
        gfx->fillRect(350, y + LINE_HEIGHT*2, 120, LINE_HEIGHT, BG);
        gfx->setTextColor(batColor(receivedData.batTotal));
        gfx->setCursor(350, y + LINE_HEIGHT*2);
        gfx->printf("%.1fV", receivedData.batTotal);
        prevBatTotal = receivedData.batTotal;
    }
    
    // SPEED
    if (forceFullUpdate || abs(receivedData.speedKmh - prevSpeed) >= 0.5) {
        gfx->fillRect(350, y + LINE_HEIGHT*3, 120, LINE_HEIGHT, BG);
        gfx->setTextColor(VALUE_COLOR);
        gfx->setCursor(350, y + LINE_HEIGHT*3);
        gfx->printf("%.1fkm/h", receivedData.speedKmh);
        prevSpeed = receivedData.speedKmh;
    }
    
    // CURRENT
    if (forceFullUpdate || abs(receivedData.current - prevCurrent) >= 0.05) {
        gfx->fillRect(350, y + LINE_HEIGHT*4, 120, LINE_HEIGHT, BG);
        gfx->setTextColor(VALUE_COLOR);
        gfx->setCursor(350, y + LINE_HEIGHT*4);
        gfx->printf("%.2fA", receivedData.current);
        prevCurrent = receivedData.current;
    }
    
    // POWER
    if (forceFullUpdate || abs(receivedData.power - prevPower) >= 0.5) {
        gfx->fillRect(350, y + LINE_HEIGHT*5, 120, LINE_HEIGHT, BG);
        gfx->setTextColor(VALUE_COLOR);
        gfx->setCursor(350, y + LINE_HEIGHT*5);
        gfx->printf("%.1fW", receivedData.power);
        prevPower = receivedData.power;
    }
    
    newDataAvailable = false;
}

void drawTelemetryScreen() {
    gfx->fillRect(0, 40, 480, 280, BG);
    gfx->setTextSize(2);
    gfx->setTextColor(LABEL_COLOR);
    int y = TELEM_Y + 30;

    // Left column labels (7 lines)
    gfx->setCursor(20, y); gfx->print("BTN1:");
    gfx->setCursor(20, y + LINE_HEIGHT); gfx->print("BTN2:");
    gfx->setCursor(20, y + LINE_HEIGHT*2); gfx->print("BRAKE:");
    gfx->setCursor(20, y + LINE_HEIGHT*3); gfx->print("TEMP1:");
    gfx->setCursor(20, y + LINE_HEIGHT*4); gfx->print("TEMP2:");
    gfx->setCursor(20, y + LINE_HEIGHT*5); gfx->print("WH:");
    gfx->setCursor(20, y + LINE_HEIGHT*6); gfx->print("VSENS:");

    // Right column labels (6 lines)
    gfx->setCursor(250, y); gfx->print("BAT1:");
    gfx->setCursor(250, y + LINE_HEIGHT); gfx->print("BAT2:");
    gfx->setCursor(250, y + LINE_HEIGHT*2); gfx->print("BAT TOT:");
    gfx->setCursor(250, y + LINE_HEIGHT*3); gfx->print("SPEED:");
    gfx->setCursor(250, y + LINE_HEIGHT*4); gfx->print("CURRENT:");
    gfx->setCursor(250, y + LINE_HEIGHT*5); gfx->print("POWER:");

    // Draw all initial values
    updateTelemetryScreen(true); // Force full update
}

/* ===================== SCREEN 3: GRAPHS ===================== */
void drawGraphsScreen() {
    gfx->fillRect(0, 40, 480, 280, BG);
    drawGraphBackground(GRAPH_X, GRAPH_Y, 210, 120, "TEMP1", GREEN);
    drawGraphBackground(GRAPH_X + 230, GRAPH_Y, 210, 120, "TEMP2", BLUE);
    drawGraphBackground(GRAPH_X, GRAPH_Y + 140, 210, 120, "BAT TOT", YELLOW);
    drawGraphBackground(GRAPH_X + 230, GRAPH_Y + 140, 210, 120, "SPEED", RED);
    graphsDrawn = true;
}

void drawGraphBackground(int x, int y, int w, int h, const char* title, uint16_t color) {
    gfx->drawRect(x, y, w, h, WHITE);
    gfx->setTextSize(1);
    gfx->setTextColor(color);
    gfx->setCursor(x + 5, y + 5);
    gfx->print(title);
    gfx->setCursor(x + w - 50, y + 5);
    gfx->print("0.0");
}

void updateGraphsScreen() {
    static unsigned long lastGraphUpdate = 0;
    if (!newDataAvailable && millis() - lastGraphUpdate < 1000) return;
    lastGraphUpdate = millis();

    int graphW = 190, graphH = 90;
    gfx->fillRect(GRAPH_X + 10, GRAPH_Y + 10, graphW, graphH, BG);
    gfx->fillRect(GRAPH_X + 230 + 10, GRAPH_Y + 10, graphW, graphH, BG);
    gfx->fillRect(GRAPH_X + 10, GRAPH_Y + 140 + 10, graphW, graphH, BG);
    gfx->fillRect(GRAPH_X + 230 + 10, GRAPH_Y + 140 + 10, graphW, graphH, BG);

    if (historyIndex > 1) {
        drawGraphData(GRAPH_X, GRAPH_Y, 210, 120, temp1History, 20, 60, GREEN);
        drawGraphData(GRAPH_X + 230, GRAPH_Y, 210, 120, temp2History, 20, 60, BLUE);
        drawGraphData(GRAPH_X, GRAPH_Y + 140, 210, 120, voltHistory, 16, 28, YELLOW);
        drawGraphData(GRAPH_X + 230, GRAPH_Y + 140, 210, 120, speedHistory, 0, 50, RED);
    }

    gfx->setTextSize(1);

    gfx->fillRect(GRAPH_X + 150, GRAPH_Y + 5, 50, 10, BG);
    gfx->setTextColor(GREEN);
    gfx->setCursor(GRAPH_X + 150, GRAPH_Y + 5);
    gfx->printf("%.1fC", temp1History[MAX_GRAPH_POINTS - 1]);

    gfx->fillRect(GRAPH_X + 380, GRAPH_Y + 5, 50, 10, BG);
    gfx->setTextColor(BLUE);
    gfx->setCursor(GRAPH_X + 380, GRAPH_Y + 5);
    gfx->printf("%.1fC", temp2History[MAX_GRAPH_POINTS - 1]);

    gfx->fillRect(GRAPH_X + 150, GRAPH_Y + 145, 50, 10, BG);
    gfx->setTextColor(YELLOW);
    gfx->setCursor(GRAPH_X + 150, GRAPH_Y + 145);
    gfx->printf("%.1fV", voltHistory[MAX_GRAPH_POINTS - 1]);

    gfx->fillRect(GRAPH_X + 380, GRAPH_Y + 145, 50, 10, BG);
    gfx->setTextColor(RED);
    gfx->setCursor(GRAPH_X + 380, GRAPH_Y + 145);
    gfx->printf("%.1fkm/h", speedHistory[MAX_GRAPH_POINTS - 1]);

    newDataAvailable = false;
}

void drawGraphData(int x, int y, int w, int h, float* data, float minVal, float maxVal, uint16_t color) {
    int graphW = w - 20, graphH = h - 30;
    for (int i = 1; i < MAX_GRAPH_POINTS; i++) {
        float v1 = data[i-1], v2 = data[i];
        if (v1 < minVal-10 || v1 > maxVal+10 || v2 < minVal-10 || v2 > maxVal+10) continue;
        int x1 = x + 10 + (i-1) * graphW / MAX_GRAPH_POINTS;
        int y1 = y + h - 20 - map(constrain(v1, minVal, maxVal), minVal, maxVal, 0, graphH);
        int x2 = x + 10 + i * graphW / MAX_GRAPH_POINTS;
        int y2 = y + h - 20 - map(constrain(v2, minVal, maxVal), minVal, maxVal, 0, graphH);
        if (y1 >= y+10 && y1 <= y+h-20 && y2 >= y+10 && y2 <= y+h-20) {
            gfx->drawLine(x1, y1, x2, y2, color);
        }
    }
}

/* ===================== SCREEN 4: DASHBOARD ===================== */
void drawDashboardScreen() {
    gfx->fillRect(0, 40, 480, 280, BG);
    gfx->drawCircle(120, 160, 90, WHITE);
    gfx->drawCircle(120, 160, 80, CYAN);
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);
    for (int i = 0; i <= 100; i += 20) {
        float rad = radians(map(i, 0, 100, 210, -30));
        int x1 = 120 + 70 * cos(rad);
        int y1 = 160 + 70 * sin(rad);
        int x2 = 120 + 80 * cos(rad);
        int y2 = 160 + 80 * sin(rad);
        gfx->drawLine(x1, y1, x2, y2, WHITE);
    }
    gfx->setCursor(90, 120); gfx->print("SPEED");
    gfx->setCursor(100, 200); gfx->print("km/h");

    gfx->drawCircle(360, 160, 90, WHITE);
    gfx->drawCircle(360, 160, 80, CYAN);
    for (int i = 0; i <= 6000; i += 1000) {
        float rad = radians(map(i, 0, 6000, 210, -30));
        int x1 = 360 + 70 * cos(rad);
        int y1 = 160 + 70 * sin(rad);
        int x2 = 360 + 80 * cos(rad);
        int y2 = 160 + 80 * sin(rad);
        gfx->drawLine(x1, y1, x2, y2, WHITE);
    }
    gfx->setCursor(340, 120); gfx->print("RPM");

    gfx->drawRect(20, 260, 200, 20, WHITE);
    gfx->setCursor(20, 245);
    gfx->print("BATTERY");

    gfx->drawRect(250, 260, 100, 20, WHITE);
    gfx->drawRect(360, 260, 100, 20, WHITE);
    gfx->setCursor(250, 245);
    gfx->print("T1");
    gfx->setCursor(360, 245);
    gfx->print("T2");
}

void updateDashboardScreen() {
    static unsigned long lastUpdate = 0;
    if (!newDataAvailable && millis() - lastUpdate < 500) return;
    lastUpdate = millis();

    gfx->fillRect(60, 140, 120, 40, BG);
    gfx->fillRect(300, 140, 120, 40, BG);
    gfx->fillRect(22, 262, 196, 16, BG);
    gfx->fillRect(252, 262, 96, 16, BG);
    gfx->fillRect(362, 262, 96, 16, BG);
    gfx->fillRect(20, 225, 180, 20, BG); // Clear Wh area

    float speedAngle = map(constrain(receivedData.speedKmh, 0, 100), 0, 100, 210, -30);
    int needleX = 120 + 70 * cos(radians(speedAngle));
    int needleY = 160 + 70 * sin(radians(speedAngle));
    gfx->drawLine(120, 160, needleX, needleY, RED);
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    gfx->setCursor(90, 150);
    gfx->printf("%.1f", receivedData.speedKmh);

    float rpmAngle = map(constrain(receivedData.rpm, 0, 6000), 0, 6000, 210, -30);
    needleX = 360 + 70 * cos(radians(rpmAngle));
    needleY = 160 + 70 * sin(radians(rpmAngle));
    gfx->drawLine(360, 160, needleX, needleY, RED);
    gfx->setCursor(330, 150);
    gfx->printf("%d", receivedData.rpm);

    float batPercent = constrainFloat(receivedData.batTotal / 30.0, 0, 1);
    int barWidth = batPercent * 196;
    uint16_t batBarColor = (receivedData.batTotal < 22) ? RED : (receivedData.batTotal < 24) ? YELLOW : GREEN;
    gfx->fillRect(22, 262, barWidth, 16, batBarColor);

    auto tempBar = [&](int x, float temp) {
        float p = constrainFloat(temp / 60.0, 0, 1);
        int w = p * 96;
        uint16_t col = temp > 40 ? RED : (temp > 35 ? YELLOW : GREEN);
        gfx->fillRect(x, 262, w, 16, col);
        gfx->setTextSize(1);
        gfx->setCursor(x + 2, 265);
        gfx->printf("%.1fC", temp);
    };
    tempBar(252, receivedData.temp1);
    tempBar(362, receivedData.temp2);
    
    // Display Wh on dashboard
    gfx->setTextSize(1);
    gfx->setTextColor(CYAN);
    gfx->setCursor(20, 225);
    gfx->printf("Energy: %.3f Wh", receivedData.wh);

    newDataAvailable = false;
}

/* ===================== SCREEN 5: DATA LOG ===================== */
void drawLogScreen() {
    gfx->fillRect(0, 40, 480, 280, BG);
    gfx->drawRect(10, 50, 460, 260, WHITE);
    gfx->setTextSize(1);
    gfx->setTextColor(LABEL_COLOR);
    gfx->setCursor(20, 55);
    gfx->print("TIME    TEMP1  TEMP2   BATT   POWER    WH     SPEED");
    gfx->drawLine(10, 65, 470, 65, WHITE);
}

void updateLogScreen() {
    static unsigned long lastLogUpdate = 0;
    if (millis() - lastLogUpdate < 1000) return;
    lastLogUpdate = millis();

    gfx->fillRect(12, 67, 456, 240, BG);

    gfx->setTextSize(1);
    for (int i = 0; i < LOG_LINES; i++) {
        int idx = (logIndex - 1 - i + LOG_LINES) % LOG_LINES;
        if (logBuffer[idx].length() > 0) {
            gfx->setCursor(20, 75 + i * 30);
            gfx->setTextColor(i == 0 ? WHITE : VALUE_COLOR);
            gfx->print(logBuffer[idx]);
        }
    }
}

/* ===================== SCREEN 6: STATUS ===================== */
void drawStatusScreen() {
    gfx->fillRect(0, 40, 480, 280, BG);
    gfx->drawRoundRect(STATUS_X, STATUS_Y, STATUS_WIDTH, STATUS_HEIGHT, 10, CYAN);
    gfx->setTextSize(1);
    gfx->setTextColor(LABEL_COLOR);
    int y = STATUS_Y + 40;
    int lh = 22;
    gfx->setCursor(STATUS_X + 20, y); gfx->print("MAC:");
    gfx->setCursor(STATUS_X + 20, y + lh); gfx->print("Signal:");
    gfx->setCursor(STATUS_X + 20, y + lh*2); gfx->print("Packets:");
    gfx->setCursor(STATUS_X + 20, y + lh*3); gfx->print("Errors:");
    gfx->setCursor(STATUS_X + 20, y + lh*4); gfx->print("Success:");
    gfx->setCursor(STATUS_X + 20, y + lh*5); gfx->print("Last pkt:");
    gfx->setCursor(STATUS_X + 20, y + lh*6); gfx->print("Battery:");
}

void updateStatusScreen() {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate < 1000) return;
    lastUpdate = millis();

    int y = STATUS_Y + 40;
    int lh = 22;
    gfx->fillRect(STATUS_X + 80, y, 200, lh * 7, BG);

    gfx->setTextColor(VALUE_COLOR);
    gfx->setCursor(STATUS_X + 80, y);
    gfx->print(WiFi.macAddress());

    if (currentRSSI == 0 && lastReceivedTime == 0) {
        gfx->setTextColor(VALUE_COLOR);
        gfx->setCursor(STATUS_X + 80, y + lh);
        gfx->print("waiting...");
    } else {
        uint16_t rssiColor = (currentRSSI > -60) ? GREEN : (currentRSSI > -70) ? YELLOW : RED;
        gfx->setTextColor(rssiColor);
        gfx->setCursor(STATUS_X + 80, y + lh);
        gfx->printf("%d dBm", currentRSSI);
    }

    gfx->setTextColor(VALUE_COLOR);
    gfx->setCursor(STATUS_X + 80, y + lh*2);
    gfx->print(receivedPacketCount);

    gfx->setTextColor(errorPacketCount > 0 ? RED : VALUE_COLOR);
    gfx->setCursor(STATUS_X + 80, y + lh*3);
    gfx->print(errorPacketCount);

    if (receivedPacketCount + errorPacketCount > 0) {
        float successRate = (float)receivedPacketCount / (receivedPacketCount + errorPacketCount) * 100;
        uint16_t successColor = successRate > 95 ? GREEN : successRate > 80 ? YELLOW : RED;
        gfx->setTextColor(successColor);
        gfx->setCursor(STATUS_X + 80, y + lh*4);
        gfx->printf("%.1f %%", successRate);
    }

    if (lastReceivedTime > 0) {
        unsigned long timeSince = (millis() - lastReceivedTime) / 1000;
        uint16_t timeColor = timeSince < 5 ? GREEN : timeSince < 10 ? YELLOW : RED;
        gfx->setTextColor(timeColor);
        gfx->setCursor(STATUS_X + 80, y + lh*5);
        gfx->printf("%lu s", timeSince);
    } else {
        gfx->setTextColor(RED);
        gfx->setCursor(STATUS_X + 80, y + lh*5);
        gfx->print("none");
    }

    float batVoltage = readBatteryVoltage();
    uint16_t batColor = batVoltage > 12.0 ? GREEN : batVoltage > 11.5 ? YELLOW : RED;
    gfx->setTextColor(batColor);
    gfx->setCursor(STATUS_X + 80, y + lh*6);
    gfx->printf("%.2f V", batVoltage);
}

/* ===================== SETUP ===================== */
void setup() {
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    pinMode(JOY1_BTN, INPUT_PULLUP);
    pinMode(JOY2_BTN, INPUT_PULLUP);

    for (int i = 0; i < 4; i++) {
        pinMode(sw3pos[i][0], INPUT_PULLUP);
        pinMode(sw3pos[i][1], INPUT_PULLUP);
        pinMode(sw2pos[i][0], INPUT_PULLUP);
        pinMode(sw2pos[i][1], INPUT_PULLUP);
    }

    gfx->begin();

    BG          = gfx->color565(0, 0, 0);
    GREEN       = gfx->color565(0, 255, 0);
    RED         = gfx->color565(255, 0, 0);
    BLUE        = gfx->color565(0, 100, 255);
    YELLOW      = gfx->color565(255, 255, 0);
    WHITE       = gfx->color565(255, 255, 255);
    CYAN        = gfx->color565(0, 255, 255);
    MAGENTA     = gfx->color565(255, 0, 255);
    DATA_BG     = gfx->color565(30, 30, 50);
    VALUE_COLOR = gfx->color565(100, 200, 255);
    LABEL_COLOR = gfx->color565(200, 200, 200);
    ALARM_COLOR = gfx->color565(255, 50, 50);
    GOOD_COLOR  = gfx->color565(50, 255, 50);
    WARNING_COLOR = gfx->color565(255, 165, 0);

    Serial.begin(115200);
    delay(1000);

    // Print CSV header (16 columns)
    Serial.println("timestamp,btn1,btn2,brake,temp1,temp2,bat1,bat2,batTotal,current,throttle,rpm,speedKmh,vsens,power,wh");

    WiFi.mode(WIFI_STA);
    esp_wifi_set_max_tx_power(76);
    esp_wifi_set_channel(0, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while(1) delay(1000);
    }
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP32-S3 Ready");
    Serial.print("My MAC: ");
    Serial.println(WiFi.macAddress());

    for (int i = 0; i < MAX_GRAPH_POINTS; i++) {
        temp1History[i] = 25.0;
        temp2History[i] = 25.0;
        voltHistory[i] = 24.0;
        speedHistory[i] = 0.0;
    }

    for (int i = 0; i < LOG_LINES; i++) {
        logBuffer[i] = "";
    }

    drawHeader();
    drawControlsScreen();
}

/* ===================== LOOP ===================== */
void loop() {
    checkScreenSwitch();
    drawHeader();

    if (screenNeedsRedraw) {
        switch (currentScreen) {
            case SCREEN_CONTROLS: drawControlsScreen(); break;
            case SCREEN_TELEMETRY: drawTelemetryScreen(); break;
            case SCREEN_GRAPHS: drawGraphsScreen(); break;
            case SCREEN_DASHBOARD: drawDashboardScreen(); break;
            case SCREEN_LOG: drawLogScreen(); break;
            case SCREEN_STATUS: drawStatusScreen(); break;
            default: break;
        }
        screenNeedsRedraw = false;
    }

    switch (currentScreen) {
        case SCREEN_CONTROLS: updateControlsScreen(); break;
        case SCREEN_TELEMETRY: updateTelemetryScreen(); break;
        case SCREEN_GRAPHS: updateGraphsScreen(); break;
        case SCREEN_DASHBOARD: updateDashboardScreen(); break;
        case SCREEN_LOG: updateLogScreen(); break;
        case SCREEN_STATUS: updateStatusScreen(); break;
        default: break;
    }

    delay(20);
}