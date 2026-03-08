#include <Arduino_GFX_Library.h>
#include <esp_now.h>
#include <WiFi.h>

/* ===================== ESP-NOW DEFINITIONS ===================== */
// Match the structure from ESP32-C3
typedef struct sensor_data_full {
    // Buttons
    uint8_t btn1;
    uint8_t btn2;
    uint8_t brake;
    
    // Temperatures
    float temp1;
    float temp2;
    
    // Batteries
    float bat1;
    float bat2;
    float batTotal;
    
    // Current and throttle
    float current;
    float throttle;
    
    // RPM and speed
    int rpm;
    float speedKmh;
    
    // Timestamp
    uint32_t timestamp;
} sensor_data_full_t;

sensor_data_full_t receivedData;
bool newDataAvailable = false;
unsigned long lastReceivedTime = 0;
int receivedPacketCount = 0;
int errorPacketCount = 0;

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
// ON / OFF / ON - Using T1 for screen switching
const int sw3pos[4][2] = {
  {15, 16},  // T1 - screen switching
  {18, 17},  // T2 - reserved
  {39, 38},  // T3 - reserved
  {41, 40}   // T4 - reserved
};

// ON / ON
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

Arduino_GFX *gfx = new Arduino_ST7796(
  bus, TFT_RST, 1, true
);

/* ===================== SCREEN DEFINITIONS ===================== */
enum ScreenMode {
  SCREEN_CONTROLS = 0,     // Control panel
  SCREEN_TELEMETRY = 1,    // Telemetry data
  SCREEN_GRAPHS = 2,       // Graphs
  SCREEN_STATUS = 3        // Status
};

ScreenMode currentScreen = SCREEN_CONTROLS;
ScreenMode previousScreen = SCREEN_CONTROLS;
bool screenNeedsRedraw = true;

/* ===================== LAYOUT FOR CONTROLS ===================== */
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

/* ===================== LAYOUT FOR TELEMETRY ===================== */
#define TELEM_X 20
#define TELEM_Y 40
#define TELEM_WIDTH 440
#define TELEM_HEIGHT 240
#define VALUE_X 250
#define LABEL_X 30
#define LINE_HEIGHT 40

/* ===================== LAYOUT FOR GRAPHS ===================== */
#define GRAPH_X 20
#define GRAPH_Y 60
#define GRAPH_WIDTH 440
#define GRAPH_HEIGHT 200
#define MAX_GRAPH_POINTS 60

/* ===================== LAYOUT FOR STATUS ===================== */
#define STATUS_X 20
#define STATUS_Y 60
#define STATUS_WIDTH 440
#define STATUS_HEIGHT 200

/* ===================== GRAPH HISTORY ===================== */
float temp1History[MAX_GRAPH_POINTS];
float temp2History[MAX_GRAPH_POINTS];
float voltHistory[MAX_GRAPH_POINTS];
float speedHistory[MAX_GRAPH_POINTS];
int historyIndex = 0;
bool graphsDrawn = false;

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
        
        // Shift data for scrolling
        for (int i = 0; i < MAX_GRAPH_POINTS - 1; i++) {
            temp1History[i] = temp1History[i + 1];
            temp2History[i] = temp2History[i + 1];
            voltHistory[i] = voltHistory[i + 1];
            speedHistory[i] = speedHistory[i + 1];
        }
        
        // Add new values at the end
        temp1History[MAX_GRAPH_POINTS - 1] = receivedData.temp1;
        temp2History[MAX_GRAPH_POINTS - 1] = receivedData.temp2;
        voltHistory[MAX_GRAPH_POINTS - 1] = receivedData.batTotal;
        speedHistory[MAX_GRAPH_POINTS - 1] = receivedData.speedKmh;
        
        historyIndex = min(historyIndex + 1, MAX_GRAPH_POINTS);
    } else {
        errorPacketCount++;
    }
}

/* ===================== SCREEN SWITCHING ===================== */
void checkScreenSwitch() {
    bool A = digitalRead(sw3pos[0][0]) == LOW;
    bool B = digitalRead(sw3pos[0][1]) == LOW;
    
    ScreenMode newScreen = currentScreen;
    
    if (A && !B) {
        newScreen = SCREEN_CONTROLS;
    } else if (!A && !B) {
        newScreen = SCREEN_TELEMETRY;
    } else if (!A && B) {
        newScreen = SCREEN_GRAPHS;
    }
    
    if (A && B) {
        newScreen = SCREEN_STATUS;
    }
    
    if (newScreen != currentScreen) {
        previousScreen = currentScreen;
        currentScreen = newScreen;
        screenNeedsRedraw = true;
        graphsDrawn = false;
        
        digitalWrite(TFT_BL, LOW);
        delay(50);
        digitalWrite(TFT_BL, HIGH);
    }
}

/* ===================== HELPER FUNCTIONS ===================== */
int joyToDelta(int v) {
    return map(v, 0, 4095, -RADIUS, RADIUS);
}

float readBatteryVoltage() {
    int raw = analogRead(BAT_ADC);
    return (raw / 4095.0) * 3.55 * 4.0;
}

void drawHeader() {
    static ScreenMode lastScreenDrawn = SCREEN_CONTROLS;
    static unsigned long lastHeaderUpdate = 0;
    static float lastBatteryVoltage = 0;
    
    if (millis() - lastHeaderUpdate < 200 && lastScreenDrawn == currentScreen) {
        return;
    }
    
    lastHeaderUpdate = millis();
    lastScreenDrawn = currentScreen;
    
    // Draw header background
    gfx->fillRect(0, 0, 480, 40, BLUE);
    
    // Screen name
    gfx->setTextSize(2);
    gfx->setTextColor(WHITE);
    
    const char* screenNames[] = {"CONTROLS", "TELEMETRY", "GRAPHS", "STATUS"};
    gfx->setCursor(10, 12);
    gfx->print(screenNames[currentScreen]);
    
    // Screen indicator
    gfx->setTextSize(1);
    gfx->setCursor(380, 15);
    gfx->print("[");
    for (int i = 0; i < 3; i++) {
        if (i == currentScreen) {
            gfx->print("#");
        } else {
            gfx->print(".");
        }
    }
    gfx->print("]");
    
    // Draw battery voltage
    float v = readBatteryVoltage();
    gfx->fillRect(280, 10, 80, 20, BLUE);
    gfx->setTextSize(1);
    gfx->setTextColor(WHITE);
    gfx->setCursor(280, 15);
    gfx->print("BAT: ");
    gfx->print(v, 2);
    gfx->print("V");
    lastBatteryVoltage = v;
}

/* ===================== SCREEN 1: CONTROLS ===================== */
void drawControlsScreen() {
    // Background
    gfx->fillRect(0, 40, 480, 280, BG);
    
    // Joystick circles
    gfx->drawCircle(J1_CX, J1_CY, RADIUS, WHITE);
    gfx->drawCircle(J2_CX, J2_CY, RADIUS, WHITE);
    
    // Labels
    gfx->setTextColor(CYAN);
    gfx->setTextSize(2);
    gfx->setCursor(90, 230);
    gfx->print("JOY 1");
    gfx->setCursor(330, 230);
    gfx->print("JOY 2");
    
    // Switches background
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
    
    // Initial joystick dots
    gfx->fillCircle(J1_CX, J1_CY, 10, GREEN);
    gfx->fillCircle(J2_CX, J2_CY, 10, GREEN);
    
    prevX1 = 0; prevY1 = 0;
    prevX2 = 0; prevY2 = 0;
}

void updateControlsScreen() {
    
    // JOY1 (GPIO 1,2) -> controls RIGHT dot on screen (J2_CX, J2_CY)
    int joy1RawX = analogRead(JOY1_X);  // GPIO 1
    int joy1RawY = analogRead(JOY1_Y);  // GPIO 2
    // Swap axes AND invert X: -joyToDelta(Y) for X movement
    int dx1 = -joyToDelta(joy1RawY);  // Inverted X axis
    int dy1 = joyToDelta(joy1RawX);   // Normal Y axis
    bool btn1 = digitalRead(JOY1_BTN) == LOW;
    
    // JOY2 (GPIO 4,5) -> controls LEFT dot on screen (J1_CX, J1_CY)  
    int joy2RawX = analogRead(JOY2_X);  // GPIO 4
    int joy2RawY = analogRead(JOY2_Y);  // GPIO 5
    // Swap axes AND invert X: -joyToDelta(Y) for X movement
    int dx2 = -joyToDelta(joy2RawY);  // Inverted X axis
    int dy2 = joyToDelta(joy2RawX);   // Normal Y axis
    bool btn2 = digitalRead(JOY2_BTN) == LOW;
    
    // Clear old dots
    gfx->fillCircle(J1_CX + prevX1, J1_CY + prevY1, 15, BG);
    gfx->fillCircle(J2_CX + prevX2, J2_CY + prevY2, 15, BG);
    
    // Redraw circles
    gfx->drawCircle(J1_CX, J1_CY, RADIUS, WHITE);
    gfx->drawCircle(J2_CX, J2_CY, RADIUS, WHITE);
    
    // JOY2 controls LEFT dot
    gfx->fillCircle(J1_CX + dx2, J1_CY + dy2, 10, btn2 ? RED : GREEN);
    // JOY1 controls RIGHT dot
    gfx->fillCircle(J2_CX + dx1, J2_CY + dy1, 10, btn1 ? RED : GREEN);
    
    // Save swapped positions
    prevX1 = dx2; prevY1 = dy2;  // Left dot uses JOY2
    prevX2 = dx1; prevY2 = dy1;  // Right dot uses JOY1
    
    // Update switches (unchanged)
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

/* ===================== TELEMETRY SCREEN ===================== */
// Global variables to track previous values
uint8_t prevBtn1 = 0, prevBtn2 = 0, prevBrake = 0;
float prevTemp1 = 0.0, prevTemp2 = 0.0;
float prevBat1 = 0.0, prevBat2 = 0.0, prevBatTotal = 0.0;
float prevSpeed = 0.0;

void drawTelemetryScreen() {
    // Background
    gfx->fillRect(0, 40, 480, 280, BG);
    
    // Static labels only - with smaller text to fit more lines
    gfx->setTextSize(2);
    gfx->setTextColor(LABEL_COLOR);
    
    int y = TELEM_Y + 30; // Adjusted starting Y
    
    // Left column labels
    gfx->setCursor(20, y);
    gfx->print("BTN1:");
    
    gfx->setCursor(20, y + LINE_HEIGHT);
    gfx->print("BTN2:");
    
    gfx->setCursor(20, y + LINE_HEIGHT * 2);
    gfx->print("BRAKE:");
    
    gfx->setCursor(20, y + LINE_HEIGHT * 3);
    gfx->print("TEMP1:");
    
    gfx->setCursor(20, y + LINE_HEIGHT * 4);
    gfx->print("TEMP2:");
    
    // Right column labels
    gfx->setCursor(250, y);
    gfx->print("BAT1:");
    
    gfx->setCursor(250, y + LINE_HEIGHT);
    gfx->print("BAT2:");
    
    gfx->setCursor(250, y + LINE_HEIGHT * 2);
    gfx->print("BAT TOT:");
    
    gfx->setCursor(250, y + LINE_HEIGHT * 3);
    gfx->print("SPEED:");
    
    // Initialize previous values
    prevBtn1 = receivedData.btn1;
    prevBtn2 = receivedData.btn2;
    prevBrake = receivedData.brake;
    prevTemp1 = receivedData.temp1;
    prevTemp2 = receivedData.temp2;
    prevBat1 = receivedData.bat1;
    prevBat2 = receivedData.bat2;
    prevBatTotal = receivedData.batTotal;
    prevSpeed = receivedData.speedKmh;
    
    // Draw initial values
    updateTelemetryScreen();
}

void updateTelemetryScreen() {
    if (!newDataAvailable) {
        return;
    }
    
    gfx->setTextSize(2);
    int y = TELEM_Y + 30;
    
    // Update values only if they changed
    
    // Button 1
    if (receivedData.btn1 != prevBtn1) {
        gfx->fillRect(100, y, 100, LINE_HEIGHT, BG);
        gfx->setTextColor(receivedData.btn1 ? GREEN : VALUE_COLOR);
        gfx->setCursor(100, y);
        gfx->print(receivedData.btn1 ? "ON" : "OFF");
        prevBtn1 = receivedData.btn1;
    }
    
    // Button 2
    if (receivedData.btn2 != prevBtn2) {
        gfx->fillRect(100, y + LINE_HEIGHT, 100, LINE_HEIGHT, BG);
        gfx->setTextColor(receivedData.btn2 ? GREEN : VALUE_COLOR);
        gfx->setCursor(100, y + LINE_HEIGHT);
        gfx->print(receivedData.btn2 ? "ON" : "OFF");
        prevBtn2 = receivedData.btn2;
    }
    
    // Brake
    if (receivedData.brake != prevBrake) {
        gfx->fillRect(100, y + LINE_HEIGHT * 2, 100, LINE_HEIGHT, BG);
        gfx->setTextColor(receivedData.brake ? RED : VALUE_COLOR);
        gfx->setCursor(100, y + LINE_HEIGHT * 2);
        gfx->print(receivedData.brake ? "ON" : "OFF");
        prevBrake = receivedData.brake;
    }
    
    // Temp1
    if (abs(receivedData.temp1 - prevTemp1) >= 0.1) {
        gfx->fillRect(100, y + LINE_HEIGHT * 3, 100, LINE_HEIGHT, BG);
        float temp1Value = receivedData.temp1;
        uint16_t temp1Color = (temp1Value > 35.0) ? ALARM_COLOR : 
                             (temp1Value > 30.0) ? WARNING_COLOR : VALUE_COLOR;
        gfx->setTextColor(temp1Color);
        gfx->setCursor(100, y + LINE_HEIGHT * 3);
        gfx->print(temp1Value, 1);
        gfx->print("C");
        prevTemp1 = temp1Value;
    }
    
    // Temp2
    if (abs(receivedData.temp2 - prevTemp2) >= 0.1) {
        gfx->fillRect(100, y + LINE_HEIGHT * 4, 100, LINE_HEIGHT, BG);
        float temp2Value = receivedData.temp2;
        uint16_t temp2Color = (temp2Value > 35.0) ? ALARM_COLOR : 
                             (temp2Value > 30.0) ? WARNING_COLOR : VALUE_COLOR;
        gfx->setTextColor(temp2Color);
        gfx->setCursor(100, y + LINE_HEIGHT * 4);
        gfx->print(temp2Value, 1);
        gfx->print("C");
        prevTemp2 = temp2Value;
    }
    
    // Right column updates
    
    // Bat1
    if (abs(receivedData.bat1 - prevBat1) >= 0.01) {
        gfx->fillRect(350, y, 120, LINE_HEIGHT, BG);
        float bat1Value = receivedData.bat1;
        uint16_t bat1Color = (bat1Value < 12.0 || bat1Value > 14.5) ? ALARM_COLOR :
                            (bat1Value < 12.5 || bat1Value > 14.0) ? WARNING_COLOR : VALUE_COLOR;
        gfx->setTextColor(bat1Color);
        gfx->setCursor(350, y);
        gfx->print(bat1Value, 1);
        gfx->print("V");
        prevBat1 = bat1Value;
    }
    
    // Bat2
    if (abs(receivedData.bat2 - prevBat2) >= 0.01) {
        gfx->fillRect(350, y + LINE_HEIGHT, 120, LINE_HEIGHT, BG);
        float bat2Value = receivedData.bat2;
        uint16_t bat2Color = (bat2Value < 12.0 || bat2Value > 14.5) ? ALARM_COLOR :
                            (bat2Value < 12.5 || bat2Value > 14.0) ? WARNING_COLOR : VALUE_COLOR;
        gfx->setTextColor(bat2Color);
        gfx->setCursor(350, y + LINE_HEIGHT);
        gfx->print(bat2Value, 1);
        gfx->print("V");
        prevBat2 = bat2Value;
    }
    
    // Bat Total
    if (abs(receivedData.batTotal - prevBatTotal) >= 0.01) {
        gfx->fillRect(350, y + LINE_HEIGHT * 2, 120, LINE_HEIGHT, BG);
        float batTotalValue = receivedData.batTotal;
        uint16_t batTotalColor = (batTotalValue < 22.0 || batTotalValue > 28.0) ? ALARM_COLOR :
                                (batTotalValue < 23.0 || batTotalValue > 26.0) ? WARNING_COLOR : VALUE_COLOR;
        gfx->setTextColor(batTotalColor);
        gfx->setCursor(350, y + LINE_HEIGHT * 2);
        gfx->print(batTotalValue, 1);
        gfx->print("V");
        prevBatTotal = batTotalValue;
    }
    
    // Speed
    if (abs(receivedData.speedKmh - prevSpeed) >= 0.1) {
        gfx->fillRect(350, y + LINE_HEIGHT * 3, 120, LINE_HEIGHT, BG);
        float speedValue = receivedData.speedKmh;
        gfx->setTextColor(VALUE_COLOR);
        gfx->setCursor(350, y + LINE_HEIGHT * 3);
        gfx->print(speedValue, 1);
        gfx->print("km/h");
        prevSpeed = speedValue;
    }
    
    newDataAvailable = false;
}

/* ===================== SCREEN 3: GRAPHS ===================== */
void drawGraphsScreen() {
    // Background
    gfx->fillRect(0, 40, 480, 280, BG);
    
    // Draw static graph backgrounds ONLY ONCE
    drawGraphBackground(GRAPH_X, GRAPH_Y, 210, 120, "TEMP1", GREEN);
    drawGraphBackground(GRAPH_X + 230, GRAPH_Y, 210, 120, "TEMP2", BLUE);
    drawGraphBackground(GRAPH_X, GRAPH_Y + 140, 210, 120, "BAT TOT", YELLOW);
    drawGraphBackground(GRAPH_X + 230, GRAPH_Y + 140, 210, 120, "SPEED", RED);
    
    graphsDrawn = true;
}

void drawGraphBackground(int x, int y, int w, int h, const char* title, uint16_t color) {
    // Border - draw once
    gfx->drawRect(x, y, w, h, WHITE);
    
    // Title - draw once
    gfx->setTextSize(1);
    gfx->setTextColor(color);
    gfx->setCursor(x + 5, y + 5);
    gfx->print(title);
    
    // Draw initial value placeholder
    gfx->setCursor(x + w - 50, y + 5);
    gfx->print("0.0");
}

void updateGraphsScreen() {
    static unsigned long lastGraphUpdate = 0;
    
    if (!newDataAvailable && millis() - lastGraphUpdate < 1000) {
        return;
    }
    
    if (newDataAvailable || millis() - lastGraphUpdate >= 1000) {
        lastGraphUpdate = millis();
        
        int graphW = 210 - 20;
        int graphH = 120 - 30;
        
        // Clear graph areas (lines only)
        gfx->fillRect(GRAPH_X + 10, GRAPH_Y + 10, graphW, graphH, BG);
        gfx->fillRect(GRAPH_X + 230 + 10, GRAPH_Y + 10, graphW, graphH, BG);
        gfx->fillRect(GRAPH_X + 10, GRAPH_Y + 140 + 10, graphW, graphH, BG);
        gfx->fillRect(GRAPH_X + 230 + 10, GRAPH_Y + 140 + 10, graphW, graphH, BG);
        
        if (historyIndex > 1) {
            drawGraphData(GRAPH_X, GRAPH_Y, 210, 120, temp1History, 20, 60, GREEN);
            drawGraphData(GRAPH_X + 230, GRAPH_Y, 210, 120, temp2History, 20, 60, BLUE);
            drawGraphData(GRAPH_X, GRAPH_Y + 140, 210, 120, voltHistory, 16, 28, YELLOW); // 24V system range
            drawGraphData(GRAPH_X + 230, GRAPH_Y + 140, 210, 120, speedHistory, 0, 50, RED); // Speed 0-50 km/h
        }
        
        gfx->setTextSize(1);
        
        // Temp1 value
        gfx->fillRect(GRAPH_X + 150, GRAPH_Y + 5, 40, 10, BG);
        gfx->setTextColor(GREEN);
        gfx->setCursor(GRAPH_X + 150, GRAPH_Y + 5);
        gfx->print(temp1History[MAX_GRAPH_POINTS - 1], 1);
        gfx->print("C");
        
        // Temp2 value
        gfx->fillRect(GRAPH_X + 380, GRAPH_Y + 5, 40, 10, BG);
        gfx->setTextColor(BLUE);
        gfx->setCursor(GRAPH_X + 380, GRAPH_Y + 5);
        gfx->print(temp2History[MAX_GRAPH_POINTS - 1], 1);
        gfx->print("C");
        
        // Bat Total value
        gfx->fillRect(GRAPH_X + 150, GRAPH_Y + 145, 40, 10, BG);
        gfx->setTextColor(YELLOW);
        gfx->setCursor(GRAPH_X + 150, GRAPH_Y + 145);
        gfx->print(voltHistory[MAX_GRAPH_POINTS - 1], 1);
        gfx->print("V");
        
        // Speed value
        gfx->fillRect(GRAPH_X + 380, GRAPH_Y + 145, 40, 10, BG);
        gfx->setTextColor(RED);
        gfx->setCursor(GRAPH_X + 380, GRAPH_Y + 145);
        gfx->print(speedHistory[MAX_GRAPH_POINTS - 1], 1);
        gfx->print("km/h");
        
        newDataAvailable = false;
    }
}

void drawGraphData(int x, int y, int w, int h, float* data, float minVal, float maxVal, uint16_t color) {
    int graphW = w - 20;
    int graphH = h - 30;
    
    for (int i = 1; i < MAX_GRAPH_POINTS; i++) {
        float val1 = data[i-1];
        float val2 = data[i];
        
        if (val1 < minVal - 10 || val1 > maxVal + 10 || val2 < minVal - 10 || val2 > maxVal + 10) {
            continue;
        }
        
        int x1 = x + 10 + (i-1) * graphW / MAX_GRAPH_POINTS;
        int y1 = y + h - 20 - map(constrain(val1, minVal, maxVal), minVal, maxVal, 0, graphH);
        
        int x2 = x + 10 + i * graphW / MAX_GRAPH_POINTS;
        int y2 = y + h - 20 - map(constrain(val2, minVal, maxVal), minVal, maxVal, 0, graphH);
        
        if (y1 >= y + 10 && y1 <= y + h - 20 && y2 >= y + 10 && y2 <= y + h - 20) {
            gfx->drawLine(x1, y1, x2, y2, color);
        }
    }
}

/* ===================== SCREEN 4: STATUS ===================== */
void drawStatusScreen() {
    // Background
    gfx->fillRect(0, 40, 480, 280, BG);
    
    // Border
    gfx->drawRoundRect(STATUS_X, STATUS_Y, STATUS_WIDTH, STATUS_HEIGHT, 10, CYAN);
    
    // Static labels
    gfx->setTextSize(1);
    gfx->setTextColor(LABEL_COLOR);
    
    int y = STATUS_Y + 50;
    int lineHeight = 25;
    
    gfx->setCursor(STATUS_X + 20, y);
    gfx->print("MAC:");
    
    gfx->setCursor(STATUS_X + 20, y + lineHeight);
    gfx->print("Signal:");
    
    gfx->setCursor(STATUS_X + 20, y + lineHeight * 2);
    gfx->print("Packets:");
    
    gfx->setCursor(STATUS_X + 20, y + lineHeight * 3);
    gfx->print("Errors:");
    
    gfx->setCursor(STATUS_X + 20, y + lineHeight * 4);
    gfx->print("Success:");
    
    gfx->setCursor(STATUS_X + 20, y + lineHeight * 5);
    gfx->print("Last pkt:");
    
    gfx->setCursor(STATUS_X + 20, y + lineHeight * 6);
    gfx->print("Battery:");
}

void updateStatusScreen() {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate < 1000) return;
    lastUpdate = millis();
    
    int y = STATUS_Y + 50;
    int lineHeight = 25;
    
    gfx->fillRect(STATUS_X + 80, y, 200, lineHeight * 7, BG);
    
    // MAC address
    gfx->setTextColor(VALUE_COLOR);
    gfx->setCursor(STATUS_X + 80, y);
    gfx->print(WiFi.macAddress());
    
    // RSSI
    int rssi = WiFi.RSSI();
    uint16_t rssiColor = (rssi > -60) ? GREEN : (rssi > -70) ? YELLOW : RED;
    gfx->setTextColor(rssiColor);
    gfx->setCursor(STATUS_X + 80, y + lineHeight);
    gfx->print(rssi);
    gfx->print(" dBm");
    
    // Packets
    gfx->setTextColor(VALUE_COLOR);
    gfx->setCursor(STATUS_X + 80, y + lineHeight * 2);
    gfx->print(receivedPacketCount);
    
    // Errors
    gfx->setTextColor(errorPacketCount > 0 ? RED : VALUE_COLOR);
    gfx->setCursor(STATUS_X + 80, y + lineHeight * 3);
    gfx->print(errorPacketCount);
    
    // Success rate
    if (receivedPacketCount + errorPacketCount > 0) {
        float successRate = (float)receivedPacketCount / (receivedPacketCount + errorPacketCount) * 100;
        uint16_t successColor = successRate > 95 ? GREEN : successRate > 80 ? YELLOW : RED;
        gfx->setTextColor(successColor);
        gfx->setCursor(STATUS_X + 80, y + lineHeight * 4);
        gfx->print(successRate, 1);
        gfx->print(" %");
    }
    
    // Time since last packet
    if (lastReceivedTime > 0) {
        unsigned long timeSince = (millis() - lastReceivedTime) / 1000;
        uint16_t timeColor = timeSince < 5 ? GREEN : timeSince < 10 ? YELLOW : RED;
        gfx->setTextColor(timeColor);
        gfx->setCursor(STATUS_X + 80, y + lineHeight * 5);
        gfx->print(timeSince);
        gfx->print(" s");
    } else {
        gfx->setTextColor(RED);
        gfx->setCursor(STATUS_X + 80, y + lineHeight * 5);
        gfx->print("none");
    }
    
    // Battery
    float batVoltage = readBatteryVoltage();
    uint16_t batColor = batVoltage > 12.0 ? GREEN : batVoltage > 11.5 ? YELLOW : RED;
    gfx->setTextColor(batColor);
    gfx->setCursor(STATUS_X + 80, y + lineHeight * 6);
    gfx->print(batVoltage, 2);
    gfx->print(" V");
}

/* ===================== SETUP ===================== */
void setup() {
    // Initialize display backlight
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);
    
    // Initialize inputs
    pinMode(JOY1_BTN, INPUT_PULLUP);
    pinMode(JOY2_BTN, INPUT_PULLUP);
    
    for (int i = 0; i < 4; i++) {
        pinMode(sw3pos[i][0], INPUT_PULLUP);
        pinMode(sw3pos[i][1], INPUT_PULLUP);
        pinMode(sw2pos[i][0], INPUT_PULLUP);
        pinMode(sw2pos[i][1], INPUT_PULLUP);
    }
    
    // Initialize display
    gfx->begin();
    
    // Define colors
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
    
    // Initialize Serial for debugging
    Serial.begin(115200);
    delay(1000);
    
    // Initialize WiFi and ESP-NOW
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while(1) delay(1000);
    }
    
    esp_now_register_recv_cb(OnDataRecv);
    
    Serial.println("ESP32-S3 Ready");
    
    // Initialize graph arrays with default values
    for (int i = 0; i < MAX_GRAPH_POINTS; i++) {
        temp1History[i] = 25.0;
        temp2History[i] = 25.0;
        voltHistory[i] = 24.0; // Default 24V
        speedHistory[i] = 0.0;
    }
    
    // Draw initial screen
    drawHeader();
    drawControlsScreen();
}

/* ===================== LOOP ===================== */
void loop() {
    // Check for screen switch
    checkScreenSwitch();
    
    // Draw header
    drawHeader();
    
    // Draw current screen if needed
    if (screenNeedsRedraw) {
        switch (currentScreen) {
            case SCREEN_CONTROLS:
                drawControlsScreen();
                break;
            case SCREEN_TELEMETRY:
                drawTelemetryScreen();
                break;
            case SCREEN_GRAPHS:
                drawGraphsScreen();
                break;
            case SCREEN_STATUS:
                drawStatusScreen();
                break;
        }
        screenNeedsRedraw = false;
    }
    
    // Update current screen
    switch (currentScreen) {
        case SCREEN_CONTROLS:
            updateControlsScreen();
            break;
        case SCREEN_TELEMETRY:
            updateTelemetryScreen();
            break;
        case SCREEN_GRAPHS:
            updateGraphsScreen();
            break;
        case SCREEN_STATUS:
            updateStatusScreen();
            break;
    }
    
    delay(20);
}