#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

// MAC adresa ESP32-S3
uint8_t receiverMac[] = {0x68, 0xB6, 0xB3, 0x27, 0x2D, 0x00};

// Štruktúra dát - VŠETKY dáta z Arduina
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

sensor_data_full_t sensorData;
esp_now_peer_info_t peerInfo;
bool peerAdded = false;

// Serial na komunikáciu s Arduino Nano
HardwareSerial SerialNano(0);  // UART0: RX=GPIO20, TX=GPIO21

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("ESP32-C3 Bridge - Full Data Mode");
    
    // Komunikácia s Arduino Nano
    SerialNano.begin(9600, SERIAL_8N1, 20, 21);
    
    // WiFi
    WiFi.mode(WIFI_STA);
    
    Serial.print("My MAC: ");
    Serial.println(WiFi.macAddress());
    
    // ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed!");
        while(1) delay(1000);
    }
    
    // Pridaj peer
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Peer added successfully");
        peerAdded = true;
    } else {
        Serial.println("Failed to add peer");
    }
    
    Serial.println("Waiting for Arduino data...");
}

void loop() {
    static String buffer = "";
    
    // Čítaj z Arduina
    while (SerialNano.available()) {
        char c = SerialNano.read();
        
        if (c == '\n') {
            if (buffer.length() > 0) {
                processData(buffer);
                buffer = "";
            }
        } else if (c != '\r') {
            buffer += c;
        }
    }
    
    delay(10);
}

// FIXED VERSION - Updated to parse 13 values (12 commas)
void processData(String data) {
    // Formát: B1,B2,BRK,TEMP1,TEMP2,BAT1,BAT2,BAT_TOTAL,CURRENT,THROTTLE,RPM,SPEED_KMH,TIMESTAMP
    // 13 hodnot, 12 čiarok
    // Príklad: "1,0,1,25.5,26.3,12.45,12.35,24.80,5.2,75,1200,15.5,1234567"
    
    // Rozdel data podľa čiarok
    int commas[12]; // Zmenené z 13 na 12 (13 hodnôt potrebuje 12 čiarok)
    int commaCount = 0;
    
    // Nájdeme všetky čiarky
    for (int i = 0; i < data.length(); i++) {
        if (data.charAt(i) == ',') {
            commas[commaCount] = i;
            commaCount++;
            if (commaCount >= 12) break; // Zmenené z 13 na 12
        }
    }
    
    if (commaCount != 12) {  // Zmenené z 13 na 12
        Serial.print("Invalid data format (expected 12 commas, got ");
        Serial.print(commaCount);
        Serial.print("): ");
        Serial.println(data);
        return;
    }
    
    // Parsuj všetky hodnoty
    int startIndex = 0;
    
    // Buttons (0-2)
    sensorData.btn1 = data.substring(startIndex, commas[0]).toInt();
    startIndex = commas[0] + 1;
    sensorData.btn2 = data.substring(startIndex, commas[1]).toInt();
    startIndex = commas[1] + 1;
    sensorData.brake = data.substring(startIndex, commas[2]).toInt();
    startIndex = commas[2] + 1;
    
    // Temperatures (3-4)
    sensorData.temp1 = data.substring(startIndex, commas[3]).toFloat();
    startIndex = commas[3] + 1;
    sensorData.temp2 = data.substring(startIndex, commas[4]).toFloat();
    startIndex = commas[4] + 1;
    
    // Batteries (5-7)
    sensorData.bat1 = data.substring(startIndex, commas[5]).toFloat();
    startIndex = commas[5] + 1;
    sensorData.bat2 = data.substring(startIndex, commas[6]).toFloat();
    startIndex = commas[6] + 1;
    sensorData.batTotal = data.substring(startIndex, commas[7]).toFloat();
    startIndex = commas[7] + 1;
    
    // Current and throttle (8-9)
    sensorData.current = data.substring(startIndex, commas[8]).toFloat();
    startIndex = commas[8] + 1;
    sensorData.throttle = data.substring(startIndex, commas[9]).toFloat();
    startIndex = commas[9] + 1;
    
    // RPM and speed (10-11)
    sensorData.rpm = data.substring(startIndex, commas[10]).toInt();
    startIndex = commas[10] + 1;
    sensorData.speedKmh = data.substring(startIndex, commas[11]).toFloat();
    startIndex = commas[11] + 1;
    
    // Timestamp (12) - posledná hodnota, žiadna čiarka za ňou
    sensorData.timestamp = data.substring(startIndex).toInt();
    
    // Debug výpis
    Serial.print("Recv: ");
    Serial.print("B1="); Serial.print(sensorData.btn1);
    Serial.print(" B2="); Serial.print(sensorData.btn2);
    Serial.print(" BRK="); Serial.print(sensorData.brake);
    Serial.print(" T1="); Serial.print(sensorData.temp1, 1);
    Serial.print(" T2="); Serial.print(sensorData.temp2, 1);
    Serial.print(" B1="); Serial.print(sensorData.bat1, 2);
    Serial.print(" B2="); Serial.print(sensorData.bat2, 2);
    Serial.print(" BT="); Serial.print(sensorData.batTotal, 2);
    Serial.print(" A="); Serial.print(sensorData.current, 1);
    Serial.print(" THR="); Serial.print(sensorData.throttle, 0);
    Serial.print(" RPM="); Serial.print(sensorData.rpm);
    Serial.print(" SPD="); Serial.print(sensorData.speedKmh, 1);
    Serial.print(" TS="); Serial.print(sensorData.timestamp);
    Serial.println();
    
    // Odošli cez ESP-NOW
    sendData();
}

void sendData() {
    if (!peerAdded) {
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            peerAdded = true;
            Serial.println("Peer re-added");
        } else {
            Serial.println("Cannot send - peer not added");
            return;
        }
    }
    
    esp_err_t result = esp_now_send(receiverMac, (uint8_t *) &sensorData, sizeof(sensorData));
    
    if (result == ESP_OK) {
        Serial.println("Sent to ESP32-S3");
    } else {
        Serial.print("Send error: ");
        Serial.println(result);
        peerAdded = false;
    }
}