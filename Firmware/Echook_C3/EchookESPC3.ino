#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

// Receiver 1 (existing)
uint8_t receiverMac[] = {0x68, 0xB6, 0xB3, 0x27, 0x2C, 0xD8};

// Receiver 2 (NEW S3)
uint8_t receiverMac2[] = {0xB8, 0xF8, 0x62, 0xFC, 0x08, 0x88};

// Data structure (UPDATED with vsens, power, and wh)
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
    
    float vsens;      // Voltage from I2C sensor
    float power;      // Power in watts (vsens * current)
    float wh;         // Watt-hours (accumulated energy)
} sensor_data_full_t;

sensor_data_full_t sensorData;

esp_now_peer_info_t peerInfo;
esp_now_peer_info_t peerInfo2;

bool peerAdded = false;
bool peerAdded2 = false;

// UART from Nano
HardwareSerial SerialNano(0);

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("ESP32-C3 Bridge - Full Data Mode (16 fields)");

    SerialNano.begin(9600, SERIAL_8N1, 20, 21);

    WiFi.mode(WIFI_STA);

    Serial.print("My MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed!");
        while(1) delay(1000);
    }

    // Peer 1
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Peer 1 added");
        peerAdded = true;
    }

    // Peer 2
    memcpy(peerInfo2.peer_addr, receiverMac2, 6);
    peerInfo2.channel = 1;
    peerInfo2.encrypt = false;

    if (esp_now_add_peer(&peerInfo2) == ESP_OK) {
        Serial.println("Peer 2 added");
        peerAdded2 = true;
    }

    Serial.println("Waiting for Arduino data (16 fields expected)...");
}

void loop() {
    static String buffer = "";

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

void processData(String data) {
    int commas[15];  // 15 commas for 16 fields
    int commaCount = 0;

    for (int i = 0; i < data.length(); i++) {
        if (data.charAt(i) == ',') {
            commas[commaCount++] = i;
            if (commaCount >= 15) break;
        }
    }

    if (commaCount != 15) {
        Serial.print("Invalid data - expected 15 commas, got ");
        Serial.println(commaCount);
        Serial.print("Data: ");
        Serial.println(data);
        return;
    }

    int i = 0;

    // Field 1: btn1
    sensorData.btn1 = data.substring(i, commas[0]).toInt(); i = commas[0]+1;
    // Field 2: btn2
    sensorData.btn2 = data.substring(i, commas[1]).toInt(); i = commas[1]+1;
    // Field 3: brake
    sensorData.brake = data.substring(i, commas[2]).toInt(); i = commas[2]+1;
    // Field 4: temp1
    sensorData.temp1 = data.substring(i, commas[3]).toFloat(); i = commas[3]+1;
    // Field 5: temp2
    sensorData.temp2 = data.substring(i, commas[4]).toFloat(); i = commas[4]+1;
    // Field 6: bat1
    sensorData.bat1 = data.substring(i, commas[5]).toFloat(); i = commas[5]+1;
    // Field 7: bat2
    sensorData.bat2 = data.substring(i, commas[6]).toFloat(); i = commas[6]+1;
    // Field 8: batTotal
    sensorData.batTotal = data.substring(i, commas[7]).toFloat(); i = commas[7]+1;
    // Field 9: current
    sensorData.current = data.substring(i, commas[8]).toFloat(); i = commas[8]+1;
    // Field 10: throttle
    sensorData.throttle = data.substring(i, commas[9]).toFloat(); i = commas[9]+1;
    // Field 11: RPM
    sensorData.rpm = data.substring(i, commas[10]).toInt(); i = commas[10]+1;
    // Field 12: speedKmh
    sensorData.speedKmh = data.substring(i, commas[11]).toFloat(); i = commas[11]+1;
    // Field 13: timestamp
    sensorData.timestamp = data.substring(i, commas[12]).toInt(); i = commas[12]+1;
    // Field 14: vsens
    sensorData.vsens = data.substring(i, commas[13]).toFloat(); i = commas[13]+1;
    // Field 15: power
    sensorData.power = data.substring(i, commas[14]).toFloat(); i = commas[14]+1;
    // Field 16: wh
    sensorData.wh = data.substring(i).toFloat();

    // Print for debugging
    Serial.printf("Received: btn1=%d, btn2=%d, brake=%d, t1=%.1f, t2=%.1f, bat1=%.2f, bat2=%.2f, batTot=%.2f, curr=%.2fA, thr=%.0f%%, rpm=%d, spd=%.1f, ts=%lu, Vsens=%.2fV, Pwr=%.1fW, Wh=%.3f\n",
                  sensorData.btn1, sensorData.btn2, sensorData.brake,
                  sensorData.temp1, sensorData.temp2,
                  sensorData.bat1, sensorData.bat2, sensorData.batTotal,
                  sensorData.current, sensorData.throttle,
                  sensorData.rpm, sensorData.speedKmh,
                  sensorData.timestamp, sensorData.vsens, sensorData.power, sensorData.wh);

    sendData();
}

void sendData() {
    if (peerAdded) {
        esp_err_t result = esp_now_send(receiverMac, (uint8_t*)&sensorData, sizeof(sensorData));
        if (result != ESP_OK) {
            Serial.println("Send to Peer 1 failed");
        }
    }

    if (peerAdded2) {
        esp_err_t result = esp_now_send(receiverMac2, (uint8_t*)&sensorData, sizeof(sensorData));
        if (result != ESP_OK) {
            Serial.println("Send to Peer 2 failed");
        }
    }

    Serial.println("Sent to both receivers");
}