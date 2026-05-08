#include <esp_now.h>
#include <WiFi.h>
#include <HardwareSerial.h>

// Receiver 1 (existing)
uint8_t receiverMac[] = {0x68, 0xB6, 0xB3, 0x27, 0x2D, 0x00};

// Receiver 2 (NEW S3)
uint8_t receiverMac2[] = {0xB8, 0xF8, 0x62, 0xFC, 0x08, 0x88};

// Data structure
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

    Serial.println("ESP32-C3 Bridge - Full Data Mode");

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

    Serial.println("Waiting for Arduino data...");
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
    int commas[12];
    int commaCount = 0;

    for (int i = 0; i < data.length(); i++) {
        if (data.charAt(i) == ',') {
            commas[commaCount++] = i;
            if (commaCount >= 12) break;
        }
    }

    if (commaCount != 12) {
        Serial.println("Invalid data");
        return;
    }

    int i = 0;

    sensorData.btn1 = data.substring(i, commas[0]).toInt(); i = commas[0]+1;
    sensorData.btn2 = data.substring(i, commas[1]).toInt(); i = commas[1]+1;
    sensorData.brake = data.substring(i, commas[2]).toInt(); i = commas[2]+1;

    sensorData.temp1 = data.substring(i, commas[3]).toFloat(); i = commas[3]+1;
    sensorData.temp2 = data.substring(i, commas[4]).toFloat(); i = commas[4]+1;

    sensorData.bat1 = data.substring(i, commas[5]).toFloat(); i = commas[5]+1;
    sensorData.bat2 = data.substring(i, commas[6]).toFloat(); i = commas[6]+1;
    sensorData.batTotal = data.substring(i, commas[7]).toFloat(); i = commas[7]+1;

    sensorData.current = data.substring(i, commas[8]).toFloat(); i = commas[8]+1;
    sensorData.throttle = data.substring(i, commas[9]).toFloat(); i = commas[9]+1;

    sensorData.rpm = data.substring(i, commas[10]).toInt(); i = commas[10]+1;
    sensorData.speedKmh = data.substring(i, commas[11]).toFloat(); i = commas[11]+1;

    sensorData.timestamp = data.substring(i).toInt();

    sendData();
}

void sendData() {
    if (peerAdded) {
        esp_now_send(receiverMac, (uint8_t*)&sensorData, sizeof(sensorData));
    }

    if (peerAdded2) {
        esp_now_send(receiverMac2, (uint8_t*)&sensorData, sizeof(sensorData));
    }

    Serial.println("Sent to both receivers");
}