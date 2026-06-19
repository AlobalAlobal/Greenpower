#include <esp_now.h>
#include <WiFi.h>

// Receiver 1
uint8_t receiverMac[] = {0x68, 0xB6, 0xB3, 0x27, 0xBE, 0xAC};

// Receiver 2
uint8_t receiverMac2[] = {0xB8, 0xF8, 0x62, 0xFC, 0x08, 0x88};

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

sensor_data_full_t sensorData;

esp_now_peer_info_t peerInfo;
esp_now_peer_info_t peerInfo2;

float totalWh = 0.0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("ESP-NOW Dummy TX");

    WiFi.mode(WIFI_STA);

    Serial.print("My MAC: ");
    Serial.println(WiFi.macAddress());

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed!");
        while (1) delay(1000);
    }

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, receiverMac, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) == ESP_OK) {
        Serial.println("Peer 1 added");
    }

    memset(&peerInfo2, 0, sizeof(peerInfo2));
    memcpy(peerInfo2.peer_addr, receiverMac2, 6);
    peerInfo2.channel = 1;
    peerInfo2.encrypt = false;

    if (esp_now_add_peer(&peerInfo2) == ESP_OK) {
        Serial.println("Peer 2 added");
    }

    randomSeed(esp_random());
}

void loop() {

    // Buttons
    sensorData.btn1 = random(0, 2);
    sensorData.btn2 = random(0, 2);
    sensorData.brake = random(0, 2);

    // Temperatures
    sensorData.temp1 = random(200, 700) / 10.0;   // 20.0 - 70.0°C
    sensorData.temp2 = random(200, 700) / 10.0;

    // Battery voltages
    sensorData.bat1 = random(1050, 1350) / 100.0; // 10.5 - 13.5V
    sensorData.bat2 = random(1050, 1350) / 100.0;
    sensorData.batTotal = sensorData.bat1 + sensorData.bat2;

    // Throttle
    sensorData.throttle = random(0, 101);

    // Current depends somewhat on throttle
    sensorData.current = sensorData.throttle * 0.35f +
                         (random(-50, 51) / 10.0f);

    if (sensorData.current < 0)
        sensorData.current = 0;

    // RPM
    sensorData.rpm = sensorData.throttle * 80 +
                     random(-300, 301);

    if (sensorData.rpm < 0)
        sensorData.rpm = 0;

    // Speed
    sensorData.speedKmh = sensorData.rpm * 0.012f;

    // Timestamp
    sensorData.timestamp = millis();

    // Sensor voltage
    sensorData.vsens = sensorData.batTotal +
                       (random(-20, 21) / 100.0f);

    // Power
    sensorData.power = sensorData.vsens * sensorData.current;

    // Energy accumulation
    totalWh += sensorData.power * (0.1f / 3600.0f);
    sensorData.wh = totalWh;

    // Send to receiver 1
    esp_now_send(
        receiverMac,
        (uint8_t*)&sensorData,
        sizeof(sensorData)
    );

    // Send to receiver 2
    esp_now_send(
        receiverMac2,
        (uint8_t*)&sensorData,
        sizeof(sensorData)
    );

    Serial.printf(
        "BTN:%d %d BR:%d | T1:%.1f T2:%.1f | "
        "BAT:%.2f %.2f %.2f | "
        "CURR:%.1fA THR:%.0f%% | "
        "RPM:%d SPD:%.1f | "
        "V:%.2f P:%.1fW WH:%.3f\n",
        sensorData.btn1,
        sensorData.btn2,
        sensorData.brake,
        sensorData.temp1,
        sensorData.temp2,
        sensorData.bat1,
        sensorData.bat2,
        sensorData.batTotal,
        sensorData.current,
        sensorData.throttle,
        sensorData.rpm,
        sensorData.speedKmh,
        sensorData.vsens,
        sensorData.power,
        sensorData.wh
    );

    delay(100); // 10 Hz update rate
}