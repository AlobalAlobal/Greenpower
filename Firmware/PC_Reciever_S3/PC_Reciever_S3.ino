#include <esp_now.h>
#include <WiFi.h>

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

volatile bool newData = false;
sensor_data_full_t data;

void onReceive(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    if (len != sizeof(data)) return;

    memcpy((void*)&data, incomingData, sizeof(data));
    newData = true;   // just flag it, DO NOT PRINT HERE
}

void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("S3 CSV Receiver");

    WiFi.mode(WIFI_STA);
    WiFi.setChannel(1);
    WiFi.setSleep(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW FAIL");
        while (1) delay(1000);
    }

    esp_now_register_recv_cb(onReceive);

    Serial.println("READY");
}

void loop() {
    if (newData) {
        newData = false;

        char line[256];

        snprintf(line, sizeof(line),
            "%d,%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%lu",
            data.btn1,
            data.btn2,
            data.brake,
            data.temp1,
            data.temp2,
            data.bat1,
            data.bat2,
            data.batTotal,
            data.current,
            data.throttle,
            data.rpm,
            data.speedKmh,
            data.timestamp
        );

        Serial.println(line);
    }

    delay(1); // keeps system happy
}