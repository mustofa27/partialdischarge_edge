#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ---- WiFi ---------------------------------------------------------------
static constexpr const char* WIFI_SSID     = "POLITEKNIK-NEGERI-MADURA";
static constexpr const char* WIFI_PASSWORD = "";

// ---- MQTT (TLS on port 8883) --------------------------------------------
static constexpr const char* MQTT_HOST      = "mqtt.icminovasi.my.id";
static constexpr uint16_t    MQTT_PORT      = 8883;
static constexpr const char* MQTT_USERNAME  = "partial_discharge";
static constexpr const char* MQTT_PASSWORD  = "PartialDischarge@2026";
static constexpr const char* MQTT_DEVICE_ID = "bnd-9bf3";
static constexpr const char* MQTT_TOPIC     = "partial_discharge/bnd-9bf3/pd_signal";

WiFiClientSecure espClient;
PubSubClient mqtt(espClient);

void connectWiFi() {
    Serial.println();
    Serial.print("Connecting to WiFi: ");
    Serial.println(WIFI_SSID);

    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println();
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}

void connectMQTT() {
    while (!mqtt.connected()) {
        Serial.print("Connecting to MQTT broker...");

        espClient.setInsecure();

        if (mqtt.connect(MQTT_DEVICE_ID, MQTT_USERNAME, MQTT_PASSWORD)) {
            Serial.println("connected");
            mqtt.subscribe(MQTT_TOPIC);
        } else {
            Serial.print("failed, rc=");
            Serial.print(mqtt.state());
            Serial.println(". retrying in 5 seconds");
            delay(5000);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX2=GPIO16, TX2=GPIO17

    Serial.println("ESP32 UART MQTT bridge started");
    connectWiFi();

    mqtt.setServer(MQTT_HOST, MQTT_PORT);
    mqtt.setBufferSize(512);
}

void loop() {
    if (WiFi.status() != WL_CONNECTED) {
        connectWiFi();
    }

    if (!mqtt.connected()) {
        connectMQTT();
    }

    mqtt.loop();

    while (Serial2.available() > 0) {
        String line = Serial2.readStringUntil('\n');
        line.trim();

        if (line.length() == 0) {
            continue;
        }

        Serial.print("STM32 -> ESP32: ");
        Serial.println(line);

        if (mqtt.publish(MQTT_TOPIC, line.c_str())) {
            Serial.println("MQTT publish ok");
        } else {
            Serial.println("MQTT publish failed");
        }
    }

    delay(20);
}
