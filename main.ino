#include <Adafruit_BMP085_U.h>
#include <Adafruit_Sensor.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>

const char *ssid = "";         // Wifi net name
const char *password = "";     // Wifi net password
const char *mqtt_server = "";  // mqtt server ip
const int mqtt_port = 1883;
const char *mqtt_user = "";      // mqtt user name
const char *mqtt_password = "";  // mqtt user password

const int SENSOR_1_PIN = 34;
const int SENSOR_2_PIN = 35;
const int SENSOR_3_PIN = 36;

const int RELAY_PIN = 12;
const int SERVO_PIN = 13;

Servo myservo;

Adafruit_BMP085_Unified bmp;

WiFiClient espClient;
PubSubClient client(espClient);

unsigned long previousMillis = 0;
const long interval = 3000;

void setup() {
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(callback);

    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);

    myservo.attach(SERVO_PIN);

    if (!bmp.begin()) {
        Serial.println("Could not find a valid BMP180 sensor, check wiring!");
        while (1)
            ;
    }
}

void setup_wifi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.println("Connecting to WiFi...");
        delay(1000);
    }
    Serial.println("Connected to WiFi");
}

void callback(char *topic, byte *payload, unsigned int length) {
    String topicStr = String(topic);

    if (topicStr.equals("relay/control")) {
        String message = "";
        for (int i = 0; i < length; i++) {
            message += (char)payload[i];
        }
        message.trim();

        Serial.print("Received message on topic 'relay/control': ");
        Serial.println(message);

        if (message.equals("on")) {
            digitalWrite(RELAY_PIN, HIGH);
            Serial.println("Relay turned on");
        } else if (message.equals("off")) {
            digitalWrite(RELAY_PIN, LOW);
            Serial.println("Relay turned off");
        } else {
            Serial.println("Invalid command");
        }
    } else if (topicStr.equals("servo/control")) {
        String message = "";
        for (int i = 0; i < length; i++) {
            message += (char)payload[i];
        }
        message.trim();

        Serial.print("Received message on topic 'servo/control': ");
        Serial.println(message);

        int percent = message.toInt();

        if (percent >= 0 && percent <= 100) {
            int angle = map(percent, 0, 100, 0, 90);
            myservo.write(angle);
            Serial.print("Servo set to ");
            Serial.print(percent);
            Serial.println("%");
        } else {
            Serial.println(
                "Invalid percentage. Percentage should be between 0 and 100.");
        }
    }
}

void reconnect() {
    while (!client.connected()) {
        Serial.println("Connecting to MQTT Broker...");
        if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
            Serial.println("Connected to MQTT Broker");
            client.subscribe("relay/control");
            client.subscribe("servo/control");
        } else {
            Serial.print("Failed, rc=");
            Serial.print(client.state());
            Serial.println(" Try again in 5 seconds");
            delay(5000);
        }
    }
}

void sendSensorData() {
    int sensor1_value = analogRead(SENSOR_1_PIN);
    int sensor2_value = analogRead(SENSOR_2_PIN);
    int sensor3_value = analogRead(SENSOR_3_PIN);

    int percent1 = map(sensor1_value, 2750, 1175, 0, 100);
    int percent2 = map(sensor2_value, 2750, 1175, 0, 100);
    int percent3 = map(sensor3_value, 2750, 1175, 0, 100);

    sensors_event_t event;
    bmp.getEvent(&event);

    if (event.pressure) {
        float temperature;
        bmp.getTemperature(&temperature);

        float pressure_mmHg = event.pressure * 0.75006375541921;

        char payload[200];
        snprintf(payload, sizeof(payload),
                 "{\"sensor1\": %d, \"sensor2\": %d, \"sensor3\": %d, "
                 "\"temperature\": %.2f, \"pressure\": %.2f}",
                 percent1, percent2, percent3, temperature, pressure_mmHg);
        client.publish("sensor_data", payload);
    } else {
        Serial.println("Error reading BMP180 sensor.");
    }
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }

    client.loop();

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        sendSensorData();
    }
}