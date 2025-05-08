#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <SPIFFS.h>
#include "MPU6050Sensor.h"
#include "DroneController.h"

// WiFi AP credentials
const char* ssid = "DroneControlAP";
const char* password = "drone1234";

// Web server and WebSocket
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// Sensor and controller
MPU6050Sensor sensor(100.0f, 0.989f);
DroneController drone(100.0f);

// Control inputs
float rollSetpoint = 0.0f, pitchSetpoint = 0.0f, throttle = 0.0f, yawSetpoint = 0.0f;

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
        data[len] = 0;
        DynamicJsonDocument doc(512);
        deserializeJson(doc, data);

        // Parse joystick inputs
        if (doc.containsKey("throttle")) {
            throttle = doc["throttle"];
        }
        if (doc.containsKey("yaw")) {
            yawSetpoint = doc["yaw"];
        }
        if (doc.containsKey("roll")) {
            rollSetpoint = doc["roll"];
        }
        if (doc.containsKey("pitch")) {
            pitchSetpoint = doc["pitch"];
        }

        // Parse PID tunings
        if (doc.containsKey("kpRoll")) {
            float kpRoll = doc["kpRoll"];
            float kiRoll = doc["kiRoll"];
            float kdRoll = doc["kdRoll"];
            float kpPitch = doc["kpPitch"];
            float kiPitch = doc["kiPitch"];
            float kdPitch = doc["kdPitch"];
            drone.setPIDTunings(kpRoll, kiRoll, kdRoll, kpPitch, kiPitch, kdPitch);
        }

        // Parse motor speed slider
        if (doc.containsKey("motorSpeed")) {
            float speed = doc["motorSpeed"];
            float speeds[4] = {speed, speed, speed, speed};
            drone.setMotorSpeeds(speeds);
        }

        // Parse arm/disarm
        if (doc.containsKey("arm")) {
            if (doc["arm"]) {
                drone.arm();
            } else {
                drone.disarm();
            }
        }
    }
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_CONNECT) {
        Serial.println("WebSocket client connected");
    } else if (type == WS_EVT_DISCONNECT) {
        Serial.println("WebSocket client disconnected");
    } else if (type == WS_EVT_DATA) {
        handleWebSocketMessage(arg, data, len);
    }
}

void setup() {
    Serial.begin(115200);

    // Initialize SPIFFS
    if (!SPIFFS.begin(true)) {
        Serial.println("Failed to mount SPIFFS");
        while (1) {
            delay(1000);
        }
    }

    // Configure WiFi as Access Point
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP SSID: ");
    Serial.println(ssid);
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Initialize sensor and drone
    if (!sensor.begin() || !drone.begin()) {
        Serial.println("Initialization failed");
        while (1) {
            delay(1000);
        }
    }

    // Web server routes
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(SPIFFS, "/index.html", "text/html");
    });

    // WebSocket
    ws.onEvent(onWsEvent);
    server.addHandler(&ws);
    server.begin();
}

void loop() {
    static unsigned long lastWsUpdate = 0;
    float roll, pitch, yaw;

    // Control loop at 100 Hz
    if (sensor.getAngles(roll, pitch, yaw)) {
        drone.update(roll, pitch, yaw, rollSetpoint, pitchSetpoint, throttle, yawSetpoint);

        // Send data to WebSocket clients at 10 Hz
        if (millis() - lastWsUpdate >= 100) {
            float motorSpeeds[4];
            drone.getMotorSpeeds(motorSpeeds);

            DynamicJsonDocument doc(512);
            doc["roll"] = roll;
            doc["pitch"] = pitch;
            doc["yaw"] = yaw;
            doc["motor1"] = motorSpeeds[0];
            doc["motor2"] = motorSpeeds[1];
            doc["motor3"] = motorSpeeds[2];
            doc["motor4"] = motorSpeeds[3];
            doc["armed"] = drone.isArmed();

            String output;
            serializeJson(doc, output);
            ws.textAll(output);

            lastWsUpdate = millis();
        }
    }

    // Clean up WebSocket clients
    ws.cleanupClients();
}