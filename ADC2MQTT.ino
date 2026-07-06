/**
 * M5StickC Plus2 - Precision Voltage Monitor (MQTT/JSON)
 * -------------------------------------------------------
 * @author  kuni
 * @date    2026-05-10
 * @license MIT License (c) 2026 Hiroyasu Kuniyoshi
 * Description:
 * This program measures analog voltage via GPIO 36, displays it with 4-decimal
 * precision on the built-in LCD, and publishes the data to an MQTT broker 
 * in JSON format.
 * 
 * Features:
 * - Independent intervals for sampling/display and MQTT publishing.
 * - LCD layout optimized for visibility with SSID and Topic information.
 * - Uses M5Unified for hardware abstraction and ArduinoJson V7 for data serialization.
 */

#include <M5Unified.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>  // Supports ArduinoJson V7

// ==========================================
// Configuration Area (Directly defined)
// ==========================================
const char* ssid = "RMS-10X-N02";
const char* password = "28093305";
const char* mqtt_server = "192.168.212.1";

IPAddress local_IP(192, 168, 212, 80); // M5StickC Plus2に割り当てたい固定IP
IPAddress gateway(192, 168, 212, 1);    // ルーター（ゲートウェイ）のIP
IPAddress subnet(255, 255, 255, 0);   // サブネットマスク
IPAddress primaryDNS(192, 168, 212, 1); // プライマリDNS（通常はルーターと同じでOK）

const int mqtt_port = 1883;
const char* mqtt_user = "mqtt";
const char* mqtt_pass = "sI7G@DijuY";
const char* client_id = "M5StickCPlus2_ADC";
const char* mqtt_topic = "0/WHISPERER/RMS-1000-N07/battery";

unsigned long update_interval = 2000;   // Interval for MQTT publishing (ms)
unsigned long sampling_interval = 100;  // Interval for ADC and display refresh (ms)
// ==========================================

const int analogPin = 36;
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
unsigned long lastSampleTime = 0;
float currentVoltage = 0.0;
float VoltageRatio = 10.77;

// Function to connect to WiFi
void setup_wifi() {
  M5.Lcd.print("WiFi Connecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    M5.Lcd.print(".");
  }
  M5.Lcd.println("\nConnected!");
  delay(800);
  M5.Lcd.fillScreen(BLACK);
}

// Function to handle MQTT reconnection
void reconnect() {
  while (!client.connected()) {
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setFont(&fonts::FreeSansBold9pt7b);
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.print("MQTT Reconnecting...");

    if (client.connect(client_id, mqtt_user, mqtt_pass)) {
      M5.Lcd.fillScreen(BLACK);
    } else {
      delay(5000);
    }
  }
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Lcd.setRotation(1);  // Landscape mode
  M5.Lcd.fillScreen(BLACK);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  pinMode(analogPin, ANALOG);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long now = millis();

  // --- 1. ADC and Display Refresh (based on sampling_interval) ---
  if (now - lastSampleTime > sampling_interval) {
    lastSampleTime = now;

    // Measure voltage with averaging (20 samples)
    long sum = 0;
    for (int i = 0; i < 20; i++) {
      sum += analogRead(analogPin);
      delay(1);
    }
    currentVoltage = VoltageRatio * (sum / 20.0 * 3.3) / 4095.0;

    // Rendering to screen
    M5.Lcd.startWrite();

    // Display SSID
    M5.Lcd.setFont(&fonts::Font0);
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setCursor(5, 5);
    M5.Lcd.printf("SSID: %s", WiFi.SSID().c_str());

    // Display MQTT Topic
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(5, 15);
    M5.Lcd.printf("Topic: %s", mqtt_topic);

    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(125, 80);
    M5.Lcd.printf("V");
    M5.Lcd.setCursor(210, 80);
    M5.Lcd.print("%");

    // Display Voltage Value (Large Font 7)
    //M5.Lcd.setFont(&fonts::DSEG7);
    M5.Display.setFont(&fonts::Font7);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Display.setTextSize(0.9, 1.6);
    M5.Display.drawFloat(currentVoltage, 2, 2, 40);
    M5.Display.drawFloat(currentVoltage, 0, 152, 40);

    M5.Lcd.endWrite();
    M5.Display.endWrite();
  }

  // --- 2. MQTT Publishing (JSON Payload) ---
  if (now - lastMsgTime > update_interval) {
    lastMsgTime = now;

    // Creating JSON Payload (ArduinoJson V7)
    JsonDocument outDoc;
    outDoc["voltage"] = currentVoltage;

    String jsonString;
    serializeJson(outDoc, jsonString);

    // Publish JSON data
    client.publish(mqtt_topic, jsonString.c_str());
  }
}