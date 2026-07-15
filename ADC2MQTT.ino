/**
 * M5StickC Plus2 - Precision Voltage Monitor (MQTT/JSON)
 * -------------------------------------------------------
 * @author    kuni
 * @date      2026-05-10
 * @license   MIT License (c)
 * Description:
 * This program measures analog voltage via GPIO 36, displays it with 4-decimal
 * precision on the built-in LCD, and publishes the data to an MQTT broker 
 * in JSON format. It also subscribes to a topic to receive a JSON payload
 * and triggers a brake flag when v_mps is near zero.
 */

#include <M5Unified.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>  // Supports ArduinoJson V7
#include <math.h>         // 絶対値計算（fabs）に必要

// ==========================================
// Configuration Area (Directly defined)
// ==========================================
const char* ssid = "RMS-SKIP-AAA09";
const char* password = "11379959";
const char* mqtt_server = "192.168.212.1";

IPAddress local_IP(192, 168, 212, 80);   // M5StickC Plus2に割り当てたい固定IP
IPAddress gateway(192, 168, 212, 1);     // ルーター（ゲートウェイ）のIP
IPAddress subnet(255, 255, 255, 0);      // サブネットマスク
IPAddress primaryDNS(192, 168, 212, 1);  // プライマリDNS（通常はルーターと同じでOK）

const int mqtt_port = 1883;
const char* mqtt_user = "mqtt";
const char* mqtt_pass = "sI7G@DijuY";
const char* client_id = "M5StickCPlus2_ADC";
const char* mqtt_topic = "0/WHISPERER/RMS-SKIP-AAA09/battery";

const char* sub_topic = "0/WHISPERER/RMS-SKIP-AAA09/vel2D_DWO"; // 受信したいトピック
bool brakeFlag = false; // 受信した条件（|v_mps| < 0.002）を満たすかのフラグ

// デバッグ・受信用グローバル変数
String rxPayloadStr = "Waiting..."; 
String debugTopic = "None";
int debugLen = 0;
int rxCount = 0;

unsigned long update_interval = 1000;   // Interval for MQTT publishing (ms)
unsigned long sampling_interval = 100;  // Interval for ADC and display refresh (ms)
// ==========================================

const int analogPin = 36;
WiFiClient espClient;
PubSubClient client(espClient);

unsigned long lastMsgTime = 0;
unsigned long lastSampleTime = 0;
float currentVoltage = 0.0;
float VoltageRatio = 10.77;
float batteryLevel = 99.00;
float filteredBattery = 99.00;
const float ALPHA = 0.01f;

char buf[8];

// MQTTメッセージ受信時のコールバック関数
void callback(char* topic, byte* payload, unsigned int length) {
  rxCount++;
  debugTopic = String(topic);
  debugLen = length;

  // 届いたバイト配列を文字列（String）に変換して保存
  rxPayloadStr = "";
  for (int i = 0; i < length; i++) {
    rxPayloadStr += (char)payload[i];
  }

  // 特定のトピック宛てか確認
  if (String(topic) == String(sub_topic)) {
    JsonDocument inDoc;
    DeserializationError error = deserializeJson(inDoc, payload, length);

    if (error) {
      rxPayloadStr = "JSON Err: " + rxPayloadStr;
      return;
    }

    // JSON内に "v_mps" というキーが存在するか確認
    if (inDoc.containsKey("v_mps")) {
      float v_mps = inDoc["v_mps"].as<float>();

      // 絶対値が 0.01 未満かどうかを判定
      if (fabs(v_mps) < 0.01f) {
        brakeFlag = true;
      } else {
        brakeFlag = false;
      }
    }
  }
}

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
      
      // 接続成功後に対象のトピックを購読（Subscribe）
      client.subscribe(sub_topic);
    } else {
      delay(5000);
    }
  }
}

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);
  // USBシリアルモニタ通信開始まで最大1.5秒待つ（シリアルデバッグの頭欠け防止）
  unsigned long startWait = millis();
  while (!Serial && (millis() - startWait < 1500)) {
    delay(10);
  }

  M5.Lcd.setRotation(1);  // Landscape mode
  M5.Lcd.fillScreen(BLACK);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback); // コールバック関数の登録
  
  pinMode(analogPin, ANALOG);
  Serial.println("--- Setup Completed ---");
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
    currentVoltage = round(100 * VoltageRatio * (sum / 20.0 * 3.3) / 4095) / 100;
    batteryLevel = currentVoltage * 59.63 - 1508;
    batteryLevel = constrain(batteryLevel, 0, 99);

    if (brakeFlag) {
      filteredBattery = (filteredBattery * (1.0f - ALPHA)) + ((float)batteryLevel * ALPHA);
    } 
    
    // Rendering to screen
    M5.Lcd.startWrite();

    // Display SSID
    M5.Lcd.setFont(&fonts::Font0);
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setCursor(5, 5);
    M5.Lcd.printf("SSID:%s", WiFi.SSID().c_str());

    // Display MQTT Topic
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.setCursor(5, 15);
    M5.Lcd.printf("Topic: %s", mqtt_topic);

    // デバッグ用：MQTTの受信パケット情報を画面に表示
    M5.Lcd.setFont(&fonts::Font0);
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setTextColor(TFT_PINK, TFT_BLACK);
    M5.Lcd.setCursor(5, 25);
    //M5.Lcd.printf("RX Cnt:%d  Len:%d", rxCount, debugLen);

    // BRAKEの状態を描画
    if (brakeFlag) {
      M5.Lcd.setTextColor(TFT_DARKGRAY, WHITE);
      M5.Lcd.drawString("BRAKE: ON ", 150, 25); 
    } else {
      M5.Lcd.setTextColor(TFT_DARKGRAY, RED);
      M5.Lcd.drawString("BRAKE: OFF", 150, 25);
    }

    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setTextSize(5);
    M5.Lcd.setCursor(125, 80);
    M5.Lcd.printf("V");
    M5.Lcd.setCursor(210, 80);
    M5.Lcd.print("%");

    // Display Voltage Value (Large Font 7)
    M5.Display.setFont(&fonts::Font7);
    M5.Lcd.setTextColor(YELLOW, BLACK);
    M5.Display.setTextSize(0.9, 1.6);
    M5.Display.drawFloat(currentVoltage, 2, 2, 40);

    // 画面のバッテリー残量表示
    sprintf(buf, "%2d", (int)(filteredBattery + 0.5f));
    M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
    M5.Display.drawString(buf, 152, 40);

    // 届いた生メッセージ（最大25文字）を画面の最下部に表示
    M5.Lcd.setFont(&fonts::Font0);
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.setCursor(5, 115);
    //M5.Lcd.printf("Payload: %-25s", rxPayloadStr.substring(0, 25).c_str());

    // --- 【新規追加】画面最下部のバッテリーバー描画処理 ---
    int barMaxW = 240;                              // バーの最大横幅（ピクセル）
    int barX = 5;                                   // 開始X座標
    int barY = 120;                                 // 開始Y座標
    int barH = 20;                                   // バーの厚み（高さ）
    int barW = (int)(barMaxW * (filteredBattery / 99.0f)); // 残量に応じた長さ計算
    barW = constrain(barW, 0, barMaxW);             // 範囲を安全に制限

    // 残量に応じた色判定 (左30%は赤、中央は黄、右30%は緑)
    uint16_t barColor = TFT_GREEN;
    if (filteredBattery <= 30.0f) {
      barColor = TFT_RED;
    } else if (filteredBattery <= 70.0f) {
      barColor = TFT_YELLOW;
    }

    // バーの描画（現在の残量分を色付け、減った残像分を黒で消去）
    if (barW > 0) {
      M5.Lcd.fillRect(barX, barY, barW, barH, barColor);
    }
    if (barW < barMaxW) {
      M5.Lcd.fillRect(barX + barW, barY, barMaxW - barW, barH, TFT_BLACK);
    }
    // -----------------------------------------------------

    M5.Lcd.endWrite();
    M5.Display.endWrite();
    
    // シリアルへの電圧デバッグ出力
    Serial.printf("Current Voltage: %.2f V\n", currentVoltage);
  }

  // --- 2. MQTT Publishing (JSON Payload) ---
  if (now - lastMsgTime > update_interval) {
    lastMsgTime = now;

    // 10段階に切り上げる計算（例：81.3% -> 90%）
    int leveledBattery = (int)(ceil(filteredBattery / 10.0f)) ;
    
    // 安全用リミッター
    if (leveledBattery < 0) {
      leveledBattery = 0;
    } else if (leveledBattery > 100) {
      leveledBattery = 100;
    }

    // Creating JSON Payload (ArduinoJson V7)
    JsonDocument outDoc;
    outDoc["voltage"] = currentVoltage;
    outDoc["batteryLevel"] = leveledBattery; 

    String jsonString;
    serializeJson(outDoc, jsonString);

    // Publish JSON data
    client.publish(mqtt_topic, jsonString.c_str());
  }
}