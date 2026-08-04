/**
 * M5StickC Plus2 - 24V LiFePO4 Battery Monitor (MQTT/JSON)
 *
 * Setup AP:
 *   SSID     : M5StickSetup
 *   Password : none
 *   IP       : 192.168.212.80
 *
 * Setup mode starts when:
 *   1. BtnA is held while powering on
 *   2. WiFi connection fails for 2 minutes
 *   3. MQTT connection fails for 2 minutes
 *   4. WiFi/MQTT is lost during operation and cannot reconnect for 2 minutes
 *
 * Setup page:
 *   http://192.168.212.80/
 *
 * Settings are stored in ESP32 NVS using Preferences.
 */

#include <M5Unified.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <math.h>

// =====================================================
// Default WiFi Configuration
// =====================================================

const char* DEFAULT_SSID = "YOUR_WIFI_SSID";
const char* DEFAULT_WIFI_PASSWORD = "YOUR_WIFI_PASSWORD";

const bool DEFAULT_USE_DHCP = false;
const char* DEFAULT_LOCAL_IP = "192.168.212.80";
const char* DEFAULT_GATEWAY  = "192.168.212.1";
const char* DEFAULT_SUBNET   = "255.255.255.0";
const char* DEFAULT_DNS      = "192.168.212.1";

// =====================================================
// Default MQTT Configuration
// =====================================================

const char* DEFAULT_MQTT_SERVER = "192.168.212.1";
const int DEFAULT_MQTT_PORT = 1883;
const char* DEFAULT_MQTT_USER = "mqtt";
const char* DEFAULT_MQTT_PASSWORD = "sI7G@DijuY";
const char* DEFAULT_MQTT_CLIENT_ID = "M5StickCPlus2_ADC";
const char* DEFAULT_MQTT_PUB_TOPIC = "0/WHISPERER/RMS-SKIP-AAA09/battery";
const char* DEFAULT_MQTT_SUB_TOPIC = "0/WHISPERER/RMS-SKIP-AAA09/vel2D_DWO";

// =====================================================
// Default Measurement Configuration
// =====================================================

const float DEFAULT_VOLTAGE_OFFSET = 0.0f;

// 24V系LiFePO4用。
// SOC近似式 batteryLevel = voltage * 43.046 - 1070 で
// ほぼ0%になる電圧を有効レンジ下限の初期値とする。
// この値未満ではMQTTにバッテリー情報をPublishしない。
const float DEFAULT_MIN_VALID_VOLTAGE = 24.86f;

// =====================================================
// Setup AP Configuration
// =====================================================

const char* SETUP_AP_SSID = "M5StickSetup";

IPAddress setupAP_IP(192, 168, 212, 80);
IPAddress setupAP_GW(192, 168, 212, 80);
IPAddress setupAP_SN(255, 255, 255, 0);

const unsigned long CONNECTION_TIMEOUT = 120000;
const unsigned long RETRY_INTERVAL = 5000;

// =====================================================
// Saved Configuration
// =====================================================

String wifiSSID;
String wifiPassword;
bool useDHCP;

String wifiLocalIP;
String wifiGateway;
String wifiSubnet;
String wifiDNS;

String mqttServer;
int mqttPort;
String mqttUser;
String mqttPassword;
String mqttClientId;
String mqttPubTopic;
String mqttSubTopic;

float voltageOffset = DEFAULT_VOLTAGE_OFFSET;
float minValidVoltage = DEFAULT_MIN_VALID_VOLTAGE;

// =====================================================
// Objects
// =====================================================

Preferences preferences;
WebServer webServer(80);

WiFiClient espClient;
PubSubClient client(espClient);

// =====================================================
// Application Variables
// =====================================================

const int analogPin = 36;

unsigned long update_interval = 1000;
unsigned long sampling_interval = 100;

unsigned long lastMsgTime = 0;
unsigned long lastSampleTime = 0;

float currentVoltage = 0.0f;
float VoltageRatio = 10.77f;

float batteryLevel = 99.0f;
float filteredBattery = 99.0f;

const float ALPHA = 0.01f;

bool brakeFlag = false;

String rxPayloadStr = "Waiting...";
String debugTopic = "None";

int debugLen = 0;
int rxCount = 0;

int lastDisplayedVoltage100 = -99999;
int lastDisplayedBattery = -1;
bool unitDrawn = false;

char buf[8];

// =====================================================
// HTML Escape
// =====================================================

String htmlEscape(const String& src) {
  String dst;
  dst.reserve(src.length() + 16);

  for (unsigned int i = 0; i < src.length(); i++) {
    char c = src[i];

    switch (c) {
      case '&':  dst += "&amp;";  break;
      case '<':  dst += "&lt;";   break;
      case '>':  dst += "&gt;";   break;
      case '"':  dst += "&quot;"; break;
      case '\'': dst += "&#39;";  break;
      default:   dst += c;        break;
    }
  }

  return dst;
}

// =====================================================
// Load Settings
// =====================================================

void loadSettings() {
  preferences.begin("config", true);

  wifiSSID = preferences.getString("ssid", DEFAULT_SSID);
  wifiPassword = preferences.getString("wifipass", DEFAULT_WIFI_PASSWORD);
  useDHCP = preferences.getBool("dhcp", DEFAULT_USE_DHCP);

  wifiLocalIP = preferences.getString("ip", DEFAULT_LOCAL_IP);
  wifiGateway = preferences.getString("gateway", DEFAULT_GATEWAY);
  wifiSubnet = preferences.getString("subnet", DEFAULT_SUBNET);
  wifiDNS = preferences.getString("dns", DEFAULT_DNS);

  mqttServer = preferences.getString("mqtthost", DEFAULT_MQTT_SERVER);
  mqttPort = preferences.getInt("mqttport", DEFAULT_MQTT_PORT);
  mqttUser = preferences.getString("mqttuser", DEFAULT_MQTT_USER);
  mqttPassword = preferences.getString("mqttpass", DEFAULT_MQTT_PASSWORD);
  mqttClientId = preferences.getString("clientid", DEFAULT_MQTT_CLIENT_ID);
  mqttPubTopic = preferences.getString("pubtopic", DEFAULT_MQTT_PUB_TOPIC);
  mqttSubTopic = preferences.getString("subtopic", DEFAULT_MQTT_SUB_TOPIC);

  voltageOffset = preferences.getFloat("voffset", DEFAULT_VOLTAGE_OFFSET);
  minValidVoltage = preferences.getFloat("minvolt", DEFAULT_MIN_VALID_VOLTAGE);

  preferences.end();

  Serial.println();
  Serial.println("========== Loaded Settings ==========");
  Serial.printf("WiFi SSID        : %s\n", wifiSSID.c_str());
  Serial.printf("IP Mode          : %s\n", useDHCP ? "DHCP" : "Static");

  if (!useDHCP) {
    Serial.printf("IP               : %s\n", wifiLocalIP.c_str());
    Serial.printf("Gateway          : %s\n", wifiGateway.c_str());
    Serial.printf("Subnet           : %s\n", wifiSubnet.c_str());
    Serial.printf("DNS              : %s\n", wifiDNS.c_str());
  }

  Serial.printf("MQTT Server      : %s\n", mqttServer.c_str());
  Serial.printf("MQTT Port        : %d\n", mqttPort);
  Serial.printf("MQTT User        : %s\n", mqttUser.c_str());
  Serial.printf("Client ID        : %s\n", mqttClientId.c_str());
  Serial.printf("Pub Topic        : %s\n", mqttPubTopic.c_str());
  Serial.printf("Sub Topic        : %s\n", mqttSubTopic.c_str());
  Serial.printf("Voltage Offset   : %+.3f V\n", voltageOffset);
  Serial.printf("Min Valid Voltage: %.2f V\n", minValidVoltage);
  Serial.println("=====================================");
}

// =====================================================
// Save Settings
// =====================================================

void saveSettings() {
  preferences.begin("config", false);

  preferences.putString("ssid", wifiSSID);
  preferences.putString("wifipass", wifiPassword);
  preferences.putBool("dhcp", useDHCP);

  preferences.putString("ip", wifiLocalIP);
  preferences.putString("gateway", wifiGateway);
  preferences.putString("subnet", wifiSubnet);
  preferences.putString("dns", wifiDNS);

  preferences.putString("mqtthost", mqttServer);
  preferences.putInt("mqttport", mqttPort);
  preferences.putString("mqttuser", mqttUser);
  preferences.putString("mqttpass", mqttPassword);
  preferences.putString("clientid", mqttClientId);
  preferences.putString("pubtopic", mqttPubTopic);
  preferences.putString("subtopic", mqttSubTopic);

  preferences.putFloat("voffset", voltageOffset);
  preferences.putFloat("minvolt", minValidVoltage);

  preferences.end();

  Serial.println("Settings saved to NVS.");
}

// =====================================================
// Setup Web Page
// =====================================================

String makeSetupPage() {
  String html;
  html.reserve(7500);

  html += R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>M5Stick Setup</title>

<style>
body {
  font-family: Arial, sans-serif;
  background: #f2f2f2;
  margin: 0;
  padding: 20px;
}
.container {
  max-width: 520px;
  margin: auto;
  background: white;
  padding: 22px;
  border-radius: 10px;
}
h2 { margin-top: 0; }
h3 {
  margin-top: 28px;
  border-bottom: 1px solid #ccc;
  padding-bottom: 5px;
}
label {
  display: block;
  margin-top: 12px;
  font-weight: bold;
}
input[type=text],
input[type=password],
input[type=number] {
  width: 100%;
  box-sizing: border-box;
  padding: 9px;
  margin-top: 4px;
}
.radio { margin-top: 10px; }
button {
  width: 100%;
  margin-top: 25px;
  padding: 13px;
  font-size: 17px;
}
.note {
  color: #666;
  font-size: 13px;
}
</style>

<script>
function updateIPFields() {
  var dhcp = document.querySelector('input[name="mode"]:checked').value === "dhcp";
  var fields = document.querySelectorAll(".staticField");

  for (var i = 0; i < fields.length; i++) {
    fields[i].disabled = dhcp;
  }
}
</script>

</head>

<body onload="updateIPFields()">
<div class="container">

<h2>M5Stick Setup</h2>

<form action="/save" method="POST">

<h3>WiFi</h3>

<label>SSID</label>
<input type="text" name="ssid" required value=")rawliteral";

  html += htmlEscape(wifiSSID);

  html += R"rawliteral(">

<label>Password</label>
<input type="password" name="wifipass" value=")rawliteral";

  html += htmlEscape(wifiPassword);

  html += R"rawliteral(">

<h3>IP Configuration</h3>

<div class="radio">

<label>
<input type="radio" name="mode" value="dhcp" onchange="updateIPFields()"
)rawliteral";

  if (useDHCP) html += " checked";

  html += R"rawliteral(>
DHCP
</label>

<label>
<input type="radio" name="mode" value="static" onchange="updateIPFields()"
)rawliteral";

  if (!useDHCP) html += " checked";

  html += R"rawliteral(>
Static IP
</label>

</div>

<label>IP Address</label>
<input class="staticField" type="text" name="ip" value=")rawliteral";

  html += htmlEscape(wifiLocalIP);

  html += R"rawliteral(">

<label>Gateway</label>
<input class="staticField" type="text" name="gateway" value=")rawliteral";

  html += htmlEscape(wifiGateway);

  html += R"rawliteral(">

<label>Subnet Mask</label>
<input class="staticField" type="text" name="subnet" value=")rawliteral";

  html += htmlEscape(wifiSubnet);

  html += R"rawliteral(">

<label>DNS Server</label>
<input class="staticField" type="text" name="dns" value=")rawliteral";

  html += htmlEscape(wifiDNS);

  html += R"rawliteral(">

<h3>MQTT</h3>

<label>MQTT Server</label>
<input type="text" name="mqtthost" required value=")rawliteral";

  html += htmlEscape(mqttServer);

  html += R"rawliteral(">

<label>MQTT Port</label>
<input type="number" name="mqttport" min="1" max="65535" value=")rawliteral";

  html += String(mqttPort);

  html += R"rawliteral(">

<label>MQTT User</label>
<input type="text" name="mqttuser" value=")rawliteral";

  html += htmlEscape(mqttUser);

  html += R"rawliteral(">

<label>MQTT Password</label>
<input type="password" name="mqttpass" value=")rawliteral";

  html += htmlEscape(mqttPassword);

  html += R"rawliteral(">

<label>MQTT Client ID</label>
<input type="text" name="clientid" required value=")rawliteral";

  html += htmlEscape(mqttClientId);

  html += R"rawliteral(">

<label>Publish Topic</label>
<input type="text" name="pubtopic" required value=")rawliteral";

  html += htmlEscape(mqttPubTopic);

  html += R"rawliteral(">

<label>Subscribe Topic</label>
<input type="text" name="subtopic" required value=")rawliteral";

  html += htmlEscape(mqttSubTopic);

  html += R"rawliteral(">

<h3>Voltage Measurement</h3>

<label>Voltage Offset [V]</label>
<input type="number"
       name="voffset"
       step="0.001"
       min="-5.0"
       max="5.0"
       value=")rawliteral";

  html += String(voltageOffset, 3);

  html += R"rawliteral(">

<p class="note">
Correction added to the measured battery voltage.<br>
Example: meter = 26.20 V, M5Stick = 26.05 V &rarr; set +0.15 V
</p>

<label>Minimum Valid Voltage [V]</label>
<input type="number"
       name="minvolt"
       step="0.01"
       min="0"
       max="40"
       value=")rawliteral";

  html += String(minValidVoltage, 2);

  html += R"rawliteral(">

<p class="note">
If the corrected battery voltage is below this value,
battery data will not be published to MQTT.
Default: 24.86 V for the current 24 V LiFePO4 conversion formula.
</p>

<button type="submit">Save and Restart</button>

<p class="note">
Settings are stored in internal flash memory.
</p>

</form>
</div>
</body>
</html>
)rawliteral";

  return html;
}

// =====================================================
// Web Server
// =====================================================

void setupWebServer() {
  webServer.on("/", HTTP_GET, []() {
    webServer.send(200, "text/html; charset=UTF-8", makeSetupPage());
  });

  webServer.on("/save", HTTP_POST, []() {
    wifiSSID = webServer.arg("ssid");
    wifiPassword = webServer.arg("wifipass");
    useDHCP = (webServer.arg("mode") == "dhcp");

    if (!useDHCP) {
      wifiLocalIP = webServer.arg("ip");
      wifiGateway = webServer.arg("gateway");
      wifiSubnet = webServer.arg("subnet");
      wifiDNS = webServer.arg("dns");
    }

    mqttServer = webServer.arg("mqtthost");

    int newPort = webServer.arg("mqttport").toInt();
    mqttPort = (newPort >= 1 && newPort <= 65535) ? newPort : DEFAULT_MQTT_PORT;

    mqttUser = webServer.arg("mqttuser");
    mqttPassword = webServer.arg("mqttpass");
    mqttClientId = webServer.arg("clientid");
    mqttPubTopic = webServer.arg("pubtopic");
    mqttSubTopic = webServer.arg("subtopic");

    voltageOffset = constrain(webServer.arg("voffset").toFloat(), -5.0f, 5.0f);
    minValidVoltage = constrain(webServer.arg("minvolt").toFloat(), 0.0f, 40.0f);

    saveSettings();

    String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Saved</title>
</head>
<body style="font-family:Arial;text-align:center;margin-top:50px;">
<h2>Settings saved</h2>
<p>M5StickC Plus2 is restarting...</p>
</body>
</html>
)rawliteral";

    webServer.send(200, "text/html; charset=UTF-8", html);

    Serial.println("Configuration saved.");
    Serial.printf("Voltage Offset    = %+.3f V\n", voltageOffset);
    Serial.printf("Min Valid Voltage = %.2f V\n", minValidVoltage);
    Serial.println("Restarting...");

    delay(1200);
    ESP.restart();
  });

  webServer.onNotFound([]() {
    webServer.sendHeader("Location", "http://192.168.212.80/", true);
    webServer.send(302, "text/plain", "");
  });

  webServer.begin();
  Serial.println("Web server started.");
}

// =====================================================
// Setup AP
// =====================================================

void startSetupAP(const char* reason) {
  client.disconnect();

  WiFi.disconnect(true);
  delay(300);

  WiFi.mode(WIFI_AP);

  if (!WiFi.softAPConfig(setupAP_IP, setupAP_GW, setupAP_SN)) {
    Serial.println("ERROR: softAPConfig failed.");
  }

  if (!WiFi.softAP(SETUP_AP_SSID)) {
    Serial.println("ERROR: Setup AP creation failed.");
  }

  Serial.println();
  Serial.println("=====================================");
  Serial.println("SETUP MODE");
  Serial.printf("Reason   : %s\n", reason);
  Serial.printf("SSID     : %s\n", SETUP_AP_SSID);
  Serial.println("Password : none");
  Serial.printf("IP       : %s\n", WiFi.softAPIP().toString().c_str());
  Serial.println("Open http://192.168.212.80/");
  Serial.println("=====================================");

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setFont(&fonts::Font0);

  M5.Lcd.setTextSize(2);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.setCursor(5, 5);
  M5.Lcd.println("SETUP MODE");

  M5.Lcd.setTextSize(1.5);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(5, 30);
  M5.Lcd.printf("Reason: %s", reason);

  M5.Lcd.setCursor(5, 52);
  M5.Lcd.println("WiFi AP:");

  M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
  M5.Lcd.setCursor(5, 67);
  M5.Lcd.println(SETUP_AP_SSID);

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(5, 88);
  M5.Lcd.println("Open browser:");

  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setCursor(5, 103);
  M5.Lcd.println("192.168.212.80");

  setupWebServer();

  while (true) {
    webServer.handleClient();
    M5.update();
    delay(2);
  }
}

// =====================================================
// Static IP Configuration
// =====================================================

bool configureStaticIP() {
  if (useDHCP) return true;

  IPAddress ip;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns;

  if (!ip.fromString(wifiLocalIP) ||
      !gateway.fromString(wifiGateway) ||
      !subnet.fromString(wifiSubnet) ||
      !dns.fromString(wifiDNS)) {
    Serial.println("ERROR: Invalid static IP configuration.");
    return false;
  }

  if (!WiFi.config(ip, gateway, subnet, dns)) {
    Serial.println("ERROR: WiFi.config() failed.");
    return false;
  }

  return true;
}

// =====================================================
// WiFi Connection
// =====================================================

bool connectWiFi() {
  Serial.println();
  Serial.println("----- WiFi Connection -----");
  Serial.printf("Connecting to SSID: %s\n", wifiSSID.c_str());

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setFont(&fonts::Font0);
  M5.Lcd.setTextSize(1.5);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.setCursor(5, 10);
  M5.Lcd.println("WiFi Connecting...");

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(5, 30);
  M5.Lcd.println(wifiSSID);

  WiFi.disconnect(true);
  delay(200);

  WiFi.mode(WIFI_STA);

  if (!configureStaticIP()) return false;

  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());

  unsigned long startTime = millis();
  unsigned long lastStatusTime = 0;

  while (WiFi.status() != WL_CONNECTED) {
    M5.update();

    unsigned long now = millis();

    if (now - startTime >= CONNECTION_TIMEOUT) {
      Serial.println();
      Serial.println("ERROR: WiFi connection timed out after 2 minutes.");

      M5.Lcd.fillScreen(TFT_BLACK);
      M5.Lcd.setFont(&fonts::Font0);
      M5.Lcd.setTextSize(1.5);
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
      M5.Lcd.setCursor(5, 20);
      M5.Lcd.println("WiFi FAILED");

      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      M5.Lcd.setCursor(5, 45);
      M5.Lcd.println("Starting Setup AP...");

      delay(1500);
      return false;
    }

    if (now - lastStatusTime >= 5000) {
      lastStatusTime = now;
      unsigned long elapsed = (now - startTime) / 1000;

      Serial.printf("WiFi waiting... %lu / 120 sec\n", elapsed);

      M5.Lcd.fillRect(0, 55, 240, 20, TFT_BLACK);
      M5.Lcd.setCursor(5, 55);
      M5.Lcd.printf("Waiting %lu / 120 sec", elapsed);
    }

    delay(20);
  }

  Serial.println("WiFi connected.");
  Serial.printf("IP Address: %s\n", WiFi.localIP().toString().c_str());

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setCursor(5, 20);
  M5.Lcd.println("WiFi Connected");

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(5, 40);
  M5.Lcd.println(WiFi.localIP().toString());

  delay(800);

  return true;
}

// =====================================================
// MQTT Callback
// =====================================================

void callback(char* topic, byte* payload, unsigned int length) {
  rxCount++;
  debugTopic = String(topic);
  debugLen = length;

  rxPayloadStr = "";

  for (unsigned int i = 0; i < length; i++) {
    rxPayloadStr += (char)payload[i];
  }

  if (String(topic) == mqttSubTopic) {
    JsonDocument inDoc;
    DeserializationError error = deserializeJson(inDoc, payload, length);

    if (error) {
      rxPayloadStr = "JSON Err: " + rxPayloadStr;
      return;
    }

    if (inDoc.containsKey("v_mps")) {
      float v_mps = inDoc["v_mps"].as<float>();
      brakeFlag = (fabs(v_mps) < 0.01f);
    }
  }
}

// =====================================================
// MQTT Connection
// =====================================================

bool connectMQTT() {
  Serial.println();
  Serial.println("----- MQTT Connection -----");
  Serial.printf("Server : %s:%d\n", mqttServer.c_str(), mqttPort);
  Serial.printf("Client : %s\n", mqttClientId.c_str());

  M5.Lcd.fillScreen(TFT_BLACK);
  M5.Lcd.setFont(&fonts::Font0);
  M5.Lcd.setTextSize(1.5);
  M5.Lcd.setTextColor(TFT_YELLOW, TFT_BLACK);
  M5.Lcd.setCursor(5, 10);
  M5.Lcd.println("MQTT Connecting...");

  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  M5.Lcd.setCursor(5, 30);
  M5.Lcd.printf("%s:%d", mqttServer.c_str(), mqttPort);

  client.setServer(mqttServer.c_str(), mqttPort);

  unsigned long startTime = millis();
  unsigned long lastAttemptTime = millis() - RETRY_INTERVAL;

  while (!client.connected()) {
    M5.update();

    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("ERROR: WiFi disconnected while connecting MQTT.");
      return false;
    }

    unsigned long now = millis();

    if (now - startTime >= CONNECTION_TIMEOUT) {
      Serial.println();
      Serial.println("ERROR: MQTT connection timed out after 2 minutes.");
      Serial.printf("Last MQTT state: %d\n", client.state());

      M5.Lcd.fillScreen(TFT_BLACK);
      M5.Lcd.setFont(&fonts::Font0);
      M5.Lcd.setTextSize(1.5);
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
      M5.Lcd.setCursor(5, 20);
      M5.Lcd.println("MQTT FAILED");

      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      M5.Lcd.setCursor(5, 42);
      M5.Lcd.printf("State: %d", client.state());

      M5.Lcd.setCursor(5, 65);
      M5.Lcd.println("Starting Setup AP...");

      delay(1500);
      return false;
    }

    if (now - lastAttemptTime >= RETRY_INTERVAL) {
      lastAttemptTime = now;

      bool result;

      if (mqttUser.length() > 0) {
        result = client.connect(mqttClientId.c_str(), mqttUser.c_str(), mqttPassword.c_str());
      } else {
        result = client.connect(mqttClientId.c_str());
      }

      if (result) {
        Serial.println("MQTT connected.");

        if (client.subscribe(mqttSubTopic.c_str())) {
          Serial.printf("Subscribed: %s\n", mqttSubTopic.c_str());
        } else {
          Serial.printf("WARNING: Subscribe failed: %s\n", mqttSubTopic.c_str());
        }

        lastDisplayedVoltage100 = -99999;
        lastDisplayedBattery = -1;
        unitDrawn = false;

        M5.Lcd.fillScreen(TFT_BLACK);
        M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
        M5.Lcd.setCursor(5, 20);
        M5.Lcd.println("MQTT Connected");

        delay(600);
        return true;
      }

      unsigned long elapsed = (now - startTime) / 1000;

      Serial.printf("MQTT connect failed. state=%d, %lu / 120 sec\n",
                    client.state(), elapsed);

      M5.Lcd.fillRect(0, 50, 240, 42, TFT_BLACK);
      M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
      M5.Lcd.setCursor(5, 50);
      M5.Lcd.printf("Failed state:%d", client.state());

      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      M5.Lcd.setCursor(5, 70);
      M5.Lcd.printf("%lu / 120 sec", elapsed);
    }

    delay(20);
  }

  return true;
}

// =====================================================
// Connection Management
// =====================================================

void ensureConnections() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println();
    Serial.println("*** WiFi connection lost ***");

    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setFont(&fonts::Font0);
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    M5.Lcd.setCursor(5, 20);
    M5.Lcd.println("WiFi LOST");

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(5, 45);
    M5.Lcd.println("Reconnecting...");

    client.disconnect();

    if (!connectWiFi()) {
      startSetupAP("WiFi timeout");
    }

    if (!connectMQTT()) {
      startSetupAP("MQTT timeout");
    }

    return;
  }

  if (!client.connected()) {
    Serial.println();
    Serial.println("*** MQTT connection lost ***");

    M5.Lcd.fillScreen(TFT_BLACK);
    M5.Lcd.setFont(&fonts::Font0);
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);
    M5.Lcd.setCursor(5, 20);
    M5.Lcd.println("MQTT LOST");

    M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    M5.Lcd.setCursor(5, 45);
    M5.Lcd.println("Reconnecting...");

    if (!connectMQTT()) {
      if (WiFi.status() != WL_CONNECTED) {
        if (!connectWiFi()) {
          startSetupAP("WiFi timeout");
        }

        if (!connectMQTT()) {
          startSetupAP("MQTT timeout");
        }
      } else {
        startSetupAP("MQTT timeout");
      }
    }
  }
}

// =====================================================
// Draw Units
// =====================================================

void drawUnits() {
  M5.Lcd.setFont(&fonts::Font0);
  M5.Lcd.setTextSize(5);
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);

  M5.Lcd.setCursor(125, 80);
  M5.Lcd.print("V");

  M5.Lcd.setCursor(210, 80);
  M5.Lcd.print("%");

  unitDrawn = true;
}

// =====================================================
// Voltage Measurement
// =====================================================

float readBatteryVoltage() {
  long sum = 0;

  for (int i = 0; i < 20; i++) {
    sum += analogRead(analogPin);
    delay(1);
  }

  // ADC値を分圧前のバッテリー電圧へ換算し、Web設定された補正値を加算
  float voltage = VoltageRatio * (sum / 20.0f * 3.3f) / 4095.0f;
  voltage += voltageOffset;

  return roundf(voltage * 100.0f) / 100.0f;
}

// =====================================================
// Setup
// =====================================================

void setup() {
  auto cfg = M5.config();
  M5.begin(cfg);

  Serial.begin(115200);

  unsigned long startWait = millis();

  while (!Serial && millis() - startWait < 1500) {
    delay(10);
  }

  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(TFT_BLACK);

  Serial.println();
  Serial.println("=== M5StickC Plus2 Battery Monitor ===");

  loadSettings();

  // BtnAを押しながら起動すると強制Setup Mode
  M5.update();
  delay(50);
  M5.update();

  if (M5.BtnA.isPressed()) {
    Serial.println("BtnA held at startup.");
    startSetupAP("Button pressed");
  }

  if (!connectWiFi()) {
    startSetupAP("WiFi timeout");
  }

  client.setCallback(callback);

  if (!connectMQTT()) {
    if (WiFi.status() != WL_CONNECTED) {
      startSetupAP("WiFi lost");
    } else {
      startSetupAP("MQTT timeout");
    }
  }

  pinMode(analogPin, ANALOG);

  currentVoltage = readBatteryVoltage();

  // 24V系LiFePO4バッテリー用の電圧→残量近似変換式
  batteryLevel = currentVoltage * 43.046f - 1070.0f;
  batteryLevel = constrain(batteryLevel, 30.0f, 99.0f);

  filteredBattery = batteryLevel;

  M5.Lcd.fillScreen(TFT_BLACK);

  Serial.println("--- Setup Completed ---");
  Serial.printf("Voltage Offset    : %+.3f V\n", voltageOffset);
  Serial.printf("Min Valid Voltage : %.2f V\n", minValidVoltage);
  Serial.printf("Current Voltage   : %.2f V\n", currentVoltage);
  Serial.printf("Battery           : %.2f %%\n", filteredBattery);

  if (currentVoltage < minValidVoltage) {
    Serial.println("WARNING: Battery voltage is below valid range. MQTT battery publish is disabled.");
  }
}

// =====================================================
// Main Loop
// =====================================================

void loop() {
  M5.update();

  ensureConnections();
  client.loop();

  unsigned long now = millis();

  // ===================================================
  // ADC / Display
  // ===================================================

  if (now - lastSampleTime >= sampling_interval) {
    lastSampleTime = now;

    currentVoltage = readBatteryVoltage();

    // 24V系LiFePO4バッテリー用の電圧→残量近似変換式
    batteryLevel = currentVoltage * 43.046f - 1070.0f;
    batteryLevel = constrain(batteryLevel, 0.0f, 99.0f);

    if (brakeFlag) {
      filteredBattery = filteredBattery * (1.0f - ALPHA) + batteryLevel * ALPHA;
    }

    int voltage100 = (int)roundf(currentVoltage * 100.0f);
    int displayBattery = (int)(filteredBattery + 0.5f);

    M5.Lcd.startWrite();

    // SSID
    M5.Lcd.setFont(&fonts::Font0);
    M5.Lcd.setTextSize(1.5);
    M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    M5.Lcd.setCursor(5, 5);
    M5.Lcd.printf("SSID:%s", WiFi.SSID().c_str());

    // MQTT Topic
    M5.Lcd.setTextColor(TFT_CYAN, TFT_BLACK);
    M5.Lcd.setCursor(5, 15);
    M5.Lcd.printf("Topic: %s", mqttPubTopic.c_str());

    // BRAKE
    M5.Lcd.setFont(&fonts::Font0);
    M5.Lcd.setTextSize(1.5);

    if (brakeFlag) {
      M5.Lcd.setTextColor(TFT_DARKGRAY, TFT_WHITE);
      M5.Lcd.drawString("BRAKE: ON ", 150, 25);
    } else {
      M5.Lcd.setTextColor(TFT_DARKGRAY, TFT_RED);
      M5.Lcd.drawString("BRAKE: OFF", 150, 25);
    }

    if (!unitDrawn) {
      drawUnits();
    }

    // Voltage
    if (voltage100 != lastDisplayedVoltage100) {
      M5.Lcd.fillRect(0, 35, 122, 82, TFT_BLACK);

      M5.Display.setFont(&fonts::Font7);
      M5.Display.setTextSize(0.9, 1.6);

      // レンジ外なら赤、正常なら黄色
      if (currentVoltage < minValidVoltage) {
        M5.Display.setTextColor(TFT_RED, TFT_BLACK);
      } else {
        M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
      }

      M5.Display.drawFloat(currentVoltage, 2, 2, 40);

      M5.Lcd.setFont(&fonts::Font0);
      M5.Lcd.setTextSize(5);
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      M5.Lcd.setCursor(125, 80);
      M5.Lcd.print("V");

      lastDisplayedVoltage100 = voltage100;
    }

    // Battery %
    if (displayBattery != lastDisplayedBattery) {
      M5.Lcd.fillRect(145, 35, 63, 82, TFT_BLACK);

      sprintf(buf, "%2d", displayBattery);

      M5.Display.setFont(&fonts::Font7);
      M5.Display.setTextSize(0.9, 1.6);
      M5.Display.setTextColor(TFT_YELLOW, TFT_BLACK);
      M5.Display.drawString(buf, 152, 40);

      M5.Lcd.setFont(&fonts::Font0);
      M5.Lcd.setTextSize(5);
      M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      M5.Lcd.setCursor(210, 80);
      M5.Lcd.print("%");

      lastDisplayedBattery = displayBattery;
    }

    // Battery bar
    int barMaxW = 240;
    int barX = 5;
    int barY = 120;
    int barH = 20;

    int barW = (int)(barMaxW * (filteredBattery / 99.0f));
    barW = constrain(barW, 0, barMaxW);

    uint16_t barColor = TFT_GREEN;

    if (filteredBattery <= 30.0f) {
      barColor = TFT_RED;
    } else if (filteredBattery <= 70.0f) {
      barColor = TFT_YELLOW;
    }

    if (barW > 0) {
      M5.Lcd.fillRect(barX, barY, barW, barH, barColor);
    }

    if (barW < barMaxW) {
      M5.Lcd.fillRect(barX + barW, barY, barMaxW - barW, barH, TFT_BLACK);
    }

    M5.Lcd.endWrite();

    Serial.printf("Current Voltage: %.2f V, Battery: %.2f %%, Offset: %+.3f V\n",
                  currentVoltage, filteredBattery, voltageOffset);
  }

  // ===================================================
  // MQTT Publish
  // ===================================================

  if (now - lastMsgTime >= update_interval) {
    lastMsgTime = now;

    // 電圧が24V系LiFePO4の有効レンジ下限より低い場合はPublishしない
    if (currentVoltage < minValidVoltage) {
      Serial.printf("MQTT publish skipped: voltage %.2f V is below minimum %.2f V\n",
                    currentVoltage, minValidVoltage);
      return;
    }

    int leveledBattery = (int)ceil(filteredBattery / 10.0f);

    if (leveledBattery < 0) {
      leveledBattery = 0;
    } else if (leveledBattery > 100) {
      leveledBattery = 100;
    }

    JsonDocument outDoc;
    outDoc["voltage"] = currentVoltage;
    outDoc["batteryLevel"] = leveledBattery;

    String jsonString;
    serializeJson(outDoc, jsonString);

    bool publishResult = client.publish(mqttPubTopic.c_str(), jsonString.c_str());

    if (!publishResult) {
      Serial.println("WARNING: MQTT publish failed.");
    }
  }
}