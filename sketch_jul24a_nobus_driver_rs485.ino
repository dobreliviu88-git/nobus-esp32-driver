#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ==========================================
// 1. CONFIGURARE REȚEA & MQTT
// ==========================================
const char* ssid = "DIGI-d7Ft";
const char* password = "tame9tdvD9";

const char* mqtt_server = "192.168.1.1"; // IP-ul serverului Ubuntu
const int mqtt_port = 1883;

const char* TOPIC_RX = "esp/pompa/stare_raw";
const char* TOPIC_TX = "esp/pompa/comanda";

// ==========================================
// 2. CONFIGURARE HARDWARE MODBUS
// ==========================================
#define RX_PIN 16
#define TX_PIN 17
#define SLAVE_ID 176
#define BAUD_RATE 9600

WiFiClient espClient;
PubSubClient mqttClient(espClient);

unsigned long lastHeartbeat = 0;
unsigned long lastPoll = 0;
unsigned long lastWiFiCheck = 0; // Temporizator pentru reconectare WiFi

// ==========================================
// 3. FUNCȚIE CALCUL CRC16
// ==========================================
uint16_t calculateCRC16(const uint8_t *buffer, uint16_t length) {
  uint16_t crc = 0xFFFF;
  for (uint16_t pos = 0; pos < length; pos++) {
    crc ^= (uint16_t)buffer[pos];
    for (int i = 8; i != 0; i--) {
      if ((crc & 0x0001) != 0) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// ==========================================
// 4. FUNCȚII MODBUS DIRECTE
// ==========================================
bool modbusWriteSingleRegister(uint16_t reg, uint16_t val) {
  uint8_t frame[8];
  frame[0] = SLAVE_ID;
  frame[1] = 0x06; 
  frame[2] = (reg >> 8) & 0xFF;
  frame[3] = reg & 0xFF;
  frame[4] = (val >> 8) & 0xFF;
  frame[5] = val & 0xFF;
  
  uint16_t crc = calculateCRC16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;

  while (Serial1.available()) Serial1.read();
  Serial1.write(frame, 8);
  Serial1.flush();

  unsigned long startTime = millis();
  while (Serial1.available() < 8) {
    if (millis() - startTime > 300) return false;
  }

  uint8_t response[8];
  for (int i = 0; i < 8; i++) {
    response[i] = Serial1.read();
  }

  return (response[0] == SLAVE_ID && response[1] == 0x06);
}

bool modbusReadHoldingRegisters(uint16_t startReg, uint16_t count, uint16_t *outValues) {
  uint8_t frame[8];
  frame[0] = SLAVE_ID;
  frame[1] = 0x03;
  frame[2] = (startReg >> 8) & 0xFF;
  frame[3] = startReg & 0xFF;
  frame[4] = (count >> 8) & 0xFF;
  frame[5] = count & 0xFF;

  uint16_t crc = calculateCRC16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;

  while (Serial1.available()) Serial1.read();
  Serial1.write(frame, 8);
  Serial1.flush();

  int expectedBytes = 3 + (count * 2) + 2;
  unsigned long startTime = millis();
  while (Serial1.available() < expectedBytes) {
    if (millis() - startTime > 300) return false;
  }

  uint8_t response[128];
  for (int i = 0; i < expectedBytes; i++) response[i] = Serial1.read();

  if (response[0] != SLAVE_ID || response[1] != 0x03) return false;

  for (uint16_t i = 0; i < count; i++) {
    outValues[i] = ((uint16_t)response[3 + (i * 2)] << 8) | response[4 + (i * 2)];
  }
  return true;
}

bool modbusReadInputRegisters(uint16_t startReg, uint16_t count, uint16_t *outValues) {
  uint8_t frame[8];
  frame[0] = SLAVE_ID;
  frame[1] = 0x04; 
  frame[2] = (startReg >> 8) & 0xFF;
  frame[3] = startReg & 0xFF;
  frame[4] = (count >> 8) & 0xFF;
  frame[5] = count & 0xFF;

  uint16_t crc = calculateCRC16(frame, 6);
  frame[6] = crc & 0xFF;
  frame[7] = (crc >> 8) & 0xFF;

  while (Serial1.available()) Serial1.read();
  Serial1.write(frame, 8);
  Serial1.flush();

  int expectedBytes = 3 + (count * 2) + 2;
  unsigned long startTime = millis();
  while (Serial1.available() < expectedBytes) {
    if (millis() - startTime > 400) return false; 
  }

  uint8_t response[128];
  for (int i = 0; i < expectedBytes; i++) response[i] = Serial1.read();

  if (response[0] != SLAVE_ID || response[1] != 0x04) return false;

  for (uint16_t i = 0; i < count; i++) {
    outValues[i] = ((uint16_t)response[3 + (i * 2)] << 8) | response[4 + (i * 2)];
  }
  return true;
}

// ==========================================
// 5. CALLBACK MQTT & NETWORK MANAGEMENT
// ==========================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  JsonDocument doc;
  if (deserializeJson(doc, msg)) return;

  if (doc.containsKey("reg") && doc.containsKey("val")) {
    uint16_t registru = doc["reg"];
    uint16_t valoare = doc["val"];
    Serial.printf("⚡ [DRIVER TX] Scriere reg %d -> Val %d\n", registru, valoare);
    modbusWriteSingleRegister(registru, valoare);
  }
}

void checkWiFiAndMQTT() {
  // 1. Verificare WiFi Non-Blocantă
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    // Incercam reconectarea la fiecare 10 secunde, ca sa nu blocam Modbus-ul
    if (currentMillis - lastWiFiCheck >= 10000) {
      Serial.println("🔗 [WIFI] Conexiune pierduta. Se reincearca...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      lastWiFiCheck = currentMillis;
    }
    return; // Daca nu avem WiFi, nu are sens sa incercam MQTT
  }

  // 2. Verificare MQTT Non-Blocantă
  if (!mqttClient.connected()) {
    Serial.println("🔗 [MQTT] Conectare la Mosquitto...");
    if (mqttClient.connect("Esp32_DriverModbus")) {
      Serial.println("✅ [MQTT] Conectat cu succes!");
      mqttClient.subscribe(TOPIC_TX);
    }
  }
}

// ==========================================
// 6. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  Serial.print("Conectare la WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi conectat!");

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);

  Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("✅ [DRIVER INIT] Modbus activat. Let's go!");
}

// ==========================================
// 7. BUCLA PRINCIPALĂ
// ==========================================
void loop() {
  // Gestionare conexiuni retea in mod sigur (fara while-uri blocate)
  checkWiFiAndMQTT();
  if (mqttClient.connected()) {
    mqttClient.loop();
  }

  unsigned long currentTime = millis();

  // -------------------------------------------------------------
  // A. HEARTBEAT-UL CRITIC (La fiecare 1 secundă)
  // -------------------------------------------------------------
  if (currentTime - lastHeartbeat >= 1000) {
    lastHeartbeat = currentTime;
    modbusWriteSingleRegister(1, 327);
  }

  // -------------------------------------------------------------
  // B. CITIREA SENZORILOR SI EXPORT (La fiecare 3 secunde)
  // -------------------------------------------------------------
  if (currentTime - lastPoll >= 3000) {
    lastPoll = currentTime;
    
    uint16_t buffer[50]; 
    JsonDocument rxDoc;
    bool celPutinUnBlocCitit = false;

    // Bloc 2000 (Holding)
    if (modbusReadHoldingRegisters(2000, 5, buffer)) {
      rxDoc["2000"] = buffer[0];
      rxDoc["2001"] = buffer[1];
      rxDoc["2004"] = buffer[4];
      celPutinUnBlocCitit = true;
    }
    delay(20);

    // Bloc 1000 (Input)
    if (modbusReadInputRegisters(1000, 4, buffer)) {
      rxDoc["1001"] = buffer[1];
      rxDoc["1003"] = buffer[3];
      celPutinUnBlocCitit = true;
    }
    delay(20);

    // Bloc 300 (Input)
    if (modbusReadInputRegisters(300, 6, buffer)) {
      rxDoc["302"] = buffer[2];
      rxDoc["304"] = buffer[4];
      rxDoc["305"] = buffer[5];
      celPutinUnBlocCitit = true;
    }
    delay(20);

    // Bloc 8000 (Input - 46 Regiștri)
    if (modbusReadInputRegisters(8000, 46, buffer)) {
      rxDoc["8000"] = buffer[0];
      rxDoc["8001"] = buffer[1];
      rxDoc["8033"] = buffer[33];
      rxDoc["8045"] = buffer[45];
      celPutinUnBlocCitit = true;
    }

    // Publicare date
    if (celPutinUnBlocCitit && mqttClient.connected()) {
      String outputPayload;
      serializeJson(rxDoc, outputPayload);
      mqttClient.publish(TOPIC_RX, outputPayload.c_str());
    }
  }
}