#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// ==========================================
// 1. CONFIGURARE REȚEA & MQTT
// ==========================================
const char* ssid = "DIGI-d7Ft";
const char* password = "tame9tdvD9";

const char* mqtt_server = "192.168.1.1";
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
unsigned long lastWiFiCheck = 0; 
unsigned long lastMqttCheck = 0; // [FIX] Temporizator pentru reconectare MQTT

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
    delay(1); // [FIX] Permite rularea proceselor WiFi in background
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
    delay(1); // [FIX] Evita trigger WDT
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
    delay(1); // [FIX] Stabilizeaza stiva TCP/IP
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
  // [FIX] FARA STRING-URI AICI. Parsam direct array-ul de octeti. 
  // Astfel eliminam complet fragmentarea de Heap.
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);
  
  if (error) {
    Serial.print("Eroare parsare JSON: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("reg") && doc.containsKey("val")) {
    uint16_t registru = doc["reg"];
    uint16_t valoare = doc["val"];
    Serial.printf("⚡ [DRIVER TX] Scriere reg %d -> Val %d\n", registru, valoare);
    modbusWriteSingleRegister(registru, valoare);
  }
}

void checkWiFiAndMQTT() {
  unsigned long currentMillis = millis();

  // 1. Verificare WiFi Non-Blocantă
  if (WiFi.status() != WL_CONNECTED) {
    if (currentMillis - lastWiFiCheck >= 10000) {
      Serial.println("🔗 [WIFI] Conexiune pierduta. Se reincearca...");
      WiFi.disconnect();
      WiFi.begin(ssid, password);
      lastWiFiCheck = currentMillis;
    }
    return; 
  }

  // 2. Verificare MQTT Non-Blocantă
  if (!mqttClient.connected()) {
    // [FIX] Reconectare MQTT protejata temporal, ca sa nu blocheze Modbus-ul si Loop-ul
    if (currentMillis - lastMqttCheck >= 5000) {
      Serial.println("🔗 [MQTT] Conectare la Mosquitto...");
      if (mqttClient.connect("Esp32_DriverModbus")) {
        Serial.println("✅ [MQTT] Conectat cu succes!");
        mqttClient.subscribe(TOPIC_TX);
      }
      lastMqttCheck = currentMillis;
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
  
  // [FIX] MARIRE BUFFER MQTT. Implicit este 256. Payload-ul tau din loop poate depasi asta.
  mqttClient.setBufferSize(1024); 

  Serial1.begin(BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("✅ [DRIVER INIT] Modbus activat. Let's go!");
}

// ==========================================
// 7. BUCLA PRINCIPALĂ
// ==========================================
void loop() {
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

    if (modbusReadHoldingRegisters(2000, 5, buffer)) {
      rxDoc["2000"] = buffer[0];
      rxDoc["2001"] = buffer[1];
      rxDoc["2004"] = buffer[4];
      celPutinUnBlocCitit = true;
    }
    delay(20);

    if (modbusReadInputRegisters(1000, 4, buffer)) {
      rxDoc["1001"] = buffer[1];
      rxDoc["1003"] = buffer[3];
      celPutinUnBlocCitit = true;
    }
    delay(20);

    if (modbusReadInputRegisters(300, 37, buffer)) {
      rxDoc["302"] = buffer[2];
      rxDoc["304"] = buffer[4];
      rxDoc["305"] = buffer[5];
      rxDoc["336"] = buffer[36]; // Presiune instalatie (val / 10.0 bar)
      celPutinUnBlocCitit = true;
    }
    delay(20);

    if (modbusReadInputRegisters(8000, 46, buffer)) {
      rxDoc["8000"] = buffer[0];
      rxDoc["8001"] = buffer[1];
      rxDoc["8033"] = buffer[33];
      rxDoc["8045"] = buffer[45];
      celPutinUnBlocCitit = true;
    }

    if (celPutinUnBlocCitit && mqttClient.connected()) {
      String outputPayload;
      serializeJson(rxDoc, outputPayload);
      mqttClient.publish(TOPIC_RX, outputPayload.c_str());
    }
  }
}