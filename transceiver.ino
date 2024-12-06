#include <LoRa.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Bonezegei_SSD1306.h>
#include <ArduinoJson.h>
#include <OneWire.h>
#include <DallasTemperature.h>


// OLED Display Setup
Bonezegei_SSD1306 oled(128, 64);

// GPIOs
#define ANALOG_A1  4
#define ANALOG_A2  6
#define ANALOG_A3  5
#define ANALOG_A4  7
#define Relay1 48
#define IO0 0
#define DI1 16 //Digital input
#define DI2 15 //1-wire
int senderCounter = 0; // Beispiel für einen globalen Zähler

#define ONE_WIRE_BUS DI2 // Pin für den DS18B20
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// DIP-Schalter für LoRa-Adresse
const int DIP_SWITCHES[] = {47, 21, 14, 13, 12, 11, 10, 9};

// OLED I²C Pins
#define IICSDA 39
#define IICSCL 38

// LoRa SPI Pins
#define CS 41

#define MOSI 44
#define MISO 43
#define SCK 42

//buttons
#define RST 2
#define DIO0 40

// MQTT Parameter
const char* ssid = "xxx";
const char* password = "xxx";
const char* mqttServer = "192.168.xxx.xxx";
const char* mqttUserName = "xxxxx";
const char* mqttPwd = "xxxxx";
const char* clientID = "MGS-PV-Park";
int mqttPort = 1883;
int actualaddress = false;


WiFiClient espClient;
PubSubClient mqttClient(espClient);


// Globale Variablen
#define NUM_NODES 254
bool registeredNodes[NUM_NODES] = {false};

struct LoRaData {
    int self_addr;
    int inword;
    int outword;
    float analog_f[4];
    float wire1_data0;
    int relay_on;
} loraNodes[NUM_NODES];

// Funktionsprototypen
void setupWiFi();
void setupMQTT();
void registerDevice(int address, const LoRaData& data);
void updateMQTTState(int address, const LoRaData& data);
void processLoRaPacket();
void Sender_ProcessLoRaCommands();
void Sender_checkButtonAndControlRelay();
void Sender_sendPeriodicLoRaPacket();
void Sender_updateDisplay();


// Globale Variablen
int counter = 0;
int toggleState_1 = 1;
String line1text = "  Park-Light V1.0";
String line2text = "", line3text = "", line4text = "",line5text = "";
String Role;

void setup() {
  Serial.begin(921600);

  // GPIO-Initialisierung
  pinMode(Relay1, OUTPUT);
  digitalWrite(Relay1, HIGH);
  for (int i = 0; i < 8; i++) pinMode(DIP_SWITCHES[i], INPUT);
  pinMode(IO0, INPUT);
  pinMode(DI1, INPUT);
  pinMode(DI2, INPUT);

  // OLED Setup
  Wire.begin(IICSDA, IICSCL);
  oled.begin();
  oled.clear();
  oled.drawText(0, 0, line1text.c_str(), oled.Font_Arial8);
  oled.draw();

  // WiFi Setup
  setup_wifi();

  // LoRa Setup
  setup_lora();
  setupMQTT();
  sensors.begin(); // DS18B20 initialisieren
}

void setup_wifi() {
  delay(10);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi verbunden. IP-Adresse: " + WiFi.localIP().toString());
}

void Read_DIPswitch_and_calc_Role() {
  uint8_t address = 0;
  for (int i = 0; i < 8; i++) {
    address |= (digitalRead(DIP_SWITCHES[i]) << i);
  }
  actualaddress = address = 255 - address;

  Role = (address == 0 || address == 255) ? "Receiver" : "Sender";
  line2text = " Node-address = " + String(address);
  line3text = " Role = " + Role;
  line4text = "",line5text = "";
}

void updateDisplay() {
  oled.clear();
  oled.drawText(0, 0,  line1text.c_str(), oled.Font_Arial12);
  oled.drawText(0, 16, line2text.c_str(), oled.Font_Arial8);
  oled.drawText(0, 26, line3text.c_str(), oled.Font_Arial8);
  oled.drawText(0, 36, line4text.c_str(), oled.Font_Arial10);
  oled.drawText(0, 48, line5text.c_str(), oled.Font_Arial10);
  oled.draw();
}

void loop() {
  Read_DIPswitch_and_calc_Role();
  //if (Role == "Receiver") {
  if (Role == "xxx") {  
    processLoRaPacket();
    mqttClient.loop();

  } else {
    Serial.println("Sender-Modus aktiv.");
    Sender_ProcessLoRaCommands();    //check for relay update vie Lora
    Sender_checkButtonAndControlRelay(); // check button for manual switch of leay
    Sender_sendPeriodicLoRaPacket();  //broadcast lora update for sensordata to receiver
    delay(2000);
  }
}

//-----------------------------------------------------------------------------------------------------
//-------------------------------------------------ROLE Receiver---------------------------------------
//-----------------------------------------------------------------------------------------------------


//--------------------------------------------------LORA----------------------------------------------------
// Verbindet mit Lora Modul 433MHz
void setup_lora() {
  LoRa.setPins(CS, RST, DIO0);
  SPI.begin(SCK, MISO, MOSI, CS);
  while (!LoRa.begin(433E6)) {
    Serial.println("LORA Init failed.");
    delay(500);
  }
  LoRa.setSyncWord(0xA5);
  Serial.println("LORA Init success!");
}  

// execute received LoRa-Package
void processLoRaPacket() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
        String receivedPayload = "";
        while (LoRa.available()) receivedPayload += (char)LoRa.read();
        Serial.println("LORA received: " + receivedPayload);

        StaticJsonDocument<256> doc;
        if (deserializeJson(doc, receivedPayload)) {
            Serial.println("LORA JSON-error!");
            return;
        }
        updateDisplay();
        int address = doc["self_addr"];
        LoRaData data = {
            address,
            doc["inword"],
            doc["outword"],
            {doc["analog_f"][0], doc["analog_f"][1], doc["analog_f"][2], doc["analog_f"][3]},
            doc["wire1_data0"],
            doc["relay"]["on"]
        };

        if (!registeredNodes[address]) registerDevice(address, data);
        updateMQTTState(address, data);
    }
}

// Funktion zum Senden des LoRa-Kommandos für das Relay
void sendLoRaRelayCommand(int address, bool relayOn) {
    char loRaPayload[100];
    snprintf(loRaPayload, sizeof(loRaPayload), "{\"self_addr\":%d,\"relay\":\"%s\"}", address, relayOn ? "on" : "off");

    Serial.printf("LORA Sending Relay command: %s\n", loRaPayload);
    LoRa.beginPacket();
    LoRa.print(loRaPayload);
    LoRa.endPacket();
}


//--------------------------------------------------MQTT----------------------------------------------------
// Verbindet mit MQTT
void setupMQTT() {
    mqttClient.setServer(mqttServer, mqttPort);
    while (!mqttClient.connected()) {
    if (mqttClient.connect(clientID, mqttUserName, mqttPwd)) {
      Serial.println("MQTT connected.");
      mqttClient.setBufferSize(512);
    } else {
      Serial.println("MQTT-connection failed. Will repeate in 5 sec.");
      delay(5000);
    }
  }
}

void ListenMqttRelay() {
    // MQTT-Client initialisieren und Callback setzen
    mqttClient.setCallback(mqttCallback);

    // Alle MQTT-Topics für registrierte Nodes abonnieren
    for (int addr = 0; addr < NUM_NODES; addr++) {
        if (registeredNodes[addr]) {
            char commandTopic[100];
            sprintf(commandTopic, "homeassistant/sensor/alrha%08X/params/set", addr);
            mqttClient.subscribe(commandTopic);

            Serial.printf("MQTT Subscribed to topic: %s\n", commandTopic);
        }
    }
}

// Register new device and send discovery topics
void registerDevice(int address, const LoRaData& data) {
    if (registeredNodes[address]) return;

    Serial.printf("MQTT New device %08X to be registered in MQTT-Broker:\n", address);

    char configTopic[1024];
    char payload[1024];
    char stateTopic[1024];
    char commandTopic[1024];

    // State-& Command-Topics for switch & Sensors
    
    sprintf(stateTopic,   "homeassistant/sensor/alrha%08X/params/state", address);
    sprintf(commandTopic, "homeassistant/sensor/alrha%08X/params/set", address);
    
    // Relay
    sprintf(configTopic, "homeassistant/switch/alr_ALR_switch/config");
    snprintf(payload, sizeof(payload),
      "{\"name\":\"relay\","
      "\"state_on\":1,"
      "\"state_off\":0,"
      "\"payload_on\":\"1\","
      "\"payload_off\":\"0\","
      "\"value_template\":\"{{ value_json.relay.on | string }}\","
      "\"state_topic\":\"%s\","
      "\"command_topic\":\"%s\","
      "\"unique_id\":\"alr_switch\","
      "\"device\":{"
        "\"identifiers\":[\"alr\"],"
        "\"name\":\"alr evb\","
        "\"model\":\"ALR\","
        "\"manufacturer\":\"KinCony\""
      "}}",
      stateTopic, 
      commandTopic);

    Serial.printf("MQTT Config topic:%sPayload:%s\n", configTopic, payload);
    mqttClient.publish(configTopic, payload);

    // Sensors
    const char* sensorNames[] = {"inword", "outword", "analog_A1", "analog_A2", "analog_A3", "analog_A4", "DS18B20"};
    const char* valueTemplates[] = {"{{ value_json.inword }}", "{{ value_json.outword }}", "{{ value_json.analog_f[0] }}",
                                    "{{ value_json.analog_f[1] }}", "{{ value_json.analog_f[2] }}", "{{ value_json.analog_f[3] }}",
                                    "{{ value_json.wire1_data0 }}"};

    for (int i = 0; i < 7; i++) {
      delay(100);
      //sprintf(configTopic,  "homeassistant/sensor/alrha%08X/alr_ALR_param%d/config", address, i);
      sprintf(configTopic,  "homeassistant/sensor/alr_ALR_param%d/config", i);

      snprintf(payload, sizeof(payload),
        "{\"name\":\"%s\","
        "\"unit_of_measurement\":\"%s\","
        "\"value_template\":\"%s\","
        "\"state_topic\":\"%s\","
        "\"command_topic\":\"%s\","
        "\"unique_id\":\"alr_param%d\","
        "\"device\":{"
          "\"identifiers\":[\"alr\"],"
          "\"name\":\"alr evb\","
          "\"model\":\"ALR\","
          "\"manufacturer\":\"KinCony\""
          "}}",
        sensorNames[i],                          // Name des Sensors
        (i > 1 && i < 6) ? "V" : " ",            // Einheit (z. B. Volt) für 4 ADs
        valueTemplates[i],                       // Template für den Wert
        stateTopic,                              // State Topic
        commandTopic,                            // Command Topic
        i);                                      // Eindeutige ID
        
      int payloadLength = strlen(payload);
      //Serial.printf("MQTT(%d)bytes", payloadLength);
      Serial.printf("MQTT Config topic:%s Payload:%s\n", configTopic, payload);
      mqttClient.publish(configTopic, payload);
    
    }

    registeredNodes[address] = true;
    ListenMqttRelay();
    Serial.printf("MQTT Device %08X Sensor Topics registered!\n", address);
}



// update MQTT-State values
void updateMQTTState(int address, const LoRaData& data) {
    StaticJsonDocument<256> jsonDoc;
    jsonDoc["self_addr"] = data.self_addr;
    jsonDoc["inword"] = data.inword;
    jsonDoc["outword"] = data.outword;
    JsonArray analogF = jsonDoc.createNestedArray("analog_f");
    
    //for (int i = 0; i < 4; i++) analogF.add(data.analog_f[i]);
    
    // Werte in analog_f explizit mit drei Nachkommastellen als String hinzufügen
    for (int i = 0; i < 4; i++) {
        char formattedValue[10];
        snprintf(formattedValue, sizeof(formattedValue), "%.3f", data.analog_f[i]);
        analogF.add(formattedValue); // Füge die formatierte Zahl als String hinzu
    }

    //jsonDoc["wire1_data0"] = data.wire1_data0;
    // Drahtsensorwert ebenfalls runden, falls nötig
    char wire1Formatted[10];
    snprintf(wire1Formatted, sizeof(wire1Formatted), "%.3f", data.wire1_data0);
    jsonDoc["wire1_data0"] = atof(wire1Formatted); // Optional: kann auch String bleiben


    jsonDoc["relay"]["on"] = data.relay_on;

    char buffer[512];
    serializeJson(jsonDoc, buffer, sizeof(buffer));

    char stateTopic[55];
    sprintf(stateTopic, "homeassistant/sensor/alrha%08X/params/state", address);
    //sprintf(stateTopic, "homeassistant/sensor/alrhaDCDA0C7B7BD4/params/state");
    
    // MQTT-Payload serial ausgeben
    Serial.print("MQTT State Updated: ");
    Serial.print(stateTopic);
    Serial.print(": ");
    Serial.println(buffer);

    // MQTT-Payload senden
    mqttClient.publish(stateTopic, buffer);
}

// Funktion, die auf MQTT-Nachrichten reagiert
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    // Nachricht in einen String umwandeln
    char message[length + 1];
    strncpy(message, (char*)payload, length);
    message[length] = '\0';  // Nullterminator hinzufügen

    Serial.printf("MQTT Message received on topic: %s, Message: %s\n", topic, message);

    // Über alle registrierten Nodes iterieren
    for (int addr = 0; addr < NUM_NODES; addr++) {
        if (registeredNodes[addr]) {
            char commandTopic[100];
            sprintf(commandTopic, "homeassistant/sensor/alrha%08X/params/set", addr);

            if (strcmp(topic, commandTopic) == 0) {
                // Verarbeiten der Nachricht
                if (strcmp(message, "1") == 0) {
                    // Relay einschalten
                    Serial.printf("MQTT Relay ON command received for Node %d. Sending LORA message...\n", addr);
                    sendLoRaRelayCommand(addr, true);
                } else if (strcmp(message, "0") == 0) {
                    // Relay ausschalten
                    Serial.printf("MQTT Relay OFF command received for Node %d. Sending LORA message...\n", addr);
                    sendLoRaRelayCommand(addr, false);
                }
                break; // Kein weiteres Matching nötig, sobald die Adresse gefunden wurde
            }
        }
    }
}


//-----------------------------------------------------------------------
//-------------------------------ROLE SENDER-----------------------------
//-----------------------------------------------------------------------


// Task 1: Auf LoRa-Paket mit Relay-Befehl warten
void Sender_ProcessLoRaCommands() {
  if (LoRa.parsePacket()) {
    StaticJsonDocument<256> receivedJson;
    char receivedData[256];
    int len = LoRa.readBytes(receivedData, sizeof(receivedData) - 1);
    receivedData[len] = '\0';

    DeserializationError error = deserializeJson(receivedJson, receivedData);
    if (error) {
      Serial.println("Fehler beim Deserialisieren des LoRa-Pakets!");
      return;
    }

    int selfAddr = receivedJson["self_addr"];
    if (selfAddr == actualaddress) {
      const char* relayState = receivedJson["relay"];
      if (strcmp(relayState, "on") == 0) {
        digitalWrite(Relay1, HIGH);
        Serial.println("Relay ON");
      } else if (strcmp(relayState, "off") == 0) {
        digitalWrite(Relay1, LOW);
        Serial.println("Relay OFF");
      }
    }
  }
}

// Task 2: Relay bei Tastendruck ansteuern
void Sender_checkButtonAndControlRelay() {
  static bool lastButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;

  bool buttonState = digitalRead(DIO0);
  Serial.println(buttonState ? "Relay OFF (Taste)" : "Relay ON (Taste)");

  if (buttonState != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (buttonState == LOW) {
      bool currentRelayState = digitalRead(Relay1);
      digitalWrite(Relay1, !currentRelayState);
      Serial.println(currentRelayState ? "Relay OFF (Taste)" : "Relay ON (Taste)");
    }
  }
  lastButtonState = buttonState;
}

// Task 3: Alle 5 Sekunden Sensorzustand senden
void Sender_sendPeriodicLoRaPacket() {
  static unsigned long lastSendTime = 0;
  const unsigned long sendInterval = 10000; //ms

  if (millis() - lastSendTime >= sendInterval) {
    lastSendTime = millis();

    StaticJsonDocument<256> jsonDoc;
    jsonDoc["self_addr"] = actualaddress;
    jsonDoc["inword"] = digitalRead(DIO0);
    jsonDoc["outword"] = digitalRead(Relay1);

    JsonArray analogF = jsonDoc.createNestedArray("analog_f");
    analogF.add(analogRead(ANALOG_A1) / 1000.0);
    analogF.add(analogRead(ANALOG_A2) / 1000.0);
    analogF.add(analogRead(ANALOG_A3) / 1000.0);
    analogF.add(analogRead(ANALOG_A4) / 1000.0);

    jsonDoc["wire1_data0"] = readDS18B20Temperature();
    jsonDoc["relay"]["on"] = digitalRead(Relay1);

    char buffer[256];
    serializeJson(jsonDoc, buffer);

    LoRa.beginPacket();
    LoRa.print(buffer);
    LoRa.endPacket();
    Sender_updateDisplay();

    senderCounter++;
    Serial.printf("LoRa Package sent: %s\n", buffer);
  }
}

// Task 4: Display aktualisieren
void Sender_updateDisplay() {
  float temperature = readDS18B20Temperature();

  // Texte vorbereiten
  line3text = "LoRa cnt_sent: " + String(senderCounter);
  line4text = "18b20= " + String(temperature, 2) + " C";
  line5text = "cts  = " + String(temperature, 2) + " C";
  updateDisplay();
}

// Task5 reading DS18B20 values
float readDS18B20Temperature() {
    sensors.requestTemperatures();        // Temperaturdaten anfordern
    float temperature = sensors.getTempCByIndex(0); // Erste verfügbare Temperatur auslesen
    if (temperature == DEVICE_DISCONNECTED_C) {
        Serial.println("no DS18B20 detected!");
        return -127.0; // Fehlerwert zurückgeben
    }
    return temperature; // Temperatur in °C zurückgeben
}
