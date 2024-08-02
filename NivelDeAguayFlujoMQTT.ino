#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Configuración de WiFi
const char* ssid = "IZZI-D4D2";
const char* password = "COR202D4D2";

// Configuración de MQTT
const char* mqtt_server = "broker.hivemq.com";
const int mqtt_port = 1883;
const char* mqtt_topic = "100236950";

WiFiClient espClient;
PubSubClient client(espClient);

// Pines para las entradas
const int pin25 = 12; // GPIO 12 para 25%
const int pin50 = 13; // GPIO 13 para 50%
const int pin75 = 14; // GPIO 14 para 75%
const int pin100 = 15; // GPIO 15 para 100%

const int flowSensorPin = 4;  // Pin conectado al cable amarillo del sensor YF-S201
volatile uint32_t pulseCount = 0;
unsigned long oldTime = 0;
float calibrationFactor = 4.5;  // Factor de calibración para el YF-S201

void IRAM_ATTR pulseCounter() {
  pulseCount++;
}

void setup() {
  Serial.begin(115200);

  // Configurar pines de entrada
  pinMode(pin25, INPUT);
  pinMode(pin50, INPUT);
  pinMode(pin75, INPUT);
  pinMode(pin100, INPUT);
  
  // Configurar pin del sensor de flujo
  pinMode(flowSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);

  // Conexión a WiFi
  setup_wifi();
  
  // Configuración del cliente MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void setup_wifi() {
  delay(10);
  Serial.println();
  Serial.print("Conectando a ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi conectado");
  Serial.println("Dirección IP: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Callback para mensajes entrantes (no utilizado en este ejemplo)
}

void reconnect() {
  // Reintenta conexión a MQTT hasta que tenga éxito
  while (!client.connected()) {
    Serial.print("Intentando conexión MQTT...");
    if (client.connect("ESP32CAMClient")) {
      Serial.println("conectado");
    } else {
      Serial.print("falló, rc=");
      Serial.print(client.state());
      Serial.println(" intentando en 5 segundos");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // Leer y enviar datos del sensor de flujo
  float flowRate = 0.0;
  if ((millis() - oldTime) > 1000) {  // Calcular cada segundo
    detachInterrupt(digitalPinToInterrupt(flowSensorPin));
    
    flowRate = ((1000.0 / (millis() - oldTime)) * pulseCount) / calibrationFactor;
    oldTime = millis();
    pulseCount = 0;

    Serial.print("Caudal: ");
    Serial.print(flowRate);
    Serial.println(" L/min");

    attachInterrupt(digitalPinToInterrupt(flowSensorPin), pulseCounter, FALLING);
  }

  // Leer las entradas
  int nivel = 0;
  if (digitalRead(pin25) == HIGH && digitalRead(pin50) == LOW && digitalRead(pin75) == LOW && digitalRead(pin100) == LOW) {
    nivel = 25;
  } else if (digitalRead(pin25) == HIGH && digitalRead(pin50) == HIGH && digitalRead(pin75) == LOW && digitalRead(pin100) == LOW) {
    nivel = 50;
  } else if (digitalRead(pin25) == HIGH && digitalRead(pin50) == HIGH && digitalRead(pin75) == HIGH && digitalRead(pin100) == LOW) {
    nivel = 75;
  } else if (digitalRead(pin25) == HIGH && digitalRead(pin50) == HIGH && digitalRead(pin75) == HIGH && digitalRead(pin100) == HIGH) {
    nivel = 100;
  } else if (digitalRead(pin25) == LOW && digitalRead(pin50) == LOW && digitalRead(pin75) == LOW && digitalRead(pin100) == LOW) {
    nivel = 0;
  }

  // Crear mensaje JSON
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["niv2"] = nivel;
  jsonDoc["flu"] = flowRate;

  char buffer[200];
  size_t n = serializeJson(jsonDoc, buffer);

  Serial.print("Enviando mensaje JSON: ");
  Serial.println(buffer);

  client.publish(mqtt_topic, buffer, n);

  delay(1000); // Pequeño retraso para evitar spam de mensajes
}
