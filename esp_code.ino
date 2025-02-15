#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// WiFi Configuration
#define WIFI_SSID "WIFI_SSID"
#define WIFI_PASSWORD "WIFI_PASSWORD"

// MQTT Broker Configuration
#define MQTT_BROKER "test.mosquitto.org"
#define MQTT_PORT 1883
#define MQTT_CLIENT_ID "ESP8266_IoT_Device_9876"

// Pin Configurations
#define DHTPIN D2       // DHT Sensor Data Pin
#define DHTTYPE DHT22   // DHT Sensor Type (DHT11 or DHT22)
#define BULB_PIN D1     // LED/Relay Control Pin

// MQTT Topics
#define TEMPERATURE_TOPIC "home/sensors/temperature/9876"
#define HUMIDITY_TOPIC "home/sensors/humidity/9876"
#define BULB_CONTROL_TOPIC "home/devices/bulb/9876"

// Global Objects
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Sensor Reading Interval
unsigned long lastSensorReadTime = 0;
const long SENSOR_READ_INTERVAL = 5000; // 5 seconds

void setup() {
  // Initialize Serial Communication
  Serial.begin(115200);
  
  // Setup Pins
  pinMode(BULB_PIN, OUTPUT);
  digitalWrite(BULB_PIN, LOW);  // Ensure bulb is initially off
  
  // Initialize DHT Sensor
  dht.begin();
  
  // Connect to WiFi
  setupWiFi();
  
  // Configure MQTT
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}

void loop() {
  // Maintain MQTT Connection
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  
  // Read and Publish Sensor Data Periodically
  unsigned long currentMillis = millis();
  if (currentMillis - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentMillis;
    publishSensorData();
  }
}

void setupWiFi() {
  delay(10);
  Serial.println("\nConnecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void publishSensorData() {
  // Read Temperature and Humidity
  float temperature = 34.5;  //  dht.readTemperature();
  float humidity =59.3 //dht.readHumidity();
  
  // Check if reading was successful
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  
  // Create Payload String
  char temperaturePayload[50];
  char humidityPayload[50];
  snprintf(temperaturePayload, sizeof(temperaturePayload), "%.1f", temperature);
  snprintf(humidityPayload, sizeof(humidityPayload), "%.1f", humidity);
  
  // Publish Sensor Data
  mqttClient.publish(TEMPERATURE_TOPIC, temperaturePayload);
  mqttClient.publish(HUMIDITY_TOPIC, humidityPayload);
  
  // Debug Print
  Serial.print("Published Sensor Data: ");
  Serial.print("Temperature: ");
  Serial.print(temperaturePayload);
  Serial.print(", Humidity: ");
  Serial.println(humidityPayload);
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Convert payload to string
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  // Debug Print
  Serial.print("Received Message on Topic: ");
  Serial.print(topic);
  Serial.print(" - Message: ");
  Serial.println(message);
  
  // Check Bulb Control Topic
  if (String(topic) == BULB_CONTROL_TOPIC) {
    if (message == "ON") {
      digitalWrite(BULB_PIN, HIGH);
      Serial.println("Bulb Turned ON");
    } else if (message == "OFF") {
      digitalWrite(BULB_PIN, LOW);
      Serial.println("Bulb Turned OFF");
    }
  }
}

void reconnectMQTT() {
  // Loop until reconnected
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT Connection...");
    
    // Attempt to connect
    if (mqttClient.connect(MQTT_CLIENT_ID)) {
      Serial.println("Connected");
      
      // Subscribe to Bulb Control Topic
      mqttClient.subscribe(BULB_CONTROL_TOPIC);
      Serial.println("Subscribed to Bulb Control Topic");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds");
      
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}