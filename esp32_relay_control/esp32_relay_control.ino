#include <WiFi.h>
#include <PubSubClient.h>

// WiFi credentials
const char* ssid = "vanhoa";
const char* password = "11111111";

// MQTT Broker settings
const char* mqtt_server = "192.168.137.241";  // Pi's IP address
const char* mqtt_username = "";  // Leave empty if no credentials
const char* mqtt_password = "";  // Leave empty if no credentials
const int mqtt_port = 1883;
const char* mqtt_topic_relay1 = "home/relay1";
const char* mqtt_topic_relay2 = "home/relay2";
const char* mqtt_status_relay1 = "home/relay1/status";
const char* mqtt_status_relay2 = "home/relay2/status";

// Pin configuration
const int RELAY1_PIN = 26;  // GPIO26
const int RELAY2_PIN = 27;  // GPIO27
const int LED1_PIN = 2;     // Built-in LED
const int LED2_PIN = 4;     // External LED

// State tracking
bool relay1_state = false;
bool relay2_state = false;

WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println("Kết nối WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("Đã kết nối WiFi");
  Serial.println("IP: ");
  Serial.println(WiFi.localIP());
}

// Update relay state and publish status
void updateRelay(int pin, int led_pin, bool &state, const char* status_topic, bool newState) {
  state = newState;
  digitalWrite(pin, state ? HIGH : LOW);
  digitalWrite(led_pin, state ? HIGH : LOW);
  client.publish(status_topic, state ? "ON" : "OFF", true);
  Serial.print("Cập nhật trạng thái: ");
  Serial.print(status_topic);
  Serial.println(state ? " -> ON" : " -> OFF");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  bool newState = (message == "ON");
  
  if (String(topic) == mqtt_topic_relay1) {
    updateRelay(RELAY1_PIN, LED1_PIN, relay1_state, mqtt_status_relay1, newState);
  }
  else if (String(topic) == mqtt_topic_relay2) {
    updateRelay(RELAY2_PIN, LED2_PIN, relay2_state, mqtt_status_relay2, newState);
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Kết nối MQTT...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("thành công");
      
      // Subscribe to control topics
      client.subscribe(mqtt_topic_relay1);
      client.subscribe(mqtt_topic_relay2);
      
      // Publish initial states
      client.publish(mqtt_status_relay1, relay1_state ? "ON" : "OFF", true);
      client.publish(mqtt_status_relay2, relay2_state ? "ON" : "OFF", true);
    } else {
      Serial.print("thất bại, lỗi = ");
      Serial.print(client.state());
      Serial.println(" thử lại sau 5 giây");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  
  // Set initial states
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  // Visual feedback through built-in LED
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED1_PIN, HIGH);   // LED sáng khi mất kết nối WiFi
    delay(100);
    digitalWrite(LED1_PIN, LOW);
    delay(100);
  }
}
