#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

const char* ssid = "Brawl Pass";
const char* password = "012345678900";
const char* mqtt_server = "10.216.11.195";
const int mqtt_port = 1883;

const char* mqtt_topic_pub = "lab3/data";   // Publish topic
const char* mqtt_topic_sub = "lab3/cmd";    // Subscribe topic (commands)

WiFiClient espClient;
PubSubClient client(espClient);

#define DHTPIN D2
#define DHTTYPE DHT11

const int FAN_PIN_A = D1;
const int FAN_PIN_B = D5;

const int BRAKE_BUTTON_PIN = D7;
const int BRAKE_LED_PIN = D8;
const int ALARM_LED_PIN = D3;

const float MIN_TEMP = 21.0;
const float MAX_TEMP = 23.0;
const int MIN_SPIN_PWM = 400;

bool isBrakeEngaged = false;
int windAngle = 0;
bool isOverheatAlarm = false;

const long sensorInterval = 5000;
const long directionInterval = 10000;
const long autoBrakeInterval = 10000;
const long blinkInterval = 500;
const long debounceDelay = 50;

unsigned long previousSensorMillis = 0;
unsigned long previousDirectionMillis = 0;
unsigned long previousBlinkMillis = 0;
unsigned long overheatStartTime = 0;

int brakeButtonState = HIGH;
int lastBrakeButtonState = HIGH;
unsigned long lastBrakeDebounceTime = 0;

DHT dht(DHTPIN, DHTTYPE);

void setFanSpeed(int speed) {
  analogWrite(FAN_PIN_A, speed);
  digitalWrite(FAN_PIN_B, LOW);
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("MQTT message [");
  Serial.print(topic);
  Serial.print("]: ");

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, payload, length);

  if (error) {
    Serial.print("JSON parse error: ");
    Serial.println(error.c_str());
    return;
  }

  
  if (doc["brake"].is<bool>()) {
    isBrakeEngaged = doc["brake"];
    Serial.printf("Brake set to: %d\n", isBrakeEngaged);
  }

  if (doc["fanSpeedPWM"].is<int>()) {
    int s = doc["fanSpeedPWM"];
    setFanSpeed(s);
    Serial.printf("Fan PWM overridden: %d\n", s);
  }

  if (doc["windDirection"].is<int>()) {
    windAngle = doc["windDirection"];
    Serial.printf("Wind angle overridden: %d\n", windAngle);
  }

  if (doc["alarm"].is<bool>()) {
    isOverheatAlarm = doc["alarm"];
    Serial.printf("Alarm manually set: %d\n", isOverheatAlarm);
  }
}


void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
}

void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("MQTT connecting...");
    String clientId = "WemosD1Mini-" + String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("connected");

      // !!! Subscribe to commands
      client.subscribe(mqtt_topic_sub);
      Serial.println("Subscribed to lab3/cmd");
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}


void publishData(float temp, int speed, int direction) {
  JsonDocument doc;
  doc["temperature"] = temp;
  doc["fanSpeedPWM"] = speed;
  doc["windDirection"] = direction;
  doc["brakeEngaged"] = isBrakeEngaged;

  char buffer[256];
  serializeJson(doc, buffer);

  client.publish(mqtt_topic_pub, buffer);
  Serial.print("Publish: ");
  Serial.println(buffer);
}


void handleButtons(unsigned long currentMillis) {
  int reading = digitalRead(BRAKE_BUTTON_PIN);

  if (reading != lastBrakeButtonState) {
    lastBrakeDebounceTime = currentMillis;
  }

  if ((currentMillis - lastBrakeDebounceTime) > debounceDelay) {
    if (reading != brakeButtonState) {
      brakeButtonState = reading;
      if (brakeButtonState == LOW) {
        isBrakeEngaged = !isBrakeEngaged;
      }
    }
  }

  lastBrakeButtonState = reading;
}

void handleLEDs(unsigned long currentMillis) {
  digitalWrite(BRAKE_LED_PIN, isBrakeEngaged);

  if (isOverheatAlarm) {
    if (currentMillis - previousBlinkMillis >= blinkInterval) {
      previousBlinkMillis = currentMillis;
      digitalWrite(ALARM_LED_PIN, !digitalRead(ALARM_LED_PIN));
    }
  } else {
    digitalWrite(ALARM_LED_PIN, LOW);
  }
}


void setup() {
  Serial.begin(115200);

  pinMode(FAN_PIN_A, OUTPUT);
  pinMode(FAN_PIN_B, OUTPUT);
  pinMode(BRAKE_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BRAKE_LED_PIN, OUTPUT);
  pinMode(ALARM_LED_PIN, OUTPUT);

  dht.begin();

  randomSeed(analogRead(A0));

  setup_wifi();

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}


void loop() {
  unsigned long currentMillis = millis();

  if (!client.connected()) reconnect_mqtt();
  client.loop();

  handleButtons(currentMillis);
  handleLEDs(currentMillis);

  if (currentMillis - previousDirectionMillis >= directionInterval) {
    previousDirectionMillis = currentMillis;
    windAngle = random(0, 360);
  }

  if (currentMillis - previousSensorMillis >= sensorInterval) {
    previousSensorMillis = currentMillis;

    float t = dht.readTemperature();
    if (isnan(t)) {
      Serial.println("DHT Fail");
      return;
    }

    // ---- Overheat logic ----
    if (t >= 23.0) {
      isOverheatAlarm = true;
      if (overheatStartTime == 0) overheatStartTime = currentMillis;
    } else {
      isOverheatAlarm = false;
      overheatStartTime = 0;
    }

    if (overheatStartTime > 0 && (currentMillis - overheatStartTime >= autoBrakeInterval)) {
      isBrakeEngaged = true;
      overheatStartTime = 0;
    }

    int fanSpeed;
    if (isBrakeEngaged) {
        fanSpeed = 0; 
    }
    else if (t <= MIN_TEMP) {
        fanSpeed = 0;
    }
    else if (t >= MAX_TEMP) {
        fanSpeed = 1023;
    }
    else {
        fanSpeed = map(t, MIN_TEMP, MAX_TEMP, MIN_SPIN_PWM, 1023);
    }

    setFanSpeed(fanSpeed);

    publishData(t, fanSpeed, windAngle);
  }
}