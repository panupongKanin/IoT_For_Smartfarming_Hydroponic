/**
    ESP32-B With OTA version 2
    date time : Tue 30 May 2023
**/
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_system.h>


#define WiFi_LED_STATUS 2
#define MQTT_LED_STATUS 15

const char* ssid = "KOi";
const char* password = "13062544";
const char *mqtt_broker = "192.168.0.28";
const char *mqtt_username = "mqtt_user";
const char *mqtt_password = "myfarm";
AsyncWebServer server(80);

// Variable used for MQTT Discovery
bool g_InitSystem = true;
const char* g_deviceModel = "ESP32Device";                           // Hardware Model
const char* g_swVersion = "2.0";                                     // Firmware Version
const char* g_manufacturer = "Vigasan";                              // Manufacturer Name
String g_deviceName = "CustomPump";                                  // Device Name
String g_mqttStatusTopic = "esp32iotsensor/" + g_deviceName;         // MQTT Topic
String g_UniqueId;
bool g_SendMqttData = false;
int g_mqttCounterConn = 0;                                           // Device Name

const int mqtt_port = 1883;
WiFiClient espClient_B;
PubSubClient client(espClient_B);



// Pin Definitions
const int NUM_MOTORS = 4;

long dosingPumpPeriod[NUM_MOTORS] = {0, 0, 0, 0};    // Example: Array to store pump periods
long dosingPumpMillis[NUM_MOTORS] = {0, 0, 0, 0};    // Example: Array to store pump millis

// PWM properties
const int freq = 5000;
const int resolution = 8;

struct MotorPinConfig {
  int enablePin;
  int in1Pin;
  int in2Pin;
};

struct MotorPinPWM {
  int ontime;
  int minPWM;
  int maxPWM;
  int currentPWM;
};

// Pin Definitions
MotorPinConfig motorPins[NUM_MOTORS] = {
  {4, 18, 19},   // Motor A (ENA, IN1, IN2)
  {23, 21, 22},  // Motor B (ENB, IN3, IN4)
  {14, 27, 26},  // Motor C (ENC, IN5, IN6)
  {32, 25, 33}   // Motor D (END, IN7, IN8)
};

// Additional Pump Pin Definitions
const int pwmChannel[4] = {0, 1, 2, 3};                           // Example: Analog Output Pins used for pump speed control
const int dosingPumpEnablePin[4] = {4, 5, 6, 7};                  // Example: Digital Output Pins used to enable/disable pumps

MotorPinPWM motorConfig[NUM_MOTORS] = {
  {0, 0, 0, 0},              // ontime
  {0, 0, 0, 0},              // currentPWM
  {0, 0, 0, 0},              // minPWM
  {255, 255, 255, 255}       // maxPWM
};

struct SensorValues {
  float phSensorValue;
  float ecSensorValue;
  float minPhSensorValue;
  float maxPhSensorValue;
  float minEcSensorValue;
  float maxEcSensorValue;
};


void setup() {

  // Initialize Serial communication
  Serial.begin(115200);  // Set the baud rate to match your Serial Monitor


  // Configure LED pins as outputs
  pinMode(WiFi_LED_STATUS, OUTPUT);
  pinMode(MQTT_LED_STATUS, OUTPUT);

  digitalWrite(WiFi_LED_STATUS, HIGH);
  digitalWrite(MQTT_LED_STATUS, HIGH);

  connectToWiFi();

  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(MqttReceiverCallback);

  // Configure motor control pins as outputs
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motorPins[i].enablePin, OUTPUT);
    pinMode(motorPins[i].in1Pin, OUTPUT);
    pinMode(motorPins[i].in2Pin, OUTPUT);
    setPumpSpeeds(i, 255);
  }

  //Configure LED PWM functionalitites
  for (int j = 0; j < NUM_MOTORS; j++)   // The condition in for loop is -1 because the noctua fan pwm channel is the last one and we will update it separately below
  {
    ledcSetup(pwmChannel[j], freq, resolution);
    ledcAttachPin(motorPins[j].enablePin, pwmChannel[j]);
  }
}

void loop() {

  // Get the current millis
  unsigned long currentMillis = millis();

  if (!client.connected())
  {
    connectToMQTTBroker();
  }

  for (int i = 0; i < NUM_MOTORS; i++) {

    if ((dosingPumpPeriod[i] > 0) && (currentMillis - dosingPumpMillis[i] >= dosingPumpPeriod[i])) {
      setPumpPower(i, 0);
      Serial.println("Done");

      dosingPumpMillis[i] = currentMillis;
    }
  }
  client.loop();
}

void setPumpSpeeds(int pumpNumber, int pumpSpeed) {
  ledcWrite(pwmChannel[pumpNumber], (motorConfig[pumpNumber].ontime > 0) ? pumpSpeed : 0);
}

void setPumpPower(int pumpNumber, long onTime) {
  char buff[10]; // Increase the size of the buffer

  ledcWrite(pwmChannel[pumpNumber], (onTime > 0) ? motorConfig[pumpNumber].currentPWM : 0);
  setMotorDirection(pumpNumber, false, onTime);

  if (onTime > 0) {
    dosingPumpMillis[pumpNumber] = millis();
    dosingPumpPeriod[pumpNumber] = onTime;

  } else {
    dosingPumpMillis[pumpNumber] = 0;
    dosingPumpPeriod[pumpNumber] = 0;

  }

  sprintf(buff, "%d:%d", pumpNumber, onTime > 0);
  Serial.println(buff);
  client.publish("feedback/dosing", buff);                        // Send feedback (<PUMP#>:<STATE>)
}

void setMotorDirection(int pumpNumber, bool clockwise, long onTime) {
  if (pumpNumber >= 0 && pumpNumber < NUM_MOTORS) {
    if (clockwise) {
      // Set direction (clockwise)
      digitalWrite(motorPins[pumpNumber].in1Pin, HIGH); // Set IN1 pin to HIGH
      digitalWrite(motorPins[pumpNumber].in2Pin, LOW);  // Set IN2 pin to LOW
    } else {
      // Set direction (counterclockwise)
      digitalWrite(motorPins[pumpNumber].in1Pin, LOW);   // Set IN1 pin to LOW
      digitalWrite(motorPins[pumpNumber].in2Pin, HIGH);  // Set IN2 pin to HIGH
    }
  }

  // Enable or disable motor based on onTime
  digitalWrite(motorPins[pumpNumber].enablePin, (onTime > 0) ? HIGH : LOW);
}

//void controlPumps(float phSensorValue, float ecSensorValue, float minPhSensorValue, float maxPhSensorValue, float minEcSensorValue, float maxEcSensorValue) {
//  const int PUMP_COUNT = 4;
//  bool pumpStatus[PUMP_COUNT] = {false}; // Initialize all pumps to be off
//
//  // Control pump 0 based on pH sensor
//  if (phSensorValue > maxPhSensorValue) {
//    pumpStatus[0] = true;  // Turn on pump 0
//  }
//
//  // Control pump 1 based on pH sensor
//  if (phSensorValue < minPhSensorValue) {
//    pumpStatus[1] = true;  // Turn on pump 1
//  }
//
//  // Control pump 2 and pump 3 based on EC sensor
//  if (ecSensorValue < minEcSensorValue) {
//    pumpStatus[2] = true;  // Turn on pump 2
//    pumpStatus[3] = true;  // Turn on pump 3
//  }
//
//  // Set the pump power based on the pump status
//  for (int i = 0; i < PUMP_COUNT; i++) {
//    setPumpPower(i, pumpStatus[i] ? 5000 : 0);  // Turn on/off the pump based on the status
//  }
//
//  // Publish the feedback message
//  if (pumpStatus[0] || pumpStatus[1] || pumpStatus[2] || pumpStatus[3]) {
//    client.publish("feedback/relays", "1:0:1");
//  } else {
//    client.publish("feedback/relays", "1:0:0");
//  }
//}


void blinkLED(int pin) {
  int ledSequence[] = {0, 1, 0, 1, 1, 1, 1, 1, 1, 1};
  const int LED_SEQUENCE_LENGTH = sizeof(ledSequence) / sizeof(ledSequence[0]);
  int timedelay = 100;  // Define the delay between blinks in milliseconds

  for (int i = 0; i < LED_SEQUENCE_LENGTH; i++) {
    digitalWrite(pin, ledSequence[i]);
    delay(timedelay);
  }
}




void connectToWiFi() {
  byte mac[6];
  byte wifiFailCount = 0;
  byte tooManyFailures = 10;
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    blinkLED(WiFi_LED_STATUS);
    wifiFailCount++;
    // Wait 5 seconds before retrying
    delay(5000);
    if (wifiFailCount >= tooManyFailures) {
      digitalWrite(WiFi_LED_STATUS, HIGH);
      ESP.restart();
    }
  }

  digitalWrite(WiFi_LED_STATUS, LOW);

  WiFi.macAddress(mac);
  g_UniqueId = String(mac[0], HEX) + String(mac[1], HEX) + String(mac[2], HEX) + String(mac[3], HEX) + String(mac[4], HEX) + String(mac[5], HEX);
  Serial.println(" ");
  Serial.print("Unique ID: ");
  Serial.println(g_UniqueId);
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "Hi! I am ESP32-B");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  Serial.println("HTTP server started");
}

void connectToMQTTBroker() {
  byte mqttFailCount = 0;
  byte tooManyFailures = 10;
  // Loop until we're reconnected
  while (!client.connected()) {
    if (mqttFailCount <= tooManyFailures) {
      String client_id = "esp32-B-client-";
      client_id += String(WiFi.macAddress());
      Serial.printf("The client %s connects to the public MQTT broker\n", g_deviceName.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        Serial.println("Public Home Assistant MQTT broker connected");
        // Subscribe
        client.subscribe("homeassistant/status");
        client.subscribe("control/dosing");
        client.subscribe("calibrate/dosing");
        client.subscribe("control/sensors");

        delay(100);
        /* After connect to MQTT Broker */
        digitalWrite(MQTT_LED_STATUS, LOW);
      } else {
        blinkLED(MQTT_LED_STATUS);
        mqttFailCount++;
        Serial.print("Failed. Count = ");
        Serial.println(mqttFailCount);
        Serial.println("...trying again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }

    // Restart ESP32 if mqttFailCount exceeds tooManyFailures
    if (mqttFailCount >= tooManyFailures) {
      digitalWrite(MQTT_LED_STATUS, HIGH);
      ESP.restart();
    }
  }
}

void MqttReceiverCallback(char* topic, byte * payload, unsigned int length) {
  char payloadStr[length]; // Create a char array that's 1 byte longer than the incoming payload to copy it to and make room for the null terminator so it can be treated as a string.
  memcpy(payloadStr, payload, length);
  payloadStr[length] = '\0'; // Null-terminate the string

  /***************** CALLBACK: Pump Speed Adjustments *****************/

  if (strcmp(topic, "calibrate/dosing") == 0) {
    int pumpNumber;
    int pwmVal;
    char* strtokIndx1;
    char delimiter[] = ":";

    strtokIndx1 = strtok(payloadStr, delimiter);
    if (strtokIndx1 != NULL) {
      pumpNumber = atoi(strtokIndx1); // Get the pump number
      strtokIndx1 = strtok(NULL, delimiter);
      pwmVal = atoi(strtokIndx1); // Get the on time
    }
    setPumpSpeeds(pumpNumber, pwmVal);
    motorConfig[pumpNumber].currentPWM = pwmVal;
  }

  /***************** CALLBACK: Dosing *****************/
  if (strcmp(topic, "control/dosing") == 0) {
    int pumpNumber;
    long onTime;
    char* strtokIndx2;
    char delimiter[] = ":";

    strtokIndx2 = strtok(payloadStr, delimiter);
    if (strtokIndx2 != NULL) {
      pumpNumber = atoi(strtokIndx2); // Get the pump number
      strtokIndx2 = strtok(NULL, delimiter);
      onTime = atol(strtokIndx2); // Get the on time
    }
    setPumpPower(pumpNumber, onTime);
    motorConfig[pumpNumber].ontime = onTime;

  }
}
