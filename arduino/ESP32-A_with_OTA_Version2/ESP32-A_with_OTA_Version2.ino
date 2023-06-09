/**
    ESP32-A With OTA version 2
    date time : Tue 30 May 2023
**/
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <DHT.h> //https://www.arduinolibraries.info/libraries/dht-sensor-library
#include <LiquidCrystal_I2C.h>
#include <ArduinoJson.h>
#include "DFRobot_ESP_EC.h"   //https://github.com/GreenPonik/DFRobot_ESP_EC_BY_GREENPONIK.git
#include "DFRobot_ESP_PH.h"
#include "Adafruit_ADS1015.h" //https://github.com/GreenPonik/Adafruit_ADS1X15.git
#include "EEPROM.h"

#define WiFi_LED_STATUS 2
#define MQTT_LED_STATUS 15
#define ONE_WIRE_BUS 23                                /* <-- ESP32 pin GIOP05 connected to DS18B20 Temperature sensor */
#define PH_PIN 35                                      /* <-- ESP32 pin GIOP34 connected to pH sensor */
#define DHT_SENSOR_PIN  4                              /* <-- ESP32 pin GIOP33 connected to DHT22 sensor */
#define DHT_SENSOR_TYPE DHT22

const char* ssid = "KOi";
const char* password = "13062544";
const char *mqtt_broker = "192.168.0.28";
const char *mqtt_username = "mqtt_user";
const char *mqtt_password = "myfarm";
const int mqtt_port = 1883;

WiFiClient espClient_A;
PubSubClient client(espClient_A);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);                  /* <-- Pass our oneWire reference to Dallas DS18B20 Temperature sensor */
DHT dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);
DFRobot_ESP_PH ph;
DFRobot_ESP_EC ec;                                    /* <-- Pass our oneWire reference to EC SENSORS sensor */
Adafruit_ADS1115 ads;                                 /* <-- ADS1115 */
AsyncWebServer server(80);

/* Variable used for MQTT Discovery */
bool g_InitSystem = true;
const char* g_deviceModel = "ESP32Device";                    // Hardware Model
const char* g_swVersion = "2.0";                              // Firmware Version
const char* g_manufacturer = "Vigasan";                       // Manufacturer Name
String g_deviceName = "CustomSensor";                         // Device Name
String g_mqttStatusTopic = "esp32iotsensor/" + g_deviceName;  // MQTT Topic
String g_UniqueId;
bool g_SendMqttData = false;
int g_mqttCounterConn = 0;





unsigned long previousMillis = 0;
const unsigned long interval = 15000; // 60 seconds interval

// Relays
int relayPins[2][8]
{
  {14, 26, 27, 25},                               // 4 Chan Relay Board0: [0]= RELAY CH1, [1]= RELAY CH2 [2]= RELAY CH3 [3]= RELAY CH4
  {18, 19}                                        // 2 Chan Relay Board1: [0]= RELAY CH1, [1]= RELAY CH2
};

struct SensorDataConfig {
  bool enableDHT22;
  bool enableDS18B20;
  bool enableECSensor;
  bool enablePHSensor;

  // Sensor values
  float dht22Temperature;
  float dht22Humidity;
  float ds18b20Temperature;
  float ecSensorValue;
  float pHSensorValue;

  // Sensor status
  bool dht22Status;
  bool ds18b20Status;
  bool ecSensorStatus;
  bool pHSensorStatus;

};
SensorDataConfig sensorData;

#define ESPADC 4096.0   //the esp Analog Digital Convertion value
#define ESPVOLTAGE 3300 //the esp voltage supply value
float ecVoltage, ecValue, phVoltage, phValue, temperature = 25;
/**
  Read Serial to launch PH or EC listener
*/
unsigned long last[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool calibrationIsRunning = false;

int i = 0;
bool readSerial(char result[])
{
  while (Serial.available() > 0)
  {
    char inChar = Serial.read();
    if (inChar == '\n')
    {
      result[i] = '\0';
      Serial.flush();
      i = 0;
      return true;
    }
    if (inChar != '\r')
    {
      result[i] = inChar;
      i++;
    }
    delay(1);
  }
  return false;
}




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
      Serial.printf("The client %s connects to the public mqtt broker\n", g_deviceName.c_str());
      if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
        Serial.println("Public Homezassistant mqtt broker connected");
        // Subscribe
        client.subscribe("homeassistant/status");
        client.subscribe("control/relays");

        delay(100);
        /* After connect to MQTT Brocker */
        digitalWrite(MQTT_LED_STATUS, LOW);

      } else {
        blinkLED(MQTT_LED_STATUS);
        mqttFailCount ++;
        Serial.print("Failed. Count = ");
        Serial.println(mqttFailCount);
        Serial.println("...trying again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
    }
  }
}

void triggerRelay(int boardNumber, int relayNumber, int relayTrigger)
{
  char buff[50];
  char relayfeedback[10];
  sprintf(buff, "Triggering board#:%d, relay#:%d, state: %d", boardNumber, relayNumber, relayTrigger);
  sprintf(relayfeedback, "%d:%d:%d", boardNumber, relayNumber, relayTrigger);
  Serial.println(buff);
  if (relayTrigger == 1)
  {
    digitalWrite(relayPins[boardNumber][relayNumber], LOW); // Turn relay ON
    client.publish("feedback/relays", relayfeedback);

    Serial.println(relayPins[boardNumber][relayNumber]);
  }
  else if (relayTrigger == 0)
  {
    digitalWrite(relayPins[boardNumber][relayNumber], HIGH); // Turn relay OFF
    client.publish("feedback/relays", relayfeedback);

    Serial.println(relayPins[boardNumber][relayNumber]);
  }
}

void MqttReceiverCallback(char* topic, byte * payload, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  char payloadStr[length + 1];              // Create a char array that's 1 byte longer than the incoming payload to copy it to and make room for the null terminator so it can be treated as string.
  memcpy(payloadStr, payload, length);
  payloadStr[length + 1] = '\0';



  if (String(topic) == String("homeassistant/status"))
  {
    if (payloadStr == "online")
      MqttHomeAssistantDiscovery();
  }

  /***************** CALLBACK: 4-Channel Relay Board (In Control Box) *****************/

  if (String(topic) == "control/relays") // Incoming message format will be <BOARD#>:<RELAY#>:<STATE>. STATE is "1" for on, "0" for off. Example payload: "1:1:0" = on board 1, turn relay 1 ON.
  {
    //    Serial.print("<Relay:");               // Print this command to the Mega since it handles the relays.
    Serial.println(payloadStr);
    //    Serial.println('>');
  }

  int boardNumber;
  int relayNumber;
  int relayPower;
  char* strtokIndx;
  char buff[20];
  char delimiter[] = ":";

  strtokIndx = strtok(payloadStr, delimiter);
  if (strtokIndx != NULL) {
    boardNumber = atoi(strtokIndx);
    strtokIndx = strtok(NULL, delimiter);
    relayNumber = atoi(strtokIndx);
    strtokIndx = strtok(NULL, delimiter);
    relayPower = atoi(strtokIndx);
  }

  triggerRelay(boardNumber, relayNumber, relayPower);

  sprintf(buff, "<Relay FB:%d:%d:%d>", boardNumber, relayNumber, relayPower);
  Serial.println(buff);
}


void MqttHomeAssistantDiscovery() {

  String discoveryTopic;
  String payload;
  String strPayload;

  if (client.connected()) {

    Serial.println("SEND HOME ASSISTANT DISCOVERY!!!");
    StaticJsonDocument<600> payload;
    JsonObject device;
    JsonArray identifiers;

    /*********************************************************************************************

       Air Temperature [DHT22 SENSOR]

    *********************************************************************************************/
    discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_airtemp" + "/config";

    payload["name"] = g_deviceName + ".airtemp";
    payload["uniq_id"] = g_UniqueId + "_airtemp";
    payload["stat_t"] = g_mqttStatusTopic;
    payload["dev_cla"] = "airtemperature";
    payload["val_tpl"] = "{{ value_json.airtemp | is_defined }}";
    payload["unit_of_meas"] = "°C";
    device = payload.createNestedObject("device");
    device["name"] = g_deviceName;
    device["model"] = g_deviceModel;
    device["sw_version"] = g_swVersion;
    device["manufacturer"] = g_manufacturer;
    identifiers = device.createNestedArray("identifiers");
    identifiers.add(g_UniqueId);

    serializeJsonPretty(payload, Serial);
    Serial.println(" ");
    serializeJson(payload, strPayload);

    client.publish(discoveryTopic.c_str(), strPayload.c_str());

    /*********************************************************************************************

       Air Humidity [DHT22 SENSOR]

     *********************************************************************************************/
    payload.clear();
    device.clear();
    identifiers.clear();
    strPayload.clear();

    discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_airhum" + "/config";

    payload["name"] = g_deviceName + ".airhum";
    payload["uniq_id"] = g_UniqueId + "_airhum";
    payload["stat_t"] = g_mqttStatusTopic;
    payload["dev_cla"] = "airhumidity";
    payload["val_tpl"] = "{{ value_json.airhum | is_defined }}";
    payload["unit_of_meas"] = "%";
    device = payload.createNestedObject("device");
    device["name"] = g_deviceName;
    device["model"] = g_deviceModel;
    device["sw_version"] = g_swVersion;
    device["manufacturer"] = g_manufacturer;
    identifiers = device.createNestedArray("identifiers");
    identifiers.add(g_UniqueId);

    serializeJsonPretty(payload, Serial);
    Serial.println(" ");
    serializeJson(payload, strPayload);

    client.publish(discoveryTopic.c_str(), strPayload.c_str());

    /*********************************************************************************************

       Water Temperature

     *********************************************************************************************/
    payload.clear();
    device.clear();
    identifiers.clear();
    strPayload.clear();

    discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_watertemp" + "/config";

    payload["name"] = g_deviceName + ".watertemp";
    payload["uniq_id"] = g_UniqueId + "_watertemp";
    payload["stat_t"] = g_mqttStatusTopic;
    payload["dev_cla"] = "watertemperature";
    payload["val_tpl"] = "{{ value_json.watertemp | is_defined }}";
    payload["unit_of_meas"] = "°C";
    device = payload.createNestedObject("device");
    device["name"] = g_deviceName;
    device["model"] = g_deviceModel;
    device["sw_version"] = g_swVersion;
    device["manufacturer"] = g_manufacturer;
    identifiers = device.createNestedArray("identifiers");
    identifiers.add(g_UniqueId);

    serializeJsonPretty(payload, Serial);
    Serial.println(" ");
    serializeJson(payload, strPayload);

    client.publish(discoveryTopic.c_str(), strPayload.c_str());

    /*********************************************************************************************

       pH SENSOR

     *********************************************************************************************/
    payload.clear();
    device.clear();
    identifiers.clear();
    strPayload.clear();

    discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_waterph" + "/config";

    payload["name"] = g_deviceName + ".waterph";
    payload["uniq_id"] = g_UniqueId + "_waterph";
    payload["stat_t"] = g_mqttStatusTopic;
    payload["dev_cla"] = "phsonsor";
    payload["val_tpl"] = "{{ value_json.waterph | is_defined }}";
    device = payload.createNestedObject("device");
    device["name"] = g_deviceName;
    device["model"] = g_deviceModel;
    device["sw_version"] = g_swVersion;
    device["manufacturer"] = g_manufacturer;
    identifiers = device.createNestedArray("identifiers");
    identifiers.add(g_UniqueId);

    serializeJsonPretty(payload, Serial);
    Serial.println(" ");
    serializeJson(payload, strPayload);

    client.publish(discoveryTopic.c_str(), strPayload.c_str());

    /*********************************************************************************************

           EC SENSOR

     *********************************************************************************************/
    payload.clear();
    device.clear();
    identifiers.clear();
    strPayload.clear();

    discoveryTopic = "homeassistant/sensor/esp32iotsensor/" + g_deviceName + "_waterec" + "/config";

    payload["name"] = g_deviceName + ".waterec";
    payload["uniq_id"] = g_UniqueId + "_waterec";
    payload["stat_t"] = g_mqttStatusTopic;
    payload["dev_cla"] = "ecsonsor";
    payload["val_tpl"] = "{{ value_json.waterec | is_defined }}";
    device = payload.createNestedObject("device");
    device["name"] = g_deviceName;
    device["model"] = g_deviceModel;
    device["sw_version"] = g_swVersion;
    device["manufacturer"] = g_manufacturer;
    identifiers = device.createNestedArray("identifiers");
    identifiers.add(g_UniqueId);

    serializeJsonPretty(payload, Serial);
    Serial.println(" ");
    serializeJson(payload, strPayload);

    client.publish(discoveryTopic.c_str(), strPayload.c_str());


  }
}

void getWaterTemperature() {

  sensors.requestTemperatures();
  float waterTempCelsius = sensors.getTempCByIndex(0);
  float waterTempFahrenheit = sensors.toFahrenheit(waterTempCelsius);

  if (waterTempCelsius == -127) {
    Serial.println("Error reading water temperature");
    sensorData.ds18b20Temperature = 0;
    sensorData.ds18b20Status = false;

  } else {
    Serial.print("Water temperature: ");
    Serial.print(waterTempCelsius);
    Serial.println(" C");
    sensorData.ds18b20Temperature = waterTempCelsius;
    sensorData.ds18b20Status = true;
  }
}

void getAirTempAndHumi() {

  float tempC = dht_sensor.readTemperature();
  float humi = dht_sensor.readHumidity();

  if (isnan(tempC) || isnan(humi)) {
    Serial.println("Error reading air temperature and humidity");
    sensorData.dht22Temperature = 0;
    sensorData.dht22Humidity = 0;
    sensorData.dht22Status = false;
  } else {
    Serial.print("Air temperature: ");
    Serial.print(tempC);
    Serial.println(" C");
    Serial.print("Air humidity: ");
    Serial.print(humi);
    Serial.println(" %");

    sensorData.dht22Temperature = tempC;
    sensorData.dht22Humidity = humi;
    sensorData.dht22Status = true;
  }
}

void getpH() {

  //phVoltage = rawPinValue / esp32ADC * esp32Vin
  phVoltage = ((analogRead(PH_PIN) / ESPADC * ESPVOLTAGE) / 2528.9795) * 1500; // read the voltage

  temperature = sensorData.ds18b20Temperature;  // read your temperature sensor to execute temperature compensation

  phValue = ph.readPH(phVoltage, temperature) + 2.95; // convert voltage to pH with temperature compensation

  if (phValue < 0 || phValue > 14) {
    Serial.println("Error reading pH value");
    sensorData.pHSensorValue = 0;
    sensorData.pHSensorStatus = false;
  } else {
    Serial.print("pH: ");
    Serial.print(phValue);
    Serial.println(" pH");

    sensorData.pHSensorValue = phValue;
    sensorData.pHSensorStatus = true;
  }
}

void getEC() {

  temperature = sensorData.ds18b20Temperature;  // read your temperature sensor to execute temperature compensation

  ecVoltage = ads.readADC_SingleEnded(0) / 10;
  ecValue = ec.readEC(ecVoltage, temperature); // convert voltage to EC with temperature compensation

  if (ecValue < 0 || ecValue > 14) {

    sensorData.ecSensorValue = 0;
    sensorData.ecSensorStatus = false;

    Serial.print("EC: ");
    Serial.print("0");
    Serial.println(" ms/cm");
  } else {
    sensorData.ecSensorValue = ecValue;
    sensorData.ecSensorStatus = true;

    Serial.print("EC: ");
    Serial.print(ecValue, 4);
    Serial.println(" ms/cm");
  }
}

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

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < sizeof(relayPins[i]) / sizeof(relayPins[i][0]); j++) {
      pinMode(relayPins[i][j], OUTPUT);
      digitalWrite(relayPins[i][j], HIGH);
    }
  }

  EEPROM.begin(32);//needed EEPROM.begin to store calibration k in eeprom
  ec.begin();
  ph.begin();

  ads.setGain(GAIN_ONE);
  ads.begin();

  sensors.begin();
  dht_sensor.begin();

  // Set the enable flags for the sensors based on your requirements
  sensorData.enableDHT22 = true;
  sensorData.enableDS18B20 = true;
  sensorData.enableECSensor = true;
  sensorData.enablePHSensor = true;

}

void loop() {
  // Check if the interval has passed
  static unsigned long timepoint = millis();
  static const unsigned long interval = 60000U; // 60 seconds

  if (!client.connected()) {
    connectToMQTTBroker();
  }
  client.loop();

  if (g_InitSystem) {
    delay(100);
    g_InitSystem = false;
    Serial.println("INIT SYSTEM...");
    MqttHomeAssistantDiscovery(); // Send Discovery Data
  }

  unsigned long currentTime = millis();

  if (currentTime - timepoint > interval) {
    if (sensorData.enableDHT22) {
      getAirTempAndHumi();
    }

    if (sensorData.enableDS18B20) {
      getWaterTemperature();
    }

    if (sensorData.enablePHSensor) {
      getpH();
    }

    if (sensorData.enableECSensor) {
      getEC();
    }

    // Perform any additional operations with the sensor data

    StaticJsonDocument<200> payload;
    payload["airTemp_status"] = sensorData.dht22Status;
    payload["airtemp"] = sensorData.dht22Temperature;
    payload["airHumi_status"] = sensorData.dht22Status;
    payload["airhum"] = sensorData.dht22Humidity;
    payload["waterTemp_status"] = sensorData.ds18b20Status;
    payload["watertemp"] = sensorData.ds18b20Temperature;
    payload["pHvalue_status"] = sensorData.pHSensorStatus;
    payload["waterph"] = sensorData.pHSensorValue;
    payload["ecvalue_status"] = sensorData.ecSensorStatus;
    payload["waterec"] = sensorData.ecSensorValue;

    String strPayload;
    serializeJson(payload, strPayload);

    if (client.connected()) {
      client.publish(g_mqttStatusTopic.c_str(), strPayload.c_str());
      Serial.println("MQTT: Send Data!!!");
      Serial.println();
      Serial.println();
      g_SendMqttData = false;
    }

    timepoint = currentTime; // Update the timepoint
  }
}
