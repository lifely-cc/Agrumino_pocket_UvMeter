/*Agrumino_pocket_UvMeter.ino - Sample project for Agrumino Lemon.
Created by gabriele.foddis@lifely.cc - Stay tuned on lifely.cc -
This sketch read and send all data with MQTT on your smartphone.

Have fun !!!*/

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Agrumino.h>
#include <Wire.h>
#include "Adafruit_VEML6075.h"
#define SERIAL_BAUD 115200
#define DELAY 1000 ///ms Time
#define WAITING_TIME 3000 ///ms Time to have valid data
#define SLEEP_TIME_SEC 1800 ///(s Time)for Agrumino Sleep (max 3600ms)
#define CLIENT ///your mqtt client
Adafruit_VEML6075 uv = Adafruit_VEML6075();
Agrumino agrumino;

////////MQTT SETUP///////
const char* ssid = ""; ///Your ssid
const char* password =  ""; ///Your WiFi Password
const char* mqttServer = "m24.cloudmqtt.com"; ///Your Mqtt Server
const int mqttPort = ; ///Mqtt Port
const char* mqttUser = ""; ///Mqtt User
const char* mqttPassword = ""; ///Mqtt password
#define TOPIC_INDEX "/Agrumino/UVMeter/UVindex" ///setup topic for mqtt message "Index UV"
#define TOPIC_INDEX_VALUE "/Agrumino/UVMeter/UVvalue" ///setup topic for mqtt message "Index UV"
#define TOPIC_WARNING "/Agrumino/UVMeter/Warning" ///setup topic for mqtt message "Warning message"
#define TOPIC_BATTERY_L "/Agrumino/UVMeter/Battery" ///setup topic for mqtt message "Battery Voltage"
WiFiClient CLIENT;
PubSubClient client(CLIENT);
/////////////////
void setup() {
  agrumino.setup();
  agrumino.turnBoardOn();
  Serial.begin(SERIAL_BAUD);
  if (! uv.begin()) {
    Serial.println("Check wiring on sensor UV");
  }
  Serial.println("VEML6075 sensor ok");
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);
  //client.setCallback(callback);

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP8266Client", mqttUser, mqttPassword )) {

      Serial.println("connected");
      blinkLedConectionOk();
      delay(500);

    } else {

      Serial.print("Failed with state ");
      Serial.print(client.state());
      delay(500);
    }
  }

  uv.setIntegrationTime(VEML6075_200MS); ////Integration time for Lifely Agrumino Lemon is 200ms

  Serial.print("Integration time set to ");
  switch (uv.getIntegrationTime()) {
    case VEML6075_50MS: Serial.print("50"); break;
    case VEML6075_100MS: Serial.print("100"); break;
    case VEML6075_200MS: Serial.print("200"); break;
    case VEML6075_400MS: Serial.print("400"); break;
    case VEML6075_800MS: Serial.print("800"); break;
  }
  Serial.println("ms");

  // Set the high dynamic mode
  uv.setHighDynamic(false);
  // Get the mode
  if (uv.getHighDynamic()) {
    Serial.println("High dynamic reading mode");
  } else {
    Serial.println("Normal dynamic reading mode");
  }

  uv.setForcedMode(false);
  if (uv.getForcedMode()) {
    Serial.println("Forced reading mode");
  } else {
    Serial.println("Continuous reading mode");
  }

  // Set the calibration coefficients
  uv.setCoefficients(2.22, 1.33,  // UVA_A and UVA_B coefficients
                     2.95, 1.74,  // UVB_C and UVB_D coefficients
                     0.001461, 0.002591); // UVA and UVB responses

}

void readDataFromAgrumino() {

  boolean isAttachedToUSB =   agrumino.isAttachedToUSB();
  boolean isBatteryCharging = agrumino.isBatteryCharging();
  boolean isButtonPressed =   agrumino.isButtonPressed();
  float temperature =         agrumino.readTempC();
  unsigned int soilMoisture = agrumino.readSoil();
  float illuminance =         agrumino.readLux();
  float batteryVoltage =      agrumino.readBatteryVoltage();
  unsigned int batteryLevel = agrumino.readBatteryLevel();
  Serial.println("Data from Agrumino Sensors");
  Serial.println("isAttachedToUSB:   " + String(isAttachedToUSB));
  Serial.println("isBatteryCharging: " + String(isBatteryCharging));
  Serial.println("isButtonPressed:   " + String(isButtonPressed));
  Serial.println("temperature:       " + String(temperature) + "°C");
  Serial.println("soilMoisture:      " + String(soilMoisture) + "%");
  Serial.println("illuminance :      " + String(illuminance) + " lux");
  Serial.println("batteryVoltage :   " + String(batteryVoltage) + " V");
  Serial.println("batteryLevel :     " + String(batteryLevel) + "%");
  Serial.println("End");
  Serial.println("#########################\n");

}

void printDataFromVeml6075() {
  Serial.println("Data from Uv Sensor");
  Serial.print("Raw UVA reading:  "); Serial.println(uv.readUVA());
  Serial.print("Raw UVB reading:  "); Serial.println(uv.readUVB());
  Serial.print("UV Index reading: "); Serial.println(uv.readUVI());
  Serial.println("end data");
  Serial.println("");
  int uvindex = uv.readUVI();
  client.publish(TOPIC_INDEX_VALUE, (String(uv.readUVI()) + String("")).c_str());
  client.publish(TOPIC_BATTERY_L, (String(agrumino.readBatteryLevel()) + String("")).c_str());
  Serial.println("Warning Message");

  if (uvindex < 1.99) {
    client.publish(TOPIC_INDEX, "Low Level");
    client.publish(TOPIC_WARNING, "Low level, wear sunglasses and hat");
    Serial.println("Low level, wear sunglasses and hat");
    blinkLedMessageOk();
    delay(DELAY);
  }
  else if (uvindex > 2.99 || uvindex < 4.99) {
    client.publish(TOPIC_INDEX, "Medium/Low level");
    client.publish(TOPIC_WARNING, "Medium/Low level, wear sunglasses, hat and SFP 15+");
    Serial.println("Medium/Low level, wear sunglasses, hat and SFP 15+");
    blinkLedMessageOk();
    delay(DELAY);
  }
  else if (uvindex > 5 || uvindex < 6.99) {
    client.publish(TOPIC_INDEX, "High level");
    client.publish(TOPIC_WARNING, "High level, wear sunglasses, hat and SFP 30+");
    Serial.println("High level, wear sunglasses, hat and SFP 30+");
    blinkLedMessageOk();
    delay(DELAY);
  }
  else if (uvindex > 7 || uvindex < 11) {
    client.publish(TOPIC_INDEX, "Very high level");
    client.publish(TOPIC_WARNING, "Very high level, wear sunglasses, hat, sunscreen SFP 30 +, possibly avoid the sun");
    Serial.println("Very high level, wear sunglasses, hat, sunscreen SFP 30 +, possibly avoid the sun");
    blinkLedMessageOk();
    delay(DELAY);
  }
  else if (uvindex > 11) {
    client.publish(TOPIC_INDEX, "Extreme level");
    client.publish(TOPIC_WARNING, "Extreme level,very dangerous, take all possible precautions, it would be better to stay inside");
    Serial.println("Extreme level,very dangerous, take all possible precautions, it would be better to stay inside");
    blinkLedMessageOk();
    delay(DELAY);
    Serial.println("");
  }
}

void loop() {
  delay(WAITING_TIME);
  client.loop();  
  // readDataFromAgrumino(); ///uncomment for print data into a serial monitor
  printDataFromVeml6075();
  // delaySec(SLEEP_TIME_SEC); // The ESP8266 stays powered, executes the loop repeatedly
  deepSleepSec(SLEEP_TIME_SEC); // ESP8266 enter in deepSleep and after the selected time starts back from setup() and then loop()
  Serial.print("Bye Bye.... I'm sleep");
  delay(DELAY);
  agrumino.turnBoardOff();
}

void delaySec(int sec) {
  delay (sec * 1000);
}
void deepSleepSec(int sec) {
  ESP.deepSleep(sec * 1000000); // microseconds
}

void blinkLedConectionOk() {
  agrumino.turnLedOn();
  delay(100);
  agrumino.turnLedOff();
  delay(100);
  agrumino.turnLedOn();
  delay(100);
  agrumino.turnLedOff();
  agrumino.turnLedOn();
  delay(100);
  agrumino.turnLedOff();
  delay(100);
  agrumino.turnLedOn();
  delay(100);
  agrumino.turnLedOff();

}

void blinkLedMessageOk() {
  agrumino.turnLedOn();
  delay(200);
  agrumino.turnLedOff();
  delay(200);
  agrumino.turnLedOn();
  delay(200);
  agrumino.turnLedOff();

}