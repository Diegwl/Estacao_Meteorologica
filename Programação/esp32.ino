/****************************************
 * Include Libraries
 ****************************************/
#include "UbidotsEsp32Mqtt.h"

#include <ETH.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WiFiClient.h>
#include <WiFiGeneric.h>
#include <WiFiMulti.h>
#include <WiFiScan.h>
#include <WiFiServer.h>
#include <WiFiSTA.h>
#include <WiFiType.h>
#include <WiFiUdp.h>

#include <PubSubClient.h>

#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

/****************************************
 * Define Constants
 ****************************************/
const char *UBIDOTS_TOKEN = "BBFF-u2gVPyr1HZed0qEc23C26eHroookjC";  // Put here your Ubidots TOKEN
const char *WIFI_SSID = "xxxx";      // Put here your Wi-Fi SSID
const char *WIFI_PASS = "xxxxxxxx";      // Put here your Wi-Fi password
const char *DEVICE_LABEL = "esp32";   // Put here your Device label to which data  will be published
const char *VARIABLE_LABEL = "luzuv"; // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL_2 = "pressao"; // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL_3 = "temperatura"; // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL_4 = "chuva"; // Put here your Variable label to which data  will be published
const char *VARIABLE_LABEL_5 = "umidadesolo"; // Put here your Variable label to which data  will be published

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

unsigned long timer;

int UVOUT = 36;   /* Pino D36 do ESP32 conetado ao Out do sensor UV */
int REF_3V3   = 34;   /* Pino D34 do ESP32 conectado ao EN do sensor UV */

int pino_d = 5; //Pino ligado ao D0 do sensor
int pino_a = 25; //Pino ligado ao A0 do sensor
int val_d = 0; //Armazena o valor lido do pino digital
int val_a = 0; //Armazena o valor lido do pino analogico

#define pino_sinal_analogico 35

int valor_analogico;

Ubidots ubidots(UBIDOTS_TOKEN);

/****************************************
 * Auxiliar Functions
 ****************************************/

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

/****************************************
 * Main Functions
 ****************************************/

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(pino_d, INPUT);
  pinMode(pino_a, INPUT);
  pinMode(pino_sinal_analogico, INPUT);
  // ubidots.setDebug(true);  // uncomment this to make debug messages available
  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
 if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
  while (1) {}
  }
  timer = millis();
}

void loop()
{
  // put your main code here, to run repeatedly:
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  
  // Use o pino de alimentação de 3,3 V como referência para obter um valor de saída muito preciso do sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.8, 0.0, 15.0); // Converta a voltagem para um nível de intensidade de UV

  //Le e arnazena o valor do pino digital
  val_d = digitalRead(pino_d);
  //Le e armazena o valor do pino analogico
  val_a = analogRead(pino_a);

  valor_analogico = analogRead(pino_sinal_analogico);

  if (!ubidots.connected())
  {
    ubidots.reconnect();
  }
  if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
  {
    ubidots.add(VARIABLE_LABEL, uvIntensity); // Insert your variable Labels and the value to be sent
    ubidots.add(VARIABLE_LABEL_2, bmp.readPressure()); // Insert your variable Labels and the value to be sent
    ubidots.add(VARIABLE_LABEL_3, bmp.readTemperature()); // Insert your variable Labels and the value to be sent
    ubidots.add(VARIABLE_LABEL_4, val_d); // Insert your variable Labels and the value to be sent
    ubidots.add(VARIABLE_LABEL_5, valor_analogico); // Insert your variable Labels and the value to be sent
    ubidots.publish(DEVICE_LABEL);
    timer = millis();
    
  }
  ubidots.loop();
}

int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 
 
  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;
 
  return(runningValue);
}
 
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
