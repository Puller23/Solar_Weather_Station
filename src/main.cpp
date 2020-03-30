#include <Arduino.h>
#include <SPI.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "Settings.h"
#define VERSION "0.1"


// IP Settings
IPAddress ip(192, 168, 0, 146);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

//OTA
#define SENSORNAME "WeatherStation"
#define OTApassword "test123!" // change this to whatever password you want to use when you upload OTA
int OTAport = 8266;

float measured_temp;
float measured_humi;
float measured_pres;

float volt;

Adafruit_BME280 bme; 

WiFiClient net;
PubSubClient client(net);


void connect_wifi()
{
  Serial.begin(9600);
  WiFi.hostname(SENSORNAME);
  WiFi.config(ip, gateway, subnet);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {  //Wait for connection
    delay(500);
    Serial.println("Waiting to connect…");
  }

  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  //Print the local IP
}

void reconnect() {
  while (!client.connected()) {
    String clientId = "bu_Nextion-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      client.subscribe("FHEM");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      delay(5000);
    }
  }
}
void setup_OTA()
{
  ArduinoOTA.setPort(OTAport);
  ArduinoOTA.setHostname(SENSORNAME);
  ArduinoOTA.setPassword((const char *)OTApassword);

  ArduinoOTA.onStart([]() {
    Serial.println("Starting");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
}
void connect_MQTT() {
  Serial.print("---> Connecting to MQTT, ");
  client.setServer(mqtt_server, mqtt_Port);
  
  while (!client.connected()) {
    Serial.println("reconnecting MQTT...");
    reconnect(); 
  }
  Serial.println("MQTT connected ok.");
}

void send_bme()
{
  bool bme_status;
  bme_status = bme.begin(0x76);  //address either 0x76 or 0x77
  if (!bme_status) {
      Serial.println("Could not find a valid BME280 sensor, check wiring!");
  }

  bme.takeForcedMeasurement();

  // Get temperature
  measured_temp = bme.readTemperature();
  measured_temp = measured_temp;
  // print on serial monitor
  Serial.print("Temp: ");
  Serial.print(measured_temp);
  Serial.print("°C; ");

  char _measured_temp[10];
  String measured_temp_str = String(measured_temp);
  measured_temp_str.toCharArray(_measured_temp, measured_temp_str.length() +1);
  String mqttMSG_temp = mqtt_DiscoveryPrefix + mqtt_Node + "/temp";
  client.publish(mqttMSG_temp.c_str(), _measured_temp);
  Serial.print("mqttMSG_temp:");
  Serial.println(mqttMSG_temp);
  delay(50);
 
  // Get humidity
  measured_humi = bme.readHumidity();
  // print on serial monitor
  Serial.print("Humidity: ");
  Serial.print(measured_humi);
  Serial.print("%; ");

  char _measured_humi[10];
  String measured_humi_str = String(measured_humi);
  measured_humi_str.toCharArray(_measured_humi, measured_humi_str.length() +1);
  String mqttMSG_humi = mqtt_DiscoveryPrefix + mqtt_Node + "/humidity";
  client.publish(mqttMSG_humi.c_str(), _measured_humi);
  delay(50);

  // Get pressure
  measured_pres = bme.readPressure() / 100.0F;
  // print on serial monitor
  Serial.print("Pressure: ");
  Serial.print(measured_pres);
  Serial.print("hPa; ");

  char _measured_pres[10];
  String measured_pres_str = String(measured_pres);
  measured_pres_str.toCharArray(_measured_pres, measured_pres_str.length() +1);
  String mqttMSG_pres = mqtt_DiscoveryPrefix + mqtt_Node + "/pressure";
  client.publish(mqttMSG_pres.c_str(), _measured_pres);
  delay(50);
  
}
void sendBattery()
{
//******Battery Voltage Monitoring*********************************************
  
  // Voltage divider R1 = 220k+100k+220k =540k and R2=100k
  float calib_factor = 5.28; // change this value to calibrate the battery voltage
  unsigned long raw = analogRead(A0);
  volt = raw * calib_factor/1024; 
  
  Serial.print( "Voltage = ");
  Serial.print(volt, 2); // print with 2 decimal places
  Serial.println (" V");

  char _volt_pres[10];
  String volt_pres_str = String(volt);
  volt_pres_str.toCharArray(_volt_pres, volt_pres_str.length() +1);
  String mqttMSG_volt = mqtt_DiscoveryPrefix + mqtt_Node + "/volt";
  client.publish(mqttMSG_volt.c_str(), _volt_pres);
}
void goToSleep()
{
  Serial.println("INFO: Closing the MQTT connection");
  client.disconnect();
  
  Serial.println("INFO: Closing the Wifi connection");
  WiFi.disconnect();

  while (client.connected() || (WiFi.status() == WL_CONNECTED)) {
    Serial.println("Waiting for shutdown before sleeping");
    delay(10);
  }
  delay(50);
  
  Serial.print ("Going to sleep now for ");
  Serial.print (sleepTimeMin);
  Serial.print (" Minute(s).");
  ESP.deepSleep(sleepTimeMin * 60 * 1000000);
}
void setup() {
  Serial.begin(9600);
  Serial.println();
  Serial.println("Weather Station");
  connect_wifi();
  setup_OTA();
  connect_MQTT();
  send_bme();
  sendBattery();
  goToSleep();
}

void loop() {

}