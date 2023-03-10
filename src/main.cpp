#include <Arduino.h>
#include "config.h"

#include "WiFi.h"
#include <esp_wifi.h>
uint8_t newMACAddress[] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6};

#include "DHT.h"
#define DHTPIN 18
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);
float tempSensor; 
float humiditySensor;

#include <OneWire.h>
#include <DallasTemperature.h>
const int oneWireBus = 4;     
OneWire oneWire(oneWireBus);
DallasTemperature ds18b20(&oneWire);
float tempSensor_DS;

#include "AdafruitIO_WiFi.h"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, ssid, password);
AdafruitIO_Feed *tempFeed = io.feed("roomTemp");
AdafruitIO_Feed *tempFeed1 = io.feed("roomTemp1");
AdafruitIO_Feed *humFeed = io.feed("roomHumidity");

unsigned long previousReconnect = 0;
unsigned long previousRead = 0;
unsigned long intervalReconnect = 30000;
unsigned long intervalRead = 10 * 60000;

#include <SimpleKalmanFilter.h>
SimpleKalmanFilter tempKalmanFilter(2, 2, 1);
SimpleKalmanFilter humKalmanFilter(2, 2, 1);
SimpleKalmanFilter tempKalmanFilter1(2, 2, 1);
SimpleKalmanFilter humKalmanFilter1(2, 2, 1);

void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid);
  // WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}

void initAIO() {
  Serial.print("Connecting to AIO");
  io.connect();
  while (io.status() < AIO_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(WiFi.localIP());
}

float read_DHT_Temp() {
  float bufRead;
  for (int i=0; i<10; i++) {
    digitalWrite(13, HIGH);
    bufRead = tempKalmanFilter.updateEstimate(dht.readTemperature());
    delay(1000);
    digitalWrite(13, LOW);
  }
  digitalWrite(13, HIGH);
  return bufRead;
}

float read_DHT_Hum() {
  float bufRead;
  for (int i=0; i<10; i++) {
    digitalWrite(13, HIGH);
    bufRead = humKalmanFilter.updateEstimate(dht.readHumidity());
    delay(1000);
    digitalWrite(13, LOW);
  }
  digitalWrite(13, HIGH);
  return bufRead;
}

float read_DS18_Temp() {
  float bufRead;
  for (int i=0; i<10; i++) {
    digitalWrite(13, HIGH);
    ds18b20.requestTemperatures();
    bufRead = tempKalmanFilter1.updateEstimate(ds18b20.getTempCByIndex(0));
    delay(1000);
    digitalWrite(13, LOW);
  }
  digitalWrite(13, HIGH);
  return bufRead;
}

void send_log_data() {
  tempSensor = read_DHT_Temp();
  Serial.print("Temp = ");
  Serial.println(tempSensor);
  tempFeed->save(tempSensor);

  humiditySensor = read_DHT_Hum();
  Serial.print("Humidity = ");
  Serial.println(humiditySensor);
  humFeed->save(humiditySensor);

  tempSensor_DS = read_DS18_Temp();
  Serial.print("Temp1 = ");
  Serial.println(tempSensor_DS);
  tempFeed1->save(tempSensor_DS);
}

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
  // initWiFi();
  initAIO();
  dht.begin();
  ds18b20.begin();
  digitalWrite(13, HIGH);
  send_log_data();
}

void loop() {

  if ((WiFi.status() != WL_CONNECTED) && (millis() - previousReconnect >= intervalReconnect)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    previousReconnect = millis();
  }

  if (millis() - previousRead >= intervalRead) {    
    send_log_data();
    previousRead = millis();
  }

  io.run();

}