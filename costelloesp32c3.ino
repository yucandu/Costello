#include <Arduino.h>
#include <BlynkSimpleEsp32.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "time.h"
#include "Adafruit_SHT4x.h"
#include "Adafruit_VL53L0X.h"
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
sensors_event_t humidity, temp;

#define LED_PIN 10

float abshum, tempSHT, humSHT;
long distance;
bool buttonstart = false;
const char* ssid = "mikesnet";
const char* password = "springchicken";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;  //Replace with your GMT offset (secs)
const int daylightOffset_sec = 3600;   //Replace with your daylight offset (secs)
int hours, mins, secs;
bool LEDon = false;

char auth[] = "pO--Yj8ksH2fjJLMW6yW9trkHBhd9-wc";

AsyncWebServer server(80);

WidgetTerminal terminal(V10);

#define every(interval) \
    static uint32_t __every__##interval = millis(); \
    if (millis() - __every__##interval >= interval && (__every__##interval = millis()))

void goToSleep(int sleeptimeSecs) {
      Wire.end();
      pinMode(SS, INPUT_PULLUP );
      pinMode(6, INPUT_PULLUP );
      pinMode(4, INPUT_PULLUP );
      pinMode(8, INPUT_PULLUP );
      pinMode(9, INPUT_PULLUP );
      pinMode(1, INPUT_PULLUP );
      pinMode(2, INPUT_PULLUP );
      pinMode(3, INPUT_PULLUP );
      pinMode(0, INPUT_PULLUP );
      pinMode(5, INPUT_PULLUP );
      esp_sleep_enable_timer_wakeup(sleeptimeSecs * 1000000ULL);
      delay(1);
      esp_deep_sleep_start();
      delay(1000);
}

BLYNK_WRITE(V11)
{
  if (param.asInt() == 1) {buttonstart = true;}
  if (param.asInt() == 0) {buttonstart = false;}
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V11);
}


BLYNK_WRITE(V10) {
  if (String("help") == param.asStr()) {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("temps");
    terminal.println("range");
    terminal.println("blink");
    terminal.println("reset");
    terminal.println("==End of list.==");
  }
  if (String("wifi") == param.asStr()) {
    terminal.print("Connected to: ");
    terminal.println(ssid);
    terminal.print("IP address:");
    terminal.println(WiFi.localIP());
    terminal.print("Signal strength: ");
    terminal.println(WiFi.RSSI());
    printLocalTime();
  }
  if (String("temps") == param.asStr()) {

    sht4.getEvent(&humidity, &temp);
    tempSHT = temp.temperature;
    humSHT = humidity.relative_humidity;
    abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT) / (tempSHT + 243.5))) * humSHT * 2.1674) / (273.15 + tempSHT);
    terminal.print("abshum: ");
    terminal.print(abshum);
    terminal.print(", TempSHT: ");
    terminal.print(tempSHT);
    terminal.print(", HumSHT: ");
    terminal.println(humSHT);
  }
  if (String("range") == param.asStr()) {
        VL53L0X_RangingMeasurementData_t measure;
    
      terminal.print("Reading a measurement... ");
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

      if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      terminal.print("Distance (mm): "); terminal.println(measure.RangeMilliMeter);
      } else {
      terminal.println(" out of range ");
      }
   }
  if (String("blink") == param.asStr()) {
    blinkLED();
  }
    if (String("reset") == param.asStr()) {
    terminal.println("Restarting...");
    terminal.flush();
    Blynk.run();
    ESP.restart();
  }
    terminal.flush();
}

void printLocalTime() {
  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  terminal.print(asctime(timeinfo));
}

void blinkLED(){
  LEDon = !LEDon;
  if (LEDon) {digitalWrite(LED_PIN, HIGH);} 
  else {digitalWrite(LED_PIN, LOW);}
}

void setup(void) {
  
  sht4.begin();
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
  delay(100);
  sht4.getEvent(&humidity, &temp);
      tempSHT = temp.temperature;
      humSHT = humidity.relative_humidity;
      abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT)/(tempSHT + 243.5))) * humSHT * 2.1674)/(273.15 + tempSHT);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {    
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
  digitalWrite(LED_PIN, LOW);
  Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
  Blynk.connect();
    while ((!Blynk.connected()) && (millis() < 15000)){delay(250);}
  if (WiFi.status() == WL_CONNECTED) {Blynk.run();}


    if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
    Blynk.virtualWrite(V21, tempSHT);
    if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
    Blynk.virtualWrite(V22, humSHT);
    if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
    VL53L0X_RangingMeasurementData_t measure;
    lox.begin();
    lox.rangingTest(&measure, false);
    
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      distance = measure.RangeMilliMeter;
      Blynk.virtualWrite(V24, distance);  
      if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
      Blynk.virtualWrite(V24, distance);  
      if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
    } else {
      terminal.println("Salt sensor out of range ");
      terminal.flush();
    }

if (buttonstart) {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "text/plain", "Hi! I am ESP32.");
    });

    AsyncElegantOTA.begin(&server);    // Start ElegantOTA
    server.begin();
    Serial.println("HTTP server started");
    delay(250);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    delay(250);
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    hours = timeinfo.tm_hour;
    mins = timeinfo.tm_min;
    secs = timeinfo.tm_sec;
    terminal.println("***Costello 2.2 STARTED***");
    terminal.print("Connected to ");
    terminal.println(ssid);
    terminal.print("IP address: ");
    terminal.println(WiFi.localIP());
    printLocalTime();
    terminal.flush();
    if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
  }
  else   {
    pinMode(LED_PIN, INPUT);
    goToSleep(300);

  }
}

void loop() {
      if (WiFi.status() == WL_CONNECTED) {Blynk.run();}
      every(1000){
        if (distance > 550){blinkLED();}
        else {digitalWrite(LED_PIN, LOW);}
      }
      every(30000){
      VL53L0X_RangingMeasurementData_t measure;
      lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
      sht4.getEvent(&humidity, &temp);
      tempSHT = temp.temperature;
      humSHT = humidity.relative_humidity;
      abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT)/(tempSHT + 243.5))) * humSHT * 2.1674)/(273.15 + tempSHT);
    Blynk.virtualWrite(V4, abshum);
    Blynk.virtualWrite(V21, tempSHT);
    Blynk.virtualWrite(V22, humSHT);

    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      distance = measure.RangeMilliMeter;
      Blynk.virtualWrite(V24, distance);  
    } else {
      terminal.println("Salt sensor out of range ");
      terminal.flush();
    }
      }

}
