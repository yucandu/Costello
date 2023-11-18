#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include "DHT.h"
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include "time.h"
#include <Average.h>
#include <Adafruit_ADS1X15.h>
#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SH1106Wire.h"   // legacy: #include "SH1106.h"
Adafruit_ADS1115 ads;

 SH1106Wire display(0x3c, SDA, SCL); 

#define RELAY_PIN 14 //D5
#define DHT_PIN 2 //D4
DHT dht (DHT_PIN, DHT22);
const float a = 0.148571;
const float c = 9.68571; //line of best fit for relationship between outside air temp and set abs humidity

float hysteresis = 0.7;  //g/m3
float sethum = 5.0;  //g/m3

#define VREF 4.096         // analog reference voltage(Volt) of the ADC



bool relaystate = false;


float correctedGas;
float aaH = 0.0000669005032586238;
float baH = -0.0159747107540879;
float abH = -0.00748672213648967;
float bbH = 1.78146466558454; //MQ5 gas sensor temp/hum compensation factors

float correctionFactor(float temp, float hum) { //MQ5 gas sensor temp/hum compensation function
  return (aaH * hum * temp) + (baH * temp) + (abH * hum) + bbH;
}

Average<float> gasAvg(6);
int gasRead, ledValue;

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;  //Replace with your GMT offset (seconds)
const int daylightOffset_sec = 0;   //Replace with your daylight offset (seconds)

float adc0, adc1, adc2, adc3;
float volts0, volts1, volts2, volts3;


const char* ssid = "mikesnet";
const char* password = "springchicken";



char auth[] = "pO--Yj8ksH2fjJLMW6yW9trkHBhd9-wc";  //BLYNK

AsyncWebServer server(80);
float abshum, tempDHT, humDHT;
float tempprobe = 20;
unsigned long millisBlynk = 0;
unsigned long millisTFT = 0;
unsigned long millisAvg = 0;
int firstvalue = 1;



WidgetTerminal terminal(V10);

float bridgetemp;


BLYNK_WRITE(V73) {
  bridgetemp = param.asFloat();
}

BLYNK_WRITE(V10) {
  if (String("help") == param.asStr()) {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("readgas");
    terminal.println("temps");
    terminal.println("==End of list.==");
  }
  if (String("wifi") == param.asStr()) {
    terminal.print("Connected to: ");
    terminal.println(ssid);
    terminal.print("IP address:");
    terminal.println(WiFi.localIP());
    terminal.print("Signal strength: ");
    printLocalTime();
    terminal.println(WiFi.RSSI());
  }

  if (String("readgas") == param.asStr()) {
    printtemp();
  }
  if (String("temps") == param.asStr()) {
    humDHT = dht.readHumidity();
    tempDHT = dht.readTemperature();
    abshum = (6.112 * pow(2.71828, ((17.67 * tempDHT) / (tempDHT + 243.5))) * humDHT * 2.1674) / (273.15 + tempDHT);
    terminal.print("> Temp: ");
    terminal.print(tempDHT);
    terminal.print("*C, Hum: ");
    terminal.print(humDHT);
    terminal.print("%, abshum: ");
    terminal.println(abshum);
  }

  terminal.flush();
}




void printLocalTime() {
  time_t rawtime;
  struct tm* timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  terminal.print("-");
  terminal.print(asctime(timeinfo));
  terminal.print(" - ");
  terminal.flush();
}


void printtemp() {
  adc3 = ads.readADC_SingleEnded(3);
  volts3 = ads.computeVolts(adc3);
  terminal.println("----------GAS---------");
  terminal.println("AIN3: ");
  terminal.print(adc3);
  terminal.print("  ");
  terminal.print(volts3);
  terminal.println("V");
  terminal.flush();
}



char time_value[20];
int hours, mins, secs;


void setup() {
  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(DHT_PIN, INPUT_PULLUP);
   dht.begin ();
  bridgetemp = -20;
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setPhyMode(WIFI_PHY_MODE_11B);
  WiFi.begin("mikesnet", "springchicken");
  delay(10);
  display.init();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.flipScreenVertically();
  display.drawString(0,0, "Connecting...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);

  }

  delay(500);
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  delay(500);
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
  Blynk.connect();
  terminal.println("***COSTELLO THE HUMIDISTAT v2.1***");
  terminal.print("Connected to ");
  terminal.println(ssid);
  terminal.print("IP address: ");
  terminal.println(WiFi.localIP());
  printLocalTime();
  terminal.flush();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(200, "text/plain", "I am Costello");
  });

  AsyncElegantOTA.begin(&server);  // Start ElegantOTA
  server.begin();
  terminal.println("HTTP server started");
  terminal.flush();
  if (!ads.begin()) {
    terminal.println("Failed to initialize ADS.");
    while (1)
      ;
  } else {
    terminal.println("ADS initialized");
  }
  terminal.println("Startup complete.");
  terminal.flush();
  ads.setGain(GAIN_SIXTEEN);
  gasRead = ads.readADC_SingleEnded(3);
  gasAvg.push(gasRead);
    humDHT = dht.readHumidity();
    tempDHT = dht.readTemperature();
    abshum = (6.112 * pow(2.71828, ((17.67 * tempDHT) / (tempDHT + 243.5))) * humDHT * 2.1674) / (273.15 + tempDHT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();
delay(100);






  if (millis() - millisBlynk >= 30000)  //if it's been 30 seconds
  {
    if (hours > 11) {display.invertDisplay();} else {display.normalDisplay();}
    humDHT = dht.readHumidity();
    tempDHT = dht.readTemperature();
    abshum = (6.112 * pow(2.71828, ((17.67 * tempDHT) / (tempDHT + 243.5))) * humDHT * 2.1674) / (273.15 + tempDHT);
    millisBlynk = millis();
      if (tempDHT > 0) {Blynk.virtualWrite(V2, tempDHT);}
    if (humDHT > 0) {Blynk.virtualWrite(V3, humDHT);}
    Blynk.virtualWrite(V2, tempDHT);
    Blynk.virtualWrite(V3, humDHT);
    Blynk.virtualWrite(V4, abshum);

    Blynk.virtualWrite(V9, gasAvg.mean());

    float Rs = (ads.computeVolts(gasAvg.mean()) * 47000) / (5.0 - ads.computeVolts(gasAvg.mean()));
    correctedGas = (Rs / correctionFactor(tempDHT, humDHT));
    Blynk.virtualWrite(V13, correctedGas);
    Blynk.virtualWrite(V14, relaystate);
    Blynk.virtualWrite(V15, ledValue);
    Blynk.virtualWrite(V16, sethum);
    Blynk.virtualWrite(V17, bridgetemp);
  }



  if (millis() - millisAvg >= 5000)  //if it's been 5 second
  {
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    hours = timeinfo.tm_hour;
    mins = timeinfo.tm_min;
    secs = timeinfo.tm_sec;
    millisAvg = millis();
    ads.setGain(GAIN_SIXTEEN);
    gasRead = ads.readADC_SingleEnded(3);
    gasAvg.push(gasRead);
    String tempstring = "OUT TEMP: " + String(bridgetemp) + "°C";
    String humstring = "IN HUM: " + String(abshum) + "g";
    String sethumstring = "SET HUM: " + String(sethum) + "g";
    String gasstring = "GAS: " + String(correctedGas);
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0,0, tempstring);
    display.drawString(0,12, humstring);
    display.drawString(0,24, sethumstring);
    if (relaystate) {String relaystring = "RELAY: [ON]";
      display.drawString(0,36, relaystring);} else {String relaystring = "RELAY: [off]";
      display.drawString(0,36, relaystring);}
    display.display(0, 48, gasstring);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
    String tempstring = String(bridgetemp) + "°C";
    String humstring = String(abshum) + "g";
    String sethumstring = String(sethum) + "g";
    String gasstring = String(correctedGas);

    sethum = (a * bridgetemp) + c;
    if ((abshum < sethum) && (abshum > 0)) {
      if (!relaystate) {digitalWrite(RELAY_PIN, HIGH);}
      relaystate = true;
      ledValue = 255;
    }
    if (abshum > (sethum + hysteresis)) {
      if (relaystate) {digitalWrite(RELAY_PIN, LOW);}
      relaystate = false;
      ledValue = 0;
    }

  }
}
