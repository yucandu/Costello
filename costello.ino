#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include "time.h"
#include <Average.h>
#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SH1106Wire.h"   // legacy: #include "SH1106.h"
#include <Adafruit_ADS1X15.h>
#include "Adafruit_SHT4x.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
  sensors_event_t humidity, temp;

Adafruit_ADS1015 ads; 


 SH1106Wire display(0x3c, SDA, SCL); 

 
int adsmax;

#define RELAY_PIN 14 //D5

//const float a = 0.143024; //0.148571;
//const float c = 10.4; //9.68571;
const float a = -0.00361127; 
const float b = 0.156529;
const float c = 11; //line of best fit for relationship between outside air temp and set abs humidity


float hysteresis = 0.6;  //g/m3
float sethum = 5.0;  //g/m3

#define VREF 4.096         // analog reference voltage(Volt) of the ADC



bool relaystate = false;





int ledValue, ledValue2;

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;  //Replace with your GMT offset (seconds)
const int daylightOffset_sec = 3600;   //Replace with your daylight offset (seconds)


const char* ssid = "mikesnet";
const char* password = "springchicken";



char auth[] = "pO--Yj8ksH2fjJLMW6yW9trkHBhd9-wc";  //BLYNK

AsyncWebServer server(80);
float abshum, tempBME, humBME, tempSHT, humSHT, presBME;
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
    terminal.println("temps");
    terminal.println("range");
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


  if (String("temps") == param.asStr()) {
    abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT) / (tempSHT + 243.5))) * humSHT * 2.1674) / (273.15 + tempSHT);
    terminal.print("%, abshum: ");
    terminal.print(abshum);
          sht4.getEvent(&humidity, &temp);
          tempSHT = temp.temperature;
          humSHT = humidity.relative_humidity;
    tempBME = bme.readTemperature();
    presBME = bme.readPressure() / 100.0F;
    humBME = bme.readHumidity();
          terminal.print(", TempSHT: ");
          terminal.print(tempSHT);
          terminal.print(", HumSHT: ");
          terminal.println(humSHT);
    terminal.print("A, raw: ");
    terminal.println(ads.readADC_SingleEnded(0));
    terminal.print("TempBME: ");
    terminal.print(tempBME);
    terminal.print("HumBME: ");
    terminal.print(humBME);
    terminal.print("PresBME: ");
    terminal.println(presBME);
    
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





char time_value[20];
int hours, mins, secs;


void setup() {
  digitalWrite(RELAY_PIN, LOW);
  pinMode(RELAY_PIN, OUTPUT);
   ads.setGain(GAIN_ONE);
   ads.begin();
   bme.begin(0x76);
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
  display.clear();
  display.setBrightness(100);
  display.drawString(0,0, "Connecting...");
  display.display();
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

  terminal.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    terminal.println("Couldn't find SHT4x");
    while (1) delay(1);
  }
  terminal.println("Found SHT4x sensor");
  terminal.print("terminal number 0x");
  terminal.println(sht4.readSerial(), HEX);
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);

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
 // terminal.flush();

    terminal.println("Adafruit VL53L0X test");
  if (!lox.begin()) {
    terminal.println("Failed to boot VL53L0X");
  }
  else {
    terminal.println("Succeeded to boot VL53L0X");
  }
  terminal.println("Startup complete.");
  terminal.flush();

}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();



  if (millis() - millisBlynk >= 30000)  //if it's been 30 seconds
  {
    if (hours > 11) {display.invertDisplay();} else {display.normalDisplay();}

          sht4.getEvent(&humidity, &temp);
          tempSHT = temp.temperature;
          humSHT = humidity.relative_humidity;
        abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT)/(tempSHT + 243.5))) * humSHT * 2.1674)/(273.15 + tempSHT);
    millisBlynk = millis();
    tempBME = bme.readTemperature();
    presBME = bme.readPressure() / 100.0F;
    humBME = bme.readHumidity();
      if (tempBME > 0) {Blynk.virtualWrite(V2, tempBME);}
    if (humBME > 0) {Blynk.virtualWrite(V3, humBME);}

    Blynk.virtualWrite(V4, abshum);
    if (presBME > 0) {Blynk.virtualWrite(V5, presBME);}
    

    Blynk.virtualWrite(V14, relaystate);
    Blynk.virtualWrite(V15, ledValue);
    Blynk.virtualWrite(V16, sethum);
    Blynk.virtualWrite(V17, bridgetemp);
    Blynk.virtualWrite(V18, adsmax);
    if (adsmax > 900) {
      ledValue2 = 255;
      Blynk.virtualWrite(V19, true);
    } else {
      ledValue2 = 0;
      Blynk.virtualWrite(V19, false);
      }
      Blynk.virtualWrite(V23, ledValue2);
    Blynk.virtualWrite(V21, tempSHT);
    Blynk.virtualWrite(V22, humSHT);

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!
    if (measure.RangeStatus != 4) {  // phase failures have incorrect data
      Blynk.virtualWrite(V24, measure.RangeMilliMeter);  
    } else {
      terminal.println("Salt sensor out of range ");
      terminal.flush();
    }
    
  }



  if (millis() - millisAvg >= 5000)  //if it's been 5 second
  {
    adsmax = 0;
    for (int i = 0; i < 100; i++) {
      int adsread = ads.readADC_SingleEnded(0);
      if (adsread > adsmax) {adsmax = adsread;}
    }
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    hours = timeinfo.tm_hour;
    mins = timeinfo.tm_min;
    secs = timeinfo.tm_sec;
    millisAvg = millis();
    String tempstring = "OUT TEMP: ";
    String humstring = "IN HUM: "; 
    String sethumstring = "SET HUM: ";
    String relaystring = "RELAY:";
    String intempstring = "IN TEMP: "; 
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0,0, tempstring);
    display.drawString(0,12, humstring);
    display.drawString(0,24, sethumstring);
    display.drawString(0,36, relaystring);
    display.drawString(0,48, intempstring);
    display.setTextAlignment(TEXT_ALIGN_RIGHT);
     tempstring = String(bridgetemp) + "°C";
     humstring = String(abshum) + "g/m³";
     sethumstring = String(sethum) + "g/m³";
     intempstring = String(tempSHT) + "°C";
    display.drawString(128,0, tempstring);
    display.drawString(128,12, humstring);
    display.drawString(128,24, sethumstring);
    if (relaystate) {String relaystring = "[ON]"; display.drawString(128,36, relaystring);} else {String relaystring = "[off]"; display.drawString(128,36, relaystring);}
    display.drawString(128,48, intempstring);
    sethum = ((a*bridgetemp)*(a*bridgetemp)) + (b * bridgetemp) + c;
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
display.display();
}
