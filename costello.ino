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
#include "Adafruit_SHT4x.h"
#include "Adafruit_VL53L0X.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>



Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

#define BMP1_PIN D6
#define BMP2_PIN D7


#define SEALEVELPRESSURE_HPA (1013.25)

#define ATMOS_PRESSURE_DIFFERENCE 0.41 //difference due to altitude between atmos pres sensor upstairs and dynamic pres sensor in furnace, INCREASE IF BASELINE TOO LOW



Adafruit_VL53L0X lox = Adafruit_VL53L0X();

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
  sensors_event_t humidity, temp;
  




 
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
float abshum, tempSHT, humSHT;
float presBMP1, presBMP2;
sensors_event_t humAHT1, tempAHT1;

sensors_event_t humAHT2, tempAHT2;
float tempprobe = 20;
unsigned long millisBlynk = 0;
unsigned long millisTFT = 0;
unsigned long millisAvg = 0;
int firstvalue = 1;



WidgetTerminal terminal(V10);

float bridgetemp, bridgepres, presdif;


BLYNK_WRITE(V73) {
  bridgetemp = param.asFloat();
}

BLYNK_WRITE(V94) {
  bridgepres = param.asFloat();
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
    switchwire1();
    abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT) / (tempSHT + 243.5))) * humSHT * 2.1674) / (273.15 + tempSHT);
    terminal.print("abshum: ");
    terminal.print(abshum);
    sht4.getEvent(&humidity, &temp);
    tempSHT = temp.temperature;
    humSHT = humidity.relative_humidity;
    presBMP1 = bmp.readPressure() / 100.0F;
    aht.getEvent(&humAHT1, &tempAHT1);
    //Wire.end();
    switchwire2();
    presBMP2 = bmp.readPressure() / 100.0F;
    aht.getEvent(&humAHT2, &tempAHT2);
    //Wire.end();
    terminal.print(", TempSHT: ");
    terminal.print(tempSHT);
    terminal.print(", HumSHT: ");
    terminal.println(humSHT);
    terminal.print("A, raw: ");
    terminal.println(analogRead(A0));
    terminal.print("Pres1: ");
    terminal.print(presBMP1);
    terminal.print(", Pres2: ");
    terminal.println(presBMP2);
    terminal.print("tempAHT1: ");
    terminal.print(tempAHT1.temperature);
    terminal.print(", tempAHT2: ");
    terminal.println(tempAHT2.temperature);
    terminal.print("humAHT1: ");
    terminal.print(humAHT1.relative_humidity);
    terminal.print(", humAHT2: ");
    terminal.println(humAHT2.relative_humidity);
    
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

void switchwire1() {
  Wire.begin(4,5);
  sht4.begin();
  sht4.setPrecision(SHT4X_HIGH_PRECISION);
  sht4.setHeater(SHT4X_NO_HEATER);
  lox.begin();
  bmp.begin(0x77);
  aht.begin();
  delay(200);
}

void switchwire2() {
  Wire.begin(BMP2_PIN, BMP1_PIN);
  bmp.begin(0x77);
  aht.begin();
  delay(200);
}


void setup() {
    pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  bridgetemp = -69;
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.setPhyMode(WIFI_PHY_MODE_11B);
  WiFi.begin("mikesnet", "springchicken");
  delay(10);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);

  }
  Wire.begin(4,5);
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
  //Wire.begin(4,5); 
  terminal.println("Adafruit SHT4x test");
  if (! sht4.begin()) {
    terminal.println("Couldn't find SHT4x");
  }
  terminal.println("Found SHT4x sensor");
  terminal.print("terminal number 0x");
  terminal.println(sht4.readSerial(), HEX);
  terminal.flush();
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
  terminal.flush();
  if (!bmp.begin(0x77)) { // Use the I2C address of the BMP280
    terminal.println("Could not find a valid BMP280 sensor on hardware I2C, check wiring!");
  }
  terminal.flush();
    if (! aht.begin()) {
    terminal.println("Could not find AHT? Check wiring");
  }
  //Wire.end();
  Wire.begin(BMP2_PIN, BMP1_PIN);
  if (!bmp.begin(0x77)) { // Use the I2C address of the BMP280
    terminal.println("Could not find a valid BMP280 sensor on D7/D6  I2C, check wiring!");
  }
  terminal.flush();
    if (! aht.begin()) {
    terminal.println("Could not find AHT? Check wiring");
  }
  terminal.flush();
  //Wire.end();
  Wire.begin(4,5);
  delay(200);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();



  if (millis() - millisBlynk >= 30000)  //if it's been 30 seconds
  {
 
    switchwire1();
    sht4.getEvent(&humidity, &temp);
    tempSHT = temp.temperature;
    humSHT = humidity.relative_humidity;
    abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT)/(tempSHT + 243.5))) * humSHT * 2.1674)/(273.15 + tempSHT);
    millisBlynk = millis();
    presBMP1 = bmp.readPressure() / 100.0F;
    aht.getEvent(&humAHT1, &tempAHT1);
    switchwire2();
    presBMP2 = bmp.readPressure() / 100.0F;
    aht.getEvent(&humAHT2, &tempAHT2);
    //Wire.end();

    Blynk.virtualWrite(V2, tempAHT1.temperature);
    Blynk.virtualWrite(V3, humAHT1.relative_humidity);

    Blynk.virtualWrite(V4, abshum);
    if (presBMP1 > 0) {Blynk.virtualWrite(V5, presBMP1);}
    if ((presBMP1 > 0) && (presBMP2 > 0)) {
      presdif = presBMP2 - presBMP1;
      Blynk.virtualWrite(V6, presdif);
    }
    Blynk.virtualWrite(V7, tempAHT2.temperature);
    Blynk.virtualWrite(V8, humAHT2.relative_humidity);

    Blynk.virtualWrite(V14, relaystate);
    Blynk.virtualWrite(V15, ledValue);
    if (bridgetemp != -69) {
      Blynk.virtualWrite(V16, sethum);
      Blynk.virtualWrite(V17, bridgetemp);
    }
    Blynk.virtualWrite(V18, adsmax);
    if (adsmax > 650) {
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
      int adsread = analogRead(A0);
      if (adsread > adsmax) {adsmax = adsread;}
    }
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    hours = timeinfo.tm_hour;
    mins = timeinfo.tm_min;
    secs = timeinfo.tm_sec;
    millisAvg = millis();
    
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

}
