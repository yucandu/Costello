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
#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
#include "SH1106Wire.h"   // legacy: #include "SH1106.h"
#include <Adafruit_ADS1X15.h>
#include "Adafruit_SHT4x.h"

Adafruit_SHT4x sht4 = Adafruit_SHT4x();
  sensors_event_t humidity, temp;

Adafruit_ADS1015 ads; 


 SH1106Wire display(0x3c, SDA, SCL); 

 
        /* 2- AC Current Measurement */
        float mVperAmpValue = 31.25;                      // If using ACS712 current module : for 5A module key in 185, for 20A module key in 100, for 30A module key in 66
                                                          // If using "Hall-Effect" Current Transformer, key in value using this formula: mVperAmp = maximum voltage range (in milli volt) / current rating of CT
                                                          /* For example, a 20A Hall-Effect Current Transformer rated at 20A, 2.5V +/- 0.625V, mVperAmp will be 625 mV / 20A = 31.25mV/A */
        float currentSampleRead_1  = 0;                     /* to read the value of a sample*/
        unsigned long currentLastSample_1  = 0;                     /* to count time for each sample. Technically 1 milli second 1 sample is taken */
        float currentSampleSum_1   = 0;                     /* accumulation of sample readings */
        float currentSampleCount_1 = 0;                     /* to count number of sample. */
        float currentMean_1 ;                               /* to calculate the average value from all samples*/ 
        float RMSCurrentMean_1 =0 ;                         /* square roof of currentMean*/
        float FinalRMSCurrent_1 ;                           /* the final RMS current reading*/
        /*2.1 Offset AC Current */    
        float currentOffset1_1 = 0;                   // to Offset deviation and accuracy. Offset any fake current when no current operates. 
                                                          // Offset will automatically callibrate when SELECT Button on the LCD Display Shield is pressed.
                                                          // If you do not have LCD Display Shield, look into serial monitor to add or minus the value manually and key in here.
                                                          // 26 means add 26 to all analog value measured
        float currentOffset2_1 = 0;                   // to offset value due to calculation error from squared and square root.

        float currentSampleRead_2  = 0;                     /* to read the value of a sample*/
        float currentLastSample_2  = 0;                     /* to count time for each sample. Technically 1 milli second 1 sample is taken */
        float currentSampleSum_2   = 0;                     /* accumulation of sample readings */
        float currentSampleCount_2 = 0;                     /* to count number of sample. */
        float currentMean_2 ;                               /* to calculate the average value from all samples*/ 
        float RMSCurrentMean_2 =0 ;                         /* square roof of currentMean*/
        float FinalRMSCurrent_2 ;                           /* the final RMS current reading*/
        /*2.1 Offset AC Current */    
        float currentOffset1_2 = 0;                   // to Offset deviation and accuracy. Offset any fake current when no current operates. 
                                                          // Offset will automatically callibrate when SELECT Button on the LCD Display Shield is pressed.
                                                          // If you do not have LCD Display Shield, look into serial monitor to add or minus the value manually and key in here.
                                                          // 26 means add 26 to all analog value measured
        float currentOffset2_2 = 0;                   // to offset value due to calculation error from squared and square root.

#define RELAY_PIN 14 //D5
#define DHT_PIN 2 //D4
DHT dht (DHT_PIN, DHT22);
//const float a = 0.143024; //0.148571;
//const float c = 10.4; //9.68571;
const float a = -0.00361127; 
const float b = 0.156529;
const float c = 10.415; //line of best fit for relationship between outside air temp and set abs humidity


float hysteresis = 0.7;  //g/m3
float sethum = 5.0;  //g/m3

#define VREF 4.096         // analog reference voltage(Volt) of the ADC



bool relaystate = false;





int ledValue;

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -18000;  //Replace with your GMT offset (seconds)
const int daylightOffset_sec = 0;   //Replace with your daylight offset (seconds)


const char* ssid = "mikesnet";
const char* password = "springchicken";



char auth[] = "pO--Yj8ksH2fjJLMW6yW9trkHBhd9-wc";  //BLYNK

AsyncWebServer server(80);
float abshum, tempDHT, humDHT, tempSHT, humSHT;
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
    humDHT = dht.readHumidity();
    tempDHT = dht.readTemperature();
    abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT) / (tempSHT + 243.5))) * humSHT * 2.1674) / (273.15 + tempSHT);
    terminal.print("> Temp: ");
    terminal.print(tempDHT);
    terminal.print("*C, Hum: ");
    terminal.print(humDHT);
    terminal.print("%, abshum: ");
    terminal.print(abshum);
          sht4.getEvent(&humidity, &temp);
          tempSHT = temp.temperature;
          humSHT = humidity.relative_humidity;
          terminal.print(", TempSHT: ");
          terminal.print(tempSHT);
          terminal.print(", HumSHT: ");
          terminal.println(humSHT);
    terminal.print("A, raw: ");
    terminal.println(ads.readADC_SingleEnded(0));
    
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
  pinMode(DHT_PIN, INPUT_PULLUP);
   dht.begin ();
   ads.setGain(GAIN_ONE);
   ads.begin();
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
  terminal.flush();
  terminal.println("Startup complete.");
  terminal.flush();
    humDHT = dht.readHumidity();
    tempDHT = dht.readTemperature();
    abshum = (6.112 * pow(2.71828, ((17.67 * tempDHT) / (tempDHT + 243.5))) * humDHT * 2.1674) / (273.15 + tempDHT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blynk.run();



  if (millis() - millisBlynk >= 30000)  //if it's been 30 seconds
  {
    if (hours > 11) {display.invertDisplay();} else {display.normalDisplay();}
    humDHT = dht.readHumidity();
    tempDHT = dht.readTemperature();
          sht4.getEvent(&humidity, &temp);
          tempSHT = temp.temperature;
          humSHT = humidity.relative_humidity;
        abshum = (6.112 * pow(2.71828, ((17.67 * tempSHT)/(tempSHT + 243.5))) * humSHT * 2.1674)/(273.15 + tempSHT);
    millisBlynk = millis();
      if (tempDHT > 0) {Blynk.virtualWrite(V2, tempDHT);}
    if (humDHT > 0) {Blynk.virtualWrite(V3, humDHT);}
    Blynk.virtualWrite(V2, tempDHT);
    Blynk.virtualWrite(V3, humDHT);
    Blynk.virtualWrite(V4, abshum);
    int adsread = ads.readADC_SingleEnded(0);

    Blynk.virtualWrite(V14, relaystate);
    Blynk.virtualWrite(V15, ledValue);
    Blynk.virtualWrite(V16, sethum);
    Blynk.virtualWrite(V17, bridgetemp);
    Blynk.virtualWrite(V18, adsread);
    if (adsread > 900) {
      Blynk.virtualWrite(V19, true);
      Blynk.virtualWrite(V20, 255);
    } else {
      Blynk.virtualWrite(V19, false);
      Blynk.virtualWrite(V20, 0);
      }
    Blynk.virtualWrite(V21, tempSHT);
    Blynk.virtualWrite(V22, humSHT);
  }



  if (millis() - millisAvg >= 5000)  //if it's been 5 second
  {
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
     intempstring = String(tempDHT) + "°C";
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
