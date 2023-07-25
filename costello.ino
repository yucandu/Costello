#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>
#include <Adafruit_SPIDevice.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include "TFT_ILI9163C.h"
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>
#include "time.h"

#include <SPI.h>
#include <ILI9163.h>


// GPIO where the DS18B20 is connected to
const int oneWireBus = 12;     








const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -14400;   //Replace with your GMT offset (seconds)
const int   daylightOffset_sec = 0;  //Replace with your daylight offset (seconds)

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0  
#define WHITE   0xFFFF



#define __CS 2
#define __DC 16
#define __A0 0
#define TFT_CS 2
#define TFT_DC  16
#define TFT_RST 0






int Ra=25;
int R1= 80 + Ra;
int ECPin= A0;

int ECPower =10;
float PPMconversion=0.5; 
float TemperatureCoef = 0.0187;
float K=2.88;
float EC=0;
float EC25 =0;
int ppm =0;
float raw= 0;
float Vin= 5;
float Vdrop= 0;
float Rc= 0;
float buffer=0;

const char* ssid = "mikesnet";
const char* password = "springchicken";


#define SEALEVELPRESSURE_HPA (1013.25)
#define tempoffset -0.9F

char auth[] = "pO--Yj8ksH2fjJLMW6yW9trkHBhd9-wc"; //BLYNK

AsyncWebServer server(80);
Adafruit_BME280 bme; // I2C
float abshumBME, tempBME, presBME, humBME, tempprobe;
unsigned long millisBlynk = 0;
unsigned long millisTFT = 0;
int firstvalue = 1;



WidgetTerminal terminal(V10);

BLYNK_WRITE(V10)
{
    if (String("help") == param.asStr()) 
    {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("tds");
    terminal.println("temp");
    terminal.println("temp2");
    terminal.println("temp3");
     terminal.println("==End of list.==");
    }
        if (String("wifi") == param.asStr()) 
    {
        terminal.print("Connected to: ");
        terminal.println(ssid);
        terminal.print("IP address:");
        terminal.println(WiFi.localIP());
        terminal.print("Signal strength: ");
        terminal.println(WiFi.RSSI());
    }

     if (String("tds") == param.asStr()) {
            GetEC();         
      PrintReadings();
     }
     if (String("temp") == param.asStr()) {
        printtemp();
     }
         if (String("temp2") == param.asStr()) {
        printtemp2();
     }
              if (String("temp3") == param.asStr()) {
        printtemp3();
     }
    terminal.flush();

}

TFT_ILI9163C display = TFT_ILI9163C(__CS, __A0, __DC);


void setup() {

  Serial.begin(115200);


  WiFi.mode(WIFI_STA);
  WiFi.setPhyMode(WIFI_PHY_MODE_11B);
  WiFi.begin("mikesnet", "springchicken");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

 delay(1000);
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
      Blynk.config(auth, IPAddress(192, 168, 50, 197), 8080);
    Blynk.connect();
      terminal.println("**********COSTELLOOOO v0.13***********");
      printLocalTime();
    terminal.print("Connected to ");
    terminal.println(ssid);
    terminal.print("IP address: ");
    terminal.println(WiFi.localIP());
      
  
      display.begin();
   display.clearScreen();
     display.setTextColor(YELLOW);
      display.setTextSize(2);
  //display.setCursor(0,0);
  display.println("Please wait, connecting to wifi...");


    bme.begin(0x76);
 bme.setSampling(Adafruit_BME280::MODE_FORCED,
               Adafruit_BME280::SAMPLING_X1, // temperature
                Adafruit_BME280::SAMPLING_X1, // pressure
                Adafruit_BME280::SAMPLING_X1, // humidity
                Adafruit_BME280::FILTER_OFF   );
                
    terminal.flush();
                  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "I am Costello");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  terminal.println("HTTP server started");
}

void loop() {
  // put your main code here, to run repeatedly:
Blynk.run();


    if  (millis() - millisBlynk >= 30000)  //if it's been 30 seconds OR we just booted up, skip the 30 second wait
    {
      bme.takeForcedMeasurement();
        tempBME = (bme.readTemperature() + tempoffset);
        presBME = (bme.readPressure() / 100.0F);
        humBME = bme.readHumidity();

        abshumBME = (6.112 * pow(2.71828, ((17.67 * tempBME)/(tempBME + 243.5))) * humBME * 2.1674)/(273.15 + tempBME);
        millisBlynk = millis();
               // GetEC();         
              // PrintReadings();
        Blynk.virtualWrite(V1, presBME);
        Blynk.virtualWrite(V2, tempBME);
        Blynk.virtualWrite(V3, humBME);
        Blynk.virtualWrite(V4, abshumBME);
        if (tempprobe > 0) {Blynk.virtualWrite(V5, tempprobe);}
    }

    if  (millis() - millisTFT >= 10000)  //if it's been 30 seconds OR we just booted up, skip the 30 second wait
    {
  /*display.setTextColor(BLACK);  
  display.setCursor(5, 5);
  display.setTextSize(2);
  display.print("T: ");
  display.print(tempBME);
  display.println("°C");
  display.print("RH: ");
  display.print(humBME);
  display.println("%");
  display.print("AH: ");
  display.println(abshumBME);
  display.print("P: ");
  display.println(presBME);*/
      bme.takeForcedMeasurement();
        tempBME = (bme.readTemperature() + tempoffset);
        presBME = (bme.readPressure() / 100.0F);
        humBME = bme.readHumidity();
        abshumBME = (6.112 * pow(2.71828, ((17.67 * tempBME)/(tempBME + 243.5))) * humBME * 2.1674)/(273.15 + tempBME);
        millisTFT = millis();
        doDisplay();


    }
}

void doDisplay() {
  display.clearScreen();
    display.setCursor(5, 5);
  display.setTextSize(2);
  display.setCursor(5, 5);
  display.setTextColor(RED);
  display.print("T:");
  display.print(tempBME);
  display.print((char)247);
  display.println("C");
  display.setTextColor(BLUE);
  display.print("RH: ");
  display.print(humBME);
  display.println("%");
  display.setTextColor(CYAN);
  display.print("AH: ");
  display.println(abshumBME);
    display.setTextColor(GREEN);
  display.print("P: ");
  display.println(presBME);
}


void GetEC(){
 
//        sensors.requestTemperatures();
         //       sensors.requestTemperatures();
       //         tempprobe = sensors.getTempCByIndex(0);
//*********Reading Temperature Of Solution *******************//


 
 
//************Estimates Resistance of Liquid ****************//
digitalWrite(ECPower,HIGH);
raw= analogRead(ECPin);
raw= analogRead(ECPin);// This is not a mistake, First reading will be low beause if charged a capacitor
digitalWrite(ECPower,LOW);
 
 
 
 
//***************** Converts to EC **************************//
Vdrop= (Vin*raw)/1024.0;
Rc=(Vdrop*R1)/(Vin-Vdrop);
Rc=Rc-Ra; //acounting for Digital Pin Resitance
EC = 1000/(Rc*K);
 
 
//*************Compensating For Temperaure********************//
EC25  =  EC/ (1+ TemperatureCoef*(tempprobe-25.0));
ppm=(EC25)*(PPMconversion*1000);
 
 
}
//************************** End OF EC Function ***************************//
 
 
 
 
//***This Loop Is called From Main Loop- Prints to serial usefull info ***//
void PrintReadings(){
terminal.print("Rc: ");
terminal.print(Rc);
terminal.print(" EC: ");
terminal.print(EC25);
terminal.print(" Simens  ");
terminal.print(ppm);
terminal.print(" ppm  ");
terminal.print(tempprobe);
terminal.println(" *C ");
 terminal.flush();
 Blynk.virtualWrite(V6, Rc);
 Blynk.virtualWrite(V7, EC25);
 Blynk.virtualWrite(V8, ppm);
 
}


void printLocalTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  terminal.print("-");
  terminal.print(asctime(timeinfo));
  terminal.print(" - ");
  terminal.flush();
}

void printtemp() {
  tempprobe = 696969;
  float tempprobe1 = 696969;
  terminal.print(tempprobe);
  terminal.print(", ");
  terminal.print(tempprobe1);
  terminal.println(" *C ");
  terminal.flush();
}

void printtemp2() {
  terminal.println("no");
  terminal.flush();
}


void printtemp3(){

  
}



