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
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;

// GPIO where the DS18B20 is connected to
const int oneWireBus = 12;     
#define DS18B20_PIN 12
int c_temp;
char c_buffer[9], f_buffer[9];

int gasRead;




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

#define LIGHTBLUE 0x739F



#define __CS 2
#define __DC 16
#define __A0 0
#define TFT_CS 2
#define TFT_DC  16
#define TFT_RST 0




  float adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;

int measureDelay = 0;
int Ra=25;
int R1= 1000 + Ra;
int ECPin= A0;
int ECGround = 16;
int ECPower =10;
float PPMconversion=0.5; 
float TemperatureCoef = 0.0187;
float K=2;
float EC=0;
float EC25 =0;
float EC252 = 0;
int ppm =0;
int ppm2 =0;
float raw= 0;
float Vin= 3.3;
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
     if (String("md50") == param.asStr()) {
        measureDelay=50;
     }
     if (String("md100") == param.asStr()) {
        measureDelay=100;
     }
     if (String("md500") == param.asStr()) {
        measureDelay=500;
     }
          if (String("k1") == param.asStr()) {
            terminal.println("k=1");
        K=1;
     }
     if (String("k1.5") == param.asStr()) {
      terminal.println("k=1.5");
        K=1.5;
     }
     if (String("k2.5") == param.asStr()) {
      terminal.println("k=2.5");
         K=2.5;
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
      display.begin();
   display.clearScreen();
     display.setTextColor(YELLOW);
      display.setTextSize(1);
  //display.setCursor(0,0);
  display.print("Please wait, connecting to wifi...");

  WiFi.mode(WIFI_STA);
  WiFi.setPhyMode(WIFI_PHY_MODE_11B);
  WiFi.begin("mikesnet", "springchicken");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    display.print(".");
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
      terminal.println("**********COSTELLOOOO v0.5***********");
      printLocalTime();
    terminal.print("Connected to ");
    terminal.println(ssid);
    terminal.print("IP address: ");
    terminal.println(WiFi.localIP());
    terminal.flush();  
  



    bme.begin(0x76);
 bme.setSampling(Adafruit_BME280::MODE_FORCED,
               Adafruit_BME280::SAMPLING_X1, // temperature
                Adafruit_BME280::SAMPLING_X1, // pressure
                Adafruit_BME280::SAMPLING_X1, // humidity
                Adafruit_BME280::FILTER_OFF   );
                
   
                  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "I am Costello");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
  terminal.println("HTTP server started");
  terminal.flush();
  ads.setGain(GAIN_TWO);
    if (!ads.begin())
  {
    terminal.println("Failed to initialize ADS.");
    while (1);
  }
  else {terminal.println("ADS initialized");}
  terminal.println("Startup complete.");
   terminal.flush();
   display.clearScreen();
     pinMode(ECPower,OUTPUT);//Setting pin for sourcing current
  pinMode(ECGround,OUTPUT);//setting pin for sinking current
  digitalWrite(ECGround,LOW);//We can leave the ground connected permanantly
 
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
                GetEC();         
              // PrintReadings();
        gasRead = ads.readADC_SingleEnded(3);
        Blynk.virtualWrite(V1, presBME);
        Blynk.virtualWrite(V2, tempBME);
        Blynk.virtualWrite(V3, humBME);
        Blynk.virtualWrite(V4, abshumBME);
        Blynk.virtualWrite(V5, tempprobe);
         Blynk.virtualWrite(V6, raw);
 Blynk.virtualWrite(V7, ppm);
 Blynk.virtualWrite(V8, ppm2);
  Blynk.virtualWrite(V9, gasRead);
    }

    if  (millis() - millisTFT >= 5000)  //if it's been 30 seconds OR we just booted up, skip the 30 second wait
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
  //display.clearScreen();
  display.fillRect(5,5,120,70,BLACK);
    display.setCursor(5, 5);
  display.setTextSize(2);
  display.setCursor(5, 5);
  display.setTextColor(RED);
  display.print("T:");
  display.print(tempBME);
  display.print((char)247);
  display.println("C");
  display.setTextColor(LIGHTBLUE);
  display.print("RH: ");
  display.print(humBME);
  display.print("%");
  display.setTextColor(CYAN);
  display.print("AH: ");
  display.println(abshumBME);
    display.setTextColor(GREEN);
  display.print("PPM: ");
  display.println(ppm);
}


void GetEC(){
 
//        sensors.requestTemperatures();
         //       sensors.requestTemperatures();
       //         tempprobe = sensors.getTempCByIndex(0);
//*********Reading Temperature Of Solution *******************//
display.setCursor(5, 70);
  display.setTextColor(YELLOW);
  display.print("READING...");

 tempprobe = readDStemp();
 
//************Estimates Resistance of Liquid ****************//
  digitalWrite(ECPower,HIGH);
   delay(measureDelay);
raw = ads.readADC_SingleEnded(0);
raw = ads.readADC_SingleEnded(0);// This is not a mistake, First reading will be low beause if charged a capacitor
digitalWrite(ECPower,LOW);

display.setCursor(5, 70);
  display.setTextColor(BLACK);
  display.print("READING...");
  
   
 
 
//***************** Converts to EC **************************//
//Vdrop= (Vin*raw)/32768.0;
Vdrop = ads.computeVolts(raw);
Rc=(Vdrop*R1)/(Vin-Vdrop);
Rc=Rc-Ra; //acounting for Digital Pin Resitance
EC = 1000/(Rc*K);
 
 
//*************Compensating For Temperaure********************//
EC25  =  EC/ (1+ TemperatureCoef*(tempprobe-25.0));
EC252  =  EC/ (1+ TemperatureCoef*(25-25.0));
ppm=(EC25)*(PPMconversion*1000);
ppm2=(EC252)*(PPMconversion*1000);



 
 
}
//************************** End OF EC Function ***************************//
 
 
 
 
//***This Loop Is called From Main Loop- Prints to serial usefull info ***//
void PrintReadings(){
terminal.println("");
terminal.print("Raw: ");
terminal.print(raw);
terminal.print(" EC: ");
terminal.print(EC25);
terminal.print(" Vdrop:  ");
terminal.print(Vdrop);
terminal.print(" PPM:  ");
terminal.print(ppm);
terminal.print(" Temp:  ");
terminal.print(tempprobe);
terminal.println("°C ");
 terminal.flush();
 Blynk.virtualWrite(V6, raw);
 Blynk.virtualWrite(V7, ppm);
 Blynk.virtualWrite(V8, ppm2);

 
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
  digitalWrite(ECPower,HIGH);
  delay(measureDelay);
adc0 = ads.readADC_SingleEnded(0);
adc0 = ads.readADC_SingleEnded(0);// This is not a mistake, First reading will be low beause if charged a capacitor
digitalWrite(ECPower,LOW);

 
  volts0 = ads.computeVolts(adc0);

 

  terminal.println("-----------------------------------------------------------");
  terminal.print("AIN0: "); terminal.print(adc0); terminal.print("  "); terminal.print(volts0); terminal.println("V");
  terminal.flush();
}

void printtemp2() {
  gasRead = ads.readADC_SingleEnded(3);
  volts3 = ads.computeVolts(gasRead);
  terminal.print("Gas value: ");
  terminal.println(gasRead);
  terminal.print("Volts: ");
  terminal.println(volts3);
  terminal.flush();
}




bool ds18b20_start()
{
  bool ret = 0;
  digitalWrite(DS18B20_PIN, LOW);  // send reset pulse to the DS18B20 sensor
  pinMode(DS18B20_PIN, OUTPUT);
  delayMicroseconds(500);          // wait 500 us
  pinMode(DS18B20_PIN, INPUT);
  delayMicroseconds(100);          // wait to read the DS18B20 sensor response
  if (!digitalRead(DS18B20_PIN))
  {
    ret = 1;                  // DS18B20 sensor is present
    delayMicroseconds(400);   // wait 400 us
  }
  return(ret);
}

void ds18b20_write_bit(bool value)
{
  digitalWrite(DS18B20_PIN, LOW);
  pinMode(DS18B20_PIN, OUTPUT);
  delayMicroseconds(2);
  digitalWrite(DS18B20_PIN, value);
  delayMicroseconds(80);
  pinMode(DS18B20_PIN, INPUT);
  delayMicroseconds(2);
}

void ds18b20_write_byte(byte value)
{
  byte i;
  for(i = 0; i < 8; i++)
    ds18b20_write_bit(bitRead(value, i));
}

bool ds18b20_read_bit(void)
{
  bool value;
  digitalWrite(DS18B20_PIN, LOW);
  pinMode(DS18B20_PIN, OUTPUT);
  delayMicroseconds(2);
  pinMode(DS18B20_PIN, INPUT);
  delayMicroseconds(5);
  value = digitalRead(DS18B20_PIN);
  delayMicroseconds(100);
  return value;
}

uint16_t ds18b20_read_byte(void)
 {  
 byte i;
 uint16_t value=0;
  for(i = 0; i < 12 ; i++)   
  bitWrite(value, i, ds18b20_read_bit());
  //terminal.print(value);
  return value;
}

bool ds18b20_read(int *raw_temp_value)
{
  if (!ds18b20_start())  // send start pulse
    return(0);
  ds18b20_write_byte(0xCC);   // send skip ROM command
  ds18b20_write_byte(0x44);   // send start conversion command
  while(ds18b20_read_byte() == 0);  // wait for conversion complete
  if (!ds18b20_start())             // send start pulse
    return(0);                      // return 0 if error
  ds18b20_write_byte(0xCC);         // send skip ROM command
  ds18b20_write_byte(0xBE);         // send read command

  // read temperature LSB byte and store it on raw_temp_value LSB byte
  *raw_temp_value = ds18b20_read_byte();
  // read temperature MSB byte and store it on raw_temp_value MSB byte
  *raw_temp_value |= (unsigned int)(ds18b20_read_byte() << 8);

  return(1);  // OK --> return 1
}

void printtemp3(){
  tempprobe = readDStemp();
terminal.print(tempprobe);
terminal.println("°C");
}

float readDStemp(void) {
       if( ds18b20_read(&c_temp) ) {  
      // read from DS18B20 sensor OK
  
      // calculate temperature in °F (actual temperature in °F = f_temp/160)
      // °F = °C x 9/5 + 32
      int32_t f_temp = (int32_t)c_temp * 90/5 + 5120;  // 5120 = 32 x 16 x 10
  
      if(c_temp < 0) {   // if temperature < 0 °C
      c_temp = abs(c_temp);  // absolute value
      sprintf(c_buffer, "-%02u.%04u", c_temp/16, (c_temp & 0x0F) * 625);
    }
    else {
      if (c_temp/16 >= 100)    // if temperature >= 100.0 °C
        sprintf(c_buffer, "%03u.%04u", c_temp/16, (c_temp & 0x0F) * 625);
      else
        sprintf(c_buffer, " %02u.%04u", c_temp/16, (c_temp & 0x0F) * 625);
    }
  
    if(f_temp < 0) {   // if temperature < 0 °F
      f_temp = abs(f_temp);  // absolute value
      sprintf(f_buffer, "-%02u.%04u", (uint16_t)f_temp/160, (uint16_t)(f_temp*1000/16 % 10000));
    }
    else {
      if (f_temp/160 >= 100)    // if temperature >= 100.0 °F
        sprintf(f_buffer, "%03u.%04u", (uint16_t)f_temp/160, (uint16_t)(f_temp*1000/16 % 10000));
      else
        sprintf(f_buffer, " %02u.%04u", (uint16_t)f_temp/160, (uint16_t)(f_temp*1000/16 % 10000));
    }
return atof(c_buffer);
  }
  else return -69.69;
}
