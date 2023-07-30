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
#include <Average.h>
#include <SPI.h>
#include <Adafruit_ADS1X15.h>
Adafruit_ADS1115 ads;
GFXcanvas1 canvas1(128, 16);
GFXcanvas1 canvas2(128, 16);
GFXcanvas1 canvas3(128, 16);
GFXcanvas1 canvas4(128, 16);
GFXcanvas1 canvas5(128, 16);
GFXcanvas1 canvas6(128, 16);



#define VREF 4.096              // analog reference voltage(Volt) of the ADC
#define SCOUNT  30            // sum of sample point
int analogBuffer[SCOUNT];     // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float uncompensatedTdsValue = 0;
float tdsValueOld = 0;
bool firstTDS = true;
float grains = 0;
float correctedGas;
float aaH = 0.0000669005032586238;
float baH = -0.0159747107540879;
float abH = -0.00748672213648967;
float bbH = 1.78146466558454;

#define BLACK   0x0000
#define BLUE    0x001F
#define LIGHTRED 0xFA9F
#define LIGHTBLUE 0x9E3F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0  
#define WHITE   0xFFFF 
#define GREY 0x8410





#define __CS 2
#define __DC 16
#define __A0 0
#define TFT_CS 2
#define TFT_DC  16
#define TFT_RST 0


TFT_ILI9163C display = TFT_ILI9163C(__CS, __A0, __DC);

float correctionFactor(float temp, float hum) {
  return (aaH * hum * temp) + (baH * temp) + (abH * hum) + bbH;
}

int getMedianNum(int bArray[], int iFilterLen){
  int bTab[iFilterLen];
  for (byte i = 0; i<iFilterLen; i++)
  bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++) {
    for (i = 0; i < iFilterLen - j - 1; i++) {
      if (bTab[i] > bTab[i + 1]) {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0){
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}


Average<float> gasAvg(20);
Average<float> ppmAvg(20);
Average<float> ppmAvg2(20);
Average<float> a2Avg(20);
Average<float> tempAvg(20);
Average<float> humAvg(20);
float tempAvgHolder, humAvgHolder, absHumAvgHolder, tempAvgHolder2, humAvgHolder2;






// GPIO where the DS18B20 is connected to
const int oneWireBus = 12;     
#define DS18B20_PIN 12
int c_temp;
char c_buffer[9], f_buffer[9];

int gasRead;




const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = -14400;   //Replace with your GMT offset (seconds)
const int   daylightOffset_sec = 0;  //Replace with your daylight offset (seconds)





  float adc0, adc1, adc2, adc3;
  float volts0, volts1, volts2, volts3;

int measureDelay = 500;
int Ra=25;
int R1= 1000 + Ra;
int ECPin= A0;
int ECGround = 16;
int ECPower =10;
float PPMconversion=0.5; 
float TemperatureCoef = 0.0187;
float K=2;
float EC=0;
float EC2=0;
float EC25 =0;
float EC252 = 0;
int ppm =0;
int ppm2 =0;
float raw= 0;
float raw2= 0;
float Vin= 3.3;
float Vdrop= 0;
float Vdrop2= 0;
float Rc= 0;
float Rc2= 0;
float buffer=0;

const char* ssid = "mikesnet";
const char* password = "springchicken";


#define SEALEVELPRESSURE_HPA (1013.25)
#define tempoffset -0.9F

char auth[] = "pO--Yj8ksH2fjJLMW6yW9trkHBhd9-wc"; //BLYNK

AsyncWebServer server(80);
Adafruit_BME280 bme; // I2C
float abshumBME, tempBME, presBME, humBME;
float tempprobe = 20;
unsigned long millisBlynk = 0;
unsigned long millisTFT = 0;
unsigned long millisAvg = 0;
int firstvalue = 1;



WidgetTerminal terminal(V10);

BLYNK_WRITE(V10)
{
    if (String("help") == param.asStr()) 
    {
    terminal.println("==List of available commands:==");
    terminal.println("wifi");
    terminal.println("readgas");
    terminal.println("readtds");
    terminal.println("pushboth");
     terminal.println("==End of list.==");
    }
        if (String("wifi") == param.asStr()) 
    {
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
         if (String("readtds") == param.asStr()) {
        printtemp2();
     }
              if (String("pushboth") == param.asStr()) {
        printtemp3();
     }
    terminal.flush();

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

void displayLocalTime()
{
  time_t rawtime;
  struct tm * timeinfo;
  time (&rawtime);
  timeinfo = localtime (&rawtime);
  display.print(asctime(timeinfo));
}


void printtemp() {

            ads.setGain(GAIN_SIXTEEN);
adc3 = ads.readADC_SingleEnded(3);
adc3 = ads.readADC_SingleEnded(3);
        ads.setGain(GAIN_TWO);
  volts3 = ads.computeVolts(adc3);
  terminal.println("----------GAS---------");
  terminal.println("AIN3: "); terminal.print(adc3); terminal.print("  "); terminal.print(volts3); terminal.println("V");
  terminal.flush();
}

void printtemp2() {
                ads.setGain(GAIN_TWO);
adc1 = ads.readADC_SingleEnded(1);
adc1 = ads.readADC_SingleEnded(1);
        ads.setGain(GAIN_TWO);
  volts1 = ads.computeVolts(adc1);
  terminal.println("----------TDS---------");
  terminal.println("AIN1: "); terminal.print(adc1); terminal.print("  "); terminal.print(volts1); terminal.println("V");
  terminal.print("Code volts: ");
  terminal.println(averageVoltage);
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
  Blynk.virtualWrite(V9, gasAvg.mean());
  Blynk.virtualWrite(V11, ppmAvg.mean());
  Blynk.virtualWrite(V12, a2Avg.mean());
terminal.println("Gas: ");
terminal.print(gasAvg.mean());
terminal.print(", PPM from avg: ");
terminal.print(ppmAvg.mean());
terminal.print(", PPM from sensor: ");
terminal.print(tdsValue);
terminal.print(", A2 from avg: ");
terminal.print(a2Avg.mean());
terminal.print(", A2 from sensor: ");
terminal.print(ads.readADC_SingleEnded(1));
terminal.println("");
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


float oldtempAvgHolder2, oldhumAvgHolder2, oldabshumBME, oldtempprobe;
int oldtdsValue, oldgasRead;

void doDisplay() {
  int x = 5;
  int y = 5;

  canvas1.setTextSize(2);
  canvas1.setCursor(0,0);
  canvas1.fillScreen(0);
  canvas1.print("T:");
  canvas1.print(tempAvgHolder2);
  canvas1.print((char)0xF7);
  canvas1.println("C");
  display.drawBitmap(x, y, canvas1.getBuffer(), 128, 16, MAGENTA, BLACK); 
  y += 16;

  
  canvas2.setTextSize(2);
  canvas2.setCursor(0,0);
  canvas2.fillScreen(0);
  canvas2.print("RH: ");
  canvas2.print(humAvgHolder2);
  canvas2.print("%");
  display.drawBitmap(x, y, canvas2.getBuffer(), 128, 16, CYAN, BLACK);
  y += 16;

  canvas3.setTextSize(2);
  canvas3.setCursor(0,0);
  canvas3.fillScreen(0);
  canvas3.print("AH: ");
  canvas3.println(abshumBME);
  display.drawBitmap(x, y, canvas3.getBuffer(), 128, 16, YELLOW, BLACK);
  y += 16;


  canvas4.setTextSize(2);
  canvas4.setCursor(0,0);
  canvas4.fillScreen(0);
  canvas4.print("TDS: ");
  canvas4.println((int)(tdsValue));
  display.drawBitmap(x, y, canvas4.getBuffer(), 128, 16, GREEN, BLACK);
  y += 16;

  canvas5.setTextSize(2);
  canvas5.setCursor(0,0);
  canvas5.fillScreen(0);
  canvas5.print("GAS: ");
  canvas5.println((int)(gasRead));
  display.drawBitmap(x, y, canvas5.getBuffer(), 128, 16, WHITE, BLACK);
  y += 16;


  canvas6.setTextSize(2);
  canvas6.setCursor(0,0);
  canvas6.fillScreen(0);
  canvas6.print("t:");
  canvas6.print(tempprobe);
  canvas6.print((char)0xF7);
  canvas6.println("C");
  display.drawBitmap(x, y, canvas6.getBuffer(), 128, 16, RED, BLACK);
  y += 16;



}

//============================================CUBE BEGIN
const float sin_d[] = { 
  0,0.17,0.34,0.5,0.64,0.77,0.87,0.94,0.98,1,0.98,0.94,
  0.87,0.77,0.64,0.5,0.34,0.17,0,-0.17,-0.34,-0.5,-0.64,
  -0.77,-0.87,-0.94,-0.98,-1,-0.98,-0.94,-0.87,-0.77,
  -0.64,-0.5,-0.34,-0.17 };
const float cos_d[] = { 
  1,0.98,0.94,0.87,0.77,0.64,0.5,0.34,0.17,0,-0.17,-0.34,
  -0.5,-0.64,-0.77,-0.87,-0.94,-0.98,-1,-0.98,-0.94,-0.87,
  -0.77,-0.64,-0.5,-0.34,-0.17,0,0.17,0.34,0.5,0.64,0.77,
  0.87,0.94,0.98};
const float d = 10;
float px[] = { 
  -d,  d,  d, -d, -d,  d,  d, -d };
float py[] = { 
  -d, -d,  d,  d, -d, -d,  d,  d };
float pz[] = { 
  -d, -d, -d, -d,  d,  d,  d,  d };

float p2x[] = {
  0,0,0,0,0,0,0,0};
float p2y[] = {
  0,0,0,0,0,0,0,0};

int r[] = {
  0,0,0};
//=========================================================

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
      terminal.println("**********COSTELLOOOO v0.7***********");
      
    terminal.print("Connected to ");
    terminal.println(ssid);
    terminal.print("IP address: ");
    terminal.println(WiFi.localIP());
    
    printLocalTime();
    terminal.flush();  
    display.setTextColor(GREEN);
    display.println("");
    display.print("Connected to ");
    display.println(ssid);
    display.print("IP address: ");
    display.println(WiFi.localIP());
     displayLocalTime();


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
   delay(3000);
           bme.takeForcedMeasurement();       
        tempAvgHolder2 = (bme.readTemperature() + tempoffset);
        humAvgHolder2 = bme.readHumidity();
        ads.setGain(GAIN_SIXTEEN);
        gasRead = ads.readADC_SingleEnded(3);
        gasRead = ads.readADC_SingleEnded(3);
        ads.setGain(GAIN_TWO);
        gasAvg.push(gasRead);
        if (tempAvgHolder2 > 0) {tempAvg.push(tempAvgHolder2);}
        if (humAvgHolder2 > 0) {humAvg.push(humAvgHolder2);}
   display.clearScreen();
  for (int k=0;k<100;k++) {
    //display.fillScreen(BLACK);
  r[0]=r[0]+1;
  r[1]=r[1]+1;
  if (r[0] == 36) r[0] = 0;
  if (r[1] == 36) r[1] = 0;
  if (r[2] == 36) r[2] = 0;
  for (int i=0;i<8;i++)
  {
     
    float px2 = px[i];
    float py2 = cos_d[r[0]]*py[i] - sin_d[r[0]]*pz[i];
    float pz2 = sin_d[r[0]]*py[i] + cos_d[r[0]]*pz[i];

    float px3 = cos_d[r[1]]*px2 + sin_d[r[1]]*pz2;
    float py3 = py2;
    float pz3 = -sin_d[r[1]]*px2 + cos_d[r[1]]*pz2;

    float ax = cos_d[r[2]]*px3 - sin_d[r[2]]*py3;
    float ay = sin_d[r[2]]*px3 + cos_d[r[2]]*py3;
    float az = pz3-190;

    p2x[i] = ((display.width())/2)+ax*500/az;
    p2y[i] = ((display.height())/2)+ay*500/az;
  }
  for (int i=0;i<3;i++) {
    display.drawLine(p2x[i],p2y[i],p2x[i+1],p2y[i+1],WHITE);
    display.drawLine(p2x[i+4],p2y[i+4],p2x[i+5],p2y[i+5],RED);
    display.drawLine(p2x[i],p2y[i],p2x[i+4],p2y[i+4],BLUE);
  }   
  display.drawLine(p2x[3],p2y[3],p2x[0],p2y[0],MAGENTA);
  display.drawLine(p2x[7],p2y[7],p2x[4],p2y[4],GREEN);
  display.drawLine(p2x[3],p2y[3],p2x[7],p2y[7],YELLOW);
  delay(100);
    for (int i=0;i<3;i++) {
    display.drawLine(p2x[i],p2y[i],p2x[i+1],p2y[i+1],BLACK);
    display.drawLine(p2x[i+4],p2y[i+4],p2x[i+5],p2y[i+5],BLACK);
    display.drawLine(p2x[i],p2y[i],p2x[i+4],p2y[i+4],BLACK);
  }   
    display.drawLine(p2x[3],p2y[3],p2x[0],p2y[0],BLACK);
  display.drawLine(p2x[7],p2y[7],p2x[4],p2y[4],BLACK);
  display.drawLine(p2x[3],p2y[3],p2x[7],p2y[7],BLACK);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
Blynk.run();

  static unsigned long analogSampleTimepoint = millis();

  if(millis()-analogSampleTimepoint > 40U){     //every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = ads.readADC_SingleEnded(1);    //read the analog value and store into the buffer
    analogBufferIndex++;
    if(analogBufferIndex == SCOUNT){ 
      analogBufferIndex = 0;
    }
  }

    static unsigned long printTimepoint = millis();
  if(millis()-printTimepoint > 800U){
    printTimepoint = millis();
    for(copyIndex=0; copyIndex<SCOUNT; copyIndex++){
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
      float medianNum = getMedianNum(analogBufferTemp,SCOUNT);
      a2Avg.push(medianNum);
      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp,SCOUNT) * (float)VREF / 65535.0;
      
      //temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0)); 
      float compensationCoefficient = 1.0+0.02*(tempprobe-25.0);
      float compensationCoefficient2 = 1.0+0.02*(25-25.0);
      //temperature compensation
      float compensationVoltage=averageVoltage/compensationCoefficient;
      float compensationVoltage2=averageVoltage/compensationCoefficient2;
      
      //convert voltage value to tds value
      tdsValue=(133.42*compensationVoltage*compensationVoltage*compensationVoltage - 255.86*compensationVoltage*compensationVoltage + 857.39*compensationVoltage)*0.5;
      uncompensatedTdsValue=(133.42*compensationVoltage2*compensationVoltage2*compensationVoltage2 - 255.86*compensationVoltage2*compensationVoltage2 + 857.39*compensationVoltage2)*0.5;
      
      //Serial.print("voltage:");
      //Serial.print(averageVoltage,2);
      //Serial.print("V   ");
      if (!firstTDS) {
          if (tdsValue > 100) {
            ppmAvg.push(tdsValue);
            ppmAvg2.push(uncompensatedTdsValue);
          }
      }
      else {
        ppmAvg.push(tdsValue);
        ppmAvg2.push(uncompensatedTdsValue);
        }
      firstTDS = false;

    }
  }


    if  (millis() - millisBlynk >= 30000)  //if it's been 30 seconds
    {
      bme.takeForcedMeasurement();
        
        presBME = (bme.readPressure() / 100.0F);
        humAvgHolder = humAvg.mean();
        tempAvgHolder = tempAvg.mean();
        absHumAvgHolder = (6.112 * pow(2.71828, ((17.67 * tempAvgHolder)/(tempAvgHolder + 243.5))) * humAvgHolder * 2.1674)/(273.15 + tempAvgHolder);
        millisBlynk = millis();

        Blynk.virtualWrite(V1, presBME);
        Blynk.virtualWrite(V2, tempAvgHolder);
        Blynk.virtualWrite(V3, humAvgHolder);
        Blynk.virtualWrite(V4, absHumAvgHolder);
        Blynk.virtualWrite(V5, tempprobe);
       //  Blynk.virtualWrite(V6, raw);
 //Blynk.virtualWrite(V7, ppm);
 
 
 //float ppmAvgHolder = ppmAvg.mean();
  Blynk.virtualWrite(V9, gasAvg.mean());
  Blynk.virtualWrite(V11, ppmAvg.mean());
  Blynk.virtualWrite(V8, ppmAvg2.mean());
  Blynk.virtualWrite(V12, a2Avg.mean());
  float Rs = (ads.computeVolts(gasAvg.mean())*47000)/(5.0-ads.computeVolts(gasAvg.mean()));
  correctedGas = (Rs / correctionFactor(tempAvgHolder, humAvgHolder));
  Blynk.virtualWrite(V13, correctedGas);

    }

    if  (millis() - millisTFT >= 3000)  //if it's been 3 seconds
    {
      //bme.takeForcedMeasurement();
      
        //tempBME = (bme.readTemperature() + tempoffset);
        //presBME = (bme.readPressure() / 100.0F);
        //humBME = bme.readHumidity();
        abshumBME = (6.112 * pow(2.71828, ((17.67 * tempAvgHolder2)/(tempAvgHolder2 + 243.5))) * humAvgHolder2 * 2.1674)/(273.15 + tempAvgHolder2);
        millisTFT = millis();
        doDisplay();
        tempprobe = readDStemp();
    }
    
    if  (millis() - millisAvg >= 1000)  //if it's been 1 second
    {
        millisAvg = millis();
        bme.takeForcedMeasurement();       
        tempAvgHolder2 = (bme.readTemperature() + tempoffset);
        humAvgHolder2 = bme.readHumidity();
        ads.setGain(GAIN_SIXTEEN);
        gasRead = ads.readADC_SingleEnded(3);
        gasRead = ads.readADC_SingleEnded(3);
        ads.setGain(GAIN_TWO);
        gasAvg.push(gasRead);
        if (tempAvgHolder2 > 0) {tempAvg.push(tempAvgHolder2);}
        if (humAvgHolder2 > 0) {humAvg.push(humAvgHolder2);}
    }
}



