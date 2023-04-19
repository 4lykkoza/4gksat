#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <mySD.h>
#include <ToneESP32.h>
#include <Adafruit_LIS3MDL.h>
//#include <Adafruit_Sensor.h>
#include <ESP32Time.h>
ESP32Time rtc(0);  // offset in seconds GMT+2
//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define BUZZER_PIN 33
#define BUZZER_CHANNEL 5

#define SDSS 13
//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 868E6
TinyGPS gps;

Adafruit_LIS3MDL lis3mdl;
Adafruit_BMP280 bmp; // I2C
ToneESP32 buzzer(BUZZER_PIN, BUZZER_CHANNEL);

//packet counter
ext::File myFile;
int counter = 0;
int checkSensors;
unsigned long start=0;
void setup() {
  checkSensors=0;
  rtc.setTime(0, 38, 12, 9, 4, 2023);  // 17th Jan 2021 15:24:30
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 34, 12);
  Serial.println("LoRa Sender Test");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    checkSensors=checkSensors+1;
  }
  Serial.println("LoRa Initializing OK!");
    if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      checkSensors=checkSensors+2;
    }
  pinMode(SDSS, OUTPUT);
  if (!SD.begin(SDSS, 15, 2, 14)) {
    Serial.println("initialization failed!");
    checkSensors=checkSensors+4;
    }
  Serial.println("initialization done.");
  // Try to initialize!
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Failed to find LIS3MDL chip");
    checkSensors=checkSensors+8;
  }
  Serial.println("LIS3MDL Found!");
  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setRange(LIS3MDL_RANGE_16_GAUSS);
  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
  Serial.print(checkSensors);
  if (checkSensors!=0){
  for(int i=0;i<=checkSensors;i++){
    buzzer.tone(NOTE_D4, 250);
    delay(250);
  }
  }else
  {
    buzzer.tone(NOTE_C5, 250);
    delay(250);
    buzzer.tone(NOTE_B4, 250);
  }

}

void loop() {
    /*Serial.println(rtc.getTime());*/
    // formating options  http://www.cplusplus.com/reference/ctime/strftime/

    struct tm timeinfo = rtc.getTimeStruct();
    //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");   //  (tm struct) Sunday, January 17 2021 07:24:38
  
    //delay(1000);
    float t=bmp.readTemperature();
    float p=bmp.readPressure();
    float rAlt=bmp.readAltitude();
    //float rSl=bmp.readSealevelPressure();
    //float rSalt=bmp.readAltitude(101500);
   /* Serial.print("Temperature = ");
    Serial.print(t);
    Serial.println(" *C");
    Serial.print("Pressure = ");
    Serial.print(p);
    Serial.println(" Pa");
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(rAlt);
    Serial.println(" meters");*/
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  //for (unsigned long start = millis(); millis() - start < 1000;)
  if (millis()-start>5000){
    while (Serial1.available())
    {
      char c =Serial1.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
    start=millis();
  }
  
      float flat, flon;
      unsigned long age;
      gps.f_get_position(&flat, &flon, &age);
     /* Serial.print("LAT=");
      Serial.print(flat);
      Serial.print(" LON=");
      Serial.println(flon);*/
    //}
   //}
   lis3mdl.read();      // get X Y and Z data at once
  // Then print out the raw data
  /*Serial.print("X: "); Serial.print(lis3mdl.x); 
  Serial.print("Y: "); Serial.print(lis3mdl.y); 
  Serial.print("Z: "); Serial.println(lis3mdl.z); */

  /* Or....get a new sensor event, normalized to uTesla */
  sensors_event_t event; 
  lis3mdl.getEvent(&event);
  /* Display the results (magnetic field is measured in uTesla) */
  /*
  Serial.print("X: "); Serial.print(event.magnetic.x);
  Serial.print("Y: "); Serial.print(event.magnetic.y); 
  Serial.print("Z: "); Serial.print(event.magnetic.z); 
  Serial.println(" uTesla ");*/
  String line=String(counter)+","+String(millis())+","+rtc.getTime()+","+String(flon)+","+String(flat)+","+String(rAlt)+","+String(t)+","+String(p)+","+String(event.magnetic.x)+","+String(event.magnetic.y)+","+String(event.magnetic.z);
  Serial.println(line);
  LoRa.beginPacket();
  LoRa.print(line);
  LoRa.endPacket();
  myFile = SD.open("test.txt", FILE_WRITE);
  if(myFile){
    myFile.println(line);
    //Serial.println("ok to file");
  }
  myFile.close();
  counter++;
  
  //buzzer.noTone();
}
