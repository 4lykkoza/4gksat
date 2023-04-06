#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP085.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <mySD.h>
#include <ToneESP32.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>
#include <time.h>
unsigned long myTime;
//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

#define BUZZER_PIN 33
#define BUZZER_CHANNEL 5

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 868E6
TinyGPS gps;

Adafruit_LIS3MDL lis3mdl;
Adafruit_BMP085 bmp;
ToneESP32 buzzer(BUZZER_PIN, BUZZER_CHANNEL);

//packet counter
ext::File myFile;
int counter = 0;
int checkSensors;
const int chipSelect = 13;
void setup() {
   //pinMode(14, OUTPUT);
   checkSensors=0;
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 34, 12);
  Serial.println("LoRa Sender Test");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    checkSensors+=1;
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
    if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      checkSensors+=2;
      while (1) {}
    }
  pinMode(SS, OUTPUT);
  if (!SD.begin(13, 15, 2, 14)) {
    Serial.println("initialization failed!");
    checkSensors+=3;
    return;
  }
  Serial.println("initialization done.");
  // Try to initialize!
  if (! lis3mdl.begin_I2C()) {          // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Failed to find LIS3MDL chip");
    checkSensors+=4;
    while (1) { delay(10); }
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
  if (checkSensors!=0){
  for(int i=0;i<=checkSensors;i++){
    buzzer.tone(NOTE_D4, 250);
    delay(250);
  }
  }else
  {
    buzzer.tone(NOTE_C5, 250);
    buzzer.tone(NOTE_D4, 250);
    delay(250);
    buzzer.tone(NOTE_C5, 250);
    buzzer.tone(NOTE_B4, 250);
  }

}

void loop() {
    myFile = SD.open("test.txt", FILE_WRITE);
    LoRa.beginPacket();
    float t=bmp.readTemperature();
    float p=bmp.readPressure();
    float rAlt=bmp.readAltitude();
    float rSl=bmp.readSealevelPressure();
    float rSalt=bmp.readAltitude(101500);
    Serial.print("Temperature = ");
    Serial.print(t);
    Serial.println(" *C");
    LoRa.print(t);
    LoRa.print(",");
    Serial.print("Pressure = ");
    Serial.print(p);
    Serial.println(" Pa");
    LoRa.print(p);
    LoRa.print(",");
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(rAlt);
    Serial.println(" meters");
    LoRa.print(rAlt);
    LoRa.print(",");
    Serial.print("Pressure at sealevel (calculated) = ");
    Serial.print(rSl);
    Serial.println(" Pa");

  // you can get a more precise measurement of altitude
  // if you know the current sea level pressure which will
  // vary with weather and such. If it is 1015 millibars
  // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(rSalt);
    Serial.println(" meters");
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (Serial1.available())
    {
      char c =Serial1.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  
  gps.stats(&chars, &sentences, &failed);
  Serial.print(" CHARS=");
  Serial.print(chars);
  Serial.print(" SENTENCES=");
  Serial.print(sentences);
  Serial.print(" CSUM ERR=");
  Serial.println(failed);
  /*if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
    Serial.println();
    //Send LoRa packet to receiver
   else{
    if (newData)
    {*/ 
      float flat, flon;
      unsigned long age;
      gps.f_get_position(&flat, &flon, &age);
      float pflat=(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 :flat, 6);
      float pflon=(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 :flon, 6);
      Serial.print("LAT=");
      Serial.print(pflat);
      LoRa.print(pflat);
      LoRa.print(",");
      Serial.print(" LON=");
      Serial.println(pflon);
      LoRa.print(pflon);
    //}
   //}
   lis3mdl.read();      // get X Y and Z data at once
  // Then print out the raw data
  Serial.print("\nX:  "); Serial.print(lis3mdl.x); 
  Serial.print("  \tY:  "); Serial.print(lis3mdl.y); 
  Serial.print("  \tZ:  "); Serial.println(lis3mdl.z); 

  /* Or....get a new sensor event, normalized to uTesla */
  sensors_event_t event; 
  lis3mdl.getEvent(&event);
  /* Display the results (magnetic field is measured in uTesla) */
  Serial.print("\tX: "); Serial.print(event.magnetic.x);
  Serial.print(" \tY: "); Serial.print(event.magnetic.y); 
  Serial.print(" \tZ: "); Serial.print(event.magnetic.z); 
  Serial.println(" uTesla ");
  LoRa.println();
  LoRa.endPacket();
  myTime=millis();
  String timeStamp=String(myTime);
  Serial.print("Current time obtained from RTC is: ");
  Serial.println(timeStamp);
  myFile = SD.open("test.txt", FILE_WRITE);
  if(myFile){
    String line=timeStamp+","+String(t)+","+String(p);
    myFile.println(line);
    Serial.println("ok to file");
  }
  myFile.close();
  counter++;
  
  //buzzer.noTone();
}
