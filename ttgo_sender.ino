#include <SPI.h>
#include <LoRa.h>
//Libraries for OLED Display
#include <Adafruit_BMP085.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <CayenneLPP.h>
CayenneLPP lpp(52);
//define the pins used by the LoRa transceiver module
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26

//433E6 for Asia
//866E6 for Europe
//915E6 for North America
#define BAND 868E6
TinyGPS gps;
//packet counter
int counter = 0;

Adafruit_BMP085 bmp;
void setup() {
   //pinMode(14, OUTPUT);
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 34, 12);
  Serial.println("LoRa Sender Test");

  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");
    if (!bmp.begin()) {
      Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      while (1) {}
    }
}

void loop() {
    lpp.reset();
    float t=bmp.readTemperature();
    float p=bmp.readPressure();
    float rAlt=bmp.readAltitude();
    float rSl=bmp.readSealevelPressure();
    float rSalt=bmp.readAltitude(101500);
    lpp.addTemperature(1,t);
    lpp.addBarometricPressure(2,rSl/100);
    lpp.addAnalogInput(1,t);
    Serial.print("Temperature = ");
    Serial.print(t);
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(p);
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(rAlt);
    Serial.println(" meters");

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
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
    Serial.println();
    //Send LoRa packet to receiver
    if (newData)
    {
      float flat, flon;
      unsigned long age;
      gps.f_get_position(&flat, &flon, &age);
      lpp.addGPS(3,(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6),(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6),2);
      Serial.print("LAT=");
      Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      Serial.print(" LON=");
      Serial.println(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    }
  LoRa.beginPacket();
  //LoRa.write(lpp.getBuffer(), lpp.getSize());
  LoRa.print(t);
  LoRa.print(",");
  LoRa.print(p);
  LoRa.endPacket();
  
  counter++;
  delay(500);
  
}
