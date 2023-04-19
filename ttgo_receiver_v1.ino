#include <SPI.h>
#include <LoRa.h>
#include <mySD.h>

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
String rssi;
ext::File myFile;
#define SDSS 13

void getLoRaData() {
  //Serial.print("Lora packet received: ");
  // Read packet
  while (LoRa.available()) {
    String LoRaData = LoRa.readString();
    Serial.println(LoRaData); 
    myFile = SD.open("gksat2.txt", FILE_WRITE);
    if(myFile){
      myFile.println(LoRaData);
      Serial.println("ok to file");
    myFile.close();
  }
  }
 
  // Get RSSI
  //rssi = LoRa.packetRssi();
  //Serial.print(" with RSSI ");    
  //Serial.println(rssi);
}
void setup() {
   //pinMode(14, OUTPUT);
  Serial.begin(115200);
  //Serial.println("LoRa Reciver Test");
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  if (!SD.begin(SDSS, 15, 2, 14)) {
    Serial.println("initialization failed!");
   }
  Serial.println("initialization done.");
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      getLoRaData();
  }
}
