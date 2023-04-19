#include <mySD.h>
ext::File myFile;
#define SDSS 13
 
void setup() {
  Serial.begin(115200);
  
  if (!SD.begin(SDSS, 15, 2, 14)) {
    Serial.println("initialization failed!");
   }
   else{
    Serial.println("initialization done.");
  {
}

void loop() {
  if (Serial.available()) {      // If anything comes in Serial (USB),
    myFile = SD.open("gksat2.txt", FILE_WRITE);
    if(myFile){
      myFile.println(Serial.readString());
      Serial.println("ok to file");
    myFile.close();
    }
  }
}
