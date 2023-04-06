#include <AsyncEventSource.h>
#include <AsyncJson.h>
#include <AsyncWebSocket.h>
#include <AsyncWebSynchronization.h>
#include <ESPAsyncWebSrv.h>
#include <SPIFFSEditor.h>
#include <StringArray.h>
#include <WebAuthentication.h>
#include <WebHandlerImpl.h>
#include <WebResponseImpl.h>
#include <SPIFFS.h>


#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
//#include <ArduinoJson.h>
//#include <CayenneLPP.h>
//CayenneLPP lpp(52);
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
const char* ssid     = "george";
const char* password = "12345678";
String temperature;
String pressure;
String alt;
String rssi;

AsyncWebServer server(80);
void connectWiFi(){
  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void getLoRaData() {
  Serial.print("Lora packet received: ");
  // Read packet
  while (LoRa.available()) {
    String LoRaData = LoRa.readString();
    // LoRaData format: readingID/temperature&soilMoisture#batterylevel
    // String example: 1/27.43&654#95.34
    Serial.print(LoRaData); 
    
    // Get readingID, temperature and soil moisture
    int pos1 = 5;//LoRaData.indexOf('/');
    int pos2 = 14;//LoRaData.indexOf('&');
    int pos3 = 15;//LoRaData.indexOf('#');
    temperature = LoRaData.substring(0, pos1);
    pressure = LoRaData.substring(pos1+1, pos2);
    alt = LoRaData.substring(pos2+1, LoRaData.length()-1);    
  }
  // Get RSSI
  rssi = LoRa.packetRssi();
  Serial.print(" with RSSI ");    
  Serial.println(rssi);
}
String processor(const String& var){
  //Serial.println(var);
  if(var == "TEMPERATURE"){
    return temperature;
  }
  else if(var == "ALTITUDE"){
    return alt;
  }
  else if(var == "PRESSURE"){
    return pressure;
  }
  else if (var == "RRSI"){
    return String(rssi);
  }
  return String();
}

void setup() {
   //pinMode(14, OUTPUT);
  Serial.begin(115200);
   connectWiFi();
  //Serial.println("LoRa Reciver Test");
  //SPI LoRa pins
  SPI.begin(SCK, MISO, MOSI, SS);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(868E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });
  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", temperature.c_str());
  });
  server.on("/alt", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", alt.c_str());
  });
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", pressure.c_str());
  });
  server.on("/rssi", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", String(rssi).c_str());
  });
  server.on("/winter", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/winter.jpg", "image/jpg");
  });
  // Start server
  server.begin();
}

void loop() {
    int packetSize = LoRa.parsePacket();
    if (packetSize) {
      getLoRaData();
  }
     /*int packetSize = LoRa.parsePacket();
      if (packetSize) {
        // received a packet
        //Serial.print("Received packet '");
    
        // read packet
        while (LoRa.available()) {
          Serial.print((char)LoRa.read());
        }
    
        // print RSSI of packet
        //Serial.print("' with RSSI ");
        //Serial.println(LoRa.packetRssi());
      }
  
      // convert String to bytes array
      //uint8_t *buffer = (uint8_t *)payload.c_str();
      
      //StaticJsonDocument<512> jsonBuffer;
      //JsonArray root = jsonBuffer.to<JsonArray>();
      //lpp.decode((uint8_t *)buffer, packetSize, root);
      //serializeJsonPretty(root, Serial);
*/
}
