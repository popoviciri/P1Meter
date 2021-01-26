#include "SoftwareSerial.h"
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <ArduinoOTA.h>
#include "CRC16.h"

/************************* WiFi Access Point *********************************/
const char* ssid = "SSID";
const char* password = "SSIDPASSWD";
const char* hostName = "esp_meter";
/************************* WiFi Access Point *********************************/
#define broker_ip   "10.0.0.4"   // IP of MQTT broker
#define broker_port 1883         // MQTT port 1883 or 8883
#define broker_user "user"       // MQTT username
#define broker_pass "pass"       // MQTT passwd
/************************* Pinout *********************************/
#define SERIAL_RX       15
const bool outputOnSerial = false;


// Vars to store meter readings
float mEVLT = 0.0; //Meter reading Electrics - consumption low tariff
float mEVHT = 0.0; //Meter reading Electrics - consumption high tariff
float mEOLT = 0.0; //Meter reading Electrics - return low tariff
float mEOHT = 0.0; //Meter reading Electrics - return high tariff
float mEAV = 0.0;  //Meter reading Electrics - Actual consumption
float mEAT = 0.0;  //Meter reading Electrics - Actual return
float mGAS = 0.0;    //Meter reading Gas
float prevGAS = 0.0;
//split on three phases
float mEAVL1 = 0.0;  //Meter reading Electrics - Actual consumption L1
float mEAVL2 = 0.0;  //Meter reading Electrics - Actual consumption L2
float mEAVL3 = 0.0;  //Meter reading Electrics - Actual consumption L3
float mEATL1 = 0.0;  //Meter reading Electrics - Actual return L1
float mEATL2 = 0.0;  //Meter reading Electrics - Actual return L2
float mEATL3 = 0.0;  //Meter reading Electrics - Actual return L3

#define MAXLINELENGTH 64 // longest normal line is 47 char (+3 for \r\n\0)
char telegram[MAXLINELENGTH];
unsigned int currentCRC=0;

SoftwareSerial mySerial(SERIAL_RX, -1, true, MAXLINELENGTH); // (RX, TX. inverted, buffer)
//updated with new softwareserial lib; see https://github.com/esp8266/Arduino/issues/6737
//SoftwareSerial mySerial;
//constexpr SoftwareSerialConfig swSerialConfig = SWSERIAL_8N1;
//constexpr int IUTBITRATE = 115200;

/************ MQTT WiFi WiFiClient ******************/
WiFiClient MQTT_client;
Adafruit_MQTT_Client mqtt(&MQTT_client, broker_ip, broker_port, broker_user, broker_pass);
/****************************** Feeds ***************************************/
Adafruit_MQTT_Publish meter_raw = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/raw");
//consumption
Adafruit_MQTT_Publish meter_mEVLT = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEVLT");
Adafruit_MQTT_Publish meter_mEVHT = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEVHT");
Adafruit_MQTT_Publish meter_mEAV = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEAV");
Adafruit_MQTT_Publish meter_mEAVL1 = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEAVL1");
Adafruit_MQTT_Publish meter_mEAVL2 = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEAVL2");
Adafruit_MQTT_Publish meter_mEAVL3 = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEAVL3");
//return
Adafruit_MQTT_Publish meter_mEOLT = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEOLT");
Adafruit_MQTT_Publish meter_mEOHT = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEOHT");
Adafruit_MQTT_Publish meter_mEAT = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEAT");
Adafruit_MQTT_Publish meter_mEATL1 = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEATL1");
Adafruit_MQTT_Publish meter_mEATL2 = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEATL2");
Adafruit_MQTT_Publish meter_mEATL3 = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEATL3");
//gas
Adafruit_MQTT_Publish meter_mGAS = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mGAS");
//##############################################################
void setup() {
  Serial.begin(115200);
  mySerial.begin(115200);
//  mySerial.begin (IUTBITRATE, swSerialConfig, SERIAL_RX, -1, true, MAXLINELENGTH);

//  Serial.setDebugOutput(true);
  delay(1000);
  Serial.println("Booting");

  setup_wifi();
  setup_ArduinoOTA();
}
 
//##############################################################
void setup_wifi() {
  Serial.println("Connecting to wifi...");
  WiFi.disconnect() ;
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);            // Client mode
  //WiFi.setSleepMode(WIFI_NONE_SLEEP);
  //WiFi.setSleepMode(WIFI_LIGHT_SLEEP);
  WiFi.setSleepMode(WIFI_MODEM_SLEEP);
  WiFi.setOutputPower(18);        // 10dBm == 10mW, 14dBm = 25mW, 17dBm = 50mW, 20dBm = 100mW
  WiFi.begin(ssid, password);     // Start WiFi.

  delay(1000);

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
    delay(2000);
  }
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("Hostname: ");
  Serial.println(WiFi.hostname());
}
//##############################################################
void setup_ArduinoOTA() {
  ArduinoOTA.setPort(8266);
  ArduinoOTA.setHostname(hostName);
  //ArduinoOTA.setPassword((const char *)"qwerty");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}
//##############################################################


void UpdateGas() {
  char sValue[10];
  sprintf(sValue, "%f", mGAS);
  meter_mGAS.publish(mGAS/1000-0.0001);
  if(outputOnSerial){
    Serial.print("Gas meter: ");Serial.print(mGAS/1000-0.0001,3);Serial.println(" [m3]");
  }
}

void UpdateElectricity() {
  char sValue[255];
  meter_mEVLT.publish(mEVLT/1000-0.0001);
  meter_mEVHT.publish(mEVHT/1000-0.0001);
  meter_mEAV.publish(mEAV);
  meter_mEAVL1.publish(mEAVL1);
  meter_mEAVL2.publish(mEAVL2);
  meter_mEAVL3.publish(mEAVL3);

  meter_mEOLT.publish(mEOLT/1000-0.0001);
  meter_mEOHT.publish(mEOHT/1000-0.0001);
  meter_mEAT.publish(mEAT);
  meter_mEATL1.publish(mEATL1);
  meter_mEATL2.publish(mEATL2);
  meter_mEATL3.publish(mEATL3);

  if(outputOnSerial){
    Serial.print("Electricity consumption LOW tariff: ");Serial.print(mEVLT/1000-0.0001,3);Serial.println(" [kWh]");
    Serial.print("Electricity consumption HIGH tariff: ");Serial.print(mEVHT/1000-0.0001,3);Serial.println(" [kWh]");
    Serial.print("Electricity consumption NOW: ");Serial.print(mEAV/1000-0.0001,3);Serial.println(" [W]");
    Serial.print("Electricity consumption L1 NOW: ");Serial.print(mEAVL1/1000-0.0001,3);Serial.println(" [W]");
    Serial.print("Electricity consumption L2 NOW: ");Serial.print(mEAVL2/1000-0.0001,3);Serial.println(" [W]");
    Serial.print("Electricity consumption L3 NOW: ");Serial.print(mEAVL3/1000-0.0001,3);Serial.println(" [W]");
  
    Serial.print("Electricity return LOW tariff: ");Serial.print(mEOLT/1000-0.0001,3);Serial.println(" [kWh]");
    Serial.print("Electricity return HIGH tariff: ");Serial.print(mEOHT/1000-0.0001,3);Serial.println(" [kWh]");
    Serial.print("Electricity return NOW: ");Serial.print(mEAT/1000-0.0001,3);Serial.println(" [W]");
    Serial.print("Electricity return L1 NOW: ");Serial.print(mEATL1/1000-0.0001,3);Serial.println(" [W]");
    Serial.print("Electricity return L2 NOW: ");Serial.print(mEATL2/1000-0.0001,3);Serial.println(" [W]");
    Serial.print("Electricity return L3 NOW: ");Serial.print(mEATL3/1000-0.0001,3);Serial.println(" [W]");
  }
}


bool isNumber(char* res, int len) {
  for (int i = 0; i < len; i++) {
    if (((res[i] < '0') || (res[i] > '9'))  && (res[i] != '.' && res[i] != 0)) {
      return false;
    }
  }
  return true;
}

int FindCharInArrayRev(char array[], char c, int len) {
  for (int i = len - 1; i >= 0; i--) {
    if (array[i] == c) {
      return i;
    }
  }
  return -1;
}
/*
long getValidVal(long valNew, long valOld, long maxDiffer)
{
  //check if the incoming value is valid
      if(valOld > 0 && ((valNew - valOld > maxDiffer) && (valOld - valNew > maxDiffer)))
        return valOld;
      return valNew;
}
*/
long getValue(char* buffer, int maxlen) {
  int s = FindCharInArrayRev(buffer, '(', maxlen - 2);
  if (s < 8) return 0;
  if (s > 32) s = 32;
  int l = FindCharInArrayRev(buffer, '*', maxlen - 2) - s - 1;
  if (l < 4) return 0;
  if (l > 12) return 0;
  char res[16];
  memset(res, 0, sizeof(res));

  if (strncpy(res, buffer + s + 1, l)) {
    if (isNumber(res, l)) {
      return (1000 * (atof(res)+0.0001));
    }
  }
  return 0;
}

bool decodeTelegram(int len) {
  //need to check for start
  int startChar = FindCharInArrayRev(telegram, '/', len);
  int endChar = FindCharInArrayRev(telegram, '!', len);
  bool validCRCFound = false;
  if(startChar>=0)
  {
    //start found. Reset CRC calculation
    currentCRC=CRC16(0x0000,(unsigned char *) telegram+startChar, len-startChar);
    if(outputOnSerial)
    {
      for(int cnt=startChar; cnt<len-startChar;cnt++)
        Serial.print(telegram[cnt]);
    }    
    //Serial.println("Start found!");
    
  }
  else if(endChar>=0)
  {
    //add to crc calc 
    currentCRC=CRC16(currentCRC,(unsigned char*)telegram+endChar, 1);
    char messageCRC[4];
    strncpy(messageCRC, telegram + endChar + 1, 4);
    if(outputOnSerial)
    {
      for(int cnt=0; cnt<len;cnt++)
        Serial.print(telegram[cnt]);
    }    
    validCRCFound = (strtol(messageCRC, NULL, 16) == currentCRC);
    if(validCRCFound)
      Serial.println("\nVALID CRC FOUND!"); 
    else
      Serial.println("\n===INVALID CRC FOUND!===");
    currentCRC = 0;
  }
  else
  {
    currentCRC=CRC16(currentCRC, (unsigned char*)telegram, len);
    if(outputOnSerial)
    {
      for(int cnt=0; cnt<len;cnt++)
        Serial.print(telegram[cnt]);
    }
  }
/*
Meter Reading electricity delivered to client (low tariff) in 0,001 kWh1-0:1.8.1.255Use case  3:  Provide  actual meter reads through P1
Meter Reading electricity delivered to client  (normaltariff) in 0,001 kWh1-0:1.8.2.255Use  case  3:  Provide  actual meter reads through P1
Meter Reading electricity delivered by client  (lowtariff) in 0,001kWh1-0:2.8.1.255Use case 3: Provide actual meter reads through P1
Meter Reading electricity delivered by client  (normaltariff) in 0,001 kWh1-0:2.8.2.255Use case 3: Provide actual meter reads through P1
Actual electricity power delivered (+P) in 1 Watt resolution1-0:1.7.0.255Use case 3: Provide actual meter reads through P1
Actual electricity power received (-P) in 1 Watt resolution1-0:2.7.0.255Use case 3: Provide actual meter reads through P1
Instantaneous current L1 1-0:31.7.0.255 Use case 3:Provide actual meter reads through P1
Instantaneous current L2 1-0:51.7.0.255 Use case 3: Provide actual meter reads through P1
Instantaneous current L3 1-0:71.7.0.255 Use case 3: Provide actual meter reads through P1
Instantaneous active power L1 (+P) 1-0:21.7.0.255 Use case 3: Provide actual meter reads through P1
Instantaneous active power L2 (+P) 1-0:41.7.0.255 Use case 3: Provide actual meter reads through P1
Instantaneous active power L3 (+P) 1-0:61.7.0.255 Use case 3: Provide actual meter reads through P1
Instantaneous active power L1 (-P) 1-0:22.7.0.255 Use case 3: Provide actual meter reads through P1
Instantaneous active power L2 (-P) 1-0:42.7.0.255 Use case 3: Provide actual meter reads through P1
Instantaneous active power L3 (-P) 1-0:62.7.0.255 Use case 3: Provide actual meter reads through P1
Last hourly value(temperature con-verted), gas delivered to client in m3, including decimal valuesand capture time0-n:24.2.1.255Use case 3: Provide actual meter reads through P1
*/
  // 1-0:1.8.1(000992.992*kWh)
  // 1-0:1.8.1 = Elektra verbruik laag tarief (DSMR v4.0)
  if (strncmp(telegram, "1-0:1.8.1", strlen("1-0:1.8.1")) == 0) 
    mEVLT =  getValue(telegram, len);

  // 1-0:1.8.2(000560.157*kWh)
  // 1-0:1.8.2 = Elektra verbruik hoog tarief (DSMR v4.0)
  if (strncmp(telegram, "1-0:1.8.2", strlen("1-0:1.8.2")) == 0) 
    mEVHT = getValue(telegram, len);

  // 1-0:2.8.1(000348.890*kWh)
  // 1-0:2.8.1 = Elektra opbrengst laag tarief (DSMR v4.0)
  if (strncmp(telegram, "1-0:2.8.1", strlen("1-0:2.8.1")) == 0) 
    mEOLT = getValue(telegram, len);

  // 1-0:2.8.2(000859.885*kWh)
  // 1-0:2.8.2 = Elektra opbrengst hoog tarief (DSMR v4.0)
  if (strncmp(telegram, "1-0:2.8.2", strlen("1-0:2.8.2")) == 0) 
    mEOHT = getValue(telegram, len);

  // 1-0:1.7.0(00.424*kW) Actueel verbruik
  // 1-0:2.7.0(00.000*kW) Actuele teruglevering
  // 1-0:1.7.x = Electricity consumption actual usage (DSMR v4.0)
  if (strncmp(telegram, "1-0:1.7.0", strlen("1-0:1.7.0")) == 0) 
    mEAV = getValue(telegram, len);
  // 1-0:21.7.0.255 Instantaneous active power L1 (+P)
  if (strncmp(telegram, "1-0:21.7.0", strlen("1-0:21.7.0")) == 0) 
    mEAVL1 = getValue(telegram, len);
  // 1-0:41.7.0.255 Instantaneous active power L2 (+P)
  if (strncmp(telegram, "1-0:41.7.0", strlen("1-0:41.7.0")) == 0) 
    mEAVL2 = getValue(telegram, len);
  // 1-0:61.7.0.255 Instantaneous active power L3 (+P)
  if (strncmp(telegram, "1-0:61.7.0", strlen("1-0:61.7.0")) == 0) 
    mEAVL3 = getValue(telegram, len);

  // 1-0:2.7.x = Electricity return actual (DSMR v4.0)
  if (strncmp(telegram, "1-0:2.7.0", strlen("1-0:2.7.0")) == 0)
    mEAT = getValue(telegram, len);
  // 1-0:22.7.0.255 Instantaneous active power L1 (-P)
  if (strncmp(telegram, "1-0:22.7.0", strlen("1-0:22.7.0")) == 0) 
    mEATL1 = getValue(telegram, len);
  // 1-0:42.7.0.255 Instantaneous active power L2 (-P)
  if (strncmp(telegram, "1-0:42.7.0", strlen("1-0:42.7.0")) == 0) 
    mEATL2 = getValue(telegram, len);
  // 1-0:62.7.0.255 Instantaneous active power L3 (-P)
  if (strncmp(telegram, "1-0:62.7.0", strlen("1-0:62.7.0")) == 0) 
    mEATL3 = getValue(telegram, len);
  // 0-1:24.2.1(150531200000S)(00811.923*m3)
  // 0-1:24.2.1 = Gas (DSMR v4.0) on Kaifa MA105 meter
  if (strncmp(telegram, "0-1:24.2.1", strlen("0-1:24.2.1")) == 0) 
    mGAS = getValue(telegram, len);

  return validCRCFound;
}

void readTelegram() {
  if (mySerial.available()) {
    memset(telegram, 0, sizeof(telegram));
    while (mySerial.available()) {
      int len = mySerial.readBytesUntil('\n', telegram, MAXLINELENGTH);
      telegram[len] = '\n';
      telegram[len+1] = 0;
      yield();
      if(decodeTelegram(len+1))
      {
         UpdateElectricity();
         UpdateGas();
      }
    } 
  }
}

//##############################################################
void loop() {
  MQTT_connect();
  readTelegram();
//  if (millis() > 600000) {
//    ESP.restart();
//  }
  ArduinoOTA.handle();
}

//##############################################################
void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 5;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(10000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
} 
