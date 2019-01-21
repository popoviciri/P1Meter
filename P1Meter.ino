#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include "CRC16.h"

//===Change values from here===
const char* ssid = "SSID";
const char* password = "SSIDPASSWD";
const char* hostName = "ESP1_Meter";
const bool outputOnSerial = false;
#define SERIAL_RX   5           // pin for SoftwareSerial RX
#define broker_ip   "1.2.3.4"   // IP of MQTT broker
#define broker_port 8883        // MQTT port 1883 or 8883
//===Change values to here===

// Vars to store meter readings
float mEVLT = 0.0; //Meter reading Electrics - consumption low tariff
float mEVHT = 0.0; //Meter reading Electrics - consumption high tariff
float mEOLT = 0.0; //Meter reading Electrics - return low tariff
float mEOHT = 0.0; //Meter reading Electrics - return high tariff
float mEAV = 0.0;  //Meter reading Electrics - Actual consumption
float mEAT = 0.0;  //Meter reading Electrics - Actual return
float mGAS = 0.0;    //Meter reading Gas
float prevGAS = 0.0;


#define MAXLINELENGTH 64 // longest normal line is 47 char (+3 for \r\n\0)
char telegram[MAXLINELENGTH];

SoftwareSerial mySerial(SERIAL_RX, -1, true, MAXLINELENGTH); // (RX, TX. inverted, buffer)

unsigned int currentCRC=0;

/************ MQTT WiFi WiFiClient ******************/
WiFiClientSecure MQTT_client;
Adafruit_MQTT_Client mqtt(&MQTT_client, broker_ip, broker_port);
/****************************** Feeds ***************************************/
Adafruit_MQTT_Publish meter_raw = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/raw");
Adafruit_MQTT_Publish meter_mEVLT = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEVLT");
Adafruit_MQTT_Publish meter_mEVHT = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEVHT");
Adafruit_MQTT_Publish meter_mEAV = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mEAV");
Adafruit_MQTT_Publish meter_mGAS = Adafruit_MQTT_Publish(&mqtt, "mqtt/feeds/mGAS");

//##############################################################
void setup() {
  Serial.begin(115200);
//  Serial.setDebugOutput(true);
  delay(500);
  Serial.println("Booting");

  setup_wifi();
  mySerial.begin(115200);
}
 
//##############################################################
void setup_wifi() {
  Serial.println("Connecting to wifi...");
  WiFi.hostname(hostName);
  delay(500);
  WiFi.persistent(false);
  WiFi.setOutputPower(0); 
  WiFi.mode(WIFI_STA);
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, password);
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
  sprintf(sValue, "%f;%f;%f;%f;%f;%f", mEVLT, mEVHT, mEOLT, mEOHT, mEAV, mEAT);
  meter_mEVLT.publish(mEVLT/1000-0.0001);
  meter_mEVHT.publish(mEVHT/1000-0.0001);
  meter_mEAV.publish(mEAV/1000-0.0001);
  if(outputOnSerial){
    Serial.print("Electricity meter LOW tariff: ");Serial.print(mEVLT/1000-0.0001,3);Serial.println(" [kW]");
    Serial.print("Electricity meter HIGH tariff: ");Serial.print(mEVHT/1000-0.0001,3);Serial.println(" [kW]");
    Serial.print("Electricity NOW: ");Serial.print(mEAV/1000-0.0001,3);Serial.println(" [kWh]");
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

  if (strncmp(telegram, "1-0:2.7.0", strlen("1-0:2.7.0")) == 0)
    mEAT = getValue(telegram, len);

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
}

//##############################################################
void MQTT_connect() {
  int8_t ret;
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  uint8_t retries = 3;
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
       retries--;
       if (retries == 0) {
         // basically die and wait for WDT to reset me
         while (1);
       }
  }
  Serial.println("MQTT Connected!");
} 
