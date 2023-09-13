#include <time.h>
#include "deviceState.h"
#include "hardwareDefs.h"
#include "WiFiOTA.h"
#include <esp_now.h>
#include <BytebeamArduino.h>
#include "SensorPayload.h"

DeviceState state;
DeviceState &deviceState = state;
unsigned long startMillis = 0;

// sntp credentials
const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 3600;
const char *ntpServer = "pool.ntp.org";

// temperature stream name
char tempHumid[] = "device_shadow";


bool processQueuedData()
{

  bool connOutcome = false;
  if (!deviceState.hasUnprocessedTelemetry())
  {
    DEBUG_PRINTLN("NO data to process Try Again");
    return false;
  }
  int maxWifiConnectionRetries = MAX_WIFI_CONN_RETRIES;
  
  if (esp_now_unregister_recv_cb() != ESP_OK)
  {
    DEBUG_PRINTLN("cant unregister ESP nOW cb");
  }
  
  int esp_now_err = esp_now_deinit();
  if (esp_now_err != ESP_OK) {
    DEBUG_PRINTLN("ESPNow deinit failed");

  }
  
  while (maxWifiConnectionRetries > 0)
  {
    connOutcome = switchToWifiClient(PSTATE.apSSID, PSTATE.apPass, 1); 
    if (connOutcome)
    {
      break;
    }
    maxWifiConnectionRetries--;
  }

  syncTimeFromNtp();

  // begin the bytebeam client
  Bytebeam.begin();
  
  bool rc = publishToBytebeam(deviceState);
  if (!rc)
  {
    DEBUG_PRINTLN("send data failed");
  }

  storeAndReset();

  return true;
}

void storeAndReset()
{
  // store persistent state
  bool storeValue = deviceState.store();
  if (!storeValue)
  {
    DEBUG_PRINTLN("Error storing values to EEPROM");
  }
  else
  {
    DEBUG_PRINTLN("Values Stored");
  }
  SPIFFS.end();
  esp_restart();
}

// callback when data is recv from Master
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if (data_len < sizeof(SensorPayloadTH))
  {
    DEBUG_PRINTLN("Bad data from espnow");
    return;
  }
  SensorPayloadTH *payload = (SensorPayloadTH*)data;
  DEBUG_PRINTF("Temperatue%.1f", payload->temperature);
  DEBUG_PRINTF("SensorProfile%d", payload->sensorProfile);

  deviceState.enqueSensorPayload(String(macStr).c_str(), payload);

}

// function to sync time from ntp server with predefined credentials
void syncTimeFromNtp()
{
  // sync the time from ntp server
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  struct tm timeinfo;

  // get the current time
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return;
  }

  // log the time info to serial :)
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  Serial.println();
}

// function to get the time
unsigned long long getEpochMillis()
{
  time_t now;
  struct tm timeinfo;

  // get the current time i.e make sure the device is in sync with the ntp server
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("failed to obtain time");
    return 0;
  }

  // get the epoch time
  time(&now);

  // generate the epoch millis
  unsigned long long timeMillis = ((unsigned long long)now * 1000) + (millis() % 1000);

  return timeMillis;
}

String createPayload(PayloadQueueElement *telemeteryElement)
{

  String temperatureHumidStr = "";
  StaticJsonDocument<1024> doc;
  // get the current epoch millis
  unsigned long long milliseconds = 0;
  milliseconds = getEpochMillis();
  // make sure you got the millis

  JsonArray tempHumidJsonArray = doc.to<JsonArray>();
  JsonObject tempHumidJsonObj_1 = tempHumidJsonArray.createNestedObject();

  
  SensorPayloadTH *payload = (SensorPayloadTH *)telemeteryElement->_payload;
  String macStr(telemeteryElement->_mac);
  
  tempHumidJsonObj_1["node_id"]   = macStr;
  tempHumidJsonObj_1["timestamp"]   = milliseconds;
  tempHumidJsonObj_1["temperature"]    = payload->temperature;
  tempHumidJsonObj_1["humid"] = payload->humidity;

  serializeJson(tempHumidJsonArray, temperatureHumidStr);

  return temperatureHumidStr;  
}

bool publishToBytebeam(DeviceState &devState){
   bool isSent = false;
   SensorPayload *payload = nullptr;
   while (devState.hasUnprocessedTelemetry()){
          Bytebeam.loop();
          DEBUG_PRINTLN("hasUnprocessedTelemetry");
          PayloadQueueElement *telemeteryElement = devState.telemetryQueue.pop();
          payload = telemeteryElement->_payload;
          String preparedPayload;
          preparedPayload = createPayload(telemeteryElement);
          isSent = Bytebeam.publishToStream(tempHumid, preparedPayload.c_str());
          if (!isSent)
        {
          DEBUG_PRINTLN("Could not send to bytebeam");
          return isSent;
        }
         delete telemeteryElement;
      }

  return isSent;
}



void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  DEBUG_PRINTLN("This is THingHz Range of wireless sensors");
  if (!EEPROM.begin(EEPROM_STORE_SIZE))
  {
    DEBUG_PRINTLN("Problem loading EEPROM");
  }

  bool rc = deviceState.load();
  if (!rc)
  {
    DEBUG_PRINTLN("EEPROM Values not loaded");
  }
  else
  {
    DEBUG_PRINTLN("Values Loaded");
  }

  DEBUG_PRINTLN("Switching to ESPNOW");
  switchToESPNowGateway();
  esp_now_register_recv_cb(OnDataRecv);
  
}

void loop()
{

  if ((millis() - startMillis) >= (PROCESS_DATA_INTERVAL_SECS * MILLI_SECS_MULTIPLIER))
  {
    processQueuedData();
    startMillis = millis();
  }
}

