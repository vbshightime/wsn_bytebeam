#ifndef WIFI_OTA_H
#define WIFI_OTA_H
#include "hardwareDefs.h"
#include "deviceState.h"
#include <esp_now.h>
#include <WiFi.h>

const IPAddress apIP(192, 168, 4, 1);
const IPAddress netMsk(255, 255, 255, 0);




/**
   @brief:
   Function to connect to WiFi to wifi if already connected returns true
   if not then check it the station connected to AP or in portal mode and return true
   or else connectes to the WiFiSTA if not connected
   @param:
   SSID     PassKey     maxDelayRetry       AP-SSID
   @return:
   true if connected to STA or AP
   false in case of faliure
*/
bool reconnectWiFi(const String& ssid, const String& pass, int maxDelay) {
  bool connectSuccess = true;
  int loopCounter = 0;
  if (WiFi.isConnected()) {
    DEBUG_PRINTLN("already in STA mode and is connected");
    RSTATE.isAPActive = false;
    return true;
  }
  WiFi.mode(WIFI_STA);  // fixem:: why AP STA mandeep had it like this.
  DEBUG_PRINTF("ssid: %s", ssid.c_str());
  DEBUG_PRINTF("pass: %s", pass.c_str());
  WiFi.begin(ssid.c_str(), pass.c_str());
  DEBUG_PRINTLN("staring wait for connection\n");
  while (WiFi.status() != WL_CONNECTED)
  {
    DEBUG_PRINT(".");
    delay(maxDelay*1000);
    if (loopCounter == 30) {
      DEBUG_PRINTLN("timeout trying to connect to wifi\n");
      connectSuccess = false;
      break;
    }
    loopCounter++;
  }
  connectSuccess = WiFi.isConnected();
  if (connectSuccess) {
    DEBUG_PRINTLN("connected to:  ");
    DEBUG_PRINTLN(WiFi.localIP());
    RSTATE.isAPActive = false;
  }
  return connectSuccess;
}


String getLast3ByteMac(uint8_t* mac, bool fullSizeMac){
  char macStr[9] = { 0 };
  WiFi.macAddress(mac);
  if(!fullSizeMac){
    sprintf(macStr, "%02X%02X%02X",  mac[3], mac[4], mac[5]);
  }else{
    sprintf(macStr, "%02X%02X%02X%02X%02X%02X", mac[0], mac[1], mac[2],mac[3], mac[4], mac[5]);
  }
  return String(macStr);
}

bool configESPNowDeviceAP() {
  //FIXME:: these two can also fail, check for errors
  WiFi.mode(WIFI_AP_STA);
  DEBUG_PRINTLN("Confuring Device AP");
  String Prefix = AP_MODE_SSID;
  uint8_t mac[6];
  String macStr = getLast3ByteMac(mac,false);
  String ssid = Prefix + macStr;
  DEBUG_PRINTLN("Configuring to:"+ssid);
  bool result = WiFi.softAP(ssid.c_str(), "", ESPNOW_CHANNEL, 0);
  if (!result) {
    DEBUG_PRINTLN("AP Config failed.");
    return false;
  }
  return true;
}


bool switchToWifiClient(const String& ssid, const String& pass, uint8_t maxWaitSecs)
{
  if (WiFi.isConnected()) {
    return true;
  }
  return reconnectWiFi(ssid, pass, maxWaitSecs);
}

bool switchToESPNowGateway()
{
  // todo :: we need to swtich to espnow only if not already setup otherwise just return
  configESPNowDeviceAP();
  WiFi.disconnect();
  int ESPNow_err = esp_now_init();
  if (ESPNow_err != ESP_OK) {
    DEBUG_PRINTLN("ESPNow Init failed");
    return false;
  } else {
    DEBUG_PRINTLN(" ESPNow Init Success");
  }
  return true;
}


bool hasWifiConnection()
{

  return (WiFi.isConnected());
}


#endif
