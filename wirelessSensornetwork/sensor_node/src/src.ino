#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // only for esp_wifi_set_channel()
#include "hardwaredefs.h"
#include "runTime.h"
#include "sensor_payload.h"
#include "sensorRead.h"


// Global copy of gateway
esp_now_peer_info_t gateway;

#define PRINTSCANRESULTS 0
#define DELETEBEFOREPAIR 0

SensorPayloadTH   SensorTempHumid;


// Init ESP Now with fallback
void InitESPNow() {
  WiFi.disconnect();
  if (esp_now_init() == ESP_OK) {
    Serial.println("ESPNow Init Success");
  }
  else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
}

// Scan for gateways in AP mode
void ScanForgateway() {
  int16_t scanResults = WiFi.scanNetworks(false, false, false, 300, ESPNOW_CHANNEL); // Scan only on one channel
  // reset on each scan
  bool gatewayFound = 0;
  memset(&gateway, 0, sizeof(gateway));

  Serial.println("");
  if (scanResults == 0) {
    Serial.println("No WiFi devices in AP Mode found");
  } else {
    Serial.print("Found "); Serial.print(scanResults); Serial.println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        Serial.print(i + 1);
        Serial.print(": ");
        Serial.print(SSID);
        Serial.print(" (");
        Serial.print(RSSI);
        Serial.print(")");
        Serial.println("");
      }
      delay(10);
      // Check if the current device starts with `gateway`
      if (SSID.indexOf("Gateway-") == 0) {
        // SSID of interest
        Serial.println("Found a gateway.");
        Serial.print(i + 1); Serial.print(": "); Serial.print(SSID); Serial.print(" ["); Serial.print(BSSIDstr); Serial.print("]"); Serial.print(" ("); Serial.print(RSSI); Serial.print(")"); Serial.println("");
        // Get BSSID => Mac Address of the gateway
        int mac[6];
        if ( 6 == sscanf(BSSIDstr.c_str(), "%x:%x:%x:%x:%x:%x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            gateway.peer_addr[ii] = (uint8_t) mac[ii];
          }
        }

        gateway.channel = ESPNOW_CHANNEL; // pick a channel
        gateway.encrypt = 0; // no encryption

        gatewayFound = 1;
        // we are planning to have only one gateway in this example;
        // Hence, break after we find one, to be a bit efficient
        break;
      }
    }
  }

  if (gatewayFound) {
    Serial.println("gateway Found, processing..");
  } else {
    Serial.println("gateway Not Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the gateway is already paired with the master.
// If not, pair the gateway with master
bool managegateway() {
  if (gateway.channel == ESPNOW_CHANNEL) {
    if (DELETEBEFOREPAIR) {
      deletePeer();
    }

    Serial.print("gateway Status: ");
    // check if the peer exists
    bool exists = esp_now_is_peer_exist(gateway.peer_addr);
    if ( exists) {
      // gateway already paired.
      Serial.println("Already Paired");
      return true;
    } else {
      // gateway not paired, attempt pair
      esp_err_t addStatus = esp_now_add_peer(&gateway);
      if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
        return true;
      } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
        // How did we get so far!!
        Serial.println("ESPNOW Not Init");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
        Serial.println("Invalid Argument");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
        Serial.println("Peer list full");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
        Serial.println("Out of memory");
        return false;
      } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
        Serial.println("Peer Exists");
        return true;
      } else {
        Serial.println("Not sure what happened");
        return false;
      }
    }
  } else {
    // No gateway found to process
    Serial.println("No gateway found to process");
    return false;
  }
}

void deletePeer() {
  esp_err_t delStatus = esp_now_del_peer(gateway.peer_addr);
  Serial.print("gateway Delete Status: ");
  if (delStatus == ESP_OK) {
    // Delete success
    Serial.println("Success");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW Not Init");
  } else if (delStatus == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (delStatus == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}


void sendData() {
  readSensorTH();
  DEBUG_PRINTF("Temp= %.1f, battery= %d, humidity=%.1f, HW_REV=%d, sensor_profile=%d\n", SensorTempHumid.temperature, SensorTempHumid.batteryPercentage, SensorTempHumid.humidity,SensorTempHumid.hwRev,SensorTempHumid.sensorProfile);
  const uint8_t *peer_addr = gateway.peer_addr;
  esp_err_t result = esp_now_send(peer_addr, (uint8_t*)&SensorTempHumid, sizeof(SensorTempHumid));
  Serial.print("Send Status: ");
  if (result == ESP_OK) {
    Serial.println("Success");
  } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
    // How did we get so far!!
    Serial.println("ESPNOW not Init.");
  } else if (result == ESP_ERR_ESPNOW_ARG) {
    Serial.println("Invalid Argument");
  } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
    Serial.println("Internal Error");
  } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
    Serial.println("ESP_ERR_ESPNOW_NO_MEM");
  } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
    Serial.println("Peer not found.");
  } else {
    Serial.println("Not sure what happened");
  }
}


void readSensorTH() {
  shtInit();
  readSHT();
  SensorTempHumid.temperature = RSTATE.temperature;
  SensorTempHumid.humidity = RSTATE.humidity;
  //SensorTempHumid.batteryPercentage = getBatteryPercentage(readBatValue());
  SensorTempHumid.batteryPercentage = 100;
}

void readSensorAccel(){
  
}


void storeAndSleep()
{
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  DEBUG_PRINTLN("going to sleep");
  esp_sleep_enable_timer_wakeup(DEEPSLEEP_SECS * MICRO_SECS_MULITPLIER);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_deep_sleep_start();
}


// callback when data is sent from Master to gateway
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: "); Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  Serial.begin(115200);
  //Set device in STA mode to begin with
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.println("ESPNow/Basic/Master Example");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  Serial.print("STA CHANNEL "); Serial.println(WiFi.channel());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
}

void loop() {
  // In the loop we scan for gateway
  ScanForgateway();
  // If gateway is found, it would be populate in `gateway` variable
  // We will check if `gateway` is defined and then we proceed further
  if (gateway.channel == ESPNOW_CHANNEL) { // check if gateway channel is defined
    // `gateway` is defined
    // Add gateway as peer if it has not been added already
    bool isPaired = managegateway();
    if (isPaired) {
      // pair success or already paired
      // Send data to device
      sendData();
    } else {
      // gateway pair failed
      Serial.println("gateway pair failed!");
    }
  }
  else {
    // No gateway found to process
  }

storeAndSleep();
//delay(5000);
}
