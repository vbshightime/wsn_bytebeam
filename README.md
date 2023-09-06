# Bytebeam ESPNow using WiFi


### Libraries used:

#### library for Sht3x
1. [Adafruit_SHT31.h](https://github.com/adafruit/Adafruit_SHT31) 1.1.6 by adafruit 

#### library used for Json
1. [Arduino Json](https://github.com/bblanchon/ArduinoJson) 6.0.0 Benoit Blenchon

#### library used for Asynchroneous web server
1. [ESPAsyncWebserver](https://github.com/me-no-dev/ESPAsyncWebServer) by Me No Dev
2. dependency for ESP8266 [ESPAsyncTCP](https://github.com/me-no-dev/ESPAsyncTCP) by Me No Dev 
3. dependency for ESP32 [AsyncTCP](https://github.com/me-no-dev/AsyncTCP) by Me No Dev 

#### Bytebeam Arduino SDK
1. [bytebeam-arduino-sdk](https://github.com/bytebeamio/bytebeam-arduino-sdk) 1.0.2 by [Bytebeam](https://bytebeam.io/)  

#### library used for ESPNow
1. esp_now for ESP32

### Configuring Sensor Node
* Navigate to wirelessSensornetwork/sensor_node
* Add ESPNowW zip to Arduino library folder
* Go to src folder to find the code for sensor node
* Now grab couple of ESP32 and SHT sensors. Wire ESP32 and SHT sensor using I2C interface. GPIO 21 is SDA and GPIO 22 is SCL in case of ESP32
* Flash your ESP32 setup with sensor node code to configure it as sensor node.  

### Configuring Gateway
* Navigate to wirelessSensornetwork/gateway
* Go to src folder to find the code for gateway
* Grab ESP32 
* Then provision your device with bytebeam device certificate. Consider the steps mentioned [here](link here)
* Once you are done with provisioning, Flash ESP32 with gateway code to configure it as gateway.




