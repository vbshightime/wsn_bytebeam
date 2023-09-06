#ifndef HARDWAREDEFS_H
#define HARDWAREDEFS_H


// hardware identifier and rev
#define DEVICE_TYPE                     DT_Node

#define DEBUG_SERIAL                    1
// hardware rev is tied to device type, they both form a combo that decies the firmware behaviour
#define HW_REV                          1
// firmware rev
#define FW_REV                          2


// sensor configuration
#define SENSOR_PROFILE                  SensorTH


#define BATTERY_VOL_PIN     36


#define BATTERY_INITIAL_READING     0
#define INVALID_TEMP_READING        99
#define INVALID_HUMIDITY_READING    -1

// configuration
#define DEEPSLEEP_SECS                          30 //900
#define MICRO_SECS_MULITPLIER                   1000000
#define EEPROM_STORE_SIZE                       512
#define SERIAL_BAUD_RATE                        115200
#define ESPNOW_MAX_PACKET_SIZE                  249
#define ESPNOW_CHANNEL                          1
#define MAC_LEN                                 6
#define MAX_DATA_RETRY_COUNT                    2


//battery
#define BATT_VOL_0                 3.0
#define BATT_VOL_100               4.2

#ifdef DEBUG_SERIAL
#define DEBUG_PRINTF(...)           Serial.printf(__VA_ARGS__)
#define DEBUG_PRINTLN(...)          Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#define DEBUG_PRINTLN(...)
#endif


#endif // HARDWAREDEFS_H
