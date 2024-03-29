(Forked from https://github.com/sidddy/flora)


## flora

This arduino sketch implements an ESP32 BLE client for Xiaomi Mi Flora Plant sensors, pushing the measurements to an MQTT server.

## Technical requirements

Hardware:
- ESP32
- Xiaomi Mi Plant Sensor (firmware revision >= 2.6.6)

Software:
- MQTT server

Arduino Libraries (not complete)
 - https://github.com/Hieromon/PageBuilder
 - https://github.com/Hieromon/AutoConnect

## Setup instructions

1) Copy config.h.example into config.h and update seetings according to your environment:
- MQTT Server address and credentials

2) Open ino sketch in Arduino, compile & upload. 

3) If not connected to Wifi, ESP32 launches as AP with **SSID "Flora ESP32 GW"**
Connect to the Wifi and input **password "kukkaruukku"**
From here you can search for available networks and input credentials. You only need to do this once, ESP32 will remember the network(s).

## Measuring interval

The ESP32 will perform a single connection attempt to the Xiaomi Mi Plant sensor, read the sensor data & push it to the MQTT server. The ESP32 will enter deep sleep mode after all sensors have been read and sleep for X minutes before repeating the exercise...
Battery level is read every Xth wakeup.
Up to X attempst per sensor are performed when reading the data fails.

## Configuration

- SLEEP_DURATION - how long should the device sleep between sensor reads?
- EMERGENCY_HIBERNATE - how long after wakeup should the device forcefully go to sleep (e.g. when something gets stuck)?
- BATTERY_INTERVAL - how ofter should the battery status be read?
- RETRY - how ofter should a single device be tried on each run?

## Constraints

Some "nice to have" features are not yet implemented or cannot be implemented:
  - OTA updates: I didn't manage to implement OTA update capabilities due to program size constraints: BLE and WLAN brings the sketch up to 90% of the size limit, so I decided to use the remaining 10% for something more useful than OTA...

## Sketch size issues

The sketch does not fit into the default arduino parition size of around 1.3MB. You'll need to change your default parition table and increase maximum build size to at least 1.6MB.
On Arduino IDE this can be achieved using "Tools -> Partition Scheme -> No OTA"

## Credits

Many thanks go to the guys at https://github.com/open-homeautomation/miflora for figuring out the sensor protocol.


## Update 2023-04-02
Fixed formatting in snprintf and WiFi.SSID(), apparently something has changed with version. Since there's so many other things that should be done better I'll just list which versions I can compile with:

Using library ESP32_BLE_Arduino at version 1.0.1
Using library pubsubclient at version 2.8
Using library AutoConnect at version 1.1.2
Using library DNSServer at version 1.1.0
Using library WiFi at version 1.0
Using library WebServer at version 1.0
Using library EEPROM at version 1.0.3
Using library PageBuilder at version 1.3.4
Using library Preferences at version 1.0
Using library ArduinoJson at version 6.15.2
Using library Ticker at version 1.1
Using library HTTPClient at version 1.2
Using library WiFiClientSecure at version 1.0
Using library HTTPUpdate at version 1.3
Using library Update at version 1.0
Using library SPIFFS at version 1.0
Using library FS at version 1.0
Using library SPI at version 1.0
Using library SD at version 1.0.5
