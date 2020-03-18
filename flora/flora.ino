/**
   A BLE client for the Xiaomi Mi Plant Sensor, pushing measurements to an MQTT server.

   See https://github.com/nkolban/esp32-snippets/blob/master/Documentation/BLE%20C%2B%2B%20Guide.pdf
   on how bluetooth low energy and the library used are working.

   See https://github.com/ChrisScheffler/miflora/wiki/The-Basics for details on how the 
   protocol is working.
   
   MIT License

   Copyright (c) 2017 Sven Henkel
   Multiple units reading by Grega Lebar 2018

   Permission is hereby granted, free of charge, to any person obtaining a copy
   of this software and associated documentation files (the "Software"), to deal
   in the Software without restriction, including without limitation the rights
   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   copies of the Software, and to permit persons to whom the Software is
   furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in all
   copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.
*/

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEUtils.h>
#include <PubSubClient.h>

#include "config.h"

#include <AutoConnect.h>
AutoConnect portal;

// boot count used to check if battery status should be read
RTC_DATA_ATTR int bootCount = 0;

#ifndef MAX_DEVICES
#define MAX_DEVICES 64
#endif
// Root service for Flora Devices
static BLEUUID rootServiceDataUUID((uint16_t) 0xfe95);

// the remote service we wish to connect to
static BLEUUID serviceUUID("00001204-0000-1000-8000-00805f9b34fb");

// the characteristic of the remote service we are interested in
static BLEUUID uuid_version_battery("00001a02-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_sensor_data("00001a01-0000-1000-8000-00805f9b34fb");
static BLEUUID uuid_write_mode("00001a00-0000-1000-8000-00805f9b34fb");

TaskHandle_t hibernateTaskHandle = NULL;

WiFiClient espClient;
PubSubClient client(espClient);

void connectWifi()
{
  Serial.println("Starting AutoConnect Wifi portal..");
  AutoConnectConfig conf;
  conf.title = "Flora ESP32 Gateway";
  conf.title = "Flora ESP32 Gateway";
  conf.apid = "Flora GW ESP32";
  conf.psk = "kukkaruukku";
  conf.autoReconnect = true;
  
  portal.config(conf);
  portal.begin();

  while (WiFi.status() != WL_CONNECTED) {
	portal.handleClient();
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("");
}

void disconnectWifi() {
  WiFi.disconnect(true);
  Serial.println("WiFi disonnected");
}

void connectMqtt() {
  Serial.println("Connecting to MQTT...");
  client.setServer(MQTT_HOST, MQTT_PORT);

  while (!client.connected()) {
    if (!client.connect(MQTT_CLIENTID, MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.print("MQTT connection failed:");
      Serial.print(client.state());
      Serial.println("Retrying...");
      delay(MQTT_RETRY_WAIT);
    }
  }

  Serial.println("MQTT connected");
  Serial.println("");
}

void disconnectMqtt() {
  client.disconnect();
  Serial.println("MQTT disconnected");
}

uint8_t getConnDetails(char* mac, char* wifiSSID)
{
	uint8_t macAddr[6];
	WiFi.macAddress(macAddr);
	sprintf(mac, "%2X:%2X:%2X:%2X:%2X:%2X",
	macAddr[0],
	macAddr[1],
	macAddr[2],
	macAddr[3],
	macAddr[4],
	macAddr[5]);

	snprintf(wifiSSID, 32, "%s", WiFi.SSID());

	return 0; //FIXME Should return error codes in case something fails
}

BLEClient* getFloraClient(BLEAddress floraAddress) {
  BLEClient* floraClient = BLEDevice::createClient();

  if (!floraClient->connect(floraAddress)) {
    Serial.println("- Connection failed, skipping");
	Serial.println(floraAddress.toString().c_str());
    return nullptr;
  }

  Serial.println("- Connection successful");
  return floraClient;
}

BLERemoteService* getFloraService(BLEClient* floraClient) {
  BLERemoteService* floraService = nullptr;

  try {
    floraService = floraClient->getService(serviceUUID);
  }
  catch (...) {
    // something went wrong
  }
  if (floraService == nullptr) {
    Serial.println("- Failed to find data service");
  }
  else {
    Serial.println("- Found data service");
  }

  return floraService;
}

bool forceFloraServiceDataMode(BLERemoteService* floraService) {
  BLERemoteCharacteristic* floraCharacteristic;
  
  // get device mode characteristic, needs to be changed to read data
  Serial.println("- Force device in data mode");
  floraCharacteristic = nullptr;
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_write_mode);
  }
  catch (...) {
    // something went wrong
  }
  if (floraCharacteristic == nullptr) {
    Serial.println("-- Failed, skipping device");
    return false;
  }

  // write the magic data
  uint8_t buf[2] = {0xA0, 0x1F};
  floraCharacteristic->writeValue(buf, 2, true);

  delay(500);
  return true;
}

bool readFloraDataCharacteristic(BLERemoteService* floraService, String baseTopic, char* mac, char* wifiSSID) {
  BLERemoteCharacteristic* floraCharacteristic = nullptr;

  // get the main device data characteristic
  Serial.println("- Access characteristic from device");
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_sensor_data);
  }
  catch (...) {
    // something went wrong
  }
  if (floraCharacteristic == nullptr) {
    Serial.println("-- Failed, skipping device");
    return false;
  }

  // read characteristic value
  Serial.println("- Read value from characteristic");
  std::string value;
  try{
    value = floraCharacteristic->readValue();
  }
  catch (...) {
    // something went wrong
    Serial.println("-- Failed, skipping device");
    return false;
  }
  const char *val = value.c_str();

  Serial.print("Hex: ");
  for (int i = 0; i < 16; i++) {
    Serial.print((int)val[i], HEX);
    Serial.print(" ");
  }
  Serial.println(" ");

  int16_t* temp_raw = (int16_t*)val;
  float temperature = (*temp_raw) / ((float)10.0);
  Serial.print("-- Temperature: ");
  Serial.println(temperature);

  int moisture = val[7];
  Serial.print("-- Moisture: ");
  Serial.println(moisture);

  int light = val[3] + val[4] * 256;
  Serial.print("-- Light: ");
  Serial.println(light);
 
  int conductivity = val[8] + val[9] * 256;
  Serial.print("-- Conductivity: ");
  Serial.println(conductivity);

  if (temperature > 200 || temperature < -100) {
    Serial.println("-- Unreasonable values received, skip publish");
    return false;
  }

  /*
  char buffer[128];
  snprintf(buffer, 128, "{\"temperature\":%f, \"moisture\":%d, \"light\":%d, \"conductivity\":%d}", temperature, moisture, light, conductivity);
  client.publish((baseTopic + "values").c_str(), buffer); 
  */

  /*
  char buffer[64];
  snprintf(buffer, 64, "{\"value\":%f, \"unit\": \"C\"}", temperature);
  client.publish((baseTopic + "temperature").c_str(), buffer); 
  snprintf(buffer, 64, "{\"value\":%d, \"unit\": \"\%\"}", moisture); 
  client.publish((baseTopic + "moisture").c_str(), buffer);
  snprintf(buffer, 64, "{\"value\":%d, \"unit\": \"lux\"}", light);
  client.publish((baseTopic + "light").c_str(), buffer);
  snprintf(buffer, 64, "{\"value\":%d, \"unit\": \"uS/cm\"}", conductivity);
  client.publish((baseTopic + "conductivity").c_str(), buffer);
  */
  char buffer[128];
  snprintf(buffer, 128, "{\"value\":%f, \"unit\": \"C\", \"gw_mac\":\"%s\", \"wifi\":\"%s\"}", temperature, mac, wifiSSID);
  client.publish((baseTopic + "temperature").c_str(), buffer); 
  snprintf(buffer, 128, "{\"value\":%d, \"unit\": \"\%\", \"gw_mac\":\"%s\", \"wifi\":\"%s\"}", moisture, mac, wifiSSID); 
  client.publish((baseTopic + "moisture").c_str(), buffer);
  snprintf(buffer, 128, "{\"value\":%d, \"unit\": \"lux\", \"gw_mac\":\"%s\", \"wifi\":\"%s\"}", light, mac, wifiSSID);
  client.publish((baseTopic + "light").c_str(), buffer);
  snprintf(buffer, 128, "{\"value\":%d, \"unit\": \"uS/cm\", \"gw_mac\":\"%s\", \"wifi\":\"%s\"}", conductivity, mac, wifiSSID);
  client.publish((baseTopic + "conductivity").c_str(), buffer);

  return true;
}

bool readFloraBatteryCharacteristic(BLERemoteService* floraService, String baseTopic, char* mac, char* wifiSSID) {
  BLERemoteCharacteristic* floraCharacteristic = nullptr;

  // get the device battery characteristic
  Serial.println("- Access battery characteristic from device");
  try {
    floraCharacteristic = floraService->getCharacteristic(uuid_version_battery);
  }
  catch (...) {
    // something went wrong
  }
  if (floraCharacteristic == nullptr) {
    Serial.println("-- Failed, skipping battery level");
    return false;
  }

  // read characteristic value
  Serial.println("- Read value from characteristic");
  std::string value;
  try{
    value = floraCharacteristic->readValue();
  }
  catch (...) {
    // something went wrong
    Serial.println("-- Failed, skipping battery level");
    return false;
  }
  const char *val2 = value.c_str();
  int battery = val2[0];

  char buffer[128];

  Serial.print("-- Battery: ");
  Serial.println(battery);
  snprintf(buffer, 128, "{\"value\":%d, \"unit\": \"\%\", \"gw_mac\":\"%s\", \"wifi\":\"%s\"}", battery, mac, wifiSSID);
  client.publish((baseTopic + "battery").c_str(), buffer);

  return true;
}

bool processFloraService(BLERemoteService* floraService, const char* deviceMacAddress, bool readBattery) {
  // set device in data mode
  if (!forceFloraServiceDataMode(floraService)) {
    return false;
  }

  String baseTopic = MQTT_BASE_TOPIC + "/" + deviceMacAddress + "/";
  char wifiSSID[32];
  char mac[18];
  getConnDetails(mac, wifiSSID);

  bool dataSuccess = readFloraDataCharacteristic(floraService, baseTopic, mac, wifiSSID);

  bool batterySuccess = true;
  if (readBattery) {
    batterySuccess = readFloraBatteryCharacteristic(floraService, baseTopic, mac, wifiSSID);
  }

  return dataSuccess && batterySuccess;
}

bool processFloraDevice(BLEAddress floraAddress, bool getBattery, int tryCount) {
  Serial.print("Processing Flora device at ");
  Serial.print(floraAddress.toString().c_str());
  Serial.print(" (try ");
  Serial.print(tryCount);
  Serial.println(")");

  // connect to flora ble server
  BLEClient* floraClient = getFloraClient(floraAddress);
  if (floraClient == nullptr) {
    return false;
  }

  // connect data service
  BLERemoteService* floraService = getFloraService(floraClient);
  if (floraService == nullptr) {
    floraClient->disconnect();
    return false;
  }

  // process devices data
  bool success = processFloraService(floraService, floraAddress.toString().c_str(), getBattery);

  // disconnect from device
  floraClient->disconnect();

  return success;
}

/***** ADDITIONAL BLE SCANNING FUNCTIONS *****/
/*

// Callback on what to do with found devices
class AdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
	void onResult(BLEAdvertisedDevice advertisedDevice) {
		// New device found
		const char* device_addr_str = advertisedDevice.getAddress().toString().c_str();
		Serial.printf("Advertised Device: %s \n", device_addr_str);

		// check if the first three bytes match Flora pattern
		if( memcmp(FLORA_PATTERN, device_addr_str, 7) == 0 )
		{
			if( device_addr_str[9] == '6') // Fourth byte can be 6A or 6B
			{
				Serial.println("New Flora Device Found!");
				strcpy( FLORA_DEVICES_DYNAMIC[deviceCount], device_addr_str );
				deviceCount++;
			}
		}
		
	}
};

bool populateFloraList(BLEScan* scan_handle)
{
	scan_handle = BLEDevice::getScan(); // Create a scan instance
	scan_handle->setAdvertisedDeviceCallbacks(new AdvertisedDeviceCallbacks(), false);
	scan_handle->setActiveScan(true); //active scan uses more power, but get results faster
	scan_handle->setInterval(100);
	scan_handle->setWindow(99);  // less or equal setInterval value

	deviceCount = 0;
	scan_handle->start(BLE_SCAN_DURATION, false);
	Serial.printf("Scanning complete, %d Floras found\n", deviceCount);
	scan_handle->clearResults(); // Delete scan results from BLE buffer
}
*/

// Taken drom Djebouh's pull request, a  more elegant implementation of the device identification
// before setup()
static void my_gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t* param) {
	ESP_LOGW(LOG_TAG, "custom gattc event handler, event: %d", (uint8_t)event);
	if (event == ESP_GATTC_DISCONNECT_EVT) {
		Serial.print("Disconnect reason: ");
		Serial.println((int)param->disconnect.reason);
	}
}


class FloraDevicesScanner {
	public:
	// Scan BLE and return true if flora devices are found
	bool scan();

	int getDeviceCount() const {
		return _deviceCount;
	}

	std::string getDeviceAddress(int i) const {
		if (i < _deviceCount)
			return _devices[i];
		else
			return std::string();
	}

	private:
		std::string _devices[MAX_DEVICES];
		int         _deviceCount = 0;

		void registerDevice(BLEAdvertisedDevice& advertisedDevice) {
			std::string deviceAddress(advertisedDevice.getAddress().toString());
			Serial.print("Flora device found at address ");
			Serial.println(deviceAddress.c_str());

			if (_deviceCount < MAX_DEVICES)
				_devices[_deviceCount++] = deviceAddress;
			else
				Serial.println("can't register device, no remaining slot");
		}

};

bool FloraDevicesScanner::scan() {
	Serial.println("Scan BLE, looking for Flora Devices");

	// detect and register Flora devices during BLE scan
	class FloraDevicesBLEDetector: public BLEAdvertisedDeviceCallbacks {
		public:
		FloraDevicesBLEDetector(FloraDevicesScanner &floraScanner) : _floraScanner(floraScanner) { }

		void onResult(BLEAdvertisedDevice advertisedDevice)
		{
			if (advertisedDevice.haveServiceUUID()) {
				BLEUUID service = advertisedDevice.getServiceUUID();
			if (service.equals(rootServiceDataUUID))
				_floraScanner.registerDevice(advertisedDevice);
			}
		}

		private:
			FloraDevicesScanner& _floraScanner;
	};

	BLEScan* scan = BLEDevice::getScan();
	FloraDevicesBLEDetector floraDetector(*this);
	scan->setAdvertisedDeviceCallbacks(&floraDetector);
	scan->start(BLE_SCAN_DURATION);

	Serial.print("Number of Flora devices detected: ");
	Serial.println(_deviceCount);
	return (_deviceCount > 0);
}

void hibernate() {
  esp_sleep_enable_timer_wakeup(SLEEP_DURATION * 1000000ll);
  Serial.println("Going to sleep now.");
  delay(100);
  esp_deep_sleep_start();
}

void delayedHibernate(void *parameter) {
  delay(EMERGENCY_HIBERNATE*1000); // delay for five minutes
  Serial.println("Something got stuck, entering emergency hibernate...");
  hibernate();
}

void setup() {
	// all action is done when device is woken up
	Serial.begin(115200);
	delay(1000);

	// increase boot count
	bootCount++;

	// create a hibernate task in case something gets stuck
	xTaskCreate(delayedHibernate, "hibernate", 4096, NULL, 1, &hibernateTaskHandle);

	Serial.println("Initialize BLE client...");
	BLEDevice::init("");
	BLEDevice::setPower(ESP_PWR_LVL_P7);

	// connect to Wifi or start as AP
	connectWifi();


  
	
	FloraDevicesScanner floraScanner;
	if (floraScanner.scan()) {

		// connect to mqtt server
		connectMqtt();

		// check if battery status should be read - based on boot count
		bool readBattery = ((bootCount % BATTERY_INTERVAL) == 0);
		if (readBattery) Serial.println("Battery will be read during this run");

		// process devices
		for (int i = 0; i < floraScanner.getDeviceCount(); i++) {
			int tryCount = 0;
			BLEAddress floraAddress(floraScanner.getDeviceAddress(i));

			while (tryCount < RETRY) {
				tryCount++;
				if (processFloraDevice(floraAddress, readBattery, tryCount)) {
					break;
				}
				delay(1000);
			}
			delay(1500);
		}
	}
	
	// disconnect wifi and mqtt
	disconnectWifi();
	disconnectMqtt();

	// delete emergency hibernate task
	vTaskDelete(hibernateTaskHandle);

	// go to sleep now
	hibernate();
}

void loop() {
  /// we're not doing anything in the loop, only on device wakeup
  delay(10000);
}
