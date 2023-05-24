#include "BLEDevice.h"
#include<Arduino.h>

// The remote service we wish to connect to.
static BLEUUID serviceUUID("0000ffff-0000-1000-8000-00805f9b34fb");
// The characteristic of the remote service we are interested in.
//static BLEUUID    charUUID("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID    charUUID("0000ff01-0000-1000-8000-00805f9b34fb");
static BLEUUID    charRxUUID("0000ff02-0000-1000-8000-00805f9b34fb");

static bool doConnect = false;
static bool connected = false;
static bool doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteCharacteristicRx;
static BLEAdvertisedDevice* myDevice;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
/*    Serial.print("Notify callback for characteristic ");
    Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    Serial.print(" of data length ");
    Serial.println(length);
    Serial.print("data: ");
    //Serial.println((char*)pData);
    for(int i=0; i < length; i++){
      Serial.print(pData[i], HEX);
      Serial.print(" ");
    }
    Serial.print("-----");
    */
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our service");

  pRemoteCharacteristicRx = pRemoteService->getCharacteristic(charRxUUID);
    if (pRemoteCharacteristicRx == nullptr) {
      Serial.print("Failed to find our RX characteristic UUID: ");
      Serial.println(charRxUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic RX!");

  if(pRemoteCharacteristicRx->canNotify()){
      pRemoteCharacteristicRx->registerForNotify(notifyCallback);
      Serial.println("RX CAN NOTIFY!");
    }

    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);


    
    if (pRemoteCharacteristic == nullptr) {
      Serial.print("Failed to find our TX characteristic UUID: ");
      Serial.println(charUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our characteristic TX");

    

    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

    if(pRemoteCharacteristic->canNotify()){
      pRemoteCharacteristic->registerForNotify(notifyCallback);
      Serial.println("TX CAN NOTIFY!");
    }

    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


static void setup2() {
  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
} // End of setup.


// This is the Arduino main loop function.
void loop2() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    String newValue = "Time since boot: " + String(millis()/1000);
    //Serial.println("Setting new characteristic value to \"" + newValue + "\"");
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    //pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
    //A5 5A 00 10 05 FF 00 00 00 00 D5 2E 
    uint8_t array0[12] = {0xA5, 0x5A, 0x00 , 0x10 , 0x05 , 0xFF , 0x00 , 0x00 , 0x00 , 0x00 , 0xD5 , 0x2E};
    pRemoteCharacteristic->writeValue(array0, 12);
    uint8_t array[12] = {0xA5, 0x5A, 0x00 , 0x11 , 0x05 , 0xFF , 0x2D , 0x00 , 0x00 , 0x00 , 0x7C , 0x98 };    
    pRemoteCharacteristic->writeValue(array, 12);
    Serial.print(" WRITE VALUE");

   if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      Serial.print("The characteristic value was: ");
      Serial.println(value.c_str());
    }

/*
    while (pRemoteCharacteristic->canRead()) { //the size of the value in bytes
      pRemoteCharacteristic->read(); //
    if (pRemoteCharacteristic->valueLength() > 0) 
      unsigned int dataHex[1] = {};
      pRemoteCharacteristic->readValue(dataHex, 1);
      Serial.println("received byte");
      Serial.println(dataHex[0], HEX);
    }
    }
    */
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
  
  delay(1000); // Delay a second between loops.
} // End of loop