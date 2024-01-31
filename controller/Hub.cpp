
/** NimBLE_Server Demo:
 *
 *  Demonstrates many of the available features of the NimBLE client library.
 *
 *  Created: on March 24 2020
 *      Author: H2zero
 *
*/

#include "Hub.h"
#include "Spoke.h"

Hub::Hub() :
    _isConnected(false),
    _advDevice(nullptr),
    _doConnect(false),
    _scanTime(0),
    _dataReady(false)
{
}

//void scanEndedCB(NimBLEScanResults results);

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
void Hub::onConnect(NimBLEClient* pClient) {
    Serial.println("Connected");
    /** After connection we should change the parameters if we don't need fast response times.
     *  These settings are 150ms interval, 0 latency, 450ms timout.
     *  Timeout should be a multiple of the interval, minimum is 100ms.
     *  I find a multiple of 3-5 * the interval works best for quick response/reconnect.
     *  Min interval: 120 * 1.25ms = 150, Max interval: 120 * 1.25ms = 150, 0 latency, 60 * 10ms = 600ms timeout
     */
    //pClient->updateConnParams(120,120,0,60);
    pClient->updateConnParams(6, 8, 0, 400);
};

void Hub::onDisconnect(NimBLEClient* pClient) {
    Serial.print(pClient->getPeerAddress().toString().c_str());
    Serial.println(" Disconnected - Starting scan");
    NimBLEDevice::getScan()->start(_scanTime, true);
    //NimBLEDevice::getScan()->start(_scanTime, std::bind(&Hub::scanEndedCB, this, std::placeholders::_1), true);
};

/** Called when the peripheral requests a change to the connection parameters.
 *  Return true to accept and apply them or false to reject and keep
 *  the currently used parameters. Default will return true.
 */
bool Hub::onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) {
    if(params->itvl_min < 24) { /** 1.25ms units */
        return false;
    } else if(params->itvl_max > 40) { /** 1.25ms units */
        return false;
    } else if(params->latency > 2) { /** Number of intervals allowed to skip */
        return false;
    } else if(params->supervision_timeout > 100) { /** 10ms units */
        return false;
    }

    return true;
};

/********************* Security handled here **********************
****** Note: these are the same return values as defaults ********/
uint32_t Hub::onPassKeyRequest(){
    Serial.println("Client Passkey Request");
    /** return the passkey to send to the server */
    return 123456;
};

bool Hub::onConfirmPIN(uint32_t pass_key){
    Serial.print("The passkey YES/NO number: ");
    Serial.println(pass_key);
/** Return false if passkeys don't match. */
    return true;
};

/** Pairing process complete, we can check the results in ble_gap_conn_desc */
void Hub::onAuthenticationComplete(ble_gap_conn_desc* desc){
    if(!desc->sec_state.encrypted) {
        Serial.println("Encrypt connection failed - disconnecting");
        /** Find the client with the connection handle provided in desc */
        NimBLEDevice::getClientByID(desc->conn_handle)->disconnect();
        return;
    }
};


/** Define a class to handle the callbacks when advertisments are received */
void Hub::onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    Serial.print("Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());
    if(advertisedDevice->isAdvertisingService(NimBLEUUID(SATELLITE_SERVICE)))
    {
        Serial.println("Found Our Service");
        /** stop scan before connecting */
        NimBLEDevice::getScan()->stop();
        /** Save the device reference in a global for the client to use*/
        _advDevice = advertisedDevice;
        /** Ready to connect now */
        _doConnect = true;
    }
}


/** Notification / Indication receiving handler callback */
void Hub::notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify){
    std::string str = (isNotify == true) ? "Notification" : "Indication";
    str += " from ";
    /** NimBLEAddress and NimBLEUUID have std::string operators */
    str += std::string(pRemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress());
    str += ": Service = " + std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    str += ", Characteristic = " + std::string(pRemoteCharacteristic->getUUID());
    str += ", Value = " + std::string((char*)pData, length);
    //Serial.println(str.c_str());

    auto remoteServiceUUID = std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    auto remoteCharacteristicUUID = std::string(pRemoteCharacteristic->getUUID());
    if (pRemoteCharacteristic->getRemoteService()->getUUID() == NimBLEUUID(SATELLITE_SERVICE) && pRemoteCharacteristic->getUUID() == NimBLEUUID(SATELLITE_INPUT_CHARACTERISTIC)) {
        _lastReceviedSatelliteInput = *reinterpret_cast<PackedGamepadInputState*>(pData);
        //Serial.print("Received satellite input: ");
        // Serial.print(_lastReceviedSatelliteInput.buttons, HEX);
        // Serial.print(", ");
        // Serial.print(_lastReceviedSatelliteInput.x);
        // Serial.print(", ");
        // Serial.print(_lastReceviedSatelliteInput.y);
        // Serial.print(", ");
        // Serial.print(_lastReceviedSatelliteInput.z);
        // Serial.print(", ");
        // Serial.print(_lastReceviedSatelliteInput.trigger);
        _dataReady = true;
    }
}

/** Callback to process the results of the last scan or restart it */
void Hub::scanEndedCB(NimBLEScanResults results){
    Serial.println("Scan Ended");
}

/** Create a single global instance of the callback class to be used by all clients */
//static ClientCallbacks clientCB;

/** Handles the provisioning of clients and connects / interfaces with the server */
bool Hub::connectToServer() {
    NimBLEClient* pClient = nullptr;

    /** Check if we have a client we should reuse first **/
    if(NimBLEDevice::getClientListSize()) {
        /** Special case when we already know this device, we send false as the
         *  second argument in connect() to prevent refreshing the service database.
         *  This saves considerable time and power.
         */
        pClient = NimBLEDevice::getClientByPeerAddress(_advDevice->getAddress());
        if(pClient){
            if(!pClient->connect(_advDevice, false)) {
                Serial.println("Reconnect failed");
                return false;
            }
            Serial.println("Reconnected client");
        }
        /** We don't already have a client that knows this device,
         *  we will check for a client that is disconnected that we can use.
         */
        else {
            pClient = NimBLEDevice::getDisconnectedClient();
        }
    }

    /** No client to reuse? Create a new one. */
    if(!pClient) {
        if(NimBLEDevice::getClientListSize() >= NIMBLE_MAX_CONNECTIONS) {
            Serial.println("Max clients reached - no more connections available");
            return false;
        }

        pClient = NimBLEDevice::createClient();

        Serial.println("New client created");

        pClient->setClientCallbacks(this, false);
        /** Set initial connection parameters: These settings are 15ms interval, 0 latency, 120ms timout.
         *  These settings are safe for 3 clients to connect reliably, can go faster if you have less
         *  connections. Timeout should be a multiple of the interval, minimum is 100ms.
         *  Min interval: 12 * 1.25ms = 15, Max interval: 12 * 1.25ms = 15, 0 latency, 51 * 10ms = 510ms timeout
         */
        pClient->setConnectionParams(12,12,0,51);
        /** Set how long we are willing to wait for the connection to complete (seconds), default is 30. */
        pClient->setConnectTimeout(10);

        if (!pClient->connect(_advDevice)) {
            /** Created a client but failed to connect, don't need to keep it as it has no data */
            NimBLEDevice::deleteClient(pClient);
            Serial.println("Failed to connect, deleted client");
            return false;
        }
    }

    if(!pClient->isConnected()) {
        if (!pClient->connect(_advDevice)) {
            Serial.println("Failed to connect");
            return false;
        }
    }

    Serial.print("Connected to: ");
    Serial.println(pClient->getPeerAddress().toString().c_str());
    Serial.print("RSSI: ");
    Serial.println(pClient->getRssi());

    /** Now we can read/write/subscribe the charateristics of the services we are interested in */
    NimBLERemoteService* pSvc = nullptr;
    NimBLERemoteCharacteristic* pChr = nullptr;
    NimBLERemoteDescriptor* pDsc = nullptr;

    pSvc = pClient->getService(SATELLITE_SERVICE);
    if(pSvc) {     /** make sure it's not null */
        pChr = pSvc->getCharacteristic(SATELLITE_INPUT_CHARACTERISTIC);

        if(pChr) {     /** make sure it's not null */
            //Serial.println("Found satellite controller input characteristic " + pChr->getUUID().toString().c_str());

            pDsc = pChr->getDescriptor(NimBLEUUID(SATELLITE_INPUT_DESCRIPTOR));
            if(pDsc) {   /** make sure it's not null */
                Serial.print("Descriptor: ");
                Serial.print(pDsc->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(pDsc->readValue().c_str());
            } else {
                Serial.println("Descriptor not found.");
            }

            if(pChr->canRead()) {
                Serial.print(pChr->getUUID().toString().c_str());
                Serial.print(" Value: ");
                Serial.println(pChr->readValue().c_str());
            }

            if(pChr->canNotify()) {
                Serial.println("Subscribing to satellite input characteristic");
                if(!pChr->subscribe(true, std::bind(&Hub::notifyCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, true))) {
                    Serial.println("Failed to subscribe to input characteristic");
                    /** Disconnect if subscribe failed */
                    pClient->disconnect();
                    return false;
                }
            } else {
                Serial.println("Satellite controller input characteristic does not support notifications.");
            }
        } else {
            Serial.println("Satellite controller input characteristic not found.");
        }

    } else {
        Serial.println("Satellite controller service not found.");
    }

    Serial.println("Done with this device!");
    return true;
}

void Hub::init (){
    Serial.println("Starting NimBLE Client");
    /** Initialize NimBLE, no device name spcified as we are not advertising */
    //NimBLEDevice::init("");

    /** Set the IO capabilities of the device, each option will trigger a different pairing method.
     *  BLE_HS_IO_KEYBOARD_ONLY    - Passkey pairing
     *  BLE_HS_IO_DISPLAY_YESNO   - Numeric comparison pairing
     *  BLE_HS_IO_NO_INPUT_OUTPUT - DEFAULT setting - just works pairing
     */
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_KEYBOARD_ONLY); // use passkey
    //NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_YESNO); //use numeric comparison

    /** 2 different ways to set security - both calls achieve the same result.
     *  no bonding, no man in the middle protection, secure connections.
     *
     *  These are the default values, only shown here for demonstration.
     */
    //NimBLEDevice::setSecurityAuth(false, false, true);
    NimBLEDevice::setSecurityAuth(/*BLE_SM_PAIR_AUTHREQ_BOND | BLE_SM_PAIR_AUTHREQ_MITM |*/ BLE_SM_PAIR_AUTHREQ_SC);

    /** Optional: set the transmit power, default is 3db */
#ifdef ESP_PLATFORM
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
    NimBLEDevice::setPower(9); /** +9db */
#endif

    /** Optional: set any devices you don't want to get advertisments from */
    // NimBLEDevice::addIgnored(NimBLEAddress ("aa:bb:cc:dd:ee:ff"));

    /** create new scan */
    NimBLEScan* pScan = NimBLEDevice::getScan();

    /** create a callback that gets called when advertisers are found */
    pScan->setAdvertisedDeviceCallbacks(this);

    /** Set scan interval (how often) and window (how long) in milliseconds */
    pScan->setInterval(45);
    pScan->setWindow(15);

    /** Active scan will gather scan response data from advertisers
     *  but will use more energy from both devices
     */
    pScan->setActiveScan(true);
    /** Start scanning for advertisers for the scan time specified (in seconds) 0 = forever
     *  Optional callback for when scanning stops.
     */
    pScan->start(_scanTime);
}


void Hub::update (){
    /** Found a device we want to connect to, do it now */
    if(_doConnect && _advDevice != nullptr){
        if(connectToServer()) {
            Serial.println("Success! we should now be getting notifications!");
        } else {
            Serial.println("Failed to connect, starting scan");
            NimBLEDevice::getScan()->start(_scanTime, true);
        }
        _doConnect = false;
    }
}

bool Hub::isConnected (){
    return _isConnected;
}

const PackedGamepadInputState& Hub::getLastReceivedSatelliteInput (){
    return _lastReceviedSatelliteInput;
}

bool Hub::isDataReady (){
    return _dataReady;
}

void Hub::consumeInputData (){
    _dataReady = false;
}