
/** NimBLE_Server Demo:
 *
 *  Demonstrates many of the available features of the NimBLE server library.
 *
 *  Created: on March 22 2020
 *      Author: H2zero
 *
*/

#include "Spoke.h"
#include <XboxGamepadDevice.h>

Spoke::Spoke() : 
    _pServer(nullptr),
    _hubConnected(false),
    _dataReady(false),
    _lastReceviedHubInput()
{
}

/**  None of these are required as they will be handled by the library with defaults. **
 **                       Remove as you see fit for your needs                        */
void Spoke::onConnect(NimBLEServer* pServer) {
    Serial.println("Client connected");
};

void Spoke::onDisconnect(NimBLEServer* pServer) {
    Serial.println("Client disconnected - start advertising");
    _hubConnected = false;
    NimBLEDevice::startAdvertising();
};

void Spoke::onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
    Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
};

void Spoke::onAuthenticationComplete(ble_gap_conn_desc* desc){
    /** Check that encryption was successful, if not we disconnect the client */
    if(!desc->sec_state.encrypted) {
        NimBLEDevice::getServer()->disconnect(desc->conn_handle);
        Serial.println("Encrypt connection failed - disconnecting client");
        return;
    }
};

/** Handler class for characteristic actions */
void Spoke::onRead(NimBLECharacteristic* pCharacteristic){
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onRead(), value: ");
    Serial.println(pCharacteristic->getValue().c_str());
};

void Spoke::onWrite(NimBLECharacteristic* pCharacteristic) {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onWrite(), value: ");
    Serial.println(pCharacteristic->getValue().c_str());
    //auto remoteServiceUUID = std::string(pRemoteCharacteristic->getRemoteService()->getUUID());
    //auto remoteCharacteristicUUID = std::string(pRemoteCharacteristic->getUUID());
    if (pCharacteristic->getService()->getUUID() == NimBLEUUID(SATELLITE_SERVICE) && 
        pCharacteristic->getUUID() == NimBLEUUID(SATELLITE_OUTPUT_CHARACTERISTIC)) {
            _lastReceviedHubInput = pCharacteristic->getValue<XboxGamepadOutputReportData>();
            _dataReady = true;
            Serial.print("Received vibration event from hub");
    }
};

void Spoke::onNotify(NimBLECharacteristic* pCharacteristic) {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.println(": onNotify()");
};


/** The status returned in status is defined in NimBLECharacteristic.h.
 *  The value returned in code is the NimBLE host return code.
 */
void Spoke::onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code) {
    String str = ("Notification/Indication status code: ");
    str += status;
    str += ", return code: ";
    str += code;
    str += ", ";
    str += NimBLEUtils::returnCodeToString(code);
    //Serial.println(str);
};

void Spoke::onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
    if(pCharacteristic->getUUID() == NimBLEUUID(SATELLITE_INPUT_CHARACTERISTIC)){
        _hubConnected = true;
        Serial.println("Hub connected");
    }
};

/** Handler class for descriptor actions */
void Spoke::onWrite(NimBLEDescriptor* pDescriptor) {
    std::string dscVal = pDescriptor->getValue();
    Serial.print("Descriptor witten value:");
    Serial.println(dscVal.c_str());
};

void Spoke::onRead(NimBLEDescriptor* pDescriptor) {
    Serial.print(pDescriptor->getUUID().toString().c_str());
    Serial.println(" Descriptor read");
};

void Spoke::init() {
    Serial.println("Starting NimBLE Server for satellite controller");

    /** sets device name */
    NimBLEDevice::init("PicaTTY Spoke Controller");

    /** Optional: set the transmit power, default is 3db */
#ifdef ESP_PLATFORM
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */
#else
    NimBLEDevice::setPower(9); /** +9db */
#endif

    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
    NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND);

    _pServer = NimBLEDevice::createServer();
    _pServer->setCallbacks(this);

    NimBLEService* pGamepadService = _pServer->createService(SATELLITE_SERVICE);
    NimBLECharacteristic* pGamepadInputCharacteristic = pGamepadService->createCharacteristic(
                                               SATELLITE_INPUT_CHARACTERISTIC,
                                               NIMBLE_PROPERTY::READ |
                                               NIMBLE_PROPERTY::WRITE |
                                               NIMBLE_PROPERTY::NOTIFY
    );
    pGamepadInputCharacteristic->setCallbacks(this);
    PackedGamepadInputState defaultInputState;
    pGamepadInputCharacteristic->setValue((uint8_t*)&defaultInputState, sizeof(defaultInputState));

	NimBLEDescriptor* GamepadInputDescriptor = pGamepadInputCharacteristic->createDescriptor(
		SATELLITE_INPUT_DESCRIPTOR,
		NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::WRITE |
        NIMBLE_PROPERTY::WRITE_ENC,
		20
	);
    GamepadInputDescriptor->setValue("Gamepad Input");
    GamepadInputDescriptor->setCallbacks(this);

    NimBLECharacteristic* pGamepadOutputCharacteristic = pGamepadService->createCharacteristic(
                                               SATELLITE_OUTPUT_CHARACTERISTIC,
                                               NIMBLE_PROPERTY::WRITE,
                                               sizeof(XboxGamepadOutputReportData)
    );
    pGamepadOutputCharacteristic->setCallbacks(this);
    XboxGamepadOutputReportData defaultOutputState;
    pGamepadOutputCharacteristic->setValue((uint8_t*)&defaultOutputState, sizeof(defaultOutputState));

    /** Start the services when finished creating all Characteristics and Descriptors */
    Serial.println("Starting satellite service");
    pGamepadService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    /** Add the services to the advertisment data **/
    pAdvertising->addServiceUUID(pGamepadService->getUUID());
    
    /** If your device is battery powered you may consider setting scan response
     *  to false as it will extend battery life at the expense of less data sent.
     */
    pAdvertising->setScanResponse(true);

    Serial.println("Starting satellite advertisement");
    pAdvertising->start();

    Serial.println("Advertising Started");
}

void Spoke::notifyHub(const PackedGamepadInputState& state){
    if(_pServer->getConnectedCount() && _hubConnected) {
        NimBLEService* pSvc = _pServer->getServiceByUUID(SATELLITE_SERVICE);
        if(pSvc) {
            NimBLECharacteristic* pChr = pSvc->getCharacteristic(SATELLITE_INPUT_CHARACTERISTIC);
            if(pChr) {
                pChr->notify((uint8_t*)&state, sizeof(state), true);
            }
        }
    }
}

bool Spoke::isHubConnected(){
    return _hubConnected;
}

bool Spoke::isDataReady (){
    return _dataReady;
}

void Spoke::consumeInputData (){
    _dataReady = false;
}

const XboxGamepadOutputReportData& Spoke::getLastReceivedHubInput(){
    return _lastReceviedHubInput;
}