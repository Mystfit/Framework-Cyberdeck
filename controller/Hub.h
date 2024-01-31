#pragma once

#include <NimBLEDevice.h>
#include <NimBLEClient.h>
#include <NimBLEAdvertisedDevice.h>
#include "Spoke.h"

class Hub : 
    public NimBLEClientCallbacks,
    public NimBLEAdvertisedDeviceCallbacks
{
public:
    Hub();
    void init();
    void update();

    // NimBLEClientCallback implementations
    void onConnect(NimBLEClient* pClient) override;
    void onDisconnect(NimBLEClient* pClient) override;
    bool onConnParamsUpdateRequest(NimBLEClient* pClient, const ble_gap_upd_params* params) override;
    uint32_t onPassKeyRequest() override;
    bool onConfirmPIN(uint32_t pass_key) override;
    void onAuthenticationComplete(ble_gap_conn_desc* desc) override;

    // NimBLEAdvertisedDeviceCallback implementations
    void onResult(NimBLEAdvertisedDevice* advertisedDevice) override;

    void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
    void scanEndedCB(NimBLEScanResults results);

    bool isConnected();
    const PackedGamepadInputState& getLastReceivedSatelliteInput();

    void consumeInputData();
    bool isDataReady();

private:
    bool connectToServer();
    bool _isConnected = false;
    NimBLEAdvertisedDevice* _advDevice;

    bool _doConnect = false;    
    uint32_t _scanTime = 0; /** 0 = scan forever */

    bool _dataReady = true;

    PackedGamepadInputState _lastReceviedSatelliteInput;
};
