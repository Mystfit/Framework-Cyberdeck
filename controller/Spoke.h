#pragma once

#include <NimBLEDevice.h>
#include "CirqueTrackpad.h"

#define SATELLITE_SERVICE "5A00"
#define SATELLITE_INPUT_CHARACTERISTIC "5A10"
#define SATELLITE_INPUT_DESCRIPTOR "5A11"
#define SATELLITE_OUTPUT_CHARACTERISTIC "5A20"


enum SingleControllerButtons : uint16_t{
    faceNorth = 0x0001,
    faceEast = 0x0002,
    faceSouth = 0x0004,
    faceWest = 0x0008,
    trackpadBtn = 0x0010,
    trackpadTouched = 0x0020,
    triggerTouched = 0x0040,
    bumper = 0x0080,
    menuA = 0x0100,
    menuB = 0x0200
};

struct PackedGamepadInputState {
    uint16_t buttons = 0x0000;
    // int16_t x = 0;
    // int16_t y = 0;
    // int16_t z = 0;
    TrackpadAbsoluteData_t absTrackpadData;
    TrackpadRelativeData_t relTrackpadData;
    uint16_t trigger = 0;
};

struct PackedGamepadOutputState {
    uint8_t strongRumble;
    uint8_t weakRumble;
};


class Spoke : 
    public NimBLEServerCallbacks, 
    public NimBLECharacteristicCallbacks,
    public NimBLEDescriptorCallbacks
{
public:
    Spoke();
    void init();
    void update();

    // NimBLEServerCallback implementations
    void onConnect(NimBLEServer* pServer) override;
    void onDisconnect(NimBLEServer* pServer) override;
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) override;
    void onAuthenticationComplete(ble_gap_conn_desc* desc) override;

    // NimBLECharacteristicCallback implementations
    void onWrite(NimBLECharacteristic* pCharacteristic) override;
    void onRead(NimBLECharacteristic* pCharacteristic) override;
    void onNotify(NimBLECharacteristic* pCharacteristic) override;
    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code) override;
    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override;

    // NimBLEDescriptorCallback implementations
    void onWrite(NimBLEDescriptor* pDescriptor) override;
    void onRead(NimBLEDescriptor* pDescriptor) override;

    void notifyHub(const PackedGamepadInputState& state);

    bool isHubConnected();

private:
    NimBLEServer* _pServer;
    bool _hubConnected;
};
