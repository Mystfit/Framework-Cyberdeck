// ___ Using a Cirque TM0XX0XX w/ Flat Overlay and Arduino ___
// This demonstration application is built to work with a Teensy 3.1/3.2 but it can easily be adapted to
// work with Arduino-based systems.
// When using with DK000013 development kit, connect sensor to the FFC connector
// labeled 'Sensor0'.
// This application connects to a TM0XX0XX circular touch pad via SPI. To verify that your touch pad is configured
// for SPI-mode, make sure that R1 is populated with a 470k resistor (or whichever resistor connects pins 24 & 25 of the 1CA027 IC).
// The pad is configured for Absolute mode tracking.  Touch data is sent in text format over USB CDC to
// the host PC.  You can open a terminal window on the PC to the USB CDC port and see X, Y, and Z data
// fill the window when you touch the sensor. Tools->Serial Monitor can be used to view touch data.
// NOTE: all config values applied in this sample are meant for a module using REXT = 976kOhm

//  Pinnacle TM0XX0XX with Arduino
//  Hardware Interface
//  GND
//  +3.3V
//  SCK = Pin 13
//  MISO = Pin 12
//  MOSI = Pin 11
//  SS = Pin 8
//  DR = Pin 7

#pragma once

#include <SPI.h>

// Masks for Cirque Register Access Protocol (RAP)
#define WRITE_MASK  0x80
#define READ_MASK   0xA0

// Register config values for this demo
#define SYSCONFIG_1_DATA   0x00
#define SYSCONFIG_1_ADDR   0x03
#define FEEDCONFIG_1_DATA  0x81  //0x01 = relative, 0x03 = absolute, 0x80 = invert Y-axis
#define FEEDCONFIG_1_ADDR  0x04
#define FEEDCONFIG_2_DATA  0x1C // disable scroll, disable rtap
#define FEEDCONFIG_2_ADDR  0x05
#define Z_IDLE_COUNT  0x05
#define PACKETBYTE_0_ADDRESS 0x12

// Coordinate scaling values
#define PINNACLE_XMAX     2047    // max value Pinnacle can report for X
#define PINNACLE_YMAX     1535    // max value Pinnacle can report for Y
#define PINNACLE_X_LOWER  127     // min "reachable" X value
#define PINNACLE_X_UPPER  1919    // max "reachable" X value
#define PINNACLE_Y_LOWER  63      // min "reachable" Y value
#define PINNACLE_Y_UPPER  1471    // max "reachable" Y value
#define PINNACLE_X_RANGE  (PINNACLE_X_UPPER-PINNACLE_X_LOWER)
#define PINNACLE_Y_RANGE  (PINNACLE_Y_UPPER-PINNACLE_Y_LOWER)

typedef struct _TrackpadAbsoluteData
{
  uint16_t xValue;
  uint16_t yValue;
  uint16_t zValue;
  uint8_t buttonFlags;
  bool touchDown;
  bool hovering;
} TrackpadAbsoluteData_t;

typedef struct _TrackpadRelativeData
{
  int16_t xDelta;
  int16_t yDelta;
  uint8_t buttonFlags;
  int8_t scrollWheel;
} TrackpadRelativeData_t;


class CirqueTrackpad {
public:
    CirqueTrackpad(uint8_t ChipSelectPin, uint8_t DataReadyPin);
    void Init();
    void GetRelative(TrackpadRelativeData_t * result);
    void GetAbsolute(TrackpadAbsoluteData_t * result);
    void EnableFeed(bool feedEnable);
    void SetRelativeMode();
    void SetAbsoluteMode();
    void SetInverseY(bool inverseY);
    bool DataReady();

private:
    void RAP_Init();
    void RAP_Write(uint8_t address, uint8_t data);
    void RAP_ReadBytes(uint8_t address, uint8_t * data, uint8_t count);
    void ERA_ReadBytes(uint16_t address, uint8_t * data, uint16_t count);
    void ERA_WriteByte(uint16_t address, uint8_t data);

    void ClearFlags();
    void Assert_CS();
    void DeAssert_CS();

    uint8_t _ChipSelectPin;
    uint8_t _DataReadyPin;  
};

