// Trackpad includes
#include <SPI.h>

// Gamepad includes
#include <SimpleKalmanFilter.h>
#include <Arduino.h>

#include <MouseDevice.h>
#include <GamepadDevice.h>
#include <BleCompositeHID.h>

#include <MouseConfiguration.h>
#include <GamepadConfiguration.h>


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

// Hardware pin-number labels
//#define SCK_PIN   13
//#define DIN_PIN   12
//#define DOUT_PIN  11
#define CS_PIN    A4//T0 //10
#define DR_PIN    A5//T1 //9

//#define SDA_PIN   18
//#define SCL_PIN   19

#define LED_0     5
#define LED_1     20

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

// Debugging
//#define CONFIG_BT_NIMBLE_DEBUG 

// Convenient way to store and access measurements
typedef struct _absData
{
  uint16_t xValue;
  uint16_t yValue;
  uint16_t zValue;
  uint8_t buttonFlags;
  bool touchDown;
  bool hovering;
} absData_t;

absData_t touchData;

typedef struct _relData
{
  int16_t xDelta;
  int16_t yDelta;
  uint8_t buttonFlags;
  int8_t scrollWheel;
} relData_t;

relData_t relData;
relData_t lastRelData;

typedef struct _gamepadData
{
  int16_t triggerLeft;
  int16_t triggerRight;
} gamepadData_t;

gamepadData_t lastGamepadState;


// Serial output refresh time
const long SERIAL_REFRESH_TIME = 8;
long refresh_time;

// HID devices
GamepadDevice* gamepad;
MouseDevice* mouse;
BleCompositeHID compositeHID;

// Input filters
SimpleKalmanFilter LeftTriggerKalmanFilter(0.2f, 0.2f, 0.01);
SimpleKalmanFilter RightTriggerKalmanFilter(0.2f, 0.2f, 0.01);


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float get_trigger_value(int pin, SimpleKalmanFilter& filter){
    // read a reference value from A0 and map it from 0 to 100
    float real_value = analogRead(pin)/4096.0 * 100.0;
    
    // add a noise to the reference value and use as the measured value
    float measured_value = real_value; + random(-100,100)/100.0;

    // calculate the estimated value with Kalman Filter
    float estimated_value = filter.updateEstimate(measured_value);

    float mapped_value = constrain(mapfloat(estimated_value, 40.5f, 100.0f, 0.0f, 1.0f), 0.0f, 1.0f);//constrain(int(map(estimated_value, 45.0l, 100.0l, 0.0l, 32767.0l)), 0, 32767);
    /*Serial.print("Pin" + String(pin) + "estimated:");
    Serial.print(estimated_value);
    Serial.print(",");
    Serial.print("Pin" + String(pin) + "real:");
    Serial.print(real_value);
    Serial.print(",");
    Serial.print("Pin" + String(pin) + "mapped:");
    Serial.print(mapped_value);
    Serial.println(",");*/
    return mapped_value;
}

// setup() gets called once at power-up, sets up serial debug output and Cirque's Pinnacle ASIC.
void setup()
{
  Serial.begin(115200);
  analogReadResolution(12);

  Serial.println("Starting BLE work!");
  
  // Set up gamepad
  GamepadConfiguration gamepadConfig;
  gamepadConfig.setButtonCount(16);
  gamepadConfig.setHatSwitchCount(0);
  gamepadConfig.setAutoReport(false);
  gamepad = new GamepadDevice(gamepadConfig);
  Serial.println("Created gamepad device");

  // Set up mouse
  MouseConfiguration mouseConfig;
  mouseConfig.setAutoReport(false);
  mouse = new MouseDevice(mouseConfig);
  Serial.println("Created mouse device");

  // Add both devices to the composite HID device to manage them
  compositeHID.addDevice(gamepad);
  compositeHID.addDevice(mouse);

  // Start the composite HID device to broadcast HID reports
  Serial.println("Starting composite HID device...");
  compositeHID.begin();
  Serial.println("Composite HID device started");
  
  // Setup LED
  pinMode(LED_0, OUTPUT);

  // Set up trackpad
  Pinnacle_Init();
  Pinnacle_EnableFeed(true);
  Serial.println("Trackpad");
}

// loop() continuously checks to see if data-ready (DR) is high. If so, reads and reports touch data to terminal.
void loop() 
{
  // Touchpad loop
  delay(8);
  if(DR_Asserted())
  {
    Pinnacle_GetRelative(&relData);

    // Update serial debug
    if(millis() > refresh_time){
      Serial.print("X: ");
      Serial.print(relData.xDelta);
      Serial.print(" Y: ");
      Serial.print(relData.yDelta);
      Serial.println("");
    }

    if(compositeHID.isConnected()){
      // Mouse the mouse
      mouse->mouseMove(relData.xDelta, relData.yDelta, 0, 0);

      // Handle button state changes
      if(relData.buttonFlags & 0x01){
        //Serial.print(" Click: 1");
        if(!(lastRelData.buttonFlags & 0x01)){
          mouse->mousePress(MOUSE_LOGICAL_LEFT_BUTTON);
          AssertSensorLED(touchData.touchDown);
        }
      } else {
        //Serial.print(" Click: 0");
        if(lastRelData.buttonFlags & 0x01){
          mouse->mouseRelease(MOUSE_LOGICAL_LEFT_BUTTON);
          AssertSensorLED(touchData.touchDown);
        }
      }
      lastRelData = relData;

      // Send HID report
      mouse->sendMouseReport();
    }
  }

  // Gamepad loop
  if(compositeHID.isConnected()){
    float trigger_left = get_trigger_value(A0, LeftTriggerKalmanFilter);
    float trigger_right = get_trigger_value(A3, RightTriggerKalmanFilter);

    int16_t mapped_trigger_left = constrain(int16_t(trigger_left * 32767), 0, SHRT_MAX);
    int16_t mapped_trigger_right = constrain(int16_t(trigger_right * 32767), 0, SHRT_MAX);
    
    gamepadData_t gamepadData;
    gamepadData.triggerLeft = mapped_trigger_left;
    gamepadData.triggerRight = mapped_trigger_right;

    // send to Serial output every 100ms
    // use the Serial Ploter for a good visualization
    if ( (gamepadData.triggerLeft != lastGamepadState.triggerLeft || gamepadData.triggerRight != lastGamepadState.triggerRight)) 
    {
      if(millis() > refresh_time){
        Serial.print("LeftTrigger:");
        Serial.print(mapped_trigger_left);
        Serial.print(",");
        Serial.print("RightTrigger:");
        Serial.println(mapped_trigger_right);
      }

      gamepad->setLeftTrigger(gamepadData.triggerLeft);
      gamepad->setRightTrigger(gamepadData.triggerRight);
      gamepad->sendGamepadReport();
    }

    lastGamepadState = gamepadData;
  }

  refresh_time = millis() + SERIAL_REFRESH_TIME;
}

/*  Pinnacle-based TM0XX0XX Functions  */
void Pinnacle_Init()
{
  RAP_Init();
  DeAssert_CS();
  pinMode(DR_PIN, INPUT);

  // Host clears SW_CC flag
  Pinnacle_ClearFlags();

  // Host configures bits of registers 0x03 and 0x05
  RAP_Write(SYSCONFIG_1_ADDR, SYSCONFIG_1_DATA);
  RAP_Write(FEEDCONFIG_2_ADDR, FEEDCONFIG_2_DATA);

  // Host enables USB Mouse output mode (relative)
  RAP_Write(FEEDCONFIG_1_ADDR, FEEDCONFIG_1_DATA);

  // Host sets z-idle packet count to 5 (default is 30)
  RAP_Write(0x0A, Z_IDLE_COUNT);
  //Serial.println("Pinnacle Initialized...");
}

//reads relative data from Pinnacle so we can pass it on to work as a USB Mouse
void Pinnacle_GetRelative(relData_t * result)
{
  uint8_t data[4] = { 0,0,0,0 };
  RAP_ReadBytes(PACKETBYTE_0_ADDRESS, data, 4);

  Pinnacle_ClearFlags();

  result->buttonFlags = data[0] & 0x07;
  result->xDelta = data[1] | ((data[0]&0x10)<<8);
  result->yDelta = data[2] | ((data[0]&0x20)<<8);
  result->scrollWheel = data[3];
}

// Reads XYZ data from Pinnacle registers 0x14 through 0x17
// Stores result in absData_t struct with xValue, yValue, and zValue members
void Pinnacle_GetAbsolute(absData_t * result)
{
  uint8_t data[6] = { 0,0,0,0,0,0 };
  RAP_ReadBytes(0x12, data, 6);

  Pinnacle_ClearFlags();

  result->buttonFlags = data[0] & 0x3F;
  result->xValue = data[2] | ((data[4] & 0x0F) << 8);
  result->yValue = data[3] | ((data[4] & 0xF0) << 4);
  result->zValue = data[5] & 0x3F;

  result->touchDown = result->xValue != 0;
}

// Checks touch data to see if it is a z-idle packet (all zeros)
bool Pinnacle_zIdlePacket(absData_t * data)
{
  return data->xValue == 0 && data->yValue == 0 && data->zValue == 0;
}

// Clears Status1 register flags (SW_CC and SW_DR)
void Pinnacle_ClearFlags()
{
  RAP_Write(0x02, 0x00);
  delayMicroseconds(50);
}

// Enables/Disables the feed
void Pinnacle_EnableFeed(bool feedEnable)
{
  uint8_t temp;

  RAP_ReadBytes(FEEDCONFIG_1_ADDR, &temp, 1);  // Store contents of FeedConfig1 register

  if(feedEnable)
  {
    temp |= 0x01;                 // Set Feed Enable bit
    RAP_Write(FEEDCONFIG_1_ADDR, temp);
  }
  else
  {
    temp &= ~0x01;                // Clear Feed Enable bit
    RAP_Write(FEEDCONFIG_1_ADDR, temp);
  }
}

/*  ERA (Extended Register Access) Functions  */
// Reads <count> bytes from an extended register at <address> (16-bit address),
// stores values in <*data>
void ERA_ReadBytes(uint16_t address, uint8_t * data, uint16_t count)
{
  uint8_t ERAControlValue = 0xFF;

  Pinnacle_EnableFeed(false); // Disable feed

  RAP_Write(0x1C, (uint8_t)(address >> 8));     // Send upper byte of ERA address
  RAP_Write(0x1D, (uint8_t)(address & 0x00FF)); // Send lower byte of ERA address

  for(uint16_t i = 0; i < count; i++)
  {
    RAP_Write(0x1E, 0x05);  // Signal ERA-read (auto-increment) to Pinnacle

    // Wait for status register 0x1E to clear
    do
    {
      RAP_ReadBytes(0x1E, &ERAControlValue, 1);
    } while(ERAControlValue != 0x00);

    RAP_ReadBytes(0x1B, data + i, 1);

    Pinnacle_ClearFlags();
  }
}

// Writes a byte, <data>, to an extended register at <address> (16-bit address)
void ERA_WriteByte(uint16_t address, uint8_t data)
{
  uint8_t ERAControlValue = 0xFF;

  Pinnacle_EnableFeed(false); // Disable feed

  RAP_Write(0x1B, data);      // Send data byte to be written

  RAP_Write(0x1C, (uint8_t)(address >> 8));     // Upper byte of ERA address
  RAP_Write(0x1D, (uint8_t)(address & 0x00FF)); // Lower byte of ERA address

  RAP_Write(0x1E, 0x02);  // Signal an ERA-write to Pinnacle

  // Wait for status register 0x1E to clear
  do
  {
    RAP_ReadBytes(0x1E, &ERAControlValue, 1);
  } while(ERAControlValue != 0x00);

  Pinnacle_ClearFlags();
}

/*  RAP Functions */

void RAP_Init()
{
  pinMode(CS_PIN, OUTPUT);
  SPI.begin();
}

// Reads <count> Pinnacle registers starting at <address>
void RAP_ReadBytes(byte address, byte * data, byte count)
{
  byte cmdByte = READ_MASK | address;   // Form the READ command byte

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

  Assert_CS();
  SPI.transfer(cmdByte);  // Signal a RAP-read operation starting at <address>
  SPI.transfer(0xFC);     // Filler byte
  SPI.transfer(0xFC);     // Filler byte
  for(byte i = 0; i < count; i++)
  {
    data[i] =  SPI.transfer(0xFC);  // Each subsequent SPI transfer gets another register's contents
  }
  DeAssert_CS();

  SPI.endTransaction();
}

// Writes single-byte <data> to <address>
void RAP_Write(byte address, byte data)
{
  byte cmdByte = WRITE_MASK | address;  // Form the WRITE command byte

  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE1));

  Assert_CS();
  SPI.transfer(cmdByte);  // Signal a write to register at <address>
  SPI.transfer(data);    // Send <value> to be written to register
  DeAssert_CS();

  SPI.endTransaction();
}

/*  Logical Scaling Functions */
// Clips raw coordinates to "reachable" window of sensor
// NOTE: values outside this window can only appear as a result of noise
void ClipCoordinates(absData_t * coordinates)
{
  if(coordinates->xValue < PINNACLE_X_LOWER)
  {
    coordinates->xValue = PINNACLE_X_LOWER;
  }
  else if(coordinates->xValue > PINNACLE_X_UPPER)
  {
    coordinates->xValue = PINNACLE_X_UPPER;
  }
  if(coordinates->yValue < PINNACLE_Y_LOWER)
  {
    coordinates->yValue = PINNACLE_Y_LOWER;
  }
  else if(coordinates->yValue > PINNACLE_Y_UPPER)
  {
    coordinates->yValue = PINNACLE_Y_UPPER;
  }
}

// Scales data to desired X & Y resolution
void ScaleData(absData_t * coordinates, uint16_t xResolution, uint16_t yResolution)
{
  uint32_t xTemp = 0;
  uint32_t yTemp = 0;

  ClipCoordinates(coordinates);

  xTemp = coordinates->xValue;
  yTemp = coordinates->yValue;

  // translate coordinates to (0, 0) reference by subtracting edge-offset
  xTemp -= PINNACLE_X_LOWER;
  yTemp -= PINNACLE_Y_LOWER;

  // scale coordinates to (xResolution, yResolution) range
  coordinates->xValue = (uint16_t)(xTemp * xResolution / PINNACLE_X_RANGE);
  coordinates->yValue = (uint16_t)(yTemp * yResolution / PINNACLE_Y_RANGE);
}

/*  I/O Functions */
void Assert_CS()
{
  digitalWrite(CS_PIN, LOW);
}

void DeAssert_CS()
{
  digitalWrite(CS_PIN, HIGH);
}

void AssertSensorLED(bool state)
{
  digitalWrite(LED_0, !state);
}

bool DR_Asserted()
{
  return digitalRead(DR_PIN);
}

