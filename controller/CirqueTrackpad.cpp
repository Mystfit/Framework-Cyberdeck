#include "CirqueTrackpad.h"
#include <Arduino.h>

CirqueTrackpad::CirqueTrackpad(uint8_t ChipSelectPin, uint8_t DataReadyPin) :
    _ChipSelectPin(ChipSelectPin),
    _DataReadyPin(DataReadyPin)
{
}

/*  Pinnacle-based TM0XX0XX Functions  */
void CirqueTrackpad::Init()
{
  RAP_Init();
  DeAssert_CS();
  pinMode(_DataReadyPin, INPUT);

  // Host clears SW_CC flag
  ClearFlags();

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
void CirqueTrackpad::GetRelative(TrackpadRelativeData_t * result)
{
  uint8_t data[4] = { 0,0,0,0 };
  RAP_ReadBytes(PACKETBYTE_0_ADDRESS, data, 4);

  ClearFlags();

  result->buttonFlags = data[0] & 0x07;
  result->xDelta = data[1] | ((data[0]&0x10)<<8);
  result->yDelta = data[2] | ((data[0]&0x20)<<8);
  result->scrollWheel = data[3];
}

// Reads XYZ data from Pinnacle registers 0x14 through 0x17
// Stores result in TrackpadAbsoluteData_t struct with xValue, yValue, and zValue members
void CirqueTrackpad::GetAbsolute(TrackpadAbsoluteData_t * result)
{
  uint8_t data[6] = { 0,0,0,0,0,0 };
  RAP_ReadBytes(0x12, data, 6);

  ClearFlags();

  result->buttonFlags = data[0] & 0x3F;
  result->xValue = data[2] | ((data[4] & 0x0F) << 8);
  result->yValue = data[3] | ((data[4] & 0xF0) << 4);
  result->zValue = data[5] & 0x3F;

  result->touchDown = result->xValue != 0;
}

// Checks touch data to see if it is a z-idle packet (all zeros)
bool Pinnacle_zIdlePacket(TrackpadAbsoluteData_t * data)
{
  return data->xValue == 0 && data->yValue == 0 && data->zValue == 0;
}

// Clears Status1 register flags (SW_CC and SW_DR)
void CirqueTrackpad::ClearFlags()
{
  RAP_Write(0x02, 0x00);
  delayMicroseconds(50);
}

// Enables/Disables the feed
void CirqueTrackpad::EnableFeed(bool feedEnable)
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

// Sets the trackpad to relative mode
void CirqueTrackpad::SetRelativeMode()
{
  EnableFeed(false); // Disable feed

  uint8_t temp;

  RAP_ReadBytes(FEEDCONFIG_1_ADDR, &temp, 1);  // Store contents of FeedConfig1 register

  temp ^= 0x03;                // Clear Feed Mode bits
  temp |= 0x01;                 // Set Feed Mode bits to relative mode

  RAP_Write(FEEDCONFIG_1_ADDR, temp);
  EnableFeed(true);
  //RAP_Write(FEEDCONFIG_1_ADDR, 0x01);
}

// Sets the trackpad to absolute mode
void CirqueTrackpad::SetAbsoluteMode()
{
  EnableFeed(false); // Disable feed
  uint8_t temp;

  RAP_ReadBytes(FEEDCONFIG_1_ADDR, &temp, 1);  // Store contents of FeedConfig1 register

  temp ^= 0x03;                // Clear Feed Mode bits
  temp |= 0x03;                 // Set Feed Mode bits to absolute mode

  RAP_Write(FEEDCONFIG_1_ADDR, temp);
  EnableFeed(true);
  //RAP_Write(FEEDCONFIG_1_ADDR, 0x03);
}

void CirqueTrackpad::SetInverseY(bool inverseY)
{
  EnableFeed(false); // Disable feed
  uint8_t temp;

  RAP_ReadBytes(FEEDCONFIG_1_ADDR, &temp, 1);  // Store contents of FeedConfig1 register

  if(inverseY)
  {
    temp |= 0x80;                 // Set inverse Y bit
  }
  else
  {
    temp ^= 0x80;                // Clear inverse Y bit
  }

  RAP_Write(FEEDCONFIG_1_ADDR, temp);
  EnableFeed(true);
}

/*  ERA (Extended Register Access) Functions  */
// Reads <count> bytes from an extended register at <address> (16-bit address),
// stores values in <*data>
void CirqueTrackpad::ERA_ReadBytes(uint16_t address, uint8_t * data, uint16_t count)
{
  uint8_t ERAControlValue = 0xFF;

  EnableFeed(false); // Disable feed

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

    ClearFlags();
  }
}

// Writes a byte, <data>, to an extended register at <address> (16-bit address)
void CirqueTrackpad::ERA_WriteByte(uint16_t address, uint8_t data)
{
  uint8_t ERAControlValue = 0xFF;

  EnableFeed(false); // Disable feed

  RAP_Write(0x1B, data);      // Send data byte to be written

  RAP_Write(0x1C, (uint8_t)(address >> 8));     // Upper byte of ERA address
  RAP_Write(0x1D, (uint8_t)(address & 0x00FF)); // Lower byte of ERA address

  RAP_Write(0x1E, 0x02);  // Signal an ERA-write to Pinnacle

  // Wait for status register 0x1E to clear
  do
  {
    RAP_ReadBytes(0x1E, &ERAControlValue, 1);
  } while(ERAControlValue != 0x00);

  ClearFlags();
}

/*  RAP Functions */

void CirqueTrackpad::RAP_Init()
{
  pinMode(_ChipSelectPin, OUTPUT);
  SPI.begin();
}

// Reads <count> Pinnacle registers starting at <address>
void CirqueTrackpad::RAP_ReadBytes(byte address, byte * data, byte count)
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
void CirqueTrackpad::RAP_Write(byte address, byte data)
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
void ClipCoordinates(TrackpadAbsoluteData_t * coordinates)
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
void ScaleData(TrackpadAbsoluteData_t * coordinates, uint16_t xResolution, uint16_t yResolution)
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
void CirqueTrackpad::Assert_CS()
{
  digitalWrite(_ChipSelectPin, LOW);
}

void CirqueTrackpad::DeAssert_CS()
{
  digitalWrite(_ChipSelectPin, HIGH);
}

bool CirqueTrackpad::DataReady()
{
  return digitalRead(_DataReadyPin);
}
