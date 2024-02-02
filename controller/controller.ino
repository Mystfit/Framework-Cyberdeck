#include <Arduino.h>
#include <Ticker.h>

#include "CirqueTrackpad.h"
#include "Hub.h"
#include "Spoke.h"

#include <MouseDevice.h>
#include <XboxGamepadDevice.h>
#include <BleCompositeHID.h>

#include <SimpleKalmanFilter.h>
#include <Adafruit_BNO08x.h>

#include "MCP23S17.h"
#include "SSD1306.h"
#include "OLEDDisplayUi.h"

// Trackpad
#define TRACKPAD_CS_PIN    32
#define TRACKPAD_DR_PIN    34

// IMU
#define IMU_CS_PIN         16
#define IMU_INTERRUPT_PIN  3
#define IMU_RESET_PIN      1

// Haptics
#define HAPTIC_PIN 17

// Trigger
#define TRIGGER_PIN A0

// IO expander
#define MCP_CS_PIN         33
#define SATELLITE_ENABLE_PIN 7
#define BUTTON_1_PIN 8
#define BUTTON_2_PIN 9
#define BUTTON_3_PIN 10
#define BUTTON_4_PIN 11                                                                                     
#define BUTTON_5_PIN 12
#define BUTTON_6_PIN 13
#define BUTTON_7_PIN 14
#define BUTTON_8_PIN 15


enum class TrackpadMode {
    Mouse = 0,
    Joystick
};

enum class ControllerHand {
    Left = 0,
    Right
};

enum class ConnectionMode {
    Hub = 0,
    Spoke
};

const uint8_t activeSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00011000,
    B00100100,
    B01000010,
    B01000010,
    B00100100,
    B00011000
};

const uint8_t inactiveSymbol[] PROGMEM = {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B00011000,
    B00011000,
    B00000000,
    B00000000
};


// Serial output refresh time
const long SERIAL_REFRESH_TIME = 8;
long refresh_time;

// Input devices
MCP23S17 MCP(MCP_CS_PIN);

// Trackpad
CirqueTrackpad trackpad(TRACKPAD_CS_PIN, TRACKPAD_DR_PIN);
TrackpadRelativeData_t lastRelData;
TrackpadAbsoluteData_t lastAbsData;

// Controller modes
TrackpadMode currentTrackpadMode = TrackpadMode::Joystick;
ControllerHand currentControllerHand = ControllerHand::Left;

// Local button states
bool faceBtnNorthPressed = false;
bool faceBtnEastPressed = false;
bool faceBtnSouthPressed = false;
bool faceBtnWestPressed = false;
bool trackpadBtnPressed = false;
bool bumperPressed = false;
bool menuBtnAPressed = false;
bool menuBtnBPressed = false;

// Trigger
float triggerValue = 0.0f;
float lastTriggerValue = 0.0f;
bool satelliteTriggerTouched = false;

SimpleKalmanFilter triggerKalmanFilter(0.2f, 0.2f, 0.01);

// IMU
struct euler_t {
  float yaw;
  float pitch;
  float roll;
};

// euler_t ypr;
// Adafruit_BNO08x  imu(IMU_RESET_PIN);
// sh2_SensorValue_t imuValue;

// Output devices
XboxGamepadDevice* gamepad = nullptr;
MouseDevice* mouse = nullptr;
BleCompositeHID composite("PicaTTY Controller", "Mystfit", 100);

// Haptics
Ticker hapticTimer;
bool hapticState = false;

// Connection
ConnectionMode currentConnectionMode = ConnectionMode::Hub;
Hub* hubController = nullptr;
Spoke* spokeController = nullptr;
bool dataUpdated = false;

// Display
SSD1306Wire display(0x3c, SDA, SCL);  // ADDRESS, SDA, SCL  -  SDA and SCL usually populate automatically based on your board's pins_arduino.h e.g. https://github.com/esp8266/Arduino/blob/master/variants/nodemcu/pins_arduino.h
OLEDDisplayUi ui( &display );


float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float mapint(int x, int in_min, int in_max, int out_min, int out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

float get_trigger_value(int pin, SimpleKalmanFilter& filter){
    // read a reference value from A0 and map it from 0 to 100
    float real_value = analogRead(pin)/4096.0 * 100.0;
    
    // add a noise to the reference value and use as the measured value
    float measured_value = real_value; + random(-100,100)/100.0;

    // calculate the estimated value with Kalman Filter
    float estimated_value = filter.updateEstimate(measured_value);

    float mapped_value = constrain(mapfloat(estimated_value, 40.5f, 100.0f, 0.0f, 1.0f), 0.0f, 1.0f);
    //constrain(int(map(estimated_value, 45.0l, 100.0l, 0.0l, 32767.0l)), 0, 32767);
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


void drawLocalJoystick(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    display->setColor(WHITE);

    int circleHOffset = 15 + x;
    int circleVOffset = 32 + y;

    int circX = mapint(lastAbsData.xValue, PINNACLE_X_LOWER, PINNACLE_X_UPPER, 32 + 8, 96 - 8);
    int circY = mapint(lastAbsData.yValue, PINNACLE_Y_LOWER, PINNACLE_Y_UPPER, 16, 48);
    
    // Joystick position
    if(lastAbsData.touchDown)
        display->fillCircle(circX + circleHOffset, circY + y, 12);
    else
        display->drawCircle(64 + circleHOffset, circleVOffset, 12);

    // Outer bounds circle
    display->drawCircle(64 + circleHOffset, circleVOffset,  28);

    // Outer ring for when trackpad is depressed
    if(trackpadBtnPressed){
        display->drawCircle(64 + circleHOffset, circleVOffset, 26);
    }

    // Coordinate text
    String xPrefix = "X:";
    String yPrefix = "Y:";
    String xStr = String(lastAbsData.xValue);
    String yStr = String(lastAbsData.yValue);
    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->setFont(ArialMT_Plain_10);
    display->drawString(128 + x, 0 + y, xPrefix + xStr);
    display->drawString(128 + x, 12 + y, yPrefix + yStr);
}

FrameCallback frames[] = { drawLocalJoystick };
int frameCount = 1;


void buttonOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
    display->setColor(WHITE);

    int faceBtnHOffset = 8;
    int faceBtnVOffset = 25;
    int faceBtnRadius = 4;
    int faceBtnDiameter = faceBtnRadius * 2;

    if (faceBtnNorthPressed){
        display->fillCircle(faceBtnHOffset + faceBtnDiameter * 2, faceBtnVOffset, faceBtnRadius);
    } else {
        display->drawCircle(faceBtnHOffset + faceBtnDiameter * 2, faceBtnVOffset, faceBtnRadius);
    }

    if (faceBtnEastPressed){
        display->fillCircle(faceBtnHOffset + faceBtnDiameter * 3, faceBtnVOffset + faceBtnDiameter, faceBtnRadius);
    } else {
        display->drawCircle(faceBtnHOffset + faceBtnDiameter * 3, faceBtnVOffset + faceBtnDiameter, faceBtnRadius);
    }

    if (faceBtnSouthPressed){
        display->fillCircle(faceBtnHOffset + faceBtnDiameter * 2, faceBtnVOffset + faceBtnDiameter * 2, faceBtnRadius);
    } else {
        display->drawCircle(faceBtnHOffset + faceBtnDiameter * 2, faceBtnVOffset + faceBtnDiameter * 2, faceBtnRadius);
    }

    if (faceBtnWestPressed){
        display->fillCircle(faceBtnHOffset + faceBtnDiameter, faceBtnVOffset + faceBtnDiameter, faceBtnRadius);
    } else {
        display->drawCircle(faceBtnHOffset + faceBtnDiameter, faceBtnVOffset + faceBtnDiameter, faceBtnRadius);
    }

    // Trigger
    display->drawProgressBar(1, 6, 42, 10, int(triggerValue * 100));

    if (bumperPressed){
        display->fillRect(1, 1, 42, 5);
    } else {
        display->drawRect(1, 1, 42, 5);
    }

    if (menuBtnAPressed){
        display->fillRect(2, 45, 7, 7);
    } else {
        display->drawRect(2, 45, 7, 7);
    }

    if (menuBtnBPressed){
        display->fillRect(2, 53, 7, 7);
    } else {
        display->drawRect(2, 53, 7, 7);
    }

    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->setFont(ArialMT_Plain_10);
    display->drawString(128, 50, "Mode: " + (currentConnectionMode == ConnectionMode::Hub) ? "Hub" : "Satellite");
}


OverlayCallback overlays[] = { buttonOverlay };
int overlaysCount = 1;

void OnVibrateEvent(XboxGamepadOutputReportData data)
{
    if(data.weakMotorMagnitude > 0 || data.strongMotorMagnitude > 0){
        if(data.strongMotorMagnitude > 0)
            hapticStart(160);
        if(data.weakMotorMagnitude > 0)
            hapticStart(320);
    } else {
        hapticStop();
    }
    Serial.println("Vibration event. Weak motor: " + String(data.weakMotorMagnitude) + " Strong motor: " + String(data.strongMotorMagnitude));
}

void setup() {
    Serial.begin(115200);

    analogReadResolution(12);

    // Set up IO expander
    Serial.println("Enabling expanded IO");
    MCP.begin();
    delay(100);
    MCP.pinMode1(BUTTON_1_PIN, INPUT_PULLUP);
    MCP.pinMode1(BUTTON_2_PIN, INPUT_PULLUP);
    MCP.pinMode1(BUTTON_3_PIN, INPUT_PULLUP);
    MCP.pinMode1(BUTTON_4_PIN, INPUT_PULLUP);
    MCP.pinMode1(BUTTON_5_PIN, INPUT_PULLUP);
    MCP.pinMode1(BUTTON_6_PIN, INPUT_PULLUP);
    MCP.pinMode1(BUTTON_7_PIN, INPUT_PULLUP);
    MCP.pinMode1(BUTTON_8_PIN, INPUT_PULLUP);
    MCP.pinMode1(SATELLITE_ENABLE_PIN, INPUT_PULLUP);
    
    // Use hardware pin to set initial connection mode
    currentConnectionMode = (MCP.read1(SATELLITE_ENABLE_PIN)) ? ConnectionMode::Hub : ConnectionMode::Spoke;
    currentControllerHand = (MCP.read1(SATELLITE_ENABLE_PIN)) ? ControllerHand::Right : ControllerHand::Left;
    Serial.println("Hardware connection mode: " + String((currentConnectionMode == ConnectionMode::Hub) ? "Hub" : "Satellite"));
    Serial.println("Hardware controller hand: " + String((currentControllerHand == ControllerHand::Left) ? "Left" : "Right"));

    // Set up IMU
    // if (!imu.begin_SPI(IMU_CS_PIN, IMU_INTERRUPT_PIN)) {
    //     Serial.println("Failed to find BNO08x IMU chip");
    //     while (1) { delay(10); }
    // }
    // imu.enableReport(SH2_ARVR_STABILIZED_RV, 5000);

    // Set up haptics
    Serial.println("Enabling haptic");
    pinMode(HAPTIC_PIN, OUTPUT);

    // Set up trackpad
    Serial.println("Enabling trackpad");
    trackpad.Init();
    trackpad.EnableFeed(true);
    delay(100);

    if(currentTrackpadMode == TrackpadMode::Joystick)
        trackpad.SetAbsoluteMode();
    else
        trackpad.SetRelativeMode();
    
    trackpad.SetInverseY(false);

    // Set up gamepad
    Serial.println("Enabling gamepad");
    XboxOneSControllerDeviceConfiguration* config = new XboxOneSControllerDeviceConfiguration();
    auto hostConfig = config->getIdealHostConfiguration();  
    gamepad = new XboxGamepadDevice(config);
    
    // Set up vibration event handler
    FunctionSlot<XboxGamepadOutputReportData> vibrationSlot(OnVibrateEvent);
    gamepad->onVibrate.attach(vibrationSlot);

    composite.addDevice(gamepad);

    // Set up mouse
    //Serial.println("Enabling mouse");
    mouse = new MouseDevice();
    //composite.addDevice(mouse);   // Removed for linux dev environment

    // Set up composite HID
    Serial.println("Enabling composite HID");
    if(currentConnectionMode == ConnectionMode::Hub)
        composite.begin(hostConfig);

    // Set up ui
    // The ESP is capable of rendering 60fps in 80Mhz mode
    // but that won't give you much time for anything else
    // run it in 160Mhz mode or just set it to 30 fps
    Serial.println("Enabling UI");
    ui.setTargetFPS(30);

    ui.disableAutoTransition();

    // Customize the active and inactive symbol
    ui.setActiveSymbol(activeSymbol);
    ui.setInactiveSymbol(inactiveSymbol);

    // You can change this to
    // TOP, LEFT, BOTTOM, RIGHT
    ui.setIndicatorPosition(BOTTOM);

    // Defines where the first frame is located in the bar.
    ui.setIndicatorDirection(LEFT_RIGHT);

    // You can change the transition that is used
    // SLIDE_LEFT, SLIDE_RIGHT, SLIDE_UP, SLIDE_DOWN
    ui.setFrameAnimation(SLIDE_LEFT);

    // Add frames
    ui.setFrames(frames, frameCount);

    // Add overlays
    ui.setOverlays(overlays, overlaysCount);

    // Initialising the UI will init the display too.
    ui.init();

    // Enable satellite controller pairing
    if(currentConnectionMode == ConnectionMode::Spoke){
        Serial.println("Enabling spoke controller");
        spokeController = new Spoke();
        spokeController->init();
    } else {
        Serial.println("Enabling hub controller");
        hubController = new Hub();
        hubController->init();
    }
}

void updateLocalButtons(){
    XboxDpadFlags dPadDirection = XboxDpadFlags::NONE;

    if(!faceBtnNorthPressed && !MCP.read1(BUTTON_1_PIN)){
        faceBtnNorthPressed = true;
        dataUpdated = true;
        hapticPulse(160, 100);
        if(currentConnectionMode == ConnectionMode::Hub){
            if(currentControllerHand == ControllerHand::Left)
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection | (uint8_t)XboxDpadFlags::NORTH);
            else
                gamepad->press(XBOX_BUTTON_Y);
        }
    } 
    if (faceBtnNorthPressed && MCP.read1(BUTTON_1_PIN)) {
        faceBtnNorthPressed = false;
        dataUpdated = true;
        if(currentConnectionMode == ConnectionMode::Hub){
            if(currentControllerHand == ControllerHand::Left)
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection & ~(uint8_t)XboxDpadFlags::NORTH);
            else
                gamepad->release(XBOX_BUTTON_Y);
        }
    }

    if(!faceBtnEastPressed && !MCP.read1(BUTTON_2_PIN)){
        faceBtnEastPressed = true;
        dataUpdated = true;
        hapticPulse(160, 100);
        if(currentConnectionMode == ConnectionMode::Hub){
            if(currentControllerHand == ControllerHand::Left)
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection | (uint8_t)XboxDpadFlags::EAST);
            else
                gamepad->press(XBOX_BUTTON_B);
        }
    }  
    if (faceBtnEastPressed && MCP.read1(BUTTON_2_PIN)) {
        faceBtnEastPressed = false;
        dataUpdated = true;
        if(currentConnectionMode == ConnectionMode::Hub){
            if(currentControllerHand == ControllerHand::Left)
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection & ~(uint8_t)XboxDpadFlags::EAST);
            else
                gamepad->release(XBOX_BUTTON_B);
        }
    }

    if(!faceBtnSouthPressed && !MCP.read1(BUTTON_3_PIN)){
        faceBtnSouthPressed = true;
        dataUpdated = true;
        hapticPulse(160, 100);
        if(currentConnectionMode == ConnectionMode::Hub){
            if(currentControllerHand == ControllerHand::Left)
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection | (uint8_t)XboxDpadFlags::SOUTH);
            else
                gamepad->press(XBOX_BUTTON_A);
        }
    } 
    if (faceBtnSouthPressed && MCP.read1(BUTTON_3_PIN)) {
        faceBtnSouthPressed = false;
        dataUpdated = true;
        if(currentConnectionMode == ConnectionMode::Hub){
            if(currentControllerHand == ControllerHand::Left)
               dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection & ~(uint8_t)XboxDpadFlags::SOUTH);
            else
                gamepad->release(XBOX_BUTTON_A);
        }
    }

    if(!faceBtnWestPressed && !MCP.read1(BUTTON_4_PIN)){
        faceBtnWestPressed = true;
        dataUpdated = true;
        hapticPulse(160, 100);
        if(currentConnectionMode == ConnectionMode::Hub){
            if(currentControllerHand == ControllerHand::Left)
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection | (uint8_t)XboxDpadFlags::WEST);
            else
                gamepad->press(XBOX_BUTTON_X);
        }
    } 
    if (faceBtnWestPressed && MCP.read1(BUTTON_4_PIN)) {
        faceBtnWestPressed = false;
        dataUpdated = true;
        if(currentConnectionMode == ConnectionMode::Hub){
            if(currentControllerHand == ControllerHand::Left)
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection & ~(uint8_t)XboxDpadFlags::WEST);
            else
                gamepad->release(XBOX_BUTTON_X);
        }
    }

    // Update Dpad
    if(currentConnectionMode == ConnectionMode::Hub){
        if(currentControllerHand == ControllerHand::Left)
            gamepad->pressDPadDirectionFlag(dPadDirection);
    }
    
    if(!trackpadBtnPressed && !MCP.read1(BUTTON_5_PIN)){
        trackpadBtnPressed = true;
        dataUpdated = true;
        hapticPulse(160, 100);
        if(currentConnectionMode == ConnectionMode::Hub)
            gamepad->press((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_LS : XBOX_BUTTON_RS);
    } 
    if (trackpadBtnPressed && MCP.read1(BUTTON_5_PIN)) {
        trackpadBtnPressed = false;
        dataUpdated = true;
        if(currentConnectionMode == ConnectionMode::Hub)
            gamepad->release((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_LS : XBOX_BUTTON_RS);
    }

    if(!bumperPressed && !MCP.read1(BUTTON_6_PIN)){
        bumperPressed = true;
        dataUpdated = true;
        hapticPulse(160, 100);
        if(currentConnectionMode == ConnectionMode::Hub)
            gamepad->press((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_LB : XBOX_BUTTON_RB);
    }
    if (bumperPressed && MCP.read1(BUTTON_6_PIN)) {
        bumperPressed = false;
        dataUpdated = true;
        if(currentConnectionMode == ConnectionMode::Hub)
            gamepad->release((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_LB : XBOX_BUTTON_RB);
    }

    if(!menuBtnAPressed && !MCP.read1(BUTTON_7_PIN)){
        menuBtnAPressed = true;
        dataUpdated = true;
        hapticPulse(160, 100);
        if(currentConnectionMode == ConnectionMode::Hub)
            gamepad->press((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_SELECT : XBOX_BUTTON_START);
    } 
    if (menuBtnAPressed && MCP.read1(BUTTON_7_PIN)) {
        menuBtnAPressed = false;
        dataUpdated = true;
        if(currentConnectionMode == ConnectionMode::Hub)
            gamepad->release((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_SELECT : XBOX_BUTTON_START);
    }

    if(!menuBtnBPressed && !MCP.read1(BUTTON_8_PIN)){
        menuBtnBPressed = true;
        dataUpdated = true;
        hapticPulse(160, 100);
        if(currentConnectionMode == ConnectionMode::Hub)
            gamepad->press((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_HOME : XBOX_BUTTON_SHARE);
    }  
    if (menuBtnBPressed && MCP.read1(BUTTON_8_PIN)) {
        menuBtnBPressed = false;
        dataUpdated = true;
        if(currentConnectionMode == ConnectionMode::Hub)
            gamepad->release((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_HOME : XBOX_BUTTON_SHARE);
    }
}

void updateSatelliteButtons(){
    if(hubController){
        auto buttons = hubController->getLastReceivedSatelliteInput().buttons;
        XboxDpadFlags dPadDirection;

        // Press face button north
        if((buttons & SingleControllerButtons::faceNorth) == SingleControllerButtons::faceNorth){
            if(currentControllerHand == ControllerHand::Right){
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection | (uint8_t)XboxDpadFlags::NORTH);

            } else {
                if(!gamepad->isPressed(XBOX_BUTTON_Y)){
                    Serial.println("Pressing satellite Y button");
                    gamepad->press(XBOX_BUTTON_Y);
                }
            }
        }

        // Release face button north
        if((buttons & SingleControllerButtons::faceNorth) == 0){
            if(currentControllerHand == ControllerHand::Left){
                if(gamepad->isPressed(XBOX_BUTTON_Y)){
                    Serial.println("Releasing satellite Y button");
                    gamepad->release(XBOX_BUTTON_Y);
                }
            }
        }

        // Press face button east
        if((buttons & SingleControllerButtons::faceEast) == SingleControllerButtons::faceEast){
            if(currentControllerHand == ControllerHand::Right){
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection | (uint8_t)XboxDpadFlags::EAST);
            } else {
                if(!gamepad->isPressed(XBOX_BUTTON_B)){
                    Serial.println("Pressing satellite B button");
                    gamepad->press(XBOX_BUTTON_B);
                }
            }
        }

        // Release face button east
        if((buttons & SingleControllerButtons::faceEast) == 0){
            if(currentControllerHand == ControllerHand::Left){
                if(gamepad->isPressed(XBOX_BUTTON_B)){
                    Serial.println("Releasing satellite B button");
                    gamepad->release(XBOX_BUTTON_B);
                }
            }
        }

        // Press face button south
        if((buttons & SingleControllerButtons::faceSouth) == SingleControllerButtons::faceSouth){
            if(currentControllerHand == ControllerHand::Right){
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection | (uint8_t)XboxDpadFlags::SOUTH);
            } else {
                if(!gamepad->isPressed(XBOX_BUTTON_A)){
                    Serial.println("Pressing satellite A button");
                    gamepad->press(XBOX_BUTTON_A);
                }
            }
        }

        // Release face button south
        if((buttons & SingleControllerButtons::faceSouth) == 0){
            if(currentControllerHand == ControllerHand::Left){
                if(gamepad->isPressed(XBOX_BUTTON_A)){
                    Serial.println("Releasing satellite button A");
                    gamepad->release(XBOX_BUTTON_A);
                }
            }
        }

        // Press face button west
        if((buttons & SingleControllerButtons::faceWest) == SingleControllerButtons::faceWest){
            if(currentControllerHand == ControllerHand::Right){
                dPadDirection = (XboxDpadFlags)((uint8_t)dPadDirection | (uint8_t)XboxDpadFlags::WEST);
            } else {
                if(!gamepad->isPressed(XBOX_BUTTON_X)){
                    Serial.println("Pressing satellite X button");
                    gamepad->press(XBOX_BUTTON_X);
                }
            }
        }

        // Release face button west
        if((buttons & SingleControllerButtons::faceWest) == 0){
            if(currentControllerHand == ControllerHand::Left){
                if(gamepad->isPressed(XBOX_BUTTON_X)){
                    Serial.println("Releasing satellite X button");
                    gamepad->release(XBOX_BUTTON_X);
                }
            }
        }

        // Update Dpad
        if(currentControllerHand == ControllerHand::Right){
            gamepad->pressDPadDirectionFlag(dPadDirection);
        }

        // Press trackpad button
        if((buttons & SingleControllerButtons::trackpadBtn) == SingleControllerButtons::trackpadBtn){
            if(!gamepad->isPressed((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_LS : XBOX_BUTTON_RS)){
                Serial.println(String("Pressing satellite ") + String((currentControllerHand == ControllerHand::Right) ? "left stick" : "right stick"));
                gamepad->press((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_LS : XBOX_BUTTON_RS);
            }
        }

        // Release trackpad button
        if((buttons & SingleControllerButtons::trackpadBtn) == 0){
            if(gamepad->isPressed((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_LS : XBOX_BUTTON_RS)){
                Serial.println(String("Releasing satellite ") + String((currentControllerHand == ControllerHand::Right) ? "left stick" : "right stick"));
                gamepad->release((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_LS : XBOX_BUTTON_RS);
            }
        }

        // Press bumper
        if((buttons & SingleControllerButtons::bumper) == SingleControllerButtons::bumper){
            if(!gamepad->isPressed((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_LB : XBOX_BUTTON_RB)){
                Serial.println("Pressing satellite  " + String((currentControllerHand == ControllerHand::Right) ? "left bumper" : "right bumper"));
                gamepad->press((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_LB : XBOX_BUTTON_RB);
            }
        }

        // Release bumper
        if((buttons & SingleControllerButtons::bumper) == 0){
            if(gamepad->isPressed((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_LB : XBOX_BUTTON_RB)){
                Serial.println("Releasing satellite  " + String((currentControllerHand == ControllerHand::Right) ? "left bumper" : "right bumper"));
                gamepad->release((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_LB : XBOX_BUTTON_RB);
            }
        }

        // Press menu button A
        if((buttons & SingleControllerButtons::menuA) == SingleControllerButtons::menuA){
            if(!gamepad->isPressed((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_SELECT : XBOX_BUTTON_START)){
                Serial.println("Pressing satellite " + String((currentControllerHand == ControllerHand::Right) ? "select" : "start"));
                gamepad->press((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_SELECT : XBOX_BUTTON_START);
            }
        }

        // Release menu button A
        if((buttons & SingleControllerButtons::menuA) == 0){
            if(gamepad->isPressed((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_SELECT : XBOX_BUTTON_START)){
                Serial.println("Releasing satellite " + String((currentControllerHand == ControllerHand::Right) ? "select" : "start"));
                gamepad->release((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_SELECT : XBOX_BUTTON_START);
            }
        }

        // Press menu button B
        if((buttons & SingleControllerButtons::menuB) == SingleControllerButtons::menuB){
            if(!gamepad->isPressed((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_HOME : XBOX_BUTTON_SHARE)){
                Serial.println("Pressing satellite " + String((currentControllerHand == ControllerHand::Right) ? "home" : "share"));
                gamepad->press((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_HOME : XBOX_BUTTON_SHARE);
            }
        }

        // Release menu button B
        if((buttons & SingleControllerButtons::menuB) == 0){
            if(gamepad->isPressed((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_HOME : XBOX_BUTTON_SHARE)){
                Serial.println("Releasing satellite " + String((currentControllerHand == ControllerHand::Right) ? "home" : "share"));
                gamepad->release((currentControllerHand == ControllerHand::Right) ? XBOX_BUTTON_HOME : XBOX_BUTTON_SHARE);
            }
        }
    }
}

void updateLocalTrackpad(){
    if(trackpad.DataReady())
    {
        TrackpadRelativeData_t relData;
        TrackpadAbsoluteData_t absData;

        // Update serial debug
        if(millis() > refresh_time){
            if(currentTrackpadMode == TrackpadMode::Joystick)
            {
                trackpad.GetAbsolute(&absData);
                Serial.print("Absolute X: ");
                Serial.print(absData.xValue);
                Serial.print(" Y: ");
                Serial.println(absData.yValue);
                lastAbsData = absData;
            } else {
                trackpad.GetRelative(&relData);
                Serial.print("Relative X: ");
                Serial.print(relData.xDelta);
                Serial.print(" Y: ");
                Serial.println(relData.yDelta);
                lastRelData = relData;
            }

            switch(currentTrackpadMode) {
            case TrackpadMode::Mouse:
                mouse->mouseMove(relData.xDelta, relData.yDelta);
                break;
            case TrackpadMode::Joystick:
            {
                int x = mapint(absData.xValue, PINNACLE_X_LOWER, PINNACLE_X_UPPER, -32767, 32767);
                int y = mapint(absData.yValue, PINNACLE_Y_LOWER, PINNACLE_Y_UPPER, -32767, 32767);

                if(!absData.touchDown){
                    x = 0;
                    y = 0;
                }

                if(currentControllerHand == ControllerHand::Left)
                    gamepad->setLeftThumb(x, y);
                else
                    gamepad->setRightThumb(x, y);
                    break;
            }
            default:
                break;
            }
        
            refresh_time = millis() + SERIAL_REFRESH_TIME;
        }

        dataUpdated = true;
    }
}

void updateSatelliteTrackpad(){
    if(hubController){
        if(hubController->isDataReady()){
            int x = mapint(hubController->getLastReceivedSatelliteInput().absTrackpadData.xValue, PINNACLE_X_LOWER, PINNACLE_X_UPPER, -32767, 32767);
            int y = mapint(hubController->getLastReceivedSatelliteInput().absTrackpadData.yValue, PINNACLE_Y_LOWER, PINNACLE_Y_UPPER, -32767, 32767);

            if(currentTrackpadMode == TrackpadMode::Joystick){
                if(hubController->getLastReceivedSatelliteInput().absTrackpadData.touchDown){
                    Serial.print("Satellite Absolute X: ");
                    Serial.print(x);
                    Serial.print(" Y: ");
                    Serial.println(y);
                } else {
                    x = 0;
                    y = 0;
                }
            }

            // The satellite controller will be in the opposite hand to the hub controller
            if(currentControllerHand == ControllerHand::Left){
                gamepad->setRightThumb(x, y);
            } else {
                gamepad->setLeftThumb(x, y);
            }
        }
    }
}

void updateLocalTrigger(){
    // Update trigger
    triggerValue = get_trigger_value(TRIGGER_PIN, triggerKalmanFilter);
    if(triggerValue > lastTriggerValue + 0.01 || triggerValue < lastTriggerValue - 0.01){
        satelliteTriggerTouched = true;
        Serial.println("Trigger: " + String(triggerValue));
        int16_t mapped_trigger = constrain(int16_t(triggerValue * XBOX_TRIGGER_MAX), 0, XBOX_TRIGGER_MAX);
        if(currentControllerHand == ControllerHand::Left)
            gamepad->setLeftTrigger(mapped_trigger);
        else
            gamepad->setRightTrigger(mapped_trigger);
        lastTriggerValue = triggerValue;
        dataUpdated = true;
    } else {
        if(satelliteTriggerTouched){
            satelliteTriggerTouched = false;
            dataUpdated = true;
        }
    }
}

void updateSatelliteTrigger(){
    if(hubController){
        if((hubController->getLastReceivedSatelliteInput().buttons & SingleControllerButtons::triggerTouched) == SingleControllerButtons::triggerTouched){
            auto triggerValue = hubController->getLastReceivedSatelliteInput().trigger;
            Serial.println("Satellite trigger touched: " + String(triggerValue));
            if(currentControllerHand == ControllerHand::Right)
                gamepad->setLeftTrigger(triggerValue);
            else
                gamepad->setRightTrigger(triggerValue);
        }
    }
}

void loop() {
    // Get IMU values
    // if (imu.wasReset()) {
    //     Serial.print("sensor was reset ");
    //     imu.enableReport(SH2_ARVR_STABILIZED_RV, 5000);
    // }

    // if (imu.getSensorEvent(&imuValue)) {
    //     quaternionToEulerRV(&imuValue.un.arvrStabilizedRV, &ypr, true);
    //     Serial.println("Yaw: " + String(ypr.yaw) + " Pitch: " + String(ypr.pitch) + " Roll: " + String(ypr.roll));
    // }

    updateLocalButtons();
    updateLocalTrackpad();
    updateLocalTrigger();

    if(currentConnectionMode == ConnectionMode::Hub)
    {
        if(hubController){
            hubController->update();
            updateSatelliteButtons();
            updateSatelliteTrackpad();
            updateSatelliteTrigger();
            hubController->consumeInputData();
        }
        
    } else {
        if(spokeController){
            if(dataUpdated)
                spokeController->notifyHub(getPackedGamepadState());    
        }
    }
    
    // Update the screen
    ui.update();

    dataUpdated = false;

    delay(8);
}

// Toggles the haptic drive transistor every time it's called
void hapticTimerInterrupt(bool* hapticState)
{
    *hapticState = !(*hapticState);
    digitalWrite(HAPTIC_PIN, *hapticState);
}

// Create a pulse at <freq> for <duration> milliseconds
void hapticPulse(uint16_t freq, uint16_t duration)
{
  hapticStart(freq);
  delay(duration);
  hapticStop();
}

// Start driving the haptic at <freq> Hz
void hapticStart(uint32_t freq)
{
    hapticTimer.detach();
    hapticState = false;
    float half_interval = 0.5f / freq;
    hapticTimer.attach(half_interval, hapticTimerInterrupt, &hapticState);
}

// Stop driving the haptic
void hapticStop(void)
{
  hapticTimer.detach();
  digitalWrite(HAPTIC_PIN, LOW);
  hapticState = false;
}

PackedGamepadInputState getPackedGamepadState(){
    PackedGamepadInputState state;

    if(faceBtnNorthPressed)
        state.buttons |= SingleControllerButtons::faceNorth;
    else
        state.buttons &= ~SingleControllerButtons::faceNorth;

    if(faceBtnEastPressed)
        state.buttons |= SingleControllerButtons::faceEast;
    else
        state.buttons &= ~SingleControllerButtons::faceEast;

    if(faceBtnSouthPressed)
        state.buttons |= SingleControllerButtons::faceSouth;
    else
        state.buttons &= ~SingleControllerButtons::faceSouth;

    if(faceBtnWestPressed)
        state.buttons |= SingleControllerButtons::faceWest;
    else
        state.buttons &= ~SingleControllerButtons::faceWest;

    if(trackpadBtnPressed)
        state.buttons |= SingleControllerButtons::trackpadBtn;
    else
        state.buttons &= ~SingleControllerButtons::trackpadBtn;

    if(bumperPressed)
        state.buttons |= SingleControllerButtons::bumper;
    else
        state.buttons &= ~SingleControllerButtons::bumper;
    
    if(menuBtnAPressed)
        state.buttons |= SingleControllerButtons::menuA;
    else
        state.buttons &= ~SingleControllerButtons::menuA;
    
    if(menuBtnBPressed)
        state.buttons |= SingleControllerButtons::menuB;
    else
        state.buttons &= ~SingleControllerButtons::menuB;

    if(lastAbsData.touchDown)
        state.buttons |= SingleControllerButtons::trackpadTouched;
    else
        state.buttons &= ~SingleControllerButtons::trackpadTouched;

    if(satelliteTriggerTouched)
        state.buttons |= SingleControllerButtons::triggerTouched;
    else
        state.buttons &= ~SingleControllerButtons::triggerTouched;

    // state.x = lastAbsData.xValue;
    // state.y = lastAbsData.yValue;
    // state.z = lastAbsData.zValue;
    state.absTrackpadData = lastAbsData;
    state.trigger = constrain(int16_t(triggerValue * XBOX_TRIGGER_MAX), 0, XBOX_TRIGGER_MAX);

    return state;
}