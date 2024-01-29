#include <Arduino.h>
#include <Ticker.h>

#include "CirqueTrackpad.h"

#include <MouseDevice.h>
#include <XboxGamepadDevice.h>
#include <BleCompositeHID.h>

#include "MCP23S17.h"
#include "SSD1306.h"
#include "OLEDDisplayUi.h"

#define MCP_CS_PIN 33
#define TRACKPAD_CS_PIN    32
#define TRACKPAD_DR_PIN    34

#define BUTTON_1_PIN 8
#define BUTTON_2_PIN 9
#define BUTTON_3_PIN 10
#define BUTTON_4_PIN 11
#define BUTTON_5_PIN 12
#define BUTTON_6_PIN 13
#define BUTTON_7_PIN 14
#define BUTTON_8_PIN 15

#define HAPTIC_PIN 17


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

TrackpadRelativeData_t lastRelData;
TrackpadAbsoluteData_t lastAbsData;

// Serial output refresh time
const long SERIAL_REFRESH_TIME = 8;
long refresh_time;

enum class TrackpadMode {
    Mouse = 0,
    Joystick
};

enum class ControllerHand {
    Left = 0,
    Right
};

// Input devices
MCP23S17 MCP(MCP_CS_PIN);
CirqueTrackpad trackpad(TRACKPAD_CS_PIN, TRACKPAD_DR_PIN);

TrackpadMode currentTrackpadMode = TrackpadMode::Joystick;
ControllerHand currentControllerHand = ControllerHand::Left;
bool button1Pressed = false;
bool button2Pressed = false;
bool button3Pressed = false;
bool button4Pressed = false;
bool button5Pressed = false;
bool button6Pressed = false;
bool button7Pressed = false;
bool button8Pressed = false;

// Output devices
XboxGamepadDevice* gamepad = nullptr;
MouseDevice* mouse = nullptr;
BleCompositeHID composite("PicaTTY Controller", "Mystfit", 100);

// Haptics
Ticker hapticTimer;
bool hapticState = false;

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

void drawJoystick(OLEDDisplay *display, OLEDDisplayUiState* state, int16_t x, int16_t y) {
    display->setColor(WHITE);

    int circX = mapint(lastAbsData.xValue, PINNACLE_X_LOWER, PINNACLE_X_UPPER, 32 + 8, 96 - 8);
    int circY = mapint(lastAbsData.yValue, PINNACLE_Y_LOWER, PINNACLE_Y_UPPER, 16, 48);

    if(lastAbsData.touchDown)
        display->fillCircle(circX + x, circY + y, 16);
    else
        display->drawCircle(64 + x, 32 + y, 16);

    display->drawCircle(64 + x, 32 + x,  32);

    String xPrefix = "X:";
    String yPrefix = "Y:";
    String xStr = String(lastAbsData.xValue);
    String yStr = String(lastAbsData.yValue);

    // for(size_t charCount = 4 - xStr.length() + 1; charCount > 0; charCount--){
    //     xPrefix += " ";
    // }
    // for(size_t charCount = 4 - yStr.length() + 1; charCount >=0; charCount--){
    //     yPrefix += " ";
    // }
    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->setFont(ArialMT_Plain_10);
    display->drawString(128, 0, xPrefix + xStr);
    display->drawString(128, 12, yPrefix + yStr);
}

FrameCallback frames[] = { drawJoystick };
int frameCount = 1;


void buttonOverlay(OLEDDisplay *display, OLEDDisplayUiState* state) {
    display->setColor(WHITE);
    if (button1Pressed){
        display->fillCircle(5, 56, 2);
    } else {
        display->drawCircle(5, 56, 2);
    }

     if (button2Pressed){
        display->fillCircle(10, 56, 2);
    } else {
        display->drawCircle(10, 56, 2);
    }

    if (button3Pressed){
        display->fillCircle(15, 56, 2);
    } else {
        display->drawCircle(15, 56, 2);
    }

    if (button4Pressed){
        display->fillCircle(20, 56, 2);
    } else {
        display->drawCircle(20, 56, 2);
    }

    if (button5Pressed){
        display->fillCircle(5, 60, 2);
    } else {
        display->drawCircle(5, 60, 2);
    }

    if (button6Pressed){
        display->fillCircle(10, 60, 2);
    } else {
        display->drawCircle(10, 60, 2);
    }

    if (button7Pressed){
        display->fillCircle(15, 60, 2);
    } else {
        display->drawCircle(15, 60, 2);
    }

    if (button8Pressed){
        display->fillCircle(20, 60, 2);
    } else {
        display->drawCircle(20, 60, 2);
    }
}
OverlayCallback overlays[] = { buttonOverlay };
int overlaysCount = 1;


void setup() {
    Serial.begin(115200);

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

    // Set up haptics
    Serial.println("Enabling haptic");
    pinMode(HAPTIC_PIN, OUTPUT);

    // Set up trackpad
    Serial.println("Enabling trackpad");
    trackpad.Init();
    trackpad.EnableFeed(true);

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
    composite.addDevice(gamepad);

    // Set up mouse
    //Serial.println("Enabling mouse");
    mouse = new MouseDevice();
    //composite.addDevice(mouse);   // Removed for linux dev environment

    // Set up composite HID
    Serial.println("Enabling composite HID");
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
}

void loop() {
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
    }

    button1Pressed = !MCP.read1(BUTTON_1_PIN);
    button2Pressed = !MCP.read1(BUTTON_2_PIN);
    button3Pressed = !MCP.read1(BUTTON_3_PIN);
    button4Pressed = !MCP.read1(BUTTON_4_PIN);
    button5Pressed = !MCP.read1(BUTTON_5_PIN);
    button6Pressed = !MCP.read1(BUTTON_6_PIN);
    button7Pressed = !MCP.read1(BUTTON_7_PIN);
    button8Pressed = !MCP.read1(BUTTON_8_PIN);

    if(!button1Pressed && !MCP.read1(BUTTON_1_PIN)){
        button1Pressed = true;
        hapticPulse(160, 100);
        if(currentControllerHand == ControllerHand::Left)
            gamepad->pressDPadDirection(XBOX_BUTTON_DPAD_SOUTH);
        else
            gamepad->press(XBOX_BUTTON_A);
    } 
    if (button1Pressed && MCP.read1(BUTTON_1_PIN)) {
        button1Pressed = false;
        if(currentControllerHand == ControllerHand::Left)
            gamepad->releaseDPadDirection(XBOX_BUTTON_DPAD_SOUTH);
        else
            gamepad->release(XBOX_BUTTON_A);
    }

    if(!button2Pressed && !MCP.read1(BUTTON_2_PIN)){
        button2Pressed = true;
        hapticPulse(160, 100);
        if(currentControllerHand == ControllerHand::Left)
            gamepad->pressDPadDirection(XBOX_BUTTON_DPAD_WEST);
        else
            gamepad->press(XBOX_BUTTON_X);
    }  
    if (button2Pressed && MCP.read1(BUTTON_2_PIN)) {
        button2Pressed = false;
        if(currentControllerHand == ControllerHand::Left)
            gamepad->releaseDPadDirection(XBOX_BUTTON_DPAD_WEST);
        else
            gamepad->release(XBOX_BUTTON_X);
    }

    if(!button3Pressed && !MCP.read1(BUTTON_3_PIN)){
        button3Pressed = true;
        hapticPulse(160, 100);
        if(currentControllerHand == ControllerHand::Left)
            gamepad->pressDPadDirection(XBOX_BUTTON_DPAD_EAST);
        else
            gamepad->press(XBOX_BUTTON_B);
    } 
    if (button3Pressed && MCP.read1(BUTTON_3_PIN)) {
        button3Pressed = false;
        if(currentControllerHand == ControllerHand::Left)
            gamepad->releaseDPadDirection(XBOX_BUTTON_DPAD_EAST);
        else
            gamepad->release(XBOX_BUTTON_B);
    }

    if(!button4Pressed && !MCP.read1(BUTTON_4_PIN)){
        button4Pressed = true;
        hapticPulse(160, 100);
        if(currentControllerHand == ControllerHand::Left)
            gamepad->pressDPadDirection(XBOX_BUTTON_DPAD_NORTH);
        else
            gamepad->press(XBOX_BUTTON_Y);
    } 
    if (button4Pressed && MCP.read1(BUTTON_4_PIN)) {
        button4Pressed = false;
        if(currentControllerHand == ControllerHand::Left)
            gamepad->releaseDPadDirection(XBOX_BUTTON_DPAD_NORTH);
        else
            gamepad->release(XBOX_BUTTON_Y);
    }

    if(!button5Pressed && !MCP.read1(BUTTON_5_PIN)){
        button5Pressed = true;
        hapticPulse(160, 100);
        gamepad->press((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_LB : XBOX_BUTTON_RB);
    } 
    if (button5Pressed && MCP.read1(BUTTON_5_PIN)) {
        button5Pressed = false;
        gamepad->release((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_LB : XBOX_BUTTON_RB);
    }

    if(!button6Pressed && !MCP.read1(BUTTON_6_PIN)){
        button6Pressed = true;
        hapticPulse(160, 100);
        gamepad->press((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_LS : XBOX_BUTTON_RS);
    }
    if (button6Pressed && MCP.read1(BUTTON_6_PIN)) {
        button6Pressed = false;
        gamepad->release((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_LS : XBOX_BUTTON_RS);
    }

    if(!button7Pressed && !MCP.read1(BUTTON_7_PIN)){
        button7Pressed = true;
        hapticPulse(160, 100);
        gamepad->press((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_SELECT : XBOX_BUTTON_START);
    } 
    if (button7Pressed && MCP.read1(BUTTON_7_PIN)) {
        button7Pressed = false;
        gamepad->release((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_SELECT : XBOX_BUTTON_START);
    }

    if(!button8Pressed && !MCP.read1(BUTTON_8_PIN)){
        button8Pressed = true;
        hapticPulse(160, 100);
        gamepad->press((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_HOME : XBOX_BUTTON_SHARE);
    }  
    if (button8Pressed && MCP.read1(BUTTON_8_PIN)) {
        button8Pressed = false;
        gamepad->release((currentControllerHand == ControllerHand::Left) ? XBOX_BUTTON_HOME : XBOX_BUTTON_SHARE);
    }

    //gamepad->setLeftTrigger(button6Pressed ? 32767 : 0);
    

/*
    if(MCP.read1(BUTTON_1_PIN) == LOW)
    {
        button1Pressed = true;
        if(currentTrackpadMode == TrackpadMode::Joystick)
        {
            currentTrackpadMode = TrackpadMode::Mouse;
            trackpad.SetRelativeMode();
        }
        else
        {
            currentTrackpadMode = TrackpadMode::Joystick;
            trackpad.SetAbsoluteMode();
        }
    } else {
        button1Pressed = false;
    }

    if(MCP.read1(BUTTON_2_PIN) == LOW)
    {
        button2Pressed = true;
        if(currentControllerHand == ControllerHand::Left)
            currentControllerHand = ControllerHand::Right;
        else
            currentControllerHand = ControllerHand::Left;
    } else {
        button2Pressed = false;
    }

    if(MCP.read1(BUTTON_3_PIN) == LOW)
    {
        button3Pressed = true;
    } else {
        button3Pressed = false;
    }
*/
    // Update the screen
    ui.update();
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