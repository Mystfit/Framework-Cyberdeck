#pragma once
#include <stdint.h>
#include <Ticker.h>
#include <Arduino.h>

#define HORIZ_FREQ 160
#define VERT_FREQ 320

class Haptics {
public:
    Haptics(uint8_t hapticPin);

    // Create a pulse at <freq> for <duration> milliseconds
    void hapticStartHorizontal(int duration = -1);
    void hapticStartVertical(int duration = -1);
    
    // Stop driving the haptic
    void hapticStopHorizontal();
    void hapticStopVertical();
    void hapticsStopAll();

private:
    // Toggles the haptic drive transistor every time it's called
    //static void hapticTimerInterrupt(bool* hapticState);
    static void hapticTimerInterrupt160(Haptics* instance);
    static void hapticTimerInterrupt320(Haptics* instance);
    
    uint8_t _hapticPin;
    Ticker _hapticTimer160;
    Ticker _hapticTimer320;
    int _hapticState160;
    int _hapticState320;

    int _horizEndTime;
    int _vertEndTime;
};
