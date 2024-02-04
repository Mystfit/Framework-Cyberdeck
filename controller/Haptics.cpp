#include "Haptics.h"

#define LED_PIN 5

Haptics::Haptics(uint8_t hapticPin) :
    _hapticPin(hapticPin),
    _hapticState160(0),
    _hapticState320(0),
    _horizEndTime(-1),
    _vertEndTime(-1)
{
    pinMode(LED_PIN, OUTPUT);
    pinMode(_hapticPin, OUTPUT);
    digitalWrite(_hapticPin, LOW);
}

// Toggles the haptic drive transistor every time it's called
void Haptics::hapticTimerInterrupt160(Haptics* instance)
{
    if(instance->_horizEndTime > -1 && millis() > instance->_horizEndTime)
    {
        instance->hapticStopHorizontal();
        return;
    }

    instance->_hapticState160 = (instance->_hapticState160 > 0) ? -1 : 1;
    digitalWrite(instance->_hapticPin, instance->_hapticState160 > 0);
    digitalWrite(LED_PIN, (instance->_hapticState160 > 0) ? HIGH : LOW);
}

// Toggles the haptic drive transistor every time it's called
void Haptics::hapticTimerInterrupt320(Haptics* instance)
{
    if(instance->_vertEndTime > -1 && millis() > instance->_vertEndTime)
    {
        instance->hapticStopVertical();
        return;
    }

    instance->_hapticState320 = (instance->_hapticState320 > 0) ? -1 : 1;
    digitalWrite(instance->_hapticPin, instance->_hapticState320 > 0);
    digitalWrite(LED_PIN, (instance->_hapticState320 > 0) ? HIGH : LOW);
}

// Create a pulse at <freq> for <duration> milliseconds
void Haptics::hapticStartHorizontal(int duration)
{
    if(duration > -1)
        _horizEndTime = millis() + duration;
    
    //_hapticTimer160.detach();
    //_hapticState160 = false;
    _hapticTimer160.attach(0.5f / HORIZ_FREQ, Haptics::hapticTimerInterrupt160, this);
}

// Create a pulse at <freq> for <duration> milliseconds
void Haptics::hapticStartVertical(int duration)
{
    if(duration > -1)
        _vertEndTime = millis() + duration;
    
    //_hapticTimer320.detach();
    //_hapticState320 = 0;
    _hapticTimer320.attach(0.5 / VERT_FREQ, Haptics::hapticTimerInterrupt320, this);
}

// Stop driving the haptic
void Haptics::hapticStopHorizontal()
{
    _hapticTimer160.detach();
    _hapticState160 = 0;
    _horizEndTime = -1;

    bool pinState = ((_hapticState160 > 0) ? HIGH : LOW) | ((_hapticState320 > 0) ? HIGH : LOW);
    digitalWrite(_hapticPin, pinState);
    digitalWrite(LED_PIN, !pinState);
}

// Stop driving the haptic
void Haptics::hapticStopVertical()
{
    _hapticTimer320.detach();
    _hapticState320 = 0;
    _vertEndTime = -1;

    bool pinState = ((_hapticState160 > 0) ? HIGH : LOW) | ((_hapticState320 > 0) ? HIGH : LOW);
    digitalWrite(_hapticPin, pinState);
    digitalWrite(LED_PIN, !pinState);
}

void Haptics::hapticsStopAll()
{
    hapticStopHorizontal();
    hapticStopVertical();
}
