#include <Arduino.h>
#include <Ticker.h>

#define HAPTIC_PIN 17

// Haptics
Ticker hapticTimer;
bool hapticState = false;

void setup(){
    Serial.begin(115200);
    pinMode(HAPTIC_PIN, OUTPUT);
    delay(100);
}

void loop(){
    hapticPulse(160, 1000);
    delay(1000);

    hapticPulse(320, 1000);
    delay(1000);
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
