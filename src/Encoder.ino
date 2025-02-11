#include "Encoder.h"

portMUX_TYPE Encoder::mux = portMUX_INITIALIZER_UNLOCKED;
volatile unsigned long Encoder::count = 0;
volatile unsigned long Encoder::lastEdgeTime = 0;
volatile unsigned long Encoder::debounceTime = 20000UL; // Start with MAX_DEBOUNCE
unsigned long Encoder::MIN_DEBOUNCE = 8000UL;
unsigned long Encoder::MAX_DEBOUNCE = 20000UL;

Encoder::Encoder(uint8_t encoderPin, int slotsPerRev, int intervaluser,
                unsigned long minDebounce, unsigned long maxDebounce)
    : encoderPin(encoderPin), slotsPerRev(slotsPerRev),interval(intervaluser),
      Lasttime(0) {
    MIN_DEBOUNCE = minDebounce;
    MAX_DEBOUNCE = maxDebounce;
}

void Encoder::begin() {
    pinMode(encoderPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);
    lastEdgeTime = micros();
    Lasttime=millis();
    
}

float Encoder::getRPM() {
    unsigned long currentCount;
    unsigned long currentTime = millis();
    float rpm = 0;

    if (currentTime-Lasttime>=interval){
    portENTER_CRITICAL(&mux);
    currentCount = count;
    count = 0;
    rpm = (float)((((float)currentCount / slotsPerRev) * 60.0 )/((float)interval/1000));
    Lasttime = currentTime;
    portEXIT_CRITICAL(&mux);

    }
    return rpm;
}
void IRAM_ATTR Encoder::encoderISR() {
    unsigned long currentMicros = micros();
    unsigned long delta = currentMicros - lastEdgeTime;

    if (delta >= debounceTime) {
        count++;
        unsigned long newDebounce = delta * 4 / 5;
        
        // Constrain debounce time
        if (newDebounce < MIN_DEBOUNCE) {
            debounceTime = MIN_DEBOUNCE;
        } else if (newDebounce > MAX_DEBOUNCE) {
            debounceTime = MAX_DEBOUNCE;
        } else {
            debounceTime = newDebounce;
        }
        
        lastEdgeTime = currentMicros;
    }
}