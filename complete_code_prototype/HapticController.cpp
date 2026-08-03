#include "HapticController.h"

HapticController::HapticController(uint8_t pinL, uint8_t pinR) 
  : leftPin(pinL), rightPin(pinR), lastPulseTime(0), state(false) {}

void HapticController::begin() {
  pinMode(leftPin, OUTPUT);
  pinMode(rightPin, OUTPUT);
  stopAll();
}

void HapticController::stopAll() {
  digitalWrite(leftPin, LOW);
  digitalWrite(rightPin, LOW);
  state = false;
}

void HapticController::update(const HazardAssessment& hazard) {
  if (hazard.threatLevel == SAFE) {
    stopAll();
    return;
  }

  unsigned long now = millis();
  uint16_t pulseInterval = (hazard.threatLevel == CRITICAL) ? 90 : 250; // Fast vs standard pulse

  if (now - lastPulseTime >= pulseInterval) {
    lastPulseTime = now;
    state = !state;

    digitalWrite(leftPin, (hazard.warnLeft && state) ? HIGH : LOW);
    digitalWrite(rightPin, (hazard.warnRight && state) ? HIGH : LOW);
  }
}
