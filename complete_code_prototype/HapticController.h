#ifndef HAPTIC_CONTROLLER_H
#define HAPTIC_CONTROLLER_H

#include <Arduino.h>
#include "SafetyLogic.h"

class HapticController {
private:
  uint8_t leftPin;
  uint8_t rightPin;
  unsigned long lastPulseTime;
  bool state;

public:
  HapticController(uint8_t pinL, uint8_t pinR);
  void begin();
  void update(const HazardAssessment& hazard);
  void stopAll();
};

#endif
