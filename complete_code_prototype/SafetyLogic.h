#ifndef SAFETY_LOGIC_H
#define SAFETY_LOGIC_H

#include <Arduino.h>

enum TurnState {
  STRAIGHT,
  TURNING_LEFT,
  TURNING_RIGHT
};

enum DangerLevel {
  SAFE,
  WARNING,
  CRITICAL
};

struct HazardAssessment {
  TurnState bikeState;
  DangerLevel threatLevel;
  bool warnLeft;
  bool warnRight;
};

class SafetyLogic {
public:
  SafetyLogic();
  TurnState evaluateTurnState(float rollAngleDeg);
  HazardAssessment assessThreat(float pitchDeg, float rollDeg, float targetDistM, float targetSpeedKmh, float targetAngleDeg);
};

#endif
