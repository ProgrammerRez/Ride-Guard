#include "SafetyLogic.h"
#include "config.h"
#include <math.h>

SafetyLogic::SafetyLogic() {}

TurnState SafetyLogic::evaluateTurnState(float rollAngleDeg) {
  if (rollAngleDeg > LEAN_THRESHOLD_DEG) {
    return TURNING_RIGHT;
  } else if (rollAngleDeg < -LEAN_THRESHOLD_DEG) {
    return TURNING_LEFT;
  }
  return STRAIGHT;
}

HazardAssessment SafetyLogic::assessThreat(float pitchDeg, float rollDeg, float targetDistM, float targetSpeedKmh, float targetAngleDeg) {
  HazardAssessment result;
  result.bikeState = evaluateTurnState(rollDeg);
  result.threatLevel = SAFE;
  result.warnLeft = false;
  result.warnRight = false;

  if (targetDistM <= 0.01f) {
    return result; // Invalid distance measurement
  }

  // Calculate Time-To-Collision (TTC) if target is approaching (positive relative speed)
  float speedMps = targetSpeedKmh / 3.6f;
  float ttc = (speedMps > 0.1f) ? (targetDistM / speedMps) : 999.0f;

  // Primary Threat Escalation
  if (targetDistM < DANGER_DIST_CLOSE_M || ttc < 1.2f) {
    result.threatLevel = CRITICAL;
  } else if (targetDistM < DANGER_DIST_FAR_M || ttc < TTC_WARN_SEC) {
    result.threatLevel = WARNING;
  }

  // Turn vs. Blind-Spot Danger Vectoring
  if (result.threatLevel != SAFE) {
    if (result.bikeState == TURNING_LEFT) {
      // Vehicle in apex or blind spot while turning left
      result.warnLeft = true;
      if (targetAngleDeg > 0.0f || result.threatLevel == CRITICAL) result.warnRight = true;
    } 
    else if (result.bikeState == TURNING_RIGHT) {
      // Vehicle in apex or blind spot while turning right
      result.warnRight = true;
      if (targetAngleDeg < 0.0f || result.threatLevel == CRITICAL) result.warnLeft = true;
    } 
    else {
      // Straight motion spatial warning
      if (targetAngleDeg < 0.0f) result.warnLeft = true;
      else if (targetAngleDeg > 0.0f) result.warnRight = true;
      else {
        result.warnLeft = true;
        result.warnRight = true;
      }
    }
  }

  return result;
}
