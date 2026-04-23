#pragma once
#include <Arduino.h>
#include "sensor.h"

// ==========================================
// ACTIVITY SOURCE ENUM
// ==========================================
enum ActivitySource {
  SOURCE_UNKNOWN = 0,
  SOURCE_AUTOMATION = 1,
  SOURCE_MQTT_REMOTE = 2,
  SOURCE_PHYSICAL_REMOTE = 3,
  SOURCE_SYSTEM = 4
};

// ==========================================
// ACTIVITY TYPE ENUM
// ==========================================
enum ActivityType {
  ACT_POWER_ON = 1,
  ACT_POWER_OFF = 2,
  ACT_MODE_CHANGE = 3,
  ACT_FAN_CHANGE = 4,
  ACT_TEMP_CHANGE = 5,
  ACT_MANUAL_OVERRIDE = 6,
  ACT_AUTO_RECOVERY = 7,
  ACT_ERROR_CHANGE = 8
};

// ==========================================
// ACTIVITY EVENT STRUCT
// ==========================================
struct ActivityEvent {
  String timestamp;             // WIB formatted datetime
  ActivityType type;
  ActivitySource source;
  
  // Old & New States
  bool old_power;
  bool new_power;
  
  String old_mode;
  String new_mode;
  
  String old_fan;
  String new_fan;
  
  int old_temp;
  int new_temp;
  
  String old_error;
  String new_error;
  
  String description;           // Human-readable description
};

// ==========================================
// PUBLIC FUNCTIONS
// ==========================================

/**
 * Initialize activity tracker
 */
void activityInit();

/**
 * Track state change (called from automation.cpp & data.cpp)
 */
void activityTrackChange(
  const BPFrame& oldState, 
  const BPFrame& newState, 
  ActivitySource source
);

/**
 * Track power change explicitly
 */
void activityTrackPower(
  bool oldPower, 
  bool newPower, 
  ActivitySource source
);

/**
 * Track mode change explicitly
 */
void activityTrackMode(
  const String& oldMode, 
  const String& newMode, 
  ActivitySource source
);

/**
 * Track fan change explicitly
 */
void activityTrackFan(
  const String& oldFan, 
  const String& newFan, 
  ActivitySource source
);

/**
 * Track temperature change explicitly (optional)
 */
void activityTrackTemp(
  int oldTemp,
  int newTemp,
  ActivitySource source
);

/**
 * Track error code change
 */
void activityTrackError(
  const String& oldError,
  const String& newError
);

/**
 * Get last known state for tracking
 */
BPFrame activityGetLastKnownState();

/**
 * Track manual override event
 */
void activityTrackManualOverride();

/**
 * Track auto-recovery event
 */
void activityTrackAutoRecovery();

/**
 * Get source name as string
 */
String activitySourceToString(ActivitySource src);

/**
 * Get type name as string
 */
String activityTypeToString(ActivityType type);