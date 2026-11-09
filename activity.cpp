#include "activity.h"
#include "config.h"
#include "data.h"

#include <ArduinoJson.h>
#include <time.h>

// ==========================================
// GLOBAL STATE TRACKING
// ==========================================
static BPFrame lastKnownState = {};
static bool stateInitialized = false;

// Setpoint temperature (comes from DisplayFrame, not BPFrame)
static int lastKnownSetTemp = -1;

// ==========================================
// TOPIC
// ==========================================
static String topicActivity;

// ==========================================
// HELPER: BUILD TOPIC
// ==========================================
void activityBuildTopic(const String& device_id) {
  topicActivity = "isems/devices/" + device_id + "/activity";
  Serial.Printf("[Activity] Topic: %s\n", topicActivity.c_str());
}

// ==========================================
// INIT
// ==========================================
void activityInit() {
  activityBuildTopic(cfg.device_id);
  
  // Initialize state from sensor
  BPFrame currentFrame = sensorGetLastFrame();
  if (sensorIsHealthy()) {
    lastKnownState = currentFrame;
    stateInitialized = true;
    Serial.Println("[Activity] Initialized with current AC state");
  } else {
    Serial.Println("[Activity] Waiting for valid sensor frame...");
  }

  if (sensorIsDisplayHealthy()) {
    DisplayFrame disp = sensorGetLastDisplayFrame();
    lastKnownSetTemp = (int)disp.set_temp;
  }
}

// ==========================================
// HELPER: GET TIMESTAMP
// ==========================================
String getTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    char buf[32];
    snprintf(buf, sizeof(buf), "UPTIME-%lus", millis() / 1000);
    return String(buf);
  }
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S WIB", &timeinfo);
  return String(buf);
}

// ==========================================
// HELPER: SOURCE TO STRING
// ==========================================
String activitySourceToString(ActivitySource src) {
  switch (src) {
    case SOURCE_AUTOMATION: return "automation";
    case SOURCE_MQTT_REMOTE: return "mqtt_remote";
    case SOURCE_PHYSICAL_REMOTE: return "physical_remote";
    case SOURCE_SYSTEM: return "system";
    default: return "unknown";
  }
}

// ==========================================
// HELPER: TYPE TO STRING
// ==========================================
String activityTypeToString(ActivityType type) {
  switch (type) {
    case ACT_POWER_ON: return "power_on";
    case ACT_POWER_OFF: return "power_off";
    case ACT_MODE_CHANGE: return "mode_change";
    case ACT_FAN_CHANGE: return "fan_change";
    case ACT_TEMP_CHANGE: return "temp_change";
    case ACT_MANUAL_OVERRIDE: return "manual_override";
    case ACT_AUTO_RECOVERY: return "auto_recovery";
    case ACT_ERROR_CHANGE: return "error_change";
    default: return "unknown";
  }
}

// ==========================================
// HELPER: PUBLISH ACTIVITY EVENT
// ==========================================
static void publishActivity(const ActivityEvent& event) {
  if (!dataIsConnected()) {
    Serial.Println("[Activity] Cannot publish - MQTT not connected");
    return;
  }
  
  StaticJsonDocument<512> doc;
  
  doc["timestamp"] = event.timestamp;
  doc["type"] = activityTypeToString(event.type);
  doc["source"] = activitySourceToString(event.source);
  
  // State changes
  JsonObject changes = doc.createNestedObject("changes");
  
  if (event.type == ACT_POWER_ON || event.type == ACT_POWER_OFF) {
    changes["power"]["old"] = event.old_power;
    changes["power"]["new"] = event.new_power;
  }
  
  if (event.type == ACT_MODE_CHANGE) {
    changes["mode"]["old"] = event.old_mode;
    changes["mode"]["new"] = event.new_mode;
  }
  
  if (event.type == ACT_FAN_CHANGE) {
    changes["fan"]["old"] = event.old_fan;
    changes["fan"]["new"] = event.new_fan;
  }
  
  if (event.type == ACT_TEMP_CHANGE) {
    changes["set_temp"]["old"] = event.old_temp;
    changes["set_temp"]["new"] = event.new_temp;
  }

  if (event.type == ACT_ERROR_CHANGE) {
    changes["error"]["old"] = event.old_error;
    changes["error"]["new"] = event.new_error;
  }
  
  doc["description"] = event.description;
  
  String payload;
  payload.reserve(512);
  serializeJson(doc, payload);
  
  dataPublishRaw(topicActivity, payload);
  Serial.Println("[Activity] Published: " + event.description);
}

// ==========================================
// HELPER: BUILD DESCRIPTION
// ==========================================
static String buildDescription(const ActivityEvent& event) {
  String desc;
  String srcName = activitySourceToString(event.source);
  
  switch (event.type) {
    case ACT_POWER_ON:
      desc = "AC turned ON";
      break;
    case ACT_POWER_OFF:
      desc = "AC turned OFF";
      break;
    case ACT_MODE_CHANGE:
      desc = "Mode changed: " + event.old_mode + " -> " + event.new_mode;
      break;
    case ACT_FAN_CHANGE:
      desc = "Fan speed changed: " + event.old_fan + " -> " + event.new_fan;
      break;
    case ACT_TEMP_CHANGE:
      desc = "Set temperature changed: " + String(event.old_temp) + "C -> " + String(event.new_temp) + "C";
      break;
    case ACT_MANUAL_OVERRIDE:
      desc = "Manual control activated - Automation paused";
      break;
    case ACT_AUTO_RECOVERY:
      desc = "Auto-recovery completed - Automation resumed";
      break;
    case ACT_ERROR_CHANGE:
      desc = "Error code changed: " + event.old_error + " \u2192 " + event.new_error;
      break;
    default:
      desc = "Unknown activity";
  }
  
  desc += " (via " + srcName + ")";
  return desc;
}

// ==========================================
// TRACK STATE CHANGE (MAIN FUNCTION)
// ==========================================
void activityTrackChange(
  const BPFrame& oldState, 
  const BPFrame& newState, 
  ActivitySource source
) {
  if (!newState.valid) {
    Serial.Println("[Activity] Invalid new state, skipping");
    return;
  }
  
  // Initialize if not yet done
  if (!stateInitialized) {
    lastKnownState = newState;
    stateInitialized = true;
    Serial.Println("[Activity] State initialized");
    return;
  }
  
  bool hasChange = false;
  ActivityEvent event;
  event.timestamp = getTimestamp();
  event.source = source;
  
  // Check POWER change
  if (oldState.power != newState.power) {
    event.type = newState.power ? ACT_POWER_ON : ACT_POWER_OFF;
    event.old_power = oldState.power;
    event.new_power = newState.power;
    event.description = buildDescription(event);
    publishActivity(event);
    hasChange = true;
  }
  
  // Check MODE change (only if powered on)
  if (newState.power && oldState.mode != newState.mode) {
    event.type = ACT_MODE_CHANGE;
    event.old_mode = oldState.mode;
    event.new_mode = newState.mode;
    event.description = buildDescription(event);
    publishActivity(event);
    hasChange = true;
  }
  
  // Check FAN change (only if powered on)
  if (newState.power && oldState.fan != newState.fan) {
    event.type = ACT_FAN_CHANGE;
    event.old_fan = oldState.fan;
    event.new_fan = newState.fan;
    event.description = buildDescription(event);
    publishActivity(event);
    hasChange = true;
  }
  
  // Update last known state
  if (hasChange) {
    lastKnownState = newState;
  }
}

// ==========================================
// TRACK POWER CHANGE
// ==========================================
void activityTrackPower(
  bool oldPower, 
  bool newPower, 
  ActivitySource source
) {
  if (oldPower == newPower) return;

  // Ensure tracker considered initialized when publishing explicit events
  if (!stateInitialized) {
    stateInitialized = true;
    lastKnownState.valid = true;
  }
  
  ActivityEvent event;
  event.timestamp = getTimestamp();
  event.type = newPower ? ACT_POWER_ON : ACT_POWER_OFF;
  event.source = source;
  event.old_power = oldPower;
  event.new_power = newPower;
  event.description = buildDescription(event);
  
  publishActivity(event);
  
  // Update state
  lastKnownState.power = newPower;
}

// ==========================================
// TRACK MODE CHANGE
// ==========================================
void activityTrackMode(
  const String& oldMode, 
  const String& newMode, 
  ActivitySource source
) {
  if (oldMode == newMode) return;

  if (!stateInitialized) {
    stateInitialized = true;
    lastKnownState.valid = true;
  }
  
  ActivityEvent event;
  event.timestamp = getTimestamp();
  event.type = ACT_MODE_CHANGE;
  event.source = source;
  event.old_mode = oldMode;
  event.new_mode = newMode;
  event.description = buildDescription(event);
  
  publishActivity(event);
  
  // Update state
  lastKnownState.mode = newMode;
}

// ==========================================
// TRACK FAN CHANGE
// ==========================================
void activityTrackFan(
  const String& oldFan, 
  const String& newFan, 
  ActivitySource source
) {
  // Normalize fan speeds for comparison
  String normalizedOld = oldFan;
  String normalizedNew = newFan;
  normalizedOld.toUpperCase();
  normalizedNew.toUpperCase();
  normalizedOld.trim();
  normalizedNew.trim();
  
  // Handle variations
  if (normalizedOld == "MED") normalizedOld = "MEDIUM";
  if (normalizedNew == "MED") normalizedNew = "MEDIUM";
  if (normalizedOld == "MIN") normalizedOld = "LOW";
  if (normalizedNew == "MIN") normalizedNew = "LOW";
  if (normalizedOld == "MAX") normalizedOld = "HIGH";
  if (normalizedNew == "MAX") normalizedNew = "HIGH";
  
  // Only track if actually different
  if (normalizedOld == normalizedNew) return;

  if (!stateInitialized) {
    stateInitialized = true;
    lastKnownState.valid = true;
  }
  
  ActivityEvent event;
  event.timestamp = getTimestamp();
  event.type = ACT_FAN_CHANGE;
  event.source = source;
  event.old_fan = oldFan;
  event.new_fan = newFan;
  event.description = buildDescription(event);
  
  publishActivity(event);
  
  // Update state
  lastKnownState.fan = newFan;
}

// ==========================================
// TRACK TEMPERATURE CHANGE (OPTIONAL)
// ==========================================
void activityTrackTemp(
  int oldTemp,
  int newTemp,
  ActivitySource source
) {
  if (oldTemp == newTemp) return;

  if (!stateInitialized) {
    stateInitialized = true;
    lastKnownState.valid = true;
  }
  
  ActivityEvent event;
  event.timestamp = getTimestamp();
  event.type = ACT_TEMP_CHANGE;
  event.source = source;
  event.old_temp = oldTemp;
  event.new_temp = newTemp;
  event.description = buildDescription(event);
  
  publishActivity(event);
  
  // Update setpoint state (do not overwrite room temperature)
  lastKnownSetTemp = newTemp;
}

// ==========================================
// TRACK MANUAL OVERRIDE
// ==========================================
void activityTrackManualOverride() {
  ActivityEvent event;
  event.timestamp = getTimestamp();
  event.type = ACT_MANUAL_OVERRIDE;
  event.source = SOURCE_PHYSICAL_REMOTE;
  event.description = buildDescription(event);
  
  publishActivity(event);
}

// ==========================================
// TRACK AUTO RECOVERY
// ==========================================
void activityTrackAutoRecovery() {
  ActivityEvent event;
  event.timestamp = getTimestamp();
  event.type = ACT_AUTO_RECOVERY;
  event.source = SOURCE_SYSTEM;
  event.description = buildDescription(event);
  
  publishActivity(event);
}

// ==========================================
// TRACK ERROR CODE CHANGE
// ==========================================
void activityTrackError(
  const String& oldError,
  const String& newError
) {
  if (oldError == newError) return;

  ActivityEvent event;
  event.timestamp = getTimestamp();
  event.type = ACT_ERROR_CHANGE;
  event.source = SOURCE_SYSTEM;
  event.old_error = oldError;
  event.new_error = newError;
  event.description = buildDescription(event);

  publishActivity(event);
}

// ==========================================
// GET LAST KNOWN STATE
// ==========================================
BPFrame activityGetLastKnownState() {
  return lastKnownState;
}