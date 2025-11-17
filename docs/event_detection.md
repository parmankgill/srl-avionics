# Event Detection System

## Overview

The event detection system is the "observer" of autonomous flight sequencing. It monitors real-time flight data from the Extended Kalman Filter and detects critical milestones during flight. Each detected event represents a real physical phenomenon that has occurred (launch vibration, motor burnout, reaching apogee, parachute deployment, landing).

The system operates as a state machine with prerequisites: each event can only be detected after specific prior events have already been observed. This prevents false event claims and ensures events are reported in physically meaningful order.

## Event Hierarchy & Prerequisites
```
┌─ NOOP (initial state)
│
├─→ LAUNCH (accel > 10g)
│   │
│   ├─→ BURNOUT (low accel + upward velocity for 500ms)
│   │   │
│   │   └─→ APOGEE (negative velocity for 200ms + altitude > min)
│   │       │
│   │       ├─→ DROGUE_DEPLOYED (accel spike > 5g detected)
│   │       │   [Requires: launch, burnout, apogee]
│   │       │
│   │       └─→ MAIN_DEPLOYED (accel spike > 3g detected)
│   │           [Requires: launch, burnout, apogee, drogue_deployed]
│   │
│   └─→ LANDED (velocity ≈ 0 + altitude ≈ launch alt)
│       [Requires: launch, burnout, apogee, drogue_deployed, main_deployed]
│
└─→ COCOM_LIMIT (independent - runs anytime)
    [Requires: NOTHING]

ALSO INDEPENDENT:
└─→ DROGUE_TRIGGERED (command sent 5 sec after apogee)
    [Requires: launch, burnout, apogee - but this is metadata, not a detection]
└─→ MAIN_TRIGGERED (command sent when altitude < threshold)
    [Requires: launch, burnout, apogee - but this is metadata, not a detection]
```

## Core Detection Methods

### 1. Launch Detection
**Criteria:**
- Z-axis acceleration exceeds 10g (~98 m/s^2)
- Only detected once
- Triggers capture of launch site LLA coordinates

**Logic:**
```cpp
if (up_accel > 10*g) {
    launch_detected = true;
    // Record launch altitude/position
    // Publish PHOENIX_EVENT_LAUNCH
}
```

### 2. Burnout Detection
**Criteria:**
- Launch must have been detected
- Z-axis acceleration drops below 5g
- Upward velocity is still positive (rocket is still ascending)
- Both conditions persist for >500 ms (prevents glitches)

**Logic:**
```cpp
const bool low_accel = up_accel < 5*g;
const bool upward_velocity = up_velocity > 0;

if (low_accel && upward_velocity) {
    if (burnout_start_time == 0) {
        burnout_start_time = current_time;  // Start timer
    } else if (current_time - burnout_start_time > 500) {
        burnout_detected = true;  // Sustained condition
    }
} else {
    burnout_start_time = 0;  // Reset if condition breaks
}
```

**Why 500ms sustained?**
- Motor burn curves show brief oscillations during tail-off
- 500ms filters noise while remaining responsive
- Typical motor burnout is gradual, not instantaneous

**Edge Case Handling**
- Timer resets if condition breaks → only fires on sustained low acceleration
- Prevents false detection from sensor spike during final burn phase

### 3. Apogee Detection
**Criteria:**
- Launch AND burnout must have been detected
- Z-axis velocity becomes negative (rocket starts descending)
- Sustained negative velocity for >200 ms
- Altitude exceeds minimum threshold (100m default)

**Logic:**
```cpp
if (up_velocity < 0) {
    if (negative_vel_start_time == 0) {
        negative_vel_start_time = current_time;
    } else if ((current_time - negative_vel_start_time > 200) &&
               (altitude > min_apogee_altitude)) {
        apogee_detected = true;
        apogee_timestamp = current_time;  // Record for timing decisions
    }
} else {
    negative_vel_start_time = 0;  // Reset on upward velocity
}
```
**Why 200ms sustained?**
- Apex of trajectory can have brief velocity oscillations from sensor noise
- 200ms ensures true apogee, not measurement artifact
- Mach 3 rocket spends ~1-2 seconds near apogee

### 4. Drogue Triggered (Metadata Event)
Detects that a command has been sent to deploy the drogue

**Criteria:** Launch, burnout, AND apogee detected

**Logic:**
```cpp
if (apogee_timestamp == 0) {
    apogee_timestamp = current_time;
    return;
}

if ((current_time - apogee_timestamp) > 5000) {
    drogue_triggered = true;  // Command was sent
    publish(PHOENIX_EVENT_FIRED_DROGUE);
}
```

### 5. Drogue Deployed Detection
Detects that the drogue parachute has physically opened

**Criteria:**
1. **Prerequisites:** Launch, burnout, AND apogee must have been detected
2. Sudden spike in positive Z-acceleration (>5g)

**Logic:**
```cpp
const bool accel_spike = up_accel > 5*g;

if (accel_spike) {
    drogue_deployed_detected = true;
    publish(PHOENIX_EVENT_DEPLOYED_DROGUE);
}
```
**What triggers this spike?**
- Drogue parachute suddenly opens
- Creates upward deceleration (looks like acceleration to sensors)
- Magnitude indicates successful deployment vs. partial opening

**Key Insight:**
- This detects actual deployment, independent of commands
- If pyro charge failed to fire → no accel spike → no deployment detection
- Validates that parachute actually opened, not just that we tried to open it
- Redundant check against deployment failure
- Physical confirmation that recovery system is working

### 6. Main Triggered (Metadata Event)
Detects that a command has been sent to deploy the main parachute

**Criteria:**
1. **Prerequisite:** Launch, burnout, AND apogee detected 
2. Altitude drops below main deployment threshold
3. Rocket still descending (negative velocity)

**Logic:**
```cpp
if (altitude < main_deploy_altitude && up_velocity < 0) {
    main_triggered = true;  // Command was sent
    publish(PHOENIX_EVENT_FIRED_MAIN);
}
```

### 7. Main Deployed Detection
Detects that the main parachute has physically opened

**Criteria:**
1. **Prerequisite:** Launch, burnout, AND apogee must have been detected
2. **Prerequisite:** Drogue must have been deployed (DROGUE_DEPLOYED)
3. Spike in positive Z-acceleration (>3g)

**Logic:**
```cpp
const bool accel_spike = up_accel > 3*g;

if (accel_spike && drogue_triggered) {
    main_deployed_detected = true;
    publish(PHOENIX_EVENT_DEPLOYED_MAIN);
}
```

**Why 3g instead of 5g?**
- Main parachute is larger than drogue
- Opens more gradually (lower peak deceleration)
- 3g still indicates significant opening force
- Filters out sensor noise from wind gusts

### 8. Landing Detection

**Prerequisites:**
1. Launch, burnout, AND apogee detected
2. BOTH drogue AND main have been deployed
3. Full recovery sequence has completed

**Criteria:**
1. Z-axis velocity near zero (< 0.5 m/s)
2. Altitude within 5% of launch altitude

**Logic:**
```cpp
bool near_zero_velocity = fabs(up_velocity) < 0.5;
bool near_launch_altitude = fabs(altitude - launch_altitude) / launch_altitude < 0.05;

if (near_zero_velocity && near_launch_altitude) {
    landed_detected = true;
    publish(PHOENIX_EVENT_LANDED);
}
```

### 9. COCOM Limit Exceeded

**What it detects:** Vehicle has exceeded export control restrictions

**Prerequisites:**
- NONE - runs independently at all times

**Criteria:**
- Altitude exceeds 12,000 m OR
- Velocity magnitude exceeds 510 m/s

**Logic:**
```cpp
float32_t velocity_mag = sqrt(east_vel² + north_vel² + up_vel²);

const bool altitude_exceeded = altitude > COCOM_alt;
const bool velocity_exceeded = velocity_mag > COCOM_velocity;

if (altitude_exceeded || velocity_exceeded) {
    publish(PHOENIX_EVENT_COCOM_LIMIT_EXCEEDED);
}
```

**What is COCOM?**
- Commercial Oriented Munitions List
- U.S. export control regulation
- Limits on missile technology applications
- Affects rocket testing in some contexts

## Launch Site Determination
How do you know your launch coordinates before launch?

**Pre-launch GNSS collection**
```cpp
void set_launch_lla(const struct zbus_channel *chan) {
    // Before launch_detected, collect GNSS altitude readings
    // Keep only last 10 seconds of data in linked list
    // Average them for accurate launch altitude
    
    launch_lla.lat = gnss_msg->pos.x;
    launch_lla.lon = gnss_msg->pos.y;
    launch_lla.alt = average_of_recent_altitudes;
    
    // After launch detected, free the buffer
    if (launch_detected) {
        free_linked_list();
    }
}
```

**Why averaging?**
- GNSS has ~5-10m noise on each sample
- Averaging pre-launch readings improves accuracy
- Better reference for "landed back at launch site" check
