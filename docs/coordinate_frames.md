# Coordinate Frame Transformations

## Overview

The avionics system operates in multiple reference frames simultaneously:
- **GPS provides data in LLA** (Latitude/Longitude/Altitude)
- **Inertial sensors measure in body frame** (attached to rocket)
- **Navigation and control require ENU** (East/North/Up - local horizon frame)
- **Attitude determination uses ECI** (Earth-Centered Inertial - non-rotating reference)

This module transforms between these frames in real-time, accounting for Earth's rotation and ellipsoidal shape of Earth.

## Reference Frames
**LLA (Latitude/Longitude/Altitude)**
- GPS coordinates
- Geodetic system using WGS84 ellipsoid
- Input from external GNSS receiver

**ECEF (Earth-Centered Earth-Fixed)**
- Origin at Earth's center
- Rotates with Earth
- Intermediate frame for LLA conversion

**ECI (Earth-Centered Inertial)**
- Origin at Earth's center
- Fixed in space (inertial reference)
- Essential for attitude determination and Kalman Filter
- Requires accounting for Earth's rotation angle (GAST)

**ENU (East/North/Up)**
- Local navigation frame centered at launch site
- East: positive pointing east
- North: positive pointing north
- Up: positive pointing away from Earth's center
- Most intuitive for mission planning and recovery

## Key Algorithm: GAST (Greenwich Apparent Sidereal Time)

Earth rotates ~360 degrees per sidereal day. To transform between inertial (ECI) and rotating (ECEF) frames, we must know exactly how much Earth has rotated at a given time.

###Steps:
1. **Convert UTC time to Julian Date** - standard astronomical time representation
2. **Calculate Julian centuries since J2000.0** - reference epoch
3. **Calculate Greenwich Mean Sidereal Time (GMST)** at 0h UT
4. **Account for time elapsed since 0h UT** to get GMST at observation time
5. **Apply nutation corrections** - Moon's gravity causes small wobbles in Earth's rotation
6. **Result: GAST** - precise rotation angle to transform between frames

### Why This Matters:
Without accurate GAST calculation, the EKF cannot properly integrate inertial sensor data. Even small errors (seconds of arc) compound over flight.

## Inertial Effects: Coriolis and Centripetal Acceleration

When transforming accelerations to the inertial frame, we must account for Earth's rotation effects:

**Coriolis Acceleration:** 'a_coriolis = 2 * omega * v'
- Moving objects experience apparent deflection in rotating frame
- Significant at high altitudes and speeds (Mach 3+)

**Centripetal Acceleration:** 'a_centripetal = omega^2 * r'
- Apparent outward 'force' due to frame rotation
- Increases with distance from Earth's axis

**Implementation:**
- Calculated in ECEF frame when they're the cleanest
- Subtracted from inertial accelerations before transforming to ENU
- Ensures accelerometers measure true inertial acceleration

**Key Functions:**
- 'utc_to_JD()' - Convert UTC timestamp to Julian Date
- 'calculate GAST()' - Compute Earth's rotation angle including nutation
- 'LLAtoECI()' - Full transformation from GPS coordinates to inertial frame
- 'ECItoENU()' - Tranform inertial state to navigation frame + apply inertial corrections

**WGS84 Parameters Used:**
- Semi-major axis (equatorial radius): 6378.137 km
- Semi-minor axis (polar radius): 6356.752 km
- These define Earth's ellipsoidal shape

## Spaceshot Enhancements
1. May need to add relativistic correction terms to accelerataion transformation
2. Current code assumes spherical Earth. At extreme altitude, Earth's oblate shape effects compound. The solution would be to upgrade the gravity model from WGS84 to EGM2008 or include J2/J3 perterubations.
