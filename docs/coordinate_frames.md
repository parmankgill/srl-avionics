# Coordinate Frame Transformations

## Overview

The avionics system operates in multiple reference frames simultaneously:
- **GPS provides data in LLA** (Latitude/Longitude/Altitude)
- **Inertial sensors measure in body frame** (attached to rocket)
- **Navigation and control require ENU** (East/North/Up - local horizon frame)
- **Attitude determination uses ECI** (Earth-Centered Inertial - non-rotating reference)

This module transforms between these frames in real-time, accounting for Earth's rotation and ellipsoidal shape of Earth.

## Reference Frames
** LLA (Latitude/Longitude/Altitude)**
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
