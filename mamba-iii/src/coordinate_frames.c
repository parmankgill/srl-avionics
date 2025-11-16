#include <math.h>
#include <arm_math.h>
#include <zephyr/kernel.h>
#include <zephyr/dsp/types.h>
#include <zephyr/logging/log.h>

#include <phoenix/common/coordinate_frames.h>
#include <phoenix/common/units.h>
#include <phoenix/common/time.h>

LOG_MODULE_REGISTER(coordinate_frames, CONFIG_LOG_LEVEL_PHOENIX);

// Function to Calculate the Julian Date of Observation
double utc_to_JD(struct time_utc t) {
    // convert UTC time to Julian Date
    int Y = t.year;
    int M = t.month;

    // if statement tht treats January and February as 13th and 14th months of previous year
    if (M<=2){
        Y -= 1;
        M += 12;
    }

    // Compute correction factor for Gregorian calendar
    int A = Y / 100;
    int B = 2 - A + (A / 4);

    double JD = (int)(365.25 * (Y + 4716)) + (int)(30.6001 * (M + 1)) + t.day + B - 1524.5;

    // Convert time of day to fractional day
    double fraction_of_day = (t.hour + (t.minute / 60.0) + (t.second / 3600.0) + (t.millisecond / 3.6e6)) / 24.0;

    return JD + fraction_of_day;
}

// Helper function to calculate GAST
double calculate_GAST(struct time_utc gpstime){
    // Calculating the Julian Date of Observation
    double JD = utc_to_JD(gpstime);
    double T = (JD - 2451545.0) / 36525.0; // Julian centuries since J2000.0

    // Calculate the Greenwich Mean Sidereal Time at 0h UT
    double GMST0 = 100.46061837 + 36000.770053608 * T + 0.000387933 * T * T - (T * T * T) / 38710000.0;

    // Calculate the Greenwich Mean Sidereal Time at the given time
        // Calculate UT in seconds elapsed since 0h UT
        double UT_seconds = gpstime.hour * 3600.0 + gpstime.minute * 60.0 +
                   gpstime.second + gpstime.millisecond / 1000.0;
    double GMST = GMST0 + 360.98564736629 * UT_seconds / 86400.0; // degrees
    // Normalize GMST to the range 0-360 degrees because GMST can produce range outside of 0-360
    GMST = fmod(GMST, 360.0);
    if (GMST < 0) {
        GMST += 360.0;
    }
    double GMST_rad = GMST * PI / 180.0; // GMST in radians

    // Calculate GAST - Greenwich Apparent Sidereal Time - accounts for nutation in longitude
        // Calculating omega (longitude of ascending node of the Moon)
            // According to the NAvy website omega, L, and epsilon are expressed in degrees
            double DTT = JD - 2451545.0; // Julian centuries since J2000.0
            double omega = 125.04452 - 0.052954*DTT; // degrees
            double L = 280.47 + 0.98565*DTT; // degrees
            double epsilon = 23.439292 - 0.0000004*DTT; // degrees

                // Convert omega, L, and epsilon to radians
                double omega_rad = omega * PI / 180.0;
                double L_rad = L * PI / 180.0;;
                double epsilon_rad = epsilon * PI / 180.0;

            // Nutation calculaion
            double nutation = -0.000319*sin(omega_rad) - 0.000024*sin(2*L_rad); // radians
            double eqeq = nutation * cos(epsilon_rad); // radians

        // Calculate GAST
        double GAST = GMST_rad + eqeq; // radians
        return GAST;
}

// NEED TO CHECK CONSISTENCY WITH DEGREES/RADIANS

int32_t LLAtoECI(struct time_utc gpstime, struct LLA *gpa_lla, struct ECI *gpa_eci, struct ECEF *gpa_ecef) {
// CONVERTING LLA TO ECEF (Earth Centered Earth Fixed)
    // convert long/lat degrees to radians
    float32_t lat_rad = gpa_lla->lat * PI / 180;
    float32_t lon_rad = gpa_lla->lon * PI / 180;

    // Calculate the Earth's radius at the given latitude accounting for Earth's ellipsoidal shape
    float32_t a = 6378.137; // semi-major axis equatorial radius (km)
    float32_t b = 6356.752; // semi-minor axis polar radius (km)
    float32_t e = sqrt(1 - (b / a) * (b / a)); // eccentricity

    // Calculate the Earth's radius of curvature in the prime vertical
    float32_t R_N = a/sqrt(1 - e * e * sin(lat_rad) * sin(lat_rad));

    // Calculate ECEF coordinates
        // CHECK TO SEE IF THESE ARE OKAY IN RADIANS
    gpa_ecef->x = (R_N + gpa_lla->alt) * cos(lat_rad) * cos(lon_rad);
    gpa_ecef->y = (R_N + gpa_lla->alt) * cos(lat_rad) * sin(lon_rad);
    gpa_ecef->z = (R_N * (1 - e * e) + gpa_lla->alt) * sin(lat_rad);

// CALCULATING EARTH'S ROTATION ANGLE
    double GAST = calculate_GAST(gpstime); // GAST in radians

// ECEF TO ECI
    // Rotation matrix from ECEF to ECI
    gpa_eci->x = gpa_ecef->x * cos(GAST) + gpa_ecef->y * sin(GAST);
    gpa_eci->y = -gpa_ecef->x * sin(GAST) + gpa_ecef->y * cos(GAST);
    gpa_eci->z = gpa_ecef->z;

    return 0;
}

int32_t ECItoENU(
    struct time_utc gpstime,
    struct LLA launch_site_lla,
    struct ECI rocket_eci,
    struct ECI rocket_eci_velocity,
    struct ECI rocket_eci_accel,
    struct ENU *rocket_enu, //output
    struct ENUVelocity *rocket_enu_velocity, // output
    struct ENUAcceleration *rocket_enu_accel, // output
    float32_t *rocket_altitude // output
){
    // 1. Establish Launch site ECI position and ECEF position
    struct ECI launch_site_eci;
    struct ECEF launch_site_ecef;
    LLAtoECI(gpstime, &launch_site_lla, &launch_site_eci, &launch_site_ecef);

    // 2. Calculate relative position vector in ECI
    struct ECI relative_eci_pos;
    relative_eci_pos.x = rocket_eci.x - launch_site_eci.x;
    relative_eci_pos.y = rocket_eci.y - launch_site_eci.y;
    relative_eci_pos.z = rocket_eci.z - launch_site_eci.z;

    // 3. Get GAST for ECI-ECEF transformations
    double GAST = calculate_GAST(gpstime); // GAST in radians

    // 4. Earth's rotation rate (rad/s) - const
    float32_t earth_rotation_rate = 7.2921159e-5; // rad/s

    // 5. Convert rocket ECI to ECEF
    struct ECEF rocket_ecef;
        // ECI to ECEF transformation (inverse of ECEF to ECI)
    rocket_ecef.x = rocket_eci.x * cos(GAST) + rocket_eci.y * sin(GAST);
    rocket_ecef.y = -rocket_eci.x * sin(GAST) + rocket_eci.y * cos(GAST);
    rocket_ecef.z = rocket_eci.z;

    // 6. Calculate relative position in ECEF
    struct ECEF relative_ecef;
    relative_ecef.x = rocket_ecef.x - launch_site_ecef.x;
    relative_ecef.y = rocket_ecef.y - launch_site_ecef.y;
    relative_ecef.z = rocket_ecef.z - launch_site_ecef.z;

    // 7. Convert ECEF to ENU
    float32_t lat_rad = launch_site_lla.lat * PI / 180.0;
    float32_t lon_rad = launch_site_lla.lon * PI / 180.0;

    float32_t sin_lat = sin(lat_rad);
    float32_t cos_lat = cos(lat_rad);
    float32_t sin_lon = sin(lon_rad);
    float32_t cos_lon = cos(lon_rad);

    // Calculate the rotation from ECEF to ENU
    // Note: Converting from km to meters for ENU output
    rocket_enu->east_position = (-sin_lon * relative_ecef.x + cos_lon * relative_ecef.y) * 1000.0;
    rocket_enu->north_position = (-cos_lon * sin_lat * relative_ecef.x - sin_lon * sin_lat * relative_ecef.y + cos_lat * relative_ecef.z) * 1000.0;
    rocket_enu->up_position = (cos_lat * cos_lon * relative_ecef.x + cos_lat * sin_lon * relative_ecef.y + sin_lat * relative_ecef.z) * 1000.0;

    // 8. Convert rocket velocity from ECI to ECEF
    struct ECI rocket_ecef_velocity;
    rocket_ecef_velocity.x = rocket_eci_velocity.x * cos(GAST) + rocket_eci_velocity.y * sin(GAST);
    rocket_ecef_velocity.y = -rocket_eci_velocity.x * sin(GAST) + rocket_eci_velocity.y * cos(GAST);
    rocket_ecef_velocity.z = rocket_eci_velocity.z;

        // Account for Earth's rotation
        rocket_ecef_velocity.x -= earth_rotation_rate * rocket_ecef.y;
        rocket_ecef_velocity.y += earth_rotation_rate * rocket_ecef.x;

    // 9. Transform velocity from ECEF to ENU
    rocket_enu_velocity->east_velocity = (-sin_lon * rocket_ecef_velocity.x + cos_lon * rocket_ecef_velocity.y) * 1000.0;
    rocket_enu_velocity->north_velocity = (-cos_lon * sin_lat * rocket_ecef_velocity.x - sin_lon * sin_lat * rocket_ecef_velocity.y + cos_lat * rocket_ecef_velocity.z) * 1000.0;
    rocket_enu_velocity->up_velocity = (cos_lat * cos_lon * rocket_ecef_velocity.x + cos_lat * sin_lon * rocket_ecef_velocity.y + sin_lat * rocket_ecef_velocity.z) * 1000.0;

    // 10. Acceleration transformation
    struct ECI rocket_ecef_accel;
    rocket_ecef_accel.x = rocket_eci_accel.x * cos(GAST) + rocket_eci_accel.y * sin(GAST);
    rocket_ecef_accel.y = -rocket_eci_accel.x * sin(GAST) + rocket_eci_accel.y * cos(GAST);
    rocket_ecef_accel.z = rocket_eci_accel.z;

    float32_t omega_squared = earth_rotation_rate * earth_rotation_rate;

    // Centripetal acceleration
    struct ECEF centripetal_accel;
    centripetal_accel.x = omega_squared * rocket_ecef.x;
    centripetal_accel.y = omega_squared * rocket_ecef.y;
    centripetal_accel.z = 0.0;

    // Coriolis acceleration
    struct ECEF coriolis_accel;
    coriolis_accel.x = 2.0 * earth_rotation_rate * rocket_ecef_velocity.y;
    coriolis_accel.y = -2.0 * earth_rotation_rate * rocket_ecef_velocity.x;
    coriolis_accel.z = 0.0;

    // Substract both effects from the ECEF acceleration
    rocket_ecef_accel.x -= (centripetal_accel.x + coriolis_accel.x);
    rocket_ecef_accel.y -= (centripetal_accel.y + coriolis_accel.y);
    rocket_ecef_accel.z -= (centripetal_accel.z + coriolis_accel.z);

    // Transform ECEF acceleration to ENU
    rocket_enu_accel->east_acceleration = (-sin_lon * rocket_ecef_accel.x + cos_lon * rocket_ecef_accel.y) * 1000.0;
    rocket_enu_accel->north_acceleration = (-cos_lon * sin_lat * rocket_ecef_accel.x - sin_lon * sin_lat * rocket_ecef_accel.y + cos_lat * rocket_ecef_accel.z) * 1000.0;
    rocket_enu_accel->up_acceleration = (cos_lat * cos_lon * rocket_ecef_accel.x + cos_lat * sin_lon * rocket_ecef_accel.y + sin_lat * rocket_ecef_accel.z) * 1000.0;

    // Calculate altitude
    float32_t launch_site_mag = sqrt(launch_site_eci.x * launch_site_eci.x + launch_site_eci.y * launch_site_eci.y + launch_site_eci.z * launch_site_eci.z);

    struct ECI launch_site_unit_vector;
    launch_site_unit_vector.x = launch_site_eci.x / launch_site_mag;
    launch_site_unit_vector.y = launch_site_eci.y / launch_site_mag;
    launch_site_unit_vector.z = launch_site_eci.z / launch_site_mag;

    *rocket_altitude = (relative_eci_pos.x * launch_site_unit_vector.x +
        relative_eci_pos.y * launch_site_unit_vector.y +
        relative_eci_pos.z * launch_site_unit_vector.z) * 1000.0;

    return 0;
}
