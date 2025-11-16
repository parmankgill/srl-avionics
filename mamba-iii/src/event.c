#include <zephyr/zbus/zbus.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <phoenix/event.h>
#include <phoenix/common/coordinate_frames.h>
#include <phoenix/common/time.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>

#include <phoenix/zbus.h>
#include <phoenix/event.h>
#include <phoenix/threads/ekf.h>


LOG_MODULE_REGISTER(event_c);

#define ADC_NODE DT_NODELABEL(adc1)
#define ADC_CHANNEL DT_PROP(ADC_NODE, channel)
#define ADC_RESOLUTION 12
#define ADC_GAIN ADC_GAIN_1
#define ADC_REFERENCE ADC_REF_INTERNAL
#define ADC_BUFFER_SIZE 1

// const struct device *adc_dev = DEVICE_DT_GET(DT_IO_CHANNELS_CTLR(ADC_NODE));
// static int16_t sample_buffer[ADC_BUFFER_SIZE];
// static struct adc_channel_cfg channel_cfg = {
//     .gain = ADC_GAIN,
//     .reference = ADC_REFERENCE,
//     .acquisition_time = ADC_ACQ_TIME_DEFAULT,
//     .channel_id = ADC_CHANNEL
// };

static const struct device *adc_dev = DEVICE_DT_GET(ADC_NODE);

int check_battery_voltage() {
    //((((((((((((((()))))))))))))))
    // static const struct adc_dt_spec adc_channel = {
    //     .channel_id = 5,
    //     .resolution = 12,
    //     .oversampling = 0,
    // };
    //((((((((((((((((((()))))))))))))))))))
    static const struct adc_channel_cfg adc_cfg = {
        .gain = ADC_GAIN_1,
        .reference = ADC_REF_INTERNAL,
        .acquisition_time = ADC_ACQ_TIME_DEFAULT,
        .channel_id = 5,
    };

    // LOG_INF("Checking battery voltage -----------------");

    int err;
	uint16_t buf = 0;

	struct adc_sequence sequence = {
		.buffer = &buf,
		.buffer_size = sizeof(buf),
        .channels = BIT(5),
        .resolution = 12,
	};
    // LOG_INF("Passed sequence init");

    // err = adc_channel_setup_dt(&adc_channel);
    // if (err < 0) {
    //     printk("Could not setup channel #5 (%d)\n", err);
    //     return 0;
    // }
	err = adc_channel_setup(adc_dev, &adc_cfg);
    if (err < 0) {
        LOG_ERR("Could not setup ADC channel (%d)\n", err);
        return err;
    }

    // LOG_INF("Passed channel setup");

    // LOG_INF("Passed ADC_READY check");

    // struct adc_sequence sequence = {
    //     .channels = BIT(ADC_CHANNEL),
    //     .buffer = sample_buffer,
    //     .buffer_size = sizeof(sample_buffer),
    //     .resolution = ADC_RESOLUTION,
    //     .oversampling = 0,
    //     .calibrate = false,
    // };
    
    double bv = 0;

    err = adc_read(adc_dev, &sequence);
    if (err < 0) {
        LOG_ERR("Could not read (%d)", err);
    } else {
        // LOG_INF("buf =  %04x", buf);
        int16_t raw = 0;
        raw = (int16_t)buf;
        // LOG_INF("raw =  %04x", raw);
        bv = 12.6 * (((double)raw / (1 << ADC_RESOLUTION))/(0.948));
        LOG_INF("Battery voltage: %d mV", (int)(bv*1000));
        // *voltage = bv;
    }

    // int ret = adc_read(adc_dev, &sequence);
    // if (ret < 0) {
    //     LOG_ERR("ADC read failed: %d", ret);
    //     return ret;
    // } else {
    //     int32_t raw = sample_buffer[0];
    //     bv = 12.6 * ((double)raw / (1 << ADC_RESOLUTION));
    //     LOG_INF("Battery voltage: %f", bv);
    //     voltage = &bv;
    // }
    
    double charge = 0;
    double voltLUT[] = {11.07, 11.13, 11.19, 11.25, 11.31, 11.37, 11.40, 11.46, 11.52, 11.55, 11.61, 11.73, 11.85, 11.94, 12.06, 12.24, 12.33, 12.45, 12.60};
    if (bv < voltLUT[0]){
        charge = 0;
        // *percent = charge;
        LOG_INF("CHARGE BATTERY IMMEDIATELY");
        return -1;
    }
    if (bv > voltLUT[18]){
        charge = 100;
        // *percent = charge;
        LOG_INF("BATTERY FULL");
        return 0;
    }

    for(int i=0; i<19; ++i){
        int p2 = (i * 5) + 10;
        if (bv > voltLUT[i] && bv < voltLUT[i+1]){
            charge = (((bv - voltLUT[i])/(voltLUT[i+1] - voltLUT[i]))*5 + p2);
        }
    }
    // *percent = charge;
    LOG_INF("Percent charged: %d", (int)(charge));

    return (int)charge;
}

float32_t east_accel = 0.0;
float32_t north_accel = 0.0;
float32_t up_accel = 0.0;

float32_t east_velocity = 0.0;
float32_t north_velocity = 0.0;
float32_t up_velocity = 0.0;

float32_t east_position = 0.0;
float32_t north_position = 0.0;
float32_t up_position = 0.0;

float32_t altitude = 0.0;

void update_ekf_data(struct ENU rocket_enu, struct ENUVelocity rocket_enu_velocity, struct ENUAcceleration rocket_enu_accel, float32_t rocket_altitude) {
    // Update the EKF data with the new values
    east_accel = rocket_enu_accel.east_acceleration;
    north_accel = rocket_enu_accel.north_acceleration;
    up_accel = rocket_enu_accel.up_acceleration;

    east_velocity = rocket_enu_velocity.east_velocity;
    north_velocity = rocket_enu_velocity.north_velocity;
    up_velocity = rocket_enu_velocity.up_velocity;

    east_position = rocket_enu.east_position;
    north_position = rocket_enu.north_position;
    up_position = rocket_enu.up_position;

    altitude = rocket_altitude; // Altitude is already passed as a parameter
}


// THINGS THAT NEED TO BE CHECKED
	// Check all calls of accel, position, velocity arrays to make sure they are called correctly
    // Primarily velocity[2] 
    // Update vertical axis code
    // Define altitude

// Global flags to track event detection
bool launch_detected = false; // flag to track if launch has been detected
bool burnout_detected = false; // tracks if burnout happened
bool apogee_detected = false; // tracks if apogee has been detected
bool landed_detected = false; // Flag to track if landing has been detected
float launch_altitude = 0.0; // Altitude at which launch was detected
bool drogue_triggered = false; // Flag to track if drogue has been triggered
bool main_triggered = false; // Flag to track if main has been triggered
bool near_zero_velocity = false;
bool near_launch_altitude = false; // Flag to track if near launch altitude
bool drogue_deployed_detected = false;
bool main_deployed_detected = false; // Flag to track if main deployment has been detected

static uint64_t ns_since_utc_epoch;
static struct time_utc utc;

// Timestamps for event detection
uint64_t burnout_start_time = 0; // timestamp for sustained low acceleration
uint64_t negative_vel_start_time = 0; // timestamp for sustained negative velocity
uint64_t apogee_timestamp = 0;

// Parameters to configure
float min_apogee_altitude = 100.0; // minimum altitude for apogee detection (ADJUST)
float main_deploy_altitude = 1000.0; // altitude for main deployment (ADJUST)
float COCOM_alt = 12000.0; // COCOM altitude
float COCOM_velocity = 510.0; // COCOM velocity
float g = 9.81; // gravitational acceleration (m/s^2)

struct LLA launch_lla = {
    .lat = 0.0,
    .lon = 0.0,
    .alt = 0.0,
};

// PHOENIX EVENT NOOP
	// No operation event, nothing to handle

// PHOENIX EVENT LAUNCH
	// 1. Listens to EKF data (previous ADXL375 data)
	// 2. Detects launch event based on z-acceleration exceeding 10 m/s^2
	// 3. Publishes PHOENIX_EVENT_LAUNCH message to phoenix_event_chan
	// 4. Logs accelerometer values
	// 5. Only detects launch once
void launch_Check() {
    if (launch_detected) {
        return; // Exit if launch already detected
    }

    // Check if z-acceleration exceeds the threshold for launch detection
    if (up_accel > 10*g) {
        launch_detected = true; // Set the flag

        struct ECI eci;
        struct ECEF ecef;
        LLAtoECI(utc, &launch_lla, &eci, &ecef); // Get the launch site ECI
        launch_altitude = eci.z; // Set the launch altitude
        // its possible that during this time the rocket may have already traveled quite a bit due to the threshold and sensor noise

        // Create and publish a launch event
        struct phoenix_event_zbus_msg msg = {.event = PHOENIX_EVENT_LAUNCH};
        zbus_chan_pub(&phoenix_event_chan, &msg, K_NO_WAIT);

        LOG_INF("Launch detected: accel_z=%.2f m/s^2, altitude=%.2f m", up_accel, altitude); // Log the detected launch event
    }
    // Log the accelerometer values
    // added .2 for precision purposesses
    LOG_INF("From listener -> Launch_Msg x=%.2f, y=%.2f, z=%.2f", east_accel, north_accel, up_accel); // Log the accelerometer values
}

// PHOENIX EVENT BURNOUT
	// 1. Launch must have been detected (launch_detected == true)
	// 2. Accelerometer z-axis value from EKF is below 5 m/s^2
	// 3. Upward velocity from EKF is greater than 0
	// 4. Low acceleration condition persists for more than 500 ms
	// 5. If burnout detected, publish PHOENIX_EVENT_BURNOUT message to phoenix_event_chan

// function to check for burnout condition
	// chan acts as a pointer to the zbus_channel
	// --> pointer is a variable that stores the memory address of another variable not the actual value

void burnout_Check() {
    if (burnout_detected || !launch_detected) { // Ensure that burnout is detected only once after launch
        return;
    }

    uint64_t current_time = k_ticks_to_ms_floor64(k_uptime_ticks()); // Get the current time
    // Check for burnout conditions
    // low_accel and upward_veloctiy do not change so adding const for consistency
    const bool low_accel = up_accel < 5*g; // Acceleration below outlined threshold
    const bool upward_velocity = up_velocity > 0; // Rocket going up, not down, added _z as I think that was missed

    if (low_accel && upward_velocity) {
        if (burnout_start_time == 0) {
            burnout_start_time = current_time; // Start the timer
        }
        // Check if conditions have lasted for 500 ms
        else if (current_time - burnout_start_time > 500) {
            burnout_detected = true;
            // Publish event
            struct phoenix_event_zbus_msg msg = {.event = PHOENIX_EVENT_BURNOUT};
            zbus_chan_pub(&phoenix_event_chan, &msg, K_NO_WAIT);

            LOG_INF("Burnout detected: accel_z=%.2f m/s^2, velocity_z=%.2f m/s", up_accel, up_velocity);
        }
    } else {
        // Reset the burnout start time if the condition is not met
        burnout_start_time = 0;
    }
}

// PHOENIX EVENT APOGEE
	// 1. Launch must have been detected (launch_detected == true)
	// 2. Burnout must have been detected (burnout_detected == true)
	// 3. Velocity z-axis value from EKF is negative for a sustained period (200ms)
	// 4. Position is greater than a minimum altitude threshold (min_apogee_altitude)
	// 5. If apogee detected, publish PHOENIX_EVENT_APOGEE message to phoenix_event_chan

void apogee_Check() {
    if (apogee_detected) { // Ensure that apogee is detected only once
        return;
    }

    // Check prerequisites: launch and burnout must have been detected
    if (!launch_detected || !burnout_detected) {
        return;
    }

    // Get the current time
    uint64_t current_time = k_ticks_to_ms_floor64(k_uptime_ticks());

    if (up_velocity < 0) {
        if (negative_vel_start_time == 0) {
            negative_vel_start_time = current_time;
        }

        // Check all apogee criteria
        else if ((current_time - negative_vel_start_time > 200) &&
                    (altitude > min_apogee_altitude)) {
            apogee_detected = true;

            // Publish event
            struct phoenix_event_zbus_msg msg = {.event = PHOENIX_EVENT_APOGEE};
            zbus_chan_pub(&phoenix_event_chan, &msg, K_NO_WAIT);

            LOG_INF("Apogee detected: altitude=%.2f m, velocity=%.2f m/s", altitude, up_velocity);
        }
    } else {
        // Reset the negative velocity start time if the condition is not met
        negative_vel_start_time = 0;
    }
}

// PHOENIX EVENT DROGUE TRIGGERED
	// Trigger the drogue once launch and burnout and apogee have been detected, and 5 seconds have passed since apogee
	// 1. Launch must have been detected
	// 2. Burnout must have been detected
	// 3. Apogee must have been detected
	// 4. 5 seconds must have passed since apogee detection
	// 5. If drogue triggered, publish PHOENIX_EVENT_DROGUE_TRIGGERED message to phoenix_event_chan
	// 6. Log the drogue deployment command

void drogue_triggered_Check() {
	if (drogue_triggered) { // Ensure that drogue is triggered only once
		return;
	}

    // Get the current time
    uint64_t current_time = k_ticks_to_ms_floor64(k_uptime_ticks());

    // Check prerequisites: launch, burnout, and apogee must have been detected
    if (!launch_detected || !burnout_detected || !apogee_detected) {
        return;
    }

    if (apogee_timestamp == 0) {
        apogee_timestamp = current_time; // Set the apogee timestamp
        LOG_INF("Apogee timestamp recorded: %lld ms", apogee_timestamp);
        return;
    }

    // Check if 5 seconds have passed since apogee
    if ((current_time - apogee_timestamp) > 5000) {
        drogue_triggered = true;

        // Create and publish a drogue deployment event
        struct phoenix_event_zbus_msg msg = {.event = PHOENIX_EVENT_FIRED_DROGUE};
        zbus_chan_pub(&phoenix_event_chan, &msg, K_NO_WAIT);

        LOG_INF("Drogue triggered: altitude=%.2f m, velocity=%.2f m/s", altitude, up_velocity);
    }
}

// PHOENIX_EVENT_MAIN_TRIGGERED
	// Trigger the main once launch and burnout and apogee have been detected
	// And we reach the main_deploy_altitude specified before launch

void main_triggered_Check() {
	if (main_triggered) { // Ensures that main is triggered only once
		return;
	}

    // Check prerequisites: launch, burnout, and apogee must have been detected
    if (!launch_detected || !burnout_detected || !apogee_detected) {
        return;
    }

    if (altitude < main_deploy_altitude && up_velocity < 0) {
        main_triggered = true; // Set the flag

        // Create and publish a main deployment event
        struct phoenix_event_zbus_msg msg = {.event = PHOENIX_EVENT_FIRED_MAIN};
        zbus_chan_pub(&phoenix_event_chan, &msg, K_NO_WAIT);

        LOG_INF("Main triggered: altitude=%.2f m", altitude);
    }
}

// PHOENIX_EVENT_LANDED
// Check if landing detected
// Criteria: 1. Launch, burnout, and apogee must have been detected, and drogues/mains have been triggered
// 2. Velocity z-axis value (velocity_zbus_chan) is close to 0 for an extended period
// If landing detected, publish PHOENIX_EVENT_LANDED message to phoenix_event_chan
void landed_Check() {
    if (landed_detected) {
        return; // Exit if landing already detected
    }

    // Check if apogee, burnout, and launch have been detected
    if (!(apogee_detected && burnout_detected && launch_detected && drogue_triggered && main_triggered)) {
        return; // Exit if any of the required events have not been detected
    }

    // Check for landing conditions
    bool near_zero_velocity = fabs(up_velocity) < 0.5; // Velocity close to 0
    bool near_launch_altitude = fabs(altitude - launch_altitude) / launch_altitude < 0.05; // Altitude within 5% of launch altitude

    if (near_zero_velocity && near_launch_altitude) {
        landed_detected = true; // Set the flag

        // Create and publish a landing event
        struct phoenix_event_zbus_msg msg = {.event = PHOENIX_EVENT_LANDED};
        zbus_chan_pub(&phoenix_event_chan, &msg, K_NO_WAIT);

        LOG_INF("Landed detected: velocity=%.2f m/s, altitude=%.2f m", up_velocity, altitude);
    }
}

// PHOENIX_EVENT_DROGUE_DEPLOYED
// Check if drogue deployed detected
// Criteria: Drogue deployed detected when launch, burnout, and apogee have been detected previously AND a spike in positive (upwards) acceleration has been detected
// If drogue deployed detected, publish PHOENIX_EVENT_DROGUE_DEPLOYED message to phoenix_event_chan

void drogue_deployed_Check(){
	if (drogue_deployed_detected) { // Ensure that drogue deployed is detected only once
		return;
	}

    // Check prerequisites: launch, burnout, and apogee must have been detected
    if (!launch_detected || !burnout_detected || !apogee_detected) {
        return; // Exit if any of the required events have not been detected
    }

    const bool accel_spike = up_accel > 5*g; // Check if the z-acceleration exceeds the threshold for drogue deployment

    if (accel_spike) {
        drogue_deployed_detected = true; // Set the flag

        // Create and publish a drogue deployment event
        struct phoenix_event_zbus_msg msg = {.event = PHOENIX_EVENT_DEPLOYED_DROGUE};
        zbus_chan_pub(&phoenix_event_chan, &msg, K_NO_WAIT);

        LOG_INF("Drogue deployed detected: accel=%.2f m/s^2", up_accel); // Log the detected drogue deployment event
    }
}

// PHOENIX_EVENT_DEPLOYED_MAIN
// Check if main deployed detected
// Criteria: Launch, burnout, apogee, and drogue triggered have been detected previously AND 
// a spike in positive (upwards) acceleration has been detected

void main_deployed_Check() {
    if (main_deployed_detected) {
        return; // Exit if main deployment already detected
    }

    // Ensure prerequisites: launch, burnout, apogee, and drogue must have been triggered
    if (!launch_detected || !burnout_detected || !apogee_detected || !drogue_triggered) {
        return;
    }

    // Check for main deployment condition - spike in positive acceleration
    // Adjust the threshold value as needed for your specific rocket
    const bool accel_spike = up_accel > 3*g; // Positive acceleration spike threshold

    if (accel_spike) {
        main_deployed_detected = true; // Set the flag
        
        // Create and publish a main deployed event
        struct phoenix_event_zbus_msg msg = {.event = PHOENIX_EVENT_DEPLOYED_MAIN};
        zbus_chan_pub(&phoenix_event_chan, &msg, K_NO_WAIT);
        
        LOG_INF("Main parachute deployed detected: accel_z=%.2f m/s^2", up_accel);
    }
}

// PHOENIX_EVENT_COCOM_LIMIT_EXCEEDED
// Check if COCOM limit exceeded detected
// Criteria: COCOM limit exceeded condition (e.g., altitude, velocity)
// COCOM alt = 12000 m
// COCOM vel = 510 m/s
// If COCOM limit exceeded detected, publish PHOENIX_EVENT_COCOM_LIMIT_EXCEEDED message to phoenix_event_chan
void COCOM_limit_Check() {
    // Extract velocity and altitude data from the EKF message
    float32_t velocity_mag = sqrtf( powf(east_velocity, 2) + powf(north_velocity, 2) + powf(up_velocity, 2)); // Calculate total velocity magnitude

    // Check for COCOM limit exceeded conditions
    const bool altitude_exceeded = altitude > COCOM_alt; // COCOM altitude limit
    const bool velocity_exceeded = velocity_mag > COCOM_velocity; // COCOM velocity limit

    if (altitude_exceeded || velocity_exceeded) {
        // Create and publish a COCOM limit exceeded event
        struct phoenix_event_zbus_msg msg = {.event = PHOENIX_EVENT_COCOM_LIMIT_EXCEEDED};
        zbus_chan_pub(&phoenix_event_chan, &msg, K_NO_WAIT);

        LOG_INF("COCOM limit exceeded: altitude=%.2f m, velocity=%.2f m/s", altitude, velocity_mag);
    }
}

struct TimedLinkedListNode {
    float value;
    float timestamp;
    struct TimedLinkedListNode* next;
};

struct TimedLinkedListNode* alt_last_5_secs_head = NULL;
struct TimedLinkedListNode* alt_last_5_secs_tail = NULL;
int alt_last_5_secs_count = 0;

void set_launch_lla(const struct zbus_channel *chan) {
    if (!launch_detected && &gnss_zbus_chan == chan) { // Check if the message is from the GNSS channel
        const struct gnss_zbus_msg *gnss_msg = zbus_chan_const_msg(chan); // Get the message from the channel
        launch_lla.lat = gnss_msg->pos.x;
        launch_lla.lon = gnss_msg->pos.y;

        uint64_t current_time = k_ticks_to_ms_floor64(k_uptime_ticks());

        // Keep only the last 10 seconds of data
        while (alt_last_5_secs_head != alt_last_5_secs_tail && (current_time - alt_last_5_secs_head->timestamp > 10000)) {
            struct TimedLinkedListNode* temp = alt_last_5_secs_head;
            alt_last_5_secs_head = alt_last_5_secs_head->next;
            k_free(temp);
            alt_last_5_secs_count -= 1;
        }

        struct TimedLinkedListNode* new_node = (struct TimedLinkedListNode*)k_malloc(sizeof(struct TimedLinkedListNode));
        new_node->value = gnss_msg->pos.z;
        new_node->timestamp = current_time;
        if (alt_last_5_secs_tail != NULL){
            alt_last_5_secs_tail->next = new_node;
        }
        if (alt_last_5_secs_head == NULL) {
            alt_last_5_secs_head = new_node;
        }
        alt_last_5_secs_tail = new_node;
        alt_last_5_secs_count += 1;
 

        // Update launch altitude as the average of the last few seconds
        struct TimedLinkedListNode* curr_node = alt_last_5_secs_head;
        float alt_sum = 0.0;
        while (curr_node != NULL) {
            alt_sum += curr_node->value;
            curr_node = curr_node->next;
        }
        launch_lla.alt = alt_sum / alt_last_5_secs_count;
    }
    if (launch_detected && alt_last_5_secs_head != NULL) {
        // Free the linked list
        struct TimedLinkedListNode* curr_node = alt_last_5_secs_head;
        while (curr_node != NULL) {
            struct TimedLinkedListNode* temp = curr_node;
            curr_node = curr_node->next;
            k_free(temp);
        }
        alt_last_5_secs_head = NULL;
        alt_last_5_secs_tail = NULL;
        alt_last_5_secs_count = 0;
    }
}
ZBUS_LISTENER_DEFINE(gnss_listener, set_launch_lla); // Define the listener for the GNSS channel
ZBUS_CHAN_ADD_OBS(gnss_zbus_chan, gnss_listener, 1); // Subscribe the listener to the GNSS channel

void event_check(const struct zbus_channel *chan) {
    if (&ekf_zbus_chan == chan) { // Check if the message is from the EKF channel
        const struct ekf_zbus_msg *ekf_msg = zbus_chan_const_msg(chan); // Get the message from the channel
        struct ECI eci = {
            .x = ekf_msg->pos_x,
            .y = ekf_msg->pos_y,
            .z = ekf_msg->pos_z
        };
        struct ECI eci_velocity = {
            .x = ekf_msg->vel_x,
            .y = ekf_msg->vel_y,
            .z = ekf_msg->vel_z
        };
        struct ECI eci_accel = {
            .x = ekf_msg->acc_x,
            .y = ekf_msg->acc_y,
            .z = ekf_msg->acc_z
        };

        ns_since_utc_epoch = utc_now();
        ns_to_time_utc(ns_since_utc_epoch, &utc);

        struct ENU rocket_enu;
        struct ENUVelocity rocket_enu_velocity;
        struct ENUAcceleration rocket_enu_accel;
        float32_t rocket_altitude;
        ECItoENU(
            utc,
            launch_lla,
            eci,
            eci_velocity,
            eci_accel,
            &rocket_enu,
            &rocket_enu_velocity,
            &rocket_enu_accel,
            &rocket_altitude
        );

        update_ekf_data(rocket_enu, rocket_enu_velocity, rocket_enu_accel, rocket_altitude);

        launch_Check();
        burnout_Check();
        apogee_Check();
        drogue_triggered_Check();
        main_triggered_Check();
        landed_Check();
        drogue_deployed_Check();
        main_deployed_Check();
        COCOM_limit_Check();
    }
}
ZBUS_LISTENER_DEFINE(event_listener, event_check); // Define the listener for the EKF channel
ZBUS_CHAN_ADD_OBS(ekf_zbus_chan, event_listener, 1); // Subscribe the listener to the EKF channel
