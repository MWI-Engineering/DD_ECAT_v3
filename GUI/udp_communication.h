// udp_communication.h - Header file for UDP communication
#ifndef UDP_COMMUNICATION_H
#define UDP_COMMUNICATION_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

// System status structure
typedef struct {
    // Wheel status
    float wheel_position;       // Current wheel position (-1.0 to 1.0)
    float wheel_speed;          // Wheel speed in RPM
    uint16_t button_state;      // Button state bitmask
    
    // FFB status
    bool ffb_enabled;           // FFB enabled/disabled
    float force_multiplier;     // Force multiplier (0.0 to 2.0)
    char current_effect[64];    // Current active effect name
    
    // Motor status
    float motor_current;        // Motor current in Amperes
    float motor_temperature;    // Motor temperature in Celsius
    float motor_voltage;        // Motor voltage in Volts
    float motor_load;           // Motor load percentage (0.0 to 100.0)
    
    // System status
    bool is_calibrated;         // Calibration status
    bool emergency_stop;        // Emergency stop status
    time_t timestamp;           // Status timestamp
} system_status_t;

// Function prototypes
int udp_communication_init(void);
int udp_communication_start(void);
void udp_communication_stop(void);
void udp_get_system_status(system_status_t* status);
void udp_set_emergency_stop(bool emergency_stop);
void udp_log_event(const char* event);

#endif // UDP_COMMUNICATION_H
