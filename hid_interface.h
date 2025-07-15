// hid_interface.h - Updated header with new functions
#ifndef HID_INTERFACE_H
#define HID_INTERFACE_H

#include <stdint.h>

// FFB effect types
typedef enum {
    FFB_EFFECT_CONSTANT_FORCE = 0,
    FFB_EFFECT_SPRING,
    FFB_EFFECT_DAMPER,
    FFB_EFFECT_INERTIA,
    FFB_EFFECT_FRICTION,
    FFB_EFFECT_PERIODIC,
    FFB_EFFECT_RAMP
} ffb_effect_type_t;

// FFB effect structure
typedef struct {
    uint8_t id;
    ffb_effect_type_t type;
    float magnitude;
    float direction;
    uint32_t duration;
    uint32_t start_delay;
    uint32_t timestamp;
    
    // Additional parameters for different effect types
    float spring_coefficient;
    float damper_coefficient;
    float inertia_coefficient;
    float friction_coefficient;
} ffb_effect_t;

// Function prototypes
int hid_interface_init();
int hid_interface_start();
void hid_interface_stop();
int hid_interface_get_ffb_effect(ffb_effect_t *effect_out);
void hid_interface_send_gamepad_report(float position, unsigned int buttons);

// New functions for better control
void hid_interface_set_rate_limiting(int enable);
int hid_interface_set_blocking_mode();

#endif // HID_INTERFACE_H