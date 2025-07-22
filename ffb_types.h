// ffb_types.h - Shared FFB types and structures
#ifndef FFB_TYPES_H
#define FFB_TYPES_H

#include <stdint.h>
#include <time.h>
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

/*
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
*/

// Enhanced FFB effect structure for your motor control
typedef struct {
    uint8_t report_id;
    ffb_effect_type_t type;
    uint8_t effect_type;    // Constant force, spring, damper, etc.
    float magnitude;        // -1.0 to 1.0
    float direction;        // 0 to 360 degrees
    int duration_ms;        // Duration in milliseconds
    uint32_t duration;
    uint32_t start_delay;
    uint32_t timestamp;

    // Additional parameters for different effect types
    float spring_coefficient;
    float damper_coefficient;
    float friction_coefficient;
    float inertia_coefficient;
    float center_position;
    float dead_band;
    
    // Envelope parameters
    float attack_level;
    int attack_time_ms;
    float fade_level;
    int fade_time_ms;
    
    // Timing
    struct timespec received_time;
} ffb_motor_effect_t;

#endif // FFB_TYPES_H