// ffb_calculator.h
#ifndef FFB_CALCULATOR_H
#define FFB_CALCULATOR_H

#include <stdint.h>

// FFB Effect Types - These constants are used in your hid_interface.c
typedef enum {
    FFB_EFFECT_CONSTANT_FORCE = 1,
    FFB_EFFECT_SPRING = 2,
    FFB_EFFECT_DAMPER = 3,
    FFB_EFFECT_INERTIA = 4,
    FFB_EFFECT_FRICTION = 5,
    FFB_EFFECT_PERIODIC = 6,
    FFB_EFFECT_RAMP = 7
} ffb_effect_type_t;
// Add more as needed

// FFB Effect Structure - Based on usage in your hid_interface.c
typedef struct {
    ffb_effect_type_t type;     // Effect type (constant, spring, damper, etc.)
    uint8_t id;                 // Effect ID (0-255)
    float magnitude;            // Effect magnitude (-1.0 to 1.0 or 0.0 to 1.0)
    float direction;            // Direction in degrees (0-360)
    uint16_t duration;          // Duration in milliseconds
    // Add other fields as needed for your application
} ffb_effect_t;

/**
 * @brief Initializes the FFB calculator.
 */
void ffb_calculator_init();

/**
 * @brief Calculates the desired torque based on the FFB effect and current wheel position.
 * @param effect Pointer to the current FFB effect. Can be NULL if no effect.
 * @param current_position The current angular position of the steering wheel.
 * @param current_velocity The current angular velocity of the steering wheel (for damper/inertia).
 * @return The calculated desired torque value.
 */
float ffb_calculator_calculate_torque(const ffb_effect_t *effect, float current_position, float current_velocity);

#endif // FFB_CALCULATOR_H
