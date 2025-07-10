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

// FFB Effect Structure
typedef struct {
    ffb_effect_type_t type;     // Effect type (constant, spring, damper, etc.)
    uint8_t id;                 // Effect ID (0-255)
    float magnitude;            // Effect magnitude (-1.0 to 1.0 or 0.0 to 1.0)
    float direction;            // Direction in degrees (0-360)
    uint16_t duration;          // Duration in milliseconds
    uint16_t start_delay;       // Start delay in milliseconds
    
    // Additional parameters for condition effects
    float spring_coefficient;   // Spring coefficient (0.0 to 1.0)
    float damper_coefficient;   // Damper coefficient (0.0 to 1.0)
    float inertia_coefficient;  // Inertia coefficient (0.0 to 1.0)
    float friction_coefficient; // Friction coefficient (0.0 to 1.0)
    
    // Timestamp for effect management
    uint32_t timestamp;         // When the effect was received
} ffb_effect_t;

// FFB State Structure (if needed for device state)
typedef struct {
    uint8_t device_paused;      // Device is paused
    uint8_t actuators_enabled;  // Actuators are enabled
    // uint8_t safety_switch;   // Safety switch state, not needed for Synapticon servomotor due to intergrated safety circuit, adjust if needed.
    uint8_t actuator_override;  // Actuator override switch
    uint8_t actuator_power;     // Actuator power level (0-100)
} ffb_device_state_t;

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
