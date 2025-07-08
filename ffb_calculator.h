// ffb_calculator.h
#ifndef FFB_CALCULATOR_H
#define FFB_CALCULATOR_H

// Define effect types (should match what HID sends)
// These need to be in the header so other files can see them.
#define FFB_EFFECT_CONSTANT_FORCE 0
#define FFB_EFFECT_SPRING         1
#define FFB_EFFECT_DAMPER         2
// Add more as needed

// Define a simple structure for an FFB effect (conceptual)
typedef struct {
    int type;      // e.g., 0 for constant_force, 1 for spring, etc.
    float magnitude; // General magnitude for the effect
    int id;        // Effect ID
    // Add more parameters for specific effects (e.g., position for spring center, gain)
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
