// ffb_calculator.c
#include "ffb_calculator.h"
#include <stdio.h> // For printf (for debugging)
#include <math.h>  // For fmax, fmin

// The FFB_EFFECT defines have been moved to ffb_calculator.h

/**
 * @brief Initializes the FFB calculator.
 */
void ffb_calculator_init() {
    printf("FFB_Calculator: Initialized.\n");
    // Any setup for internal state variables can go here
}

/**
 * @brief Calculates the desired torque based on the FFB effect and current wheel position.
 * @param effect Pointer to the current FFB effect. Can be NULL if no effect.
 * @param current_position The current angular position of the steering wheel.
 * @param current_velocity The current angular velocity of the steering wheel (for damper/inertia).
 * @return The calculated desired torque value.
 */
float ffb_calculator_calculate_torque(const ffb_effect_t *effect, float current_position, float current_velocity) {
    float desired_torque = 0.0f;

    if (effect != NULL) {
        switch (effect->type) {
            case FFB_EFFECT_CONSTANT_FORCE:
                // Apply a constant force based on magnitude
                // Scale factor (e.g., 1000) converts normalized magnitude to motor units
                desired_torque = effect->magnitude * 1000.0f;
                break;
            case FFB_EFFECT_SPRING:
                // Spring effect: Torque proportional to displacement from a center point.
                // A real spring effect would also have a center position and saturation.
                float center_position = 0.0f; // Assuming center is 0
                float spring_gain = 0.8f;     // Example gain (should come from effect parameters)
                desired_torque = -(current_position - center_position) * spring_gain * 500.0f; // Scale
                break;
            case FFB_EFFECT_DAMPER:
                // Damper effect: Torque proportional to velocity, opposing motion.
                float damper_gain = 1.0f;     // Example gain (should come from effect parameters)
                desired_torque = -current_velocity * damper_gain * 200.0f; // Scale
                break;
            // Add more FFB effect types as needed (friction, inertia, ramp, sine, etc.)
            default:
                // Unknown effect type, apply no force
                desired_torque = 0.0f;
                break;
        }
    }

    // Clamp torque to motor limits
    float max_torque_output = 5000.0f; // Example max motor command units
    float min_torque_output = -5000.0f; // Example min motor command units
    desired_torque = fmaxf(min_torque_output, fminf(max_torque_output, desired_torque));

    return desired_torque;
}
