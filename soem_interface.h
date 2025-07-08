// soem_interface.h
#ifndef SOEM_INTERFACE_H
#define SOEM_INTERFACE_H

// Function prototypes for SOEM interface
/**
 * @brief Initializes the SOEM master and discovers slaves.
 * @param ifname The network interface name (e.g., "eth0").
 * @return 0 on success, -1 on failure.
 */
int soem_interface_init_master(const char *ifname);

/**
 * @brief Sends process data (e.g., target torque) and receives process data (e.g., actual position).
 * This is the core real-time loop function.
 * @param target_torque The desired torque to send to the servo motor.
 */
void soem_interface_send_and_receive_pdo(float target_torque);

/**
 * @brief Returns the last known position from the servo motor.
 * @return The current angular position.
 */
float soem_interface_get_current_position();

/**
 * @brief Returns the last known velocity from the servo motor.
 * @return The current angular velocity.
 */
float soem_interface_get_current_velocity();


/**
 * @brief Stops the SOEM master and cleans up resources.
 */
void soem_interface_stop_master();

#endif // SOEM_INTERFACE_H
