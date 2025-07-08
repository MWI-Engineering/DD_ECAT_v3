// soem_interface.h
#ifndef SOEM_INTERFACE_H
#define SOEM_INTERFACE_H
#include <stdint.h> // For int32, etc.
#include <pthread.h> // For OSAL_THREAD_FUNC if it's defined here

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
// Add these new declarations:
void soem_interface_read_inputs(void);
void soem_interface_write_outputs(void);

// Also ensure this is declared if it's external:
// int soem_interface_init_master(const char *ifname); // Assuming it's already there

// If your ecat_loop is meant to be called from main, you might declare it here too:
// OSAL_THREAD_FUNC ecat_loop(void *ptr); // Or just void *ecat_loop(void *ptr);

#endif // SOEM_INTERFACE_H
