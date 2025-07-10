// soem_interface.h - Improved header for SOEM EtherCAT interface
#ifndef SOEM_INTERFACE_H
#define SOEM_INTERFACE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- Function Prototypes ---

/**
 * @brief Initializes the SOEM master and discovers slaves.
 * @param ifname The network interface name (e.g., "eth0").
 * @return 0 on success, -1 on failure.
 */
int soem_interface_init_master(const char *ifname);

/**
 * @brief Sends target torque to the EtherCAT thread.
 * The actual PDO communication happens in the EtherCAT thread.
 * @param target_torque The desired torque to send to the servo motor.
 */
void soem_interface_send_and_receive_pdo(float target_torque);

/**
 * @brief Returns the last known position from the servo motor.
 * @return The current angular position in degrees.
 */
float soem_interface_get_current_position(void);

/**
 * @brief Returns the last known velocity from the servo motor.
 * @return The current angular velocity in degrees/second.
 */
float soem_interface_get_current_velocity(void);

/**
 * @brief Returns the communication status.
 * @return 1 if communication is OK, 0 if there are communication errors.
 */
int soem_interface_get_communication_status(void);

/**
 * @brief Stops the SOEM master and cleans up resources.
 */
void soem_interface_stop_master(void);

// --- Constants ---

// Scaling factors for unit conversion
#define SOEM_POSITION_SCALE_FACTOR (360.0f / 1000000.0f)  // Convert from encoder counts to degrees
#define SOEM_VELOCITY_SCALE_FACTOR (1.0f / 1000.0f)       // Convert from internal units to deg/s
#define SOEM_TORQUE_SCALE_FACTOR   (1000.0f)              // Convert from float torque to motor units

// EtherCAT state machine control words
#define SOEM_CONTROLWORD_SHUTDOWN           0x06  // Ready to switch on
#define SOEM_CONTROLWORD_SWITCH_ON          0x07  // Switch on
#define SOEM_CONTROLWORD_OPERATION_ENABLED  0x0F  // Operation enabled
#define SOEM_CONTROLWORD_FAULT_RESET        0x80  // Fault reset

// EtherCAT status word masks
#define SOEM_STATUSWORD_FAULT_MASK          0x08  // Fault bit mask
#define SOEM_STATUSWORD_OPERATION_ENABLED   0x37  // Operation enabled state

// Modes of operation
#define SOEM_MODE_CYCLIC_SYNC_TORQUE        0x0A  // Cyclic Synchronous Torque mode

// --- Error Codes ---
#define SOEM_SUCCESS                         0
#define SOEM_ERROR_INIT_FAILED              -1
#define SOEM_ERROR_NO_SLAVES                -2
#define SOEM_ERROR_CONFIG_FAILED            -3
#define SOEM_ERROR_STATE_FAILED             -4
#define SOEM_ERROR_THREAD_FAILED            -5

#ifdef __cplusplus
}
#endif

#endif // SOEM_INTERFACE_H