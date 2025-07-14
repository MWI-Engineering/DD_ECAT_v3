// hid_interface.h
#ifndef HID_INTERFACE_H
#define HID_INTERFACE_H

#include <stdint.h>
#include "ffb_calculator.h" // Include for ffb_effect_t

// --- External Function Prototypes (from soem_interface.h, but duplicated here for convenience) ---
// It's generally better to only include soem_interface.h if these are truly needed here,
// but for now, we'll correct the type.
#include "ethercat.h" // Needed for ec_state
int soem_interface_init_master(const char *ifname);
void soem_interface_send_and_receive_pdo(float target_torque);
float soem_interface_get_current_position(void);
float soem_interface_get_current_velocity(void);
int soem_interface_get_communication_status(void);
void soem_interface_stop_master(void);
int soem_interface_write_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex, uint16_t data_size, void *data);
// THIS IS THE LINE THAT WAS CAUSING THE ERROR:
int soem_interface_set_ethercat_state(uint16_t slave_idx, ec_state desired_state);
int soem_interface_configure_pdo_mapping(uint16_t slave_idx, uint16_t pdo_assign_idx, uint16_t pdo_map_idx, uint32_t *mapped_objects, uint8_t num_mapped_objects);

// --- HID Interface Function Prototypes ---

/**
 * @brief Initializes the HID interface.
 * @return 0 on success, -1 on failure.
 */
int hid_interface_init(void);

/**
 * @brief Starts the HID communication threads (e.g., for receiving FFB effects).
 * @return 0 on success, -1 on failure.
 */
int hid_interface_start(void);

/**
 * @brief Stops the HID communication and cleans up resources.
 */
void hid_interface_stop(void);

/**
 * @brief Retrieves the latest FFB effect from the queue.
 * @param effect_out Pointer to an ffb_effect_t struct to store the effect.
 * @return 1 if an effect was retrieved, 0 if the queue is empty.
 */
int hid_interface_get_ffb_effect(ffb_effect_t *effect_out);

/**
 * @brief Sends a gamepad report (position, buttons) to the PC via USB gadget.
 * @param position The current position of the steering wheel.
 * @param buttons The state of the buttons (bitmask).
 */
void hid_interface_send_gamepad_report(float position, unsigned int buttons);

#endif // HID_INTERFACE_H
