// hid_interface.h
#ifndef HID_INTERFACE_H
#define HID_INTERFACE_H

#include "ffb_calculator.h" // To use ffb_effect_t

// Function prototypes for HID interface
/**
 * @brief Initializes the HID interface.
 * @return 0 on success, -1 on failure.
 */
int hid_interface_init();

/**
 * @brief Starts the HID communication threads (e.g., for receiving FFB effects).
 * @return 0 on success, -1 on failure.
 */
int hid_interface_start();

/**
 * @brief Stops the HID communication and cleans up resources.
 */
void hid_interface_stop();

/**
 * @brief Retrieves the latest FFB effect from the queue.
 * @param effect_out Pointer to an ffb_effect_t struct to store the effect.
 * @return 1 if an effect was retrieved, 0 if the queue is empty.
 */
int hid_interface_get_ffb_effect(ffb_effect_t *effect_out);

/**
 * @brief Sends a gamepad report (position, buttons) to the PC.
 * @param position The current position of the steering wheel.
 * @param buttons The state of the buttons (bitmask).
 */
void hid_interface_send_gamepad_report(float position, unsigned int buttons);

#endif // HID_INTERFACE_H
