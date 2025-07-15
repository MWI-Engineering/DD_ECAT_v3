// hid_interface.h - Updated header with new functions
#ifndef HID_INTERFACE_H
#define HID_INTERFACE_H

#include "ffb_types.h"

// Function prototypes
int hid_interface_init();
int hid_interface_start();
void hid_interface_stop();
int hid_interface_get_ffb_effect(ffb_effect_t *effect_out);
void hid_interface_send_gamepad_report(float position, unsigned int buttons);

// New functions for better control
void hid_interface_set_rate_limiting(int enable);
int hid_interface_set_blocking_mode();

#endif // HID_INTERFACE_H