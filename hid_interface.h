// hid_interface.h - Updated header with new functions for better error handling
#ifndef HID_INTERFACE_H
#define HID_INTERFACE_H

#include "ffb_types.h"

// Global running flag
extern volatile int hid_running;

// Core function prototypes
int hid_interface_init();
int hid_interface_start();
void hid_interface_stop();
int hid_interface_get_ffb_effect(ffb_effect_t *effect_out);
void hid_interface_send_gamepad_report(float position, unsigned int buttons);

// Configuration functions
void hid_interface_set_rate_limiting(int enable);
void hid_interface_set_report_interval(int interval_ms);

// Status and diagnostics functions
int hid_interface_get_connection_status();
void hid_interface_get_stats(int *write_errors, int *read_errors, int *reconnects);

#endif // HID_INTERFACE_H