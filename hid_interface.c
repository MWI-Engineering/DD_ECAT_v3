// hid_interface.c
#include "hid_interface.h"
#include <stdio.h>
#include <pthread.h> // For threading

// Add this define before including unistd.h to help with usleep declaration
#include <unistd.h>  // For usleep
#include <stdlib.h>  // For malloc, free
#include <libusb-1.0/libusb.h>

// --- Conceptual USB queue ---
#define EP_OUT 0x01  // Replace with actual OUT endpoint address (Host to Device)
#define EP_IN  0x81  // Replace with actual IN endpoint address (Device to Host)
#define HID_REPORT_SIZE 64  // Adjust based on your HID report descriptor
#define USB_TIMEOUT 10  // Timeout in milliseconds

// Replace these with your actual VID and PID
#define USB_VENDOR_ID  0x046d // Example: Logitech
#define USB_PRODUCT_ID 0xc21d // Example: FFB Wheel
#define USB_INTERFACE  0      // Usually 0, unless specified

// --- Conceptual FFB effect queue ---
// In a real system, this would be populated by the HID driver/library
// receiving FFB output reports from the host PC.
#define FFB_EFFECT_QUEUE_SIZE 10
static ffb_effect_t ffb_effect_queue[FFB_EFFECT_QUEUE_SIZE];
static int queue_head = 0;
static int queue_tail = 0;
static int queue_count = 0;
static pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;

static int hid_running = 0;
static pthread_t ffb_reception_thread;

// USB context and device handle
static libusb_context *usb_ctx = NULL;
static libusb_device_handle *usb_dev_handle = NULL;

// --- Private function for simulating FFB reception ---
static void* _usb_ffb_reception_thread(void* arg) {
    uint8_t report[HID_REPORT_SIZE];
    int transferred;

    while (hid_running) {
        int rc = libusb_interrupt_transfer(
            usb_dev_handle,
            EP_OUT,
            report,
            HID_REPORT_SIZE,
            &transferred,
            USB_TIMEOUT
        );

        if (rc == 0 && transferred > 0) {
            pthread_mutex_lock(&queue_mutex);
            if (queue_count < FFB_EFFECT_QUEUE_SIZE) {
                // For now, convert report to a dummy effect
                ffb_effect_t new_effect;
                new_effect.type = FFB_EFFECT_CONSTANT_FORCE;
                new_effect.magnitude = report[1] / 255.0f;  // Example scaling
                new_effect.id = report[0];  // Or use another field
                ffb_effect_queue[queue_tail] = new_effect;
                queue_tail = (queue_tail + 1) % FFB_EFFECT_QUEUE_SIZE;
                queue_count++;
                pthread_cond_signal(&queue_cond);
            }
            pthread_mutex_unlock(&queue_mutex);
        } else if (rc != LIBUSB_ERROR_TIMEOUT) {
            fprintf(stderr, "HIDInterface: Failed to receive FFB report: %s\n", libusb_error_name(rc));
        }

        usleep(1000);  // Small delay to avoid high CPU load
    }

    return NULL;
}

/**
 * @brief Initializes the HID interface.
 * @return 0 on success, -1 on failure.
 */
int hid_interface_init() {
    int rc;

    // Initialize libusb
    rc = libusb_init(&usb_ctx);
    if (rc < 0) {
        fprintf(stderr, "HIDInterface: libusb_init failed: %s\n", libusb_error_name(rc));
        return -1;
    }

    // Open the device with known VID/PID
    usb_dev_handle = libusb_open_device_with_vid_pid(usb_ctx, USB_VENDOR_ID, USB_PRODUCT_ID);
    if (!usb_dev_handle) {
        fprintf(stderr, "HIDInterface: Unable to open USB device (VID=0x%04x, PID=0x%04x)\n", USB_VENDOR_ID, USB_PRODUCT_ID);
        libusb_exit(usb_ctx);
        return -1;
    }

    // Detach kernel driver if necessary
    if (libusb_kernel_driver_active(usb_dev_handle, USB_INTERFACE) == 1) {
        rc = libusb_detach_kernel_driver(usb_dev_handle, USB_INTERFACE);
        if (rc < 0) {
            fprintf(stderr, "HIDInterface: Failed to detach kernel driver: %s\n", libusb_error_name(rc));
            libusb_close(usb_dev_handle);
            libusb_exit(usb_ctx);
            return -1;
        }
    }

    // Claim the interface
    rc = libusb_claim_interface(usb_dev_handle, USB_INTERFACE);
    if (rc < 0) {
        fprintf(stderr, "HIDInterface: Failed to claim interface: %s\n", libusb_error_name(rc));
        libusb_close(usb_dev_handle);
        libusb_exit(usb_ctx);
        return -1;
    }

    printf("HIDInterface: USB device initialized and interface claimed.\n");
    return 0;
}

/**
 * @brief Starts the HID communication threads (e.g., for receiving FFB effects).
 * @return 0 on success, -1 on failure.
 */
int hid_interface_start() {
    hid_running = 1;
    if (pthread_create(&ffb_reception_thread, NULL, _usb_ffb_reception_thread, NULL) != 0) {
        perror("HIDInterface: Failed to create FFB reception thread");
        return -1;
    }
    printf("HIDInterface: Started FFB reception thread.\n");
    return 0; // Simulate success
}

/**
 * @brief Stops the HID communication and cleans up resources.
 */
void hid_interface_stop() {
    hid_running = 0;
    if (ffb_reception_thread) {
        pthread_join(ffb_reception_thread, NULL);
    }

    // Release USB interface and clean up
    if (usb_dev_handle) {
        libusb_release_interface(usb_dev_handle, USB_INTERFACE);
        libusb_close(usb_dev_handle);
        usb_dev_handle = NULL;
    }
    if (usb_ctx) {
        libusb_exit(usb_ctx);
        usb_ctx = NULL;
    }

    printf("HIDInterface: Stopped and cleaned up USB.\n");
}

/**
 * @brief Retrieves the latest FFB effect from the queue.
 * @param effect_out Pointer to an ffb_effect_t struct to store the effect.
 * @return 1 if an effect was retrieved, 0 if the queue is empty.
 */
int hid_interface_get_ffb_effect(ffb_effect_t *effect_out) {
    pthread_mutex_lock(&queue_mutex);
    if (queue_count > 0) {
        *effect_out = ffb_effect_queue[queue_head];
        queue_head = (queue_head + 1) % FFB_EFFECT_QUEUE_SIZE;
        queue_count--;
        pthread_mutex_unlock(&queue_mutex);
        return 1;
    }
    pthread_mutex_unlock(&queue_mutex);
    return 0;
}

/**
 * @brief Sends a gamepad report (position, buttons) to the PC.
 * @param position The current position of the steering wheel.
 * @param buttons The state of the buttons (bitmask).
 */
void hid_interface_send_gamepad_report(float position, unsigned int buttons) {
    if (!usb_dev_handle) return;

    uint8_t report[HID_REPORT_SIZE] = {0};

    // Example report format (you must adjust to match your real descriptor):
    // Byte 0: Report ID (optional)
    // Byte 1-2: Position (int16_t, scaled from -32767 to 32767)
    // Byte 3-4: Button state (bitmask)

    int16_t scaled_pos = (int16_t)(position * 32767.0f);  // Assumes position in [-1.0, 1.0]
    report[1] = scaled_pos & 0xFF;
    report[2] = (scaled_pos >> 8) & 0xFF;
    report[3] = buttons & 0xFF;
    report[4] = (buttons >> 8) & 0xFF;

    int transferred;
    int rc = libusb_interrupt_transfer(
        usb_dev_handle,
        EP_IN,
        report,
        HID_REPORT_SIZE,
        &transferred,
        USB_TIMEOUT
    );

    if (rc != 0) {
        fprintf(stderr, "HIDInterface: Failed to send gamepad report: %s\n", libusb_error_name(rc));
    } else {
        printf("HIDInterface: Sent gamepad report (%d bytes)\n", transferred);
    }
}