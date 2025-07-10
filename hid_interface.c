// hid_interface.c
// New version with exposing the HID interface on /dev/hidg0
#include "hid_interface.h"
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>     // usleep, read, write
#include <fcntl.h>      // open
#include <stdlib.h>
#include <string.h>     // memset
#include <errno.h>

#define HID_DEVICE_PATH "/dev/hidg0"
#define HID_REPORT_SIZE 64

static int hidg_fd = -1;

#define FFB_EFFECT_QUEUE_SIZE 10
static ffb_effect_t ffb_effect_queue[FFB_EFFECT_QUEUE_SIZE];
static int queue_head = 0;
static int queue_tail = 0;
static int queue_count = 0;
static pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;

static int hid_running = 0;
static pthread_t ffb_reception_thread;

// --- Private function for simulating FFB reception ---
static void* _usb_ffb_reception_thread(void* arg) {
    uint8_t report[HID_REPORT_SIZE];

    while (hid_running) {
        int len = read(hidg_fd, report, HID_REPORT_SIZE);
        if (len > 0) {
            pthread_mutex_lock(&queue_mutex);
            if (queue_count < FFB_EFFECT_QUEUE_SIZE) {
                ffb_effect_t new_effect;
                // Very basic FFB effect parsing
                switch (report[0]) {
                    case 0x01:  // Constant force
                        new_effect.type = FFB_EFFECT_CONSTANT_FORCE;
                        new_effect.magnitude = report[1] / 255.0f;
                        break;
                    case 0x02:  // Spring
                        new_effect.type = FFB_EFFECT_SPRING;
                        new_effect.magnitude = report[2] / 255.0f;
                        break;
                    case 0x03:  // Damper
                        new_effect.type = FFB_EFFECT_DAMPER;
                        new_effect.magnitude = report[2] / 255.0f;
                        break;
                    default:
                        new_effect.type = FFB_EFFECT_CONSTANT_FORCE;
                        new_effect.magnitude = 0.0f;
                }
                new_effect.id = report[0];
                ffb_effect_queue[queue_tail] = new_effect;
                queue_tail = (queue_tail + 1) % FFB_EFFECT_QUEUE_SIZE;
                queue_count++;
                pthread_cond_signal(&queue_cond);
            }
            pthread_mutex_unlock(&queue_mutex);
        }

        usleep(1000);
    }

    return NULL;
}

/**
 * @brief Initializes the HID interface.
 * @return 0 on success, -1 on failure.
 */
int hid_interface_init() {
    hidg_fd = open(HID_DEVICE_PATH, O_RDWR | O_NONBLOCK);
    if (hidg_fd < 0) {
        perror("HIDInterface: Failed to open /dev/hidg0");
        return -1;
    }

    printf("HIDInterface: Opened /dev/hidg0 for USB HID gadget.\n");
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
    return 0;
}

/**
 * @brief Stops the HID communication and cleans up resources.
 */
void hid_interface_stop() {
    hid_running = 0;
    if (ffb_reception_thread) {
        pthread_join(ffb_reception_thread, NULL);
    }

    if (hidg_fd >= 0) {
        close(hidg_fd);
        hidg_fd = -1;
    }

    printf("HIDInterface: Closed /dev/hidg0 and stopped.\n");
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
