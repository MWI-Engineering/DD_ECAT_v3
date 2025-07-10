// hid_interface.c - Corrected FFB Parser
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

// FFB Report structures matching your HID descriptor
typedef struct {
    uint8_t report_id;
    uint8_t effect_id;
    uint8_t magnitude;
    uint8_t direction;
    uint8_t duration;
    uint8_t start_delay;
    uint8_t reserved[2];
} ffb_constant_force_report_t;

typedef struct {
    uint8_t report_id;
    uint8_t effect_id;
    uint8_t spring_coefficient;
    uint8_t damper_coefficient;
    uint8_t inertia_coefficient;
    uint8_t friction_coefficient;
    uint8_t reserved[2];
} ffb_condition_report_t;

// Parse FFB reports according to your HID descriptor
static int parse_ffb_report(uint8_t *report, int len, ffb_effect_t *effect) {
    if (len < 2) return 0;
    
    uint8_t report_id = report[0];
    
    switch (report_id) {
        case 0x01: // PID State Report
            printf("FFB: PID State Report received\n");
            // Handle device state changes
            return 0; // Not a force effect
            
        case 0x02: // Constant Force Effect
            if (len >= sizeof(ffb_constant_force_report_t)) {
                ffb_constant_force_report_t *cf = (ffb_constant_force_report_t *)report;
                effect->type = FFB_EFFECT_CONSTANT_FORCE;
                effect->id = cf->effect_id;
                // Convert magnitude from 0-255 to -1.0 to +1.0
                effect->magnitude = (cf->magnitude - 128) / 128.0f;
                effect->direction = cf->direction * 360.0f / 255.0f; // Convert to degrees
                effect->duration = cf->duration * 10; // Convert to milliseconds
                printf("FFB: Constant Force - ID:%d, Mag:%.2f, Dir:%.1f°\n", 
                       effect->id, effect->magnitude, effect->direction);
                return 1;
            }
            break;
            
        case 0x03: // Spring/Damper/Inertia Effects
            if (len >= sizeof(ffb_condition_report_t)) {
                ffb_condition_report_t *cond = (ffb_condition_report_t *)report;
                effect->id = cond->effect_id;
                
                // Determine primary effect type based on strongest coefficient
                if (cond->spring_coefficient > cond->damper_coefficient && 
                    cond->spring_coefficient > cond->inertia_coefficient) {
                    effect->type = FFB_EFFECT_SPRING;
                    effect->magnitude = cond->spring_coefficient / 255.0f;
                } else if (cond->damper_coefficient > cond->inertia_coefficient) {
                    effect->type = FFB_EFFECT_DAMPER;
                    effect->magnitude = cond->damper_coefficient / 255.0f;
                } else {
                    effect->type = FFB_EFFECT_INERTIA;
                    effect->magnitude = cond->inertia_coefficient / 255.0f;
                }
                
                printf("FFB: Condition Effect - ID:%d, Type:%d, Mag:%.2f\n",
                       effect->id, effect->type, effect->magnitude);
                return 1;
            }
            break;
            
        case 0x04: // Periodic Effects (Sine, Square, etc.)
            printf("FFB: Periodic Effect received (not implemented)\n");
            return 0;
            
        case 0x05: // Ramp Effects  
            printf("FFB: Ramp Effect received (not implemented)\n");
            return 0;
            
        case 0x06: // Effect Parameters
            printf("FFB: Effect Parameters received\n");
            return 0;
            
        case 0x07: // Envelope Parameters
            printf("FFB: Envelope Parameters received\n");
            return 0;
            
        case 0x08: // Device Control
            printf("FFB: Device Control received\n");
            return 0;
            
        default:
            printf("FFB: Unknown report ID: 0x%02X\n", report_id);
            return 0;
    }
    
    return 0;
}

// --- Private function for FFB reception ---
static void* _usb_ffb_reception_thread(void* arg) {
    uint8_t report[HID_REPORT_SIZE];

    printf("FFB: Reception thread started\n");
    
    while (hid_running) {
        int len = read(hidg_fd, report, HID_REPORT_SIZE);
        if (len > 0) {
            printf("FFB: Received %d bytes: ", len);
            for (int i = 0; i < len && i < 8; i++) {
                printf("0x%02X ", report[i]);
            }
            printf("\n");
            
            ffb_effect_t new_effect;
            if (parse_ffb_report(report, len, &new_effect)) {
                pthread_mutex_lock(&queue_mutex);
                if (queue_count < FFB_EFFECT_QUEUE_SIZE) {
                    ffb_effect_queue[queue_tail] = new_effect;
                    queue_tail = (queue_tail + 1) % FFB_EFFECT_QUEUE_SIZE;
                    queue_count++;
                    pthread_cond_signal(&queue_cond);
                } else {
                    printf("FFB: Effect queue full, dropping effect\n");
                }
                pthread_mutex_unlock(&queue_mutex);
            }
        } else if (len < 0 && errno != EAGAIN) {
            perror("FFB: Error reading from hidg0");
            break;
        }

        usleep(1000); // 1ms polling
    }

    printf("FFB: Reception thread stopped\n");
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
 * @brief Sends a gamepad report (position, buttons) to the PC via USB gadget.
 * @param position The current position of the steering wheel.
 * @param buttons The state of the buttons (bitmask).
 */
void hid_interface_send_gamepad_report(float position, unsigned int buttons) {
    if (hidg_fd < 0) return;

    uint8_t report[HID_REPORT_SIZE] = {0};

    // Format matching your HID descriptor:
    // Bytes 0-1: Steering wheel position (int16_t, scaled from -32767 to 32767)
    // Bytes 2-3: Button state (16 buttons, bitmask)
    
    int16_t scaled_pos = (int16_t)(position * 32767.0f);  // Assumes position in [-1.0, 1.0]
    report[0] = scaled_pos & 0xFF;
    report[1] = (scaled_pos >> 8) & 0xFF;
    report[2] = buttons & 0xFF;
    report[3] = (buttons >> 8) & 0xFF;

    int bytes_written = write(hidg_fd, report, HID_REPORT_SIZE);
    if (bytes_written < 0) {
        perror("HIDInterface: Failed to send gamepad report");
    }
    // Remove the debug print to avoid spam
    // printf("HIDInterface: Sent gamepad report (%d bytes)\n", bytes_written);
}