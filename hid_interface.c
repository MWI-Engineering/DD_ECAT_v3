// hid_interface.c
#include "hid_interface.h"
#include <stdio.h>
#include <pthread.h> // For threading

// Add this define before including unistd.h to help with usleep declaration
#define _DEFAULT_SOURCE
#include <unistd.h>  // For usleep
#include <stdlib.h>  // For malloc, free

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

// --- Private function for simulating FFB reception ---
static void* _simulate_ffb_reception_thread(void* arg) {
    int effect_id = 0;
    while (hid_running) {
        pthread_mutex_lock(&queue_mutex);
        if (queue_count < FFB_EFFECT_QUEUE_SIZE) {
            ffb_effect_t dummy_effect;
            dummy_effect.type = FFB_EFFECT_CONSTANT_FORCE; // Simulate constant force
            dummy_effect.magnitude = 0.5f; // Example magnitude
            dummy_effect.id = effect_id;

            ffb_effect_queue[queue_tail] = dummy_effect;
            queue_tail = (queue_tail + 1) % FFB_EFFECT_QUEUE_SIZE;
            queue_count++;
            pthread_cond_signal(&queue_cond); // Signal that an item is available
            effect_id = (effect_id + 1) % 10;
        }
        pthread_mutex_unlock(&queue_mutex);
        usleep(100000); // Simulate receiving effects at 10Hz (100ms interval)
                       // A real FFB system would expect 100-1000Hz updates.
    }
    return NULL;
}

/**
 * @brief Initializes the HID interface.
 * @return 0 on success, -1 on failure.
 */
int hid_interface_init() {
    printf("HIDInterface: Initialized (conceptual).\n");
    // In a real implementation:
    // - Initialize USB device connection (e.g., libusb_init)
    // - Configure HID descriptors (this is complex for FFB)
    // - Open the USB device/interface
    return 0; // Simulate success
}

/**
 * @brief Starts the HID communication threads (e.g., for receiving FFB effects).
 * @return 0 on success, -1 on failure.
 */
int hid_interface_start() {
    hid_running = 1;
    if (pthread_create(&ffb_reception_thread, NULL, _simulate_ffb_reception_thread, NULL) != 0) {
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
    printf("HIDInterface: Stopped.\n");
    // In a real implementation:
    // - Close USB device
    // - Deinitialize USB library
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
    // In a real implementation:
    // - Create a HID report structure
    // - Populate it with position (scaled to HID report range, e.g., -32767 to 32767)
    // - Populate it with button states
    // - Send the report via USB (e.g., using hid_write or libusb_interrupt_transfer)
    // printf("HIDInterface: Sending gamepad report - Position: %.2f, Buttons: %u\n", position, buttons);
}
