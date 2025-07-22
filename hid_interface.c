// hid_interface.c - Improved version with better write error handling and adaptive timing
#include "hid_interface.h"
#include "soem_interface.h"
#include "ffb_types.h"

#include <math.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <sys/time.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sched.h> // For CPU affinity

#define HID_DEVICE_PATH "/dev/hidg0"
// Increased send interval to reduce USB bus load and buffer overflow
#define HID_SEND_INTERVAL_MS 20  // Send reports every 20ms (50Hz) - more reasonable for USB
#define USB_RECONNECT_DELAY_MS 1000  // Wait 1 second before trying to reconnect
#define ENCODER_COUNTS_PER_REV 4096.0f
#define MAX_STEERING_ANGLE 540.0f // Adjusted to 540 degrees as per main.c

// Reduced retries since we're using longer timeouts
#define MAX_WRITE_RETRIES 3 // This is effectively ignored for non-blocking writes
// Longer delay between retries to let USB buffer clear
#define WRITE_RETRY_DELAY_US 1000
// Longer select timeout for writes
#define WRITE_SELECT_TIMEOUT_US 20000  // Increased to 20ms

// Add adaptive timing parameters
#define MIN_SEND_INTERVAL_MS 16   // Minimum 16ms (62.5Hz max)
#define MAX_SEND_INTERVAL_MS 100  // Maximum 100ms (10Hz min)
static int current_send_interval_ms = HID_SEND_INTERVAL_MS;
static int consecutive_successes = 0;
static int adaptive_timing_enabled = 1;

// Structure matching standard HID gamepad Report ID 1
// Corrected: Removed padding to exactly match descriptor (1 byte ID + 2 bytes X-axis + 2 bytes Buttons = 5 bytes)
typedef struct {
    uint8_t report_id;      // Report ID = 1
    int16_t x_axis;         // 16-bit signed X-axis (-32768 to 32767)
    uint16_t buttons;       // 16-bit button field
} __attribute__((packed)) gamepad_report_t;

// Fixed FFB Input Report structures matching your HID descriptor
typedef struct {
    uint8_t report_id;      // Report ID = 2
    uint8_t effect_block_index;  // Effect block index
    uint8_t effect_type;    // Effect type
} __attribute__((packed)) ffb_pid_pool_report_t;

typedef struct {
    uint8_t report_id;      // Report ID = 3
    uint8_t magnitude;      // Effect magnitude
    uint8_t offset;         // Effect offset
    uint8_t phase;          // Phase
    uint8_t period;         // Period
    uint8_t duration_low;   // Duration low byte
    uint8_t duration_high;  // Duration high byte
} __attribute__((packed)) ffb_set_effect_report_t;

typedef struct {
    uint8_t report_id;      // Report ID = 4
    uint8_t cp_offset;      // CP Offset
    uint8_t positive_coefficient; // Positive Coefficient
    uint8_t negative_coefficient; // Negative Coefficient
    uint8_t positive_saturation;  // Positive Saturation
    uint8_t negative_saturation;  // Negative Saturation
    uint8_t dead_band;      // Dead Band
} __attribute__((packed)) ffb_set_envelope_report_t;

typedef struct {
    uint8_t report_id;      // Report ID = 5
    uint8_t condition_type; // Condition type
    uint8_t cp_offset;      // CP Offset
    uint8_t positive_coefficient; // Positive Coefficient
    uint8_t negative_coefficient; // Negative Coefficient
    uint8_t positive_saturation;  // Positive Saturation
    uint8_t negative_saturation;  // Negative Saturation
    uint8_t dead_band;      // Dead Band
    uint8_t center;         // Center
} __attribute__((packed)) ffb_set_condition_report_t;

typedef struct {
    uint8_t report_id;      // Report ID = 6
    uint8_t direction_low;  // Direction low byte
    uint8_t direction_high; // Direction high byte
    uint8_t type_specific_block_offset_low;
    uint8_t type_specific_block_offset_high;
    uint8_t type_specific_block_offset_param1;
    uint8_t type_specific_block_offset_param2;
} __attribute__((packed)) ffb_set_periodic_report_t;

typedef struct {
    uint8_t report_id;      // Report ID = 7
    uint8_t attack_level;   // Attack Level
    uint8_t attack_time_low; // Attack Time Low
    uint8_t attack_time_high; // Attack Time High
    uint8_t fade_level;     // Fade Level
    uint8_t fade_time_low;  // Fade Time Low
    uint8_t fade_time_high; // Fade Time High
    uint8_t param7;
    uint8_t param8;
    uint8_t param9;
    uint8_t param10;
    uint8_t param11;
    uint8_t param12;
    uint8_t param13;
    uint8_t param14;
    uint8_t param15;
    uint8_t param16;
} __attribute__((packed)) ffb_set_constant_force_report_t;

// Generic FFB report for any report ID
typedef struct {
    uint8_t report_id;
    uint8_t data[16];  // Maximum data size based on your largest report
} __attribute__((packed)) ffb_generic_report_t;

// Encoder state variables
static float center_position = 0.0f;
static volatile int position_initialized = 0;
static pthread_mutex_t position_mutex = PTHREAD_MUTEX_INITIALIZER;

// FFB Effect Queue
#define FFB_EFFECT_QUEUE_SIZE 16
static ffb_motor_effect_t ffb_effect_queue[FFB_EFFECT_QUEUE_SIZE];
static int queue_head = 0;
static int queue_tail = 0;
static int queue_count = 0;
static pthread_mutex_t queue_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t queue_cond = PTHREAD_COND_INITIALIZER;

// Thread running flag
volatile int hid_running = 0;

static int hidg_fd = -1;
static pthread_mutex_t hidg_fd_mutex = PTHREAD_MUTEX_INITIALIZER;

// USB connection state tracking
static volatile int usb_connected = 0;
static int consecutive_write_failures = 0;
static struct timespec last_reconnect_attempt = {0, 0};

// Error tracking
static int total_write_errors = 0;
static int total_read_errors = 0;
static int reconnect_count = 0;

// Debugging - reduced frequency to avoid spam
static int report_counter = 0;
static int debug_every_n_reports = 250;  // Debug every 250 reports (about once per 5 seconds at 50Hz)

// Threads
static pthread_t ffb_reception_thread;
static pthread_t gamepad_report_thread;

// Position caching to avoid unnecessary SOEM calls
static float cached_position = 0.0f;
static struct timespec last_position_update = {0, 0};
static pthread_mutex_t cache_mutex = PTHREAD_MUTEX_INITIALIZER;
#define POSITION_CACHE_INTERVAL_MS 5  // Update position cache every 5ms

// Add this helper function for time comparison
static long timespec_diff_ms(struct timespec *start, struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000 + (end->tv_nsec - start->tv_nsec) / 1000000;
}

// Check if HID device exists and is accessible
static int check_hid_device_exists() {
    struct stat st;
    if (stat(HID_DEVICE_PATH, &st) != 0) {
        return 0;
    }
    return S_ISCHR(st.st_mode);
}

// Safe function to close HID device
static void safe_close_hid_device() {
    pthread_mutex_lock(&hidg_fd_mutex);
    if (hidg_fd >= 0) {
        close(hidg_fd);
        hidg_fd = -1; // Invalidate the file descriptor
        usb_connected = 0;
        printf("HIDInterface: Closed HID device\n");
    }
    pthread_mutex_unlock(&hidg_fd_mutex);
}

// Attempt to open/reopen HID device
static int try_open_hid_device() {
    pthread_mutex_lock(&hidg_fd_mutex);
    
    // Close existing connection if any before trying to reopen
    if (hidg_fd >= 0) {
        close(hidg_fd);
        hidg_fd = -1;
    }
    
    // Check if device exists
    if (!check_hid_device_exists()) {
        pthread_mutex_unlock(&hidg_fd_mutex);
        return -1;
    }
    
    // Try to open the device
    hidg_fd = open(HID_DEVICE_PATH, O_RDWR | O_NONBLOCK);
    if (hidg_fd < 0) {
        perror("HIDInterface: Failed to open HID device");
        pthread_mutex_unlock(&hidg_fd_mutex);
        return -1;
    }
    
    usb_connected = 1;
    consecutive_write_failures = 0;
    consecutive_successes = 0;
    reconnect_count++;
    
    // Reset adaptive timing on reconnect
    current_send_interval_ms = HID_SEND_INTERVAL_MS;
    
    printf("HIDInterface: Successfully opened HID device (reconnect #%d)\n", reconnect_count);
    
    pthread_mutex_unlock(&hidg_fd_mutex);
    return 0;
}

// Check if we should attempt reconnection
static int should_attempt_reconnect() {
    if (usb_connected) return 0;
    
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    if (last_reconnect_attempt.tv_sec != 0 || last_reconnect_attempt.tv_nsec != 0) {
        long elapsed_ms = timespec_diff_ms(&last_reconnect_attempt, &current_time);
        if (elapsed_ms < USB_RECONNECT_DELAY_MS) {
            return 0;
        }
    }
    
    last_reconnect_attempt = current_time;
    return 1;
}

// Cached position calculation to reduce SOEM interface calls
static float get_cached_wheel_position() {
    pthread_mutex_lock(&cache_mutex);
    
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    // Check if we need to update the cache
    if (last_position_update.tv_sec == 0 || 
        timespec_diff_ms(&last_position_update, &current_time) >= POSITION_CACHE_INTERVAL_MS) {
        
        pthread_mutex_lock(&position_mutex);
        if (position_initialized) {
            float raw_pos = soem_interface_get_current_position();
            printf("HIDInterface: get_cached_wheel_position - raw_pos from SOEM: %.2f, current center_position: %.2f\n", raw_pos, center_position); // Added debug
            
            float relative_pos = raw_pos - center_position;
            printf("HIDInterface: get_cached_wheel_position - relative_pos: %.2f\n", relative_pos); // Added debug

            float angle_degrees = (relative_pos / ENCODER_COUNTS_PER_REV) * 360.0f;
            printf("HIDInterface: get_cached_wheel_position - angle_degrees: %.2f\n", angle_degrees); // Added debug

            float normalized_angle = angle_degrees / MAX_STEERING_ANGLE;
            
            // Clamp to [-1.0, 1.0] range
            if (normalized_angle > 1.0f) normalized_angle = 1.0f;
            if (normalized_angle < -1.0f) normalized_angle = -1.0f;
            
            cached_position = normalized_angle;
            last_position_update = current_time;

            printf("HIDInterface: get_cached_wheel_position - final normalized_angle: %.4f\n", normalized_angle); // Added debug
        }
        pthread_mutex_unlock(&position_mutex);
    }
    
    float result = cached_position;
    pthread_mutex_unlock(&cache_mutex);
    return result;
}

// Adaptive timing adjustment
static void adjust_send_interval() {
    if (!adaptive_timing_enabled) return;
    
    if (consecutive_write_failures > 0) {
        // Increase interval on failures (slow down)
        current_send_interval_ms += 5; // Increase by 5ms
        if (current_send_interval_ms > MAX_SEND_INTERVAL_MS) {
            current_send_interval_ms = MAX_SEND_INTERVAL_MS;
        }
        consecutive_successes = 0; // Reset successes on failure
        printf("HIDInterface: Increased send interval to %dms due to write failures\n", current_send_interval_ms);
    } else {
        consecutive_successes++;
        // Decrease interval after many successes (speed up gradually)
        if (consecutive_successes >= 50 && current_send_interval_ms > MIN_SEND_INTERVAL_MS) {
            current_send_interval_ms -= 1; // Decrease by 1ms
            consecutive_successes = 0; // Reset after adjustment
            printf("HIDInterface: Decreased send interval to %dms after successful writes\n", current_send_interval_ms);
        }
    }
}

// Updated parse FFB reports from PC with proper structure handling
static int parse_ffb_report(uint8_t *report, size_t len, ffb_motor_effect_t *effect) {
    if (len < 2) return 0;
    
    uint8_t report_id = report[0];
    
    // Initialize effect with default values
    memset(effect, 0, sizeof(ffb_motor_effect_t));
    effect->report_id = report_id;
    clock_gettime(CLOCK_REALTIME, &effect->received_time);
    
    switch (report_id) {
        case 2: // PID Pool Report
            if (len >= sizeof(ffb_pid_pool_report_t)) {
                ffb_pid_pool_report_t *pid_report = (ffb_pid_pool_report_t *)report;
                effect->effect_type = pid_report->effect_type;
                return 1;
            }
            break;
            
        case 3: // Set Effect Report
            if (len >= sizeof(ffb_set_effect_report_t)) {
                ffb_set_effect_report_t *set_effect = (ffb_set_effect_report_t *)report;
                effect->magnitude = (set_effect->magnitude - 128) / 128.0f;
                effect->duration_ms = (set_effect->duration_high << 8) | set_effect->duration_low;
                return 1;
            }
            break;
            
        case 4: // Set Envelope Report
            if (len >= sizeof(ffb_set_envelope_report_t)) {
                ffb_set_envelope_report_t *envelope = (ffb_set_envelope_report_t *)report;
                effect->spring_coefficient = envelope->positive_coefficient / 255.0f;
                effect->damper_coefficient = envelope->negative_coefficient / 255.0f;
                effect->dead_band = envelope->dead_band / 255.0f;
                return 1;
            }
            break;
            
        case 5: // Set Condition Report
            if (len >= sizeof(ffb_set_condition_report_t)) {
                ffb_set_condition_report_t *condition = (ffb_set_condition_report_t *)report;
                effect->effect_type = condition->condition_type;
                effect->spring_coefficient = condition->positive_coefficient / 255.0f;
                effect->damper_coefficient = condition->negative_coefficient / 255.0f;
                effect->center_position = condition->center / 255.0f;
                effect->dead_band = condition->dead_band / 255.0f;
                return 1;
            }
            break;
            
        case 6: // Set Periodic Report
            if (len >= sizeof(ffb_set_periodic_report_t)) {
                ffb_set_periodic_report_t *periodic = (ffb_set_periodic_report_t *)report;
                uint16_t direction = (periodic->direction_high << 8) | periodic->direction_low;
                effect->direction = direction * 360.0f / 65535.0f;
                return 1;
            }
            break;
            
        case 7: // Set Constant Force Report
            if (len >= sizeof(ffb_set_constant_force_report_t)) {
                ffb_set_constant_force_report_t *constant = (ffb_set_constant_force_report_t *)report;
                effect->effect_type = 1;
                effect->attack_level = constant->attack_level / 255.0f;
                effect->attack_time_ms = (constant->attack_time_high << 8) | constant->attack_time_low;
                effect->fade_level = constant->fade_level / 255.0f;
                effect->fade_time_ms = (constant->fade_time_high << 8) | constant->fade_time_low;
                return 1;
            }
            break;
    }
    
    return 0;
}

// FFB reception thread with improved error handling
static void* _usb_ffb_reception_thread(void* arg) {
    (void)arg;
    
    // Set CPU affinity for this thread to core 1 (example)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset); // Assign to core 1
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        perror("FFB Reception Thread: Failed to set CPU affinity");
    } else {
        printf("FFB Reception Thread: Set to CPU core 1\n");
    }

    ffb_generic_report_t ffb_report;
    int read_failures = 0;

    printf("FFB: Reception thread started\n");
    
    while (hid_running) {
        // Check if we need to reconnect
        if (!usb_connected && should_attempt_reconnect()) {
            if (try_open_hid_device() == 0) {
                read_failures = 0;
            }
        }
        
        // Only try to read if we're connected
        if (usb_connected) {
            pthread_mutex_lock(&hidg_fd_mutex); // Lock for read operation
            int fd = hidg_fd;
            
            if (fd >= 0) {
                fd_set read_fds;
                struct timeval tv;
                FD_ZERO(&read_fds);
                FD_SET(fd, &read_fds);
                tv.tv_sec = 0;
                tv.tv_usec = 5000; // 5ms timeout for polling

                int retval = select(fd + 1, &read_fds, NULL, NULL, &tv);

                if (retval == -1) {
                    perror("FFB Reception Thread: select error");
                    read_failures++;
                    total_read_errors++;
                    if (read_failures > 20) { // If many read errors, assume disconnect
                        printf("FFB Reception Thread: Too many read errors, closing device.\n");
                        safe_close_hid_device(); // This will unlock the mutex internally
                        read_failures = 0;
                    }
                } else if (retval) {
                    ssize_t len = read(fd, &ffb_report, sizeof(ffb_report));
                    if (len > 0) {
                        read_failures = 0;
                        
                        ffb_motor_effect_t new_effect;
                        if (parse_ffb_report((uint8_t*)&ffb_report, (size_t)len, &new_effect)) {
                            pthread_mutex_lock(&queue_mutex);
                            if (queue_count < FFB_EFFECT_QUEUE_SIZE) {
                                ffb_effect_queue[queue_tail] = new_effect;
                                queue_tail = (queue_tail + 1) % FFB_EFFECT_QUEUE_SIZE;
                                queue_count++;
                                pthread_cond_signal(&queue_cond);
                            }
                            pthread_mutex_unlock(&queue_mutex);
                        }
                    } else if (len < 0) {
                        if (errno != EAGAIN && errno != EWOULDBLOCK) {
                            perror("FFB Reception Thread: read error");
                            read_failures++;
                            total_read_errors++;
                            if (read_failures > 20) { // If many read errors, assume disconnect
                                printf("FFB Reception Thread: Persistent read error, closing device.\n");
                                safe_close_hid_device(); // This will unlock the mutex internally
                                read_failures = 0;
                            }
                        }
                    }
                }
            }
            pthread_mutex_unlock(&hidg_fd_mutex); // Unlock after read operation
        }
    }

    printf("FFB: Reception thread stopped\n");
    return NULL;
}

// Improved gamepad report sending thread with adaptive timing
static void* _gamepad_report_loop(void* arg) {
    (void)arg;

    // Set CPU affinity for this thread to core 1 (example)
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(1, &cpuset); // Assign to core 1, same as reception thread to keep HID on one core
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        perror("Gamepad Report Thread: Failed to set CPU affinity");
    } else {
        printf("Gamepad Report Thread: Set to CPU core 1\n");
    }

    printf("HIDInterface: Gamepad report thread started\n");

    struct timespec loop_start_time, loop_end_time;

    while (hid_running) {
        clock_gettime(CLOCK_MONOTONIC, &loop_start_time);

        // Get current wheel position using cached value
        float wheel_position = get_cached_wheel_position();
        
        // Send the report
        int send_result = hid_interface_send_gamepad_report(wheel_position, 0);
        
        // Adjust timing based on success/failure
        adjust_send_interval();
        
        // Periodic debugging output - less frequent to avoid spam
        report_counter++;
        // Keep this print for overall HID thread status, not per-report
        if (report_counter % debug_every_n_reports == 0) {
            printf("HID Report #%d: Pos=%.4f, Interval=%dms, Connected=%s, Result=%d, WriteErr=%d\n", 
                   report_counter, wheel_position, current_send_interval_ms,
                   usb_connected ? "YES" : "NO", send_result, total_write_errors);
        }
        
        // Enforce the current send interval (adaptive)
        clock_gettime(CLOCK_MONOTONIC, &loop_end_time);
        long elapsed_ms = timespec_diff_ms(&loop_start_time, &loop_end_time);
        long sleep_time_ms = current_send_interval_ms - elapsed_ms;

        if (sleep_time_ms > 0) {
            usleep(sleep_time_ms * 1000);
        }
    }

    printf("HIDInterface: Gamepad report thread stopped\n");
    return NULL;
}

/**
 * @brief Initializes the HID interface.
 */
int hid_interface_init() {
    printf("HIDInterface: Initializing HID interface...\n");
    
    // Set initial center position AFTER SOEM is ready (main.c ensures this order)
    pthread_mutex_lock(&position_mutex);
    center_position = soem_interface_get_current_position();
    printf("HIDInterface: Initial center_position set to: %.2f\n", center_position); // Added debug
    position_initialized = 1;
    pthread_mutex_unlock(&position_mutex);
    
    printf("HIDInterface: Center position set to: %.2f\n", center_position);
    
    // Try to open HID device initially
    if (try_open_hid_device() != 0) {
        printf("HIDInterface: Warning - Could not open HID device initially. Will try to reconnect later.\n");
    } else {
        // Give USB host time to recognize the device
        usleep(20000); // 20ms delay (reduced from 200ms)
    }

    printf("HIDInterface: HID interface initialized.\n");
    return 0;
}

/**
 * @brief Starts the HID communication threads.
 */
int hid_interface_start() {
    hid_running = 1;
    
    if (pthread_create(&ffb_reception_thread, NULL, _usb_ffb_reception_thread, NULL) != 0) {
        perror("HIDInterface: Failed to create FFB reception thread");
        return -1;
    }
    
    if (pthread_create(&gamepad_report_thread, NULL, _gamepad_report_loop, NULL) != 0) {
        perror("HIDInterface: Failed to create gamepad report thread");
        // If gamepad thread fails, try to cancel reception thread
        pthread_cancel(ffb_reception_thread);
        pthread_join(ffb_reception_thread, NULL); // Wait for it to clean up
        return -1;
    }
    
    printf("HIDInterface: Started FFB reception and gamepad report threads.\n");
    return 0;
}

/**
 * @brief Stops the HID communication and cleans up resources.
 */
void hid_interface_stop() {
    hid_running = 0;
    
    // Signal threads to stop and wait for them to finish
    // pthread_cancel is generally discouraged for graceful shutdown,
    // but here we ensure hid_running flag is checked.
    if (ffb_reception_thread) {
        pthread_join(ffb_reception_thread, NULL);
    }
    
    if (gamepad_report_thread) {
        pthread_join(gamepad_report_thread, NULL);
    }

    safe_close_hid_device();

    printf("HIDInterface: Stopped. Stats - Write errors: %d, Read errors: %d, Reconnects: %d\n", 
           total_write_errors, total_read_errors, reconnect_count);
}

/**
 * @brief Retrieves the latest FFB effect from the queue.
 */
int hid_interface_get_ffb_effect(ffb_motor_effect_t *effect_out) {
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
 * @brief Sends a gamepad report with improved error handling and buffering.
 * This function now attempts to send a report only if the device is immediately ready.
 * It does not block or retry if the buffer is full.
 */
int hid_interface_send_gamepad_report(float position, unsigned int buttons) {
    if (!hid_running) {
        return -1;
    }

    // Check connection and try to reconnect if needed
    // This check is outside the main mutex to allow reconnection logic to proceed
    if (!usb_connected && should_attempt_reconnect()) {
        if (try_open_hid_device() != 0) {
            return 0; // Temporary failure, couldn't reconnect yet
        }
    }
    
    // If still not connected after potential reconnect attempt, return
    if (!usb_connected) {
        return 0; 
    }

    // Clamp position to valid range
    if (position > 1.0f) position = 1.0f;
    if (position < -1.0f) position = -1.0f;
    
    // Create report
    gamepad_report_t report = {0};
    report.report_id = 1;
    
    // Convert normalized float position [-1.0, 1.0] to int16_t [-32768, 32767]
    // Use float to int16_t conversion, ensuring proper rounding for positive and negative ranges
    if (position >= 0.0f) {
        report.x_axis = (int16_t)(position * 32767.0f);
    } else {
        // For negative values, map -1.0 to -32768
        report.x_axis = (int16_t)(position * 32768.0f);
    }
    
    report.buttons = (uint16_t)(buttons & 0xFFFF);

    // Debug print for the values being sent (more frequent for debugging this specific issue)
    // This print will show for every attempt to send a report
    printf("HIDInterface: send_gamepad_report - Pos (norm): %.4f, X-axis (raw): %d, Buttons: %u\n",
           position, report.x_axis, report.buttons);
    
    // Lock the mutex for the entire write operation
    pthread_mutex_lock(&hidg_fd_mutex);
    int fd = hidg_fd; // Get the current file descriptor inside the lock

    if (fd < 0) { // Check if fd is still valid after acquiring lock
        pthread_mutex_unlock(&hidg_fd_mutex);
        return -1; // Device not open
    }

    int success = 0; // Flag to track if send was successful in this attempt

    fd_set write_fds;
    struct timeval tv;
    
    FD_ZERO(&write_fds);
    FD_SET(fd, &write_fds);
    // Use a very short timeout for select to make it almost non-blocking
    // If the buffer isn't ready immediately, we'll skip this send.
    tv.tv_sec = 0;
    tv.tv_usec = 100; // 100 microseconds timeout (effectively non-blocking for most cases)

    int retval = select(fd + 1, NULL, &write_fds, NULL, &tv);

    if (retval == -1) {
        perror("HIDInterface: select error during write (non-blocking)");
        consecutive_write_failures++;
        total_write_errors++;
        // If select fails, assume a serious issue and try to close/reconnect
        if (consecutive_write_failures > 5) { 
            printf("HIDInterface: Persistent select error, marking as disconnected.\n");
            safe_close_hid_device(); // This will unlock the mutex internally
            return -1;
        }
        // No retry, just return failure for this cycle
        pthread_mutex_unlock(&hidg_fd_mutex);
        return 0; // Not sent
    } else if (retval == 0) {
        // Timeout - buffer not ready immediately, skip sending this report
        consecutive_write_failures++; // Still count as a failure for adaptive timing
        pthread_mutex_unlock(&hidg_fd_mutex);
        return 0; // Not sent
    } else {
        // Buffer ready, attempt write
        ssize_t bytes_written = write(fd, &report, sizeof(report));
        
        if (bytes_written == sizeof(report)) {
            consecutive_write_failures = 0; // Reset failures on success
            success = 1;
        } else if (bytes_written < 0) {
            consecutive_write_failures++; // Increment failure count for any write error
            total_write_errors++;

            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Buffer full, non-blocking write would block, skip this report
                // Already handled by select == 0, but good to have here too
                printf("HIDInterface: Write buffer full (EAGAIN/EWOULDBLOCK), skipping report.\n");
            } else if (errno == EBADF || errno == ENODEV || errno == EPIPE) { // EPIPE for "Broken pipe"
                // Critical error: file descriptor bad, device gone, or transport shut down
                perror("HIDInterface: Critical write error (device lost/shutdown)");
                printf("HIDInterface: Write error (%s), marking as disconnected\n", strerror(errno));
                safe_close_hid_device(); // Immediately close to trigger reconnect
                pthread_mutex_unlock(&hidg_fd_mutex); // Unlock before returning
                return -1; // Indicate critical failure
            } else {
                // Other write error
                perror("HIDInterface: write error");
                printf("HIDInterface: Write error (%s)\n", strerror(errno));
            }
            // Not sent for this cycle
            success = 0; 
        } else {
            // Partial write (shouldn't happen for HID reports typically)
            printf("HIDInterface: Partial write (%zd/%zu bytes), skipping report.\n", bytes_written, sizeof(report));
            consecutive_write_failures++;
            total_write_errors++;
            success = 0;
        }
    }
    
    pthread_mutex_unlock(&hidg_fd_mutex); // Unlock after the entire write process

    return success; // Return 1 for sent, 0 for not sent, -1 for critical error
}

// Function to recenter the wheel position
void hid_interface_recenter_wheel() {
    pthread_mutex_lock(&position_mutex);
    center_position = soem_interface_get_current_position();
    printf("HIDInterface: Wheel recentered to position: %.2f\n", center_position);
    pthread_mutex_unlock(&position_mutex);
    
    // Clear position cache to force immediate update
    pthread_mutex_lock(&cache_mutex);
    last_position_update.tv_sec = 0;
    last_position_update.tv_nsec = 0;
    pthread_mutex_unlock(&cache_mutex);
}

// Function to get current position relative to center
float hid_interface_get_relative_position() {
    return get_cached_wheel_position();
}

/**
 * @brief Get connection status
 */
int hid_interface_get_connection_status() {
    return usb_connected;
}

/**
 * @brief Get error statistics
 */
void hid_interface_get_stats(int *write_errors, int *read_errors, int *reconnects) {
    if (write_errors) *write_errors = total_write_errors;
    if (read_errors) *read_errors = total_read_errors;
    if (reconnects) *reconnects = reconnect_count;
}

/**
 * @brief Set adaptive timing mode
 */
void hid_interface_set_adaptive_timing(int enabled) {
    adaptive_timing_enabled = enabled;
    if (enabled) {
        printf("HIDInterface: Adaptive timing enabled\n");
    } else {
        printf("HIDInterface: Adaptive timing disabled\n");
        current_send_interval_ms = HID_SEND_INTERVAL_MS;
    }
}

/**
 * @brief Get current send interval
 */
int hid_interface_get_current_interval() {
    return current_send_interval_ms;
}
