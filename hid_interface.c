// hid_interface.c - Fixed version with corrected encoder transmission
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

#define HID_DEVICE_PATH "/dev/hidg0"
#define HID_SEND_INTERVAL_MS 16  // Send reports every 16ms (62.5Hz)
#define USB_RECONNECT_DELAY_MS 500  // Wait 500ms before trying to reconnect
#define ENCODER_COUNTS_PER_REV 4096.0f  // Define encoder resolution
#define MAX_STEERING_ANGLE 900.0f  // Maximum steering angle in degrees (2.5 revolutions)

// Structure matching standard HID gamepad Report ID 1
typedef struct {
    uint8_t report_id;      // Report ID = 1
    int16_t x_axis;         // 16-bit signed X-axis (-32768 to 32767)
    uint16_t buttons;       // 16-bit button field
    uint8_t padding[3];     // Padding to make it 8 bytes total if needed
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
    // Continue for all 16 parameters as defined in your HID descriptor
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
static volatile int position_initialized = 0;  // Make volatile to ensure thread visibility
static pthread_mutex_t position_mutex = PTHREAD_MUTEX_INITIALIZER; // Protect position access

// FFB Effect Queue - Updated to use new structure
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

// Rate limiting for gamepad reports
static struct timespec last_send_time = {0, 0};
static int rate_limit_enabled = 1;

// USB connection state tracking
static volatile int usb_connected = 0;  // Make volatile for thread safety
static int consecutive_write_failures = 0;
static struct timespec last_reconnect_attempt = {0, 0};

// Error tracking
static int total_write_errors = 0;
static int total_read_errors = 0;
static int reconnect_count = 0;

// Debugging
static int report_counter = 0;
static int debug_every_n_reports = 60;  // Debug every 60 reports (about once per second at 60Hz)

// Threads
static pthread_t ffb_reception_thread;
static pthread_t gamepad_report_thread;

// Add this helper function for time comparison
static long timespec_diff_ms(struct timespec *start, struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000 + (end->tv_nsec - start->tv_nsec) / 1000000;
}

// Check if HID device exists and is accessible
static int check_hid_device_exists() {
    struct stat st;
    if (stat(HID_DEVICE_PATH, &st) != 0) {
        return 0; // Device doesn't exist
    }
    return S_ISCHR(st.st_mode); // Check if it's a character device
}

// Safe function to close HID device
static void safe_close_hid_device() {
    pthread_mutex_lock(&hidg_fd_mutex);
    if (hidg_fd >= 0) {
        close(hidg_fd);
        hidg_fd = -1;
        usb_connected = 0;
        printf("HIDInterface: Closed HID device\n");
    }
    pthread_mutex_unlock(&hidg_fd_mutex);
}

// Attempt to open/reopen HID device
static int try_open_hid_device() {
    pthread_mutex_lock(&hidg_fd_mutex);
    
    // Close existing connection if any
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
        pthread_mutex_unlock(&hidg_fd_mutex);
        return -1;
    }
    
    usb_connected = 1;
    consecutive_write_failures = 0;
    reconnect_count++;
    printf("HIDInterface: Successfully opened HID device (reconnect #%d)\n", reconnect_count);
    
    pthread_mutex_unlock(&hidg_fd_mutex);
    return 0;
}

// Check if we should attempt reconnection
static int should_attempt_reconnect() {
    if (usb_connected) return 0;
    
    struct timespec current_time;
    clock_gettime(CLOCK_MONOTONIC, &current_time);
    
    // Don't attempt reconnection too frequently
    if (last_reconnect_attempt.tv_sec != 0 || last_reconnect_attempt.tv_nsec != 0) {
        long elapsed_ms = timespec_diff_ms(&last_reconnect_attempt, &current_time);
        if (elapsed_ms < USB_RECONNECT_DELAY_MS) {
            return 0;
        }
    }
    
    last_reconnect_attempt = current_time;
    return 1;
}

// Improved position calculation function
static float calculate_wheel_position() {
    pthread_mutex_lock(&position_mutex);
    
    if (!position_initialized) {
        pthread_mutex_unlock(&position_mutex);
        return 0.0f;
    }
    
    // Get current raw encoder position
    float raw_pos = soem_interface_get_current_position();
    
    // Calculate difference from center
    float relative_pos = raw_pos - center_position;
    
    // Convert to degrees based on encoder resolution
    float angle_degrees = (relative_pos / ENCODER_COUNTS_PER_REV) * 360.0f;
    
    // Normalize to steering range (-MAX_STEERING_ANGLE to +MAX_STEERING_ANGLE)
    float normalized_angle = angle_degrees / MAX_STEERING_ANGLE;
    
    // Clamp to [-1.0, 1.0] range
    if (normalized_angle > 1.0f) normalized_angle = 1.0f;
    if (normalized_angle < -1.0f) normalized_angle = -1.0f;
    
    pthread_mutex_unlock(&position_mutex);
    
    return normalized_angle;
}

// Updated parse FFB reports from PC with proper structure handling
static int parse_ffb_report(uint8_t *report, size_t len, ffb_motor_effect_t *effect) {
    if (len < 2) return 0; // At least need report ID and one data byte
    
    uint8_t report_id = report[0];
    printf("FFB: Parsing report ID %d, length %zu\n", report_id, len);
    
    // Initialize effect with default values
    memset(effect, 0, sizeof(ffb_motor_effect_t));
    effect->report_id = report_id;
    clock_gettime(CLOCK_REALTIME, &effect->received_time);
    
    switch (report_id) {
        case 2: // PID Pool Report
            if (len >= sizeof(ffb_pid_pool_report_t)) {
                ffb_pid_pool_report_t *pid_report = (ffb_pid_pool_report_t *)report;
                effect->effect_type = pid_report->effect_type;
                printf("FFB: PID Pool - Effect Block: %d, Type: %d\n", 
                       pid_report->effect_block_index, pid_report->effect_type);
                return 1;
            }
            break;
            
        case 3: // Set Effect Report
            if (len >= sizeof(ffb_set_effect_report_t)) {
                ffb_set_effect_report_t *set_effect = (ffb_set_effect_report_t *)report;
                
                // Convert from HID format to internal format
                effect->magnitude = (set_effect->magnitude - 128) / 128.0f; // Convert 0-255 to -1.0 to 1.0
                effect->duration_ms = (set_effect->duration_high << 8) | set_effect->duration_low;
                
                printf("FFB: Set Effect - Mag: %d -> %.2f, Duration: %dms\n", 
                       set_effect->magnitude, effect->magnitude, effect->duration_ms);
                return 1;
            }
            break;
            
        case 4: // Set Envelope Report
            if (len >= sizeof(ffb_set_envelope_report_t)) {
                ffb_set_envelope_report_t *envelope = (ffb_set_envelope_report_t *)report;
                
                // Convert envelope parameters
                effect->spring_coefficient = envelope->positive_coefficient / 255.0f;
                effect->damper_coefficient = envelope->negative_coefficient / 255.0f;
                effect->dead_band = envelope->dead_band / 255.0f;
                
                printf("FFB: Set Envelope - Spring: %.2f, Damper: %.2f, DeadBand: %.2f\n",
                       effect->spring_coefficient, effect->damper_coefficient, effect->dead_band);
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
                
                printf("FFB: Set Condition - Type: %d, Spring: %.2f, Center: %.2f\n",
                       condition->condition_type, effect->spring_coefficient, effect->center_position);
                return 1;
            }
            break;
            
        case 6: // Set Periodic Report
            if (len >= sizeof(ffb_set_periodic_report_t)) {
                ffb_set_periodic_report_t *periodic = (ffb_set_periodic_report_t *)report;
                
                uint16_t direction = (periodic->direction_high << 8) | periodic->direction_low;
                effect->direction = direction * 360.0f / 65535.0f; // Convert to degrees
                
                printf("FFB: Set Periodic - Direction: %.1f degrees\n", effect->direction);
                return 1;
            }
            break;
            
        case 7: // Set Constant Force Report
            if (len >= sizeof(ffb_set_constant_force_report_t)) {
                ffb_set_constant_force_report_t *constant = (ffb_set_constant_force_report_t *)report;
                
                effect->effect_type = 1; // Constant force type
                effect->attack_level = constant->attack_level / 255.0f;
                effect->attack_time_ms = (constant->attack_time_high << 8) | constant->attack_time_low;
                effect->fade_level = constant->fade_level / 255.0f;
                effect->fade_time_ms = (constant->fade_time_high << 8) | constant->fade_time_low;
                
                printf("FFB: Set Constant Force - Attack: %.2f (%dms), Fade: %.2f (%dms)\n",
                       effect->attack_level, effect->attack_time_ms, 
                       effect->fade_level, effect->fade_time_ms);
                return 1;
            }
            break;
            
        default:
            printf("FFB: Unknown report ID %d\n", report_id);
            break;
    }
    
    return 0;
}

// FFB reception thread with improved error handling and proper report parsing
static void* _usb_ffb_reception_thread(void* arg) {
    (void)arg;
    ffb_generic_report_t ffb_report;
    int read_failures = 0;

    printf("FFB: Reception thread started\n");
    
    while (hid_running) {
        // Check if we need to reconnect
        if (!usb_connected && should_attempt_reconnect()) {
            printf("FFB: Attempting to reconnect to HID device...\n");
            if (try_open_hid_device() == 0) {
                printf("FFB: Reconnected successfully\n");
                read_failures = 0;
            } else {
                printf("FFB: Reconnection failed\n");
            }
        }
        
        // Only try to read if we're connected
        if (usb_connected) {
            pthread_mutex_lock(&hidg_fd_mutex);
            int fd = hidg_fd;
            pthread_mutex_unlock(&hidg_fd_mutex);
            
            if (fd >= 0) {
                ssize_t len = read(fd, &ffb_report, sizeof(ffb_report));
                if (len > 0) {
                    read_failures = 0;
                    printf("FFB: Received %zd bytes, Report ID: %d\n", len, ffb_report.report_id);
                    
                    ffb_motor_effect_t new_effect;
                    if (parse_ffb_report((uint8_t*)&ffb_report, (size_t)len, &new_effect)) {
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
                } else if (len < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) {
                        // No data available, this is normal
                    } else {
                        read_failures++;
                        total_read_errors++;
                        if (read_failures > 10) {
                            printf("FFB: Too many read failures, marking as disconnected\n");
                            safe_close_hid_device();
                            read_failures = 0;
                        }
                    }
                }
            }
        }

        usleep(1000); // 1ms polling
    }

    printf("FFB: Reception thread stopped\n");
    return NULL;
}

// Improved gamepad report sending thread with better error handling and debugging
static void* _gamepad_report_loop(void* arg) {
    (void)arg;
    printf("HIDInterface: Gamepad report thread started\n");

    while (hid_running) {
        // Calculate current wheel position
        float wheel_position = calculate_wheel_position();
        
        // Send the report
        int send_result = hid_interface_send_gamepad_report(wheel_position, 0);
        
        // Periodic debugging output
        report_counter++;
        if (report_counter % debug_every_n_reports == 0) {
            printf("HID Report #%d: Position=%.4f, Raw=%.1f, Center=%.1f, Connected=%s, Result=%d\n", 
                   report_counter, wheel_position, soem_interface_get_current_position(), 
                   center_position, usb_connected ? "YES" : "NO", send_result);
        }
        
        usleep(HID_SEND_INTERVAL_MS * 1000);  // Convert to microseconds
    }

    printf("HIDInterface: Gamepad report thread stopped\n");
    return NULL;
}

/**
 * @brief Initializes the HID interface.
 * @return 0 on success, -1 on failure.
 */
int hid_interface_init() {
    printf("HIDInterface: Initializing HID interface...\n");
    
    // Wait a moment to ensure SOEM interface is ready
    usleep(100000); // 100ms delay
    
    // Set initial center position with thread safety
    pthread_mutex_lock(&position_mutex);
    center_position = soem_interface_get_current_position();
    position_initialized = 1;
    pthread_mutex_unlock(&position_mutex);
    
    printf("HIDInterface: Center position set to: %.2f\n", center_position);
    
    // Try to open HID device initially, but don't fail if it's not available
    if (try_open_hid_device() != 0) {
        printf("HIDInterface: Warning - Could not open HID device initially. Will try to reconnect later.\n");
    } else {
        // Send initial report to establish connection
        gamepad_report_t initial_report = {
            .report_id = 1,
            .x_axis = 0,
            .buttons = 0,
            .padding = {0, 0, 0}
        };
        
        pthread_mutex_lock(&hidg_fd_mutex);
        if (hidg_fd >= 0) {
            ssize_t bytes_written = write(hidg_fd, &initial_report, sizeof(initial_report));
            if (bytes_written == sizeof(initial_report)) {
                printf("HIDInterface: Initial report sent successfully\n");
            } else {
                printf("HIDInterface: Warning - Failed to send initial report (wrote %zd/%zu bytes)\n", 
                       bytes_written, sizeof(initial_report));
            }
        }
        pthread_mutex_unlock(&hidg_fd_mutex);
        
        // Give the USB host a moment to recognize the device
        usleep(100000); // 100ms delay
    }

    printf("HIDInterface: HID interface initialized.\n");
    return 0;
}

/**
 * @brief Starts the HID communication threads.
 * @return 0 on success, -1 on failure.
 */
int hid_interface_start() {
    hid_running = 1;
    
    if (pthread_create(&ffb_reception_thread, NULL, _usb_ffb_reception_thread, NULL) != 0) {
        perror("HIDInterface: Failed to create FFB reception thread");
        return -1;
    }
    
    if (pthread_create(&gamepad_report_thread, NULL, _gamepad_report_loop, NULL) != 0) {
        perror("HIDInterface: Failed to create gamepad report thread");
        pthread_cancel(ffb_reception_thread);
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
 * @param effect_out Pointer to store the effect.
 * @return 1 if an effect was retrieved, 0 if queue is empty.
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
 * @brief Sends a gamepad report with proper structure matching HID descriptor.
 * @param position The current position of the steering wheel (-1.0 to 1.0).
 * @param buttons The state of the buttons.
 * @return 1 on success, 0 on temporary failure, -1 on error
 */
int hid_interface_send_gamepad_report(float position, unsigned int buttons) {
    if (!hid_running) {
        return -1;
    }

    // Check connection status and try to reconnect if needed
    if (!usb_connected && should_attempt_reconnect()) {
        if (try_open_hid_device() != 0) {
            return 0; // Temporary failure, will retry
        }
    }
    
    if (!usb_connected) {
        return 0; // Not connected, temporary failure
    }

    // Clamp position to valid range
    if (position > 1.0f) position = 1.0f;
    if (position < -1.0f) position = -1.0f;
    
    // Create properly structured report
    gamepad_report_t report = {0};
    report.report_id = 1;
    
    // Convert normalized position (-1.0 to 1.0) to 16-bit signed integer
    // Use full range: -32768 to 32767
    if (position >= 0.0f) {
        report.x_axis = (int16_t)(position * 32767.0f);
    } else {
        report.x_axis = (int16_t)(position * 32768.0f);
    }
    
    report.buttons = (uint16_t)(buttons & 0xFFFF);  // Only use lower 16 bits
    
    // Send the report with proper size
    pthread_mutex_lock(&hidg_fd_mutex);
    int fd = hidg_fd;
    pthread_mutex_unlock(&hidg_fd_mutex);
    
    if (fd >= 0) {
        ssize_t bytes_written = write(fd, &report, sizeof(report));
        
        if (bytes_written == sizeof(report)) {
            consecutive_write_failures = 0;
            return 1; // Success
        } else if (bytes_written < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return 0; // Temporary failure
            } else {
                consecutive_write_failures++;
                total_write_errors++;
                
                if (consecutive_write_failures > 5) {
                    printf("HIDInterface: Too many write failures (%s), marking as disconnected\n", strerror(errno));
                    safe_close_hid_device();
                }
                return -1; // Error
            }
        } else {
            printf("HIDInterface: Partial write - %zd/%zu bytes\n", bytes_written, sizeof(report));
            return -1; // Partial write is an error
        }
    }
    
    return -1; // Error - invalid file descriptor
}

// Function to recenter the wheel position
void hid_interface_recenter_wheel() {
    pthread_mutex_lock(&position_mutex);
    center_position = soem_interface_get_current_position();
    printf("HIDInterface: Wheel recentered to position: %.2f\n", center_position);
    pthread_mutex_unlock(&position_mutex);
}

// Function to get current position relative to center
float hid_interface_get_relative_position() {
    return calculate_wheel_position();
}

/**
 * @brief Get connection status
 * @return 1 if connected, 0 if not connected
 */
int hid_interface_get_connection_status() {
    return usb_connected;
}

/**
 * @brief Get error statistics
 * @param write_errors Pointer to store write error count
 * @param read_errors Pointer to store read error count
 * @param reconnects Pointer to store reconnection count
 */
void hid_interface_get_stats(int *write_errors, int *read_errors, int *reconnects) {
    if (write_errors) *write_errors = total_write_errors;
    if (read_errors) *read_errors = total_read_errors;
    if (reconnects) *reconnects = reconnect_count;
}

/**
 * @brief Set the HID report send interval
 * @param interval_ms Interval in milliseconds (minimum 8ms recommended)
 */
void hid_interface_set_report_interval(int interval_ms) {
    if (interval_ms < 8) interval_ms = 8; // Minimum 8ms for USB HID
    // Note: This would require making HID_SEND_INTERVAL_MS non-const
    // For now, it's fixed at compile time
    printf("HIDInterface: Report interval setting not implemented in this version\n");
}
