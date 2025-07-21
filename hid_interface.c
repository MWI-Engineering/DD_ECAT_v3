// hid_interface.c - Improved version with better error handling and USB reconnection
#include "hid_interface.h"
#include "soem_interface.h"
//
#include "ffb_types.h"
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
#include <linux/time.h>

#define HID_DEVICE_PATH "/dev/hidg0"
#define HID_REPORT_SIZE 64
#define HID_SEND_INTERVAL_MS 16  // Send reports every 16ms (62.5Hz) - more reasonable for USB HID
#define USB_RECONNECT_DELAY_MS 500  // Wait 500ms before trying to reconnect


// Encoder
static float center_position = 0.0f;  // Store the center position
static int position_initialized = 0;  // Flag to track if center has been set

// FFB Effect Queue
#define FFB_EFFECT_QUEUE_SIZE 16
static ffb_effect_t ffb_effect_queue[FFB_EFFECT_QUEUE_SIZE];
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
static int usb_connected = 0;
static int consecutive_write_failures = 0;
static struct timespec last_reconnect_attempt = {0, 0};

// Error tracking
static int total_write_errors = 0;
static int total_read_errors = 0;
static int reconnect_count = 0;

// Add this helper function for time comparison
static long timespec_diff_ms(struct timespec *start, struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000 + (end->tv_nsec - start->tv_nsec) / 1000000;
}

static pthread_t ffb_reception_thread;

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

static pthread_t gamepad_report_thread;

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

typedef struct {
    uint8_t report_id;
    uint8_t effect_id;
    uint8_t waveform;
    uint8_t magnitude;
    uint8_t offset;
    uint8_t frequency;
    uint8_t phase;
    uint8_t reserved;
} ffb_periodic_report_t;

typedef struct {
    uint8_t report_id;
    uint8_t effect_id;
    uint8_t start_magnitude;
    uint8_t end_magnitude;
    uint8_t duration;
    uint8_t reserved[3];
} ffb_ramp_report_t;

typedef struct {
    uint8_t report_id;
    uint8_t effect_id;
    uint8_t parameter_type;
    uint8_t value;
    uint8_t reserved[4];
} ffb_effect_param_report_t;

typedef struct {
    uint8_t report_id;
    uint8_t effect_id;
    uint8_t attack_level;
    uint8_t attack_time;
    uint8_t fade_level;
    uint8_t fade_time;
    uint8_t reserved[2];
} ffb_envelope_report_t;

typedef struct {
    uint8_t report_id;
    uint8_t control_type;
    uint8_t reserved[6];
} ffb_device_control_report_t;

// Parse FFB reports according to your HID descriptor
static int parse_ffb_report(uint8_t *report, size_t len, ffb_effect_t *effect) {
    if (len < 2) return 0;
    
    uint8_t report_id = report[0];
    
    switch (report_id) {
        case 0x01: // PID State Report
            printf("FFB: PID State Report received\n");
            return 0;
            
        case 0x02: // Constant Force Effect
            if (len >= sizeof(ffb_constant_force_report_t)) {
                ffb_constant_force_report_t *cf = (ffb_constant_force_report_t *)report;
                effect->type = FFB_EFFECT_CONSTANT_FORCE;
                effect->id = cf->effect_id;
                effect->magnitude = (cf->magnitude - 128) / 128.0f;
                effect->direction = cf->direction * 360.0f / 255.0f;
                effect->duration = cf->duration * 10;
                effect->start_delay = cf->start_delay * 10;
                printf("FFB: Constant Force - ID:%d, Mag:%.2f, Dir:%.1f°, Dur:%dms\n", 
                       effect->id, effect->magnitude, effect->direction, effect->duration);
                return 1;
            }
            break;
            
        case 0x03: // Spring/Damper/Inertia Effects
            if (len >= sizeof(ffb_condition_report_t)) {
                ffb_condition_report_t *cond = (ffb_condition_report_t *)report;
                effect->id = cond->effect_id;
                
                effect->spring_coefficient = cond->spring_coefficient / 255.0f;
                effect->damper_coefficient = cond->damper_coefficient / 255.0f;
                effect->inertia_coefficient = cond->inertia_coefficient / 255.0f;
                effect->friction_coefficient = cond->friction_coefficient / 255.0f;
                
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
                
                printf("FFB: Condition Effect - ID:%d, Type:%d, Mag:%.2f, Spring:%.2f, Damper:%.2f\n",
                       effect->id, effect->type, effect->magnitude, effect->spring_coefficient, effect->damper_coefficient);
                return 1;
            }
            break;
            
        case 0x04: // Periodic Effects
            if (len >= sizeof(ffb_periodic_report_t)) {
                ffb_periodic_report_t *per = (ffb_periodic_report_t *)report;
                effect->type = FFB_EFFECT_PERIODIC;
                effect->id = per->effect_id;
                effect->magnitude = per->magnitude / 255.0f;
                effect->direction = 0.0f;
                effect->duration = 0;
                
                effect->start_delay = per->waveform;
                effect->timestamp = (per->frequency << 16) | (per->phase << 8) | per->offset;
                
                printf("FFB: Periodic Effect - ID:%d, Waveform:%d, Mag:%.2f, Freq:%d, Phase:%d\n",
                       effect->id, per->waveform, effect->magnitude, per->frequency, per->phase);
                return 1;
            }
            break;
            
        case 0x05: // Ramp Effects  
            if (len >= sizeof(ffb_ramp_report_t)) {
                ffb_ramp_report_t *ramp = (ffb_ramp_report_t *)report;
                effect->type = FFB_EFFECT_RAMP;
                effect->id = ramp->effect_id;
                effect->magnitude = ramp->start_magnitude / 255.0f;
                effect->direction = ramp->end_magnitude / 255.0f;
                effect->duration = ramp->duration * 10;
                
                printf("FFB: Ramp Effect - ID:%d, Start:%.2f, End:%.2f, Dur:%dms\n",
                       effect->id, effect->magnitude, effect->direction, effect->duration);
                return 1;
            }
            break;
            
        case 0x06: // Effect Parameters
            if (len >= sizeof(ffb_effect_param_report_t)) {
                ffb_effect_param_report_t *param = (ffb_effect_param_report_t *)report;
                printf("FFB: Effect Parameters - ID:%d, Type:%d, Value:%d\n",
                       param->effect_id, param->parameter_type, param->value);
                return 0;
            }
            break;
            
        case 0x07: // Envelope Parameters
            if (len >= sizeof(ffb_envelope_report_t)) {
                ffb_envelope_report_t *env = (ffb_envelope_report_t *)report;
                printf("FFB: Envelope Parameters - ID:%d, Attack:%d/%d, Fade:%d/%d\n",
                       env->effect_id, env->attack_level, env->attack_time, 
                       env->fade_level, env->fade_time);
                return 0;
            }
            break;
            
        case 0x08: // Device Control
            if (len >= sizeof(ffb_device_control_report_t)) {
                ffb_device_control_report_t *ctrl = (ffb_device_control_report_t *)report;
                printf("FFB: Device Control - Type:%d ", ctrl->control_type);
                
                switch (ctrl->control_type) {
                    case 0:
                        printf("(Enable Actuators)\n");
                        break;
                    case 1:
                        printf("(Disable Actuators)\n");
                        break;
                    case 2:
                        printf("(Stop All Effects)\n");
                        pthread_mutex_lock(&queue_mutex);
                        queue_head = queue_tail = queue_count = 0;
                        pthread_mutex_unlock(&queue_mutex);
                        break;
                    case 3:
                        printf("(Device Reset)\n");
                        pthread_mutex_lock(&queue_mutex);
                        queue_head = queue_tail = queue_count = 0;
                        pthread_mutex_unlock(&queue_mutex);
                        break;
                    case 4:
                        printf("(Device Pause)\n");
                        break;
                    case 5:
                        printf("(Device Continue)\n");
                        break;
                    default:
                        printf("(Unknown: %d)\n", ctrl->control_type);
                        break;
                }
                return 0;
            }
            break;
            
        default:
            printf("FFB: Unknown report ID: 0x%02X\n", report_id);
            return 0;
    }
    
    return 0;
}

// Improved FFB reception thread with reconnection handling
static void* _usb_ffb_reception_thread(void* arg) {
    (void)arg;
    uint8_t report[HID_REPORT_SIZE];
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
                ssize_t len = read(fd, report, HID_REPORT_SIZE);
                if (len > 0) {
                    read_failures = 0;
                    printf("FFB: Received %zd bytes: ", len);
                    for (ssize_t i = 0; i < len && i < 8; i++) {
                        printf("0x%02X ", report[i]);
                    }
                    printf("\n");
                    
                    ffb_effect_t new_effect;
                    if (parse_ffb_report(report, (size_t)len, &new_effect)) {
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

/**
 * @brief Initializes the HID interface.
 * @return 0 on success, -1 on failure.
 */
int hid_interface_init() {
    printf("HIDInterface: Initializing HID interface...\n");
    
    // Set initial center position
    center_position = soem_interface_get_current_position();
    position_initialized = 1;
    printf("HIDInterface: Center position set to: %.2f\n", center_position);
    
    if (try_open_hid_device() != 0) {
        printf("HIDInterface: Warning - Could not open HID device initially. Will try to reconnect later.\n");
    }

    printf("HIDInterface: HID interface initialized.\n");
    return 0;
}

static void* _gamepad_report_loop(void* arg) {
    (void)arg;
    printf("HIDInterface: Gamepad report thread started\n");

    while (hid_running) {
        float raw_pos = soem_interface_get_current_position();
        
        // Calculate relative position from center
        float relative_pos = raw_pos - center_position;
        
        // Convert to revolutions (assuming 4096 counts per revolution)
        float revolutions = relative_pos / 4096.0f;
        
        // Normalize to [-0.5, 0.5] range (half revolution each direction)
        float normalized = fmodf(revolutions + 0.5f, 1.0f) - 0.5f;
        
        // Scale to [-1.0, 1.0] range for full steering range
        float output = normalized * 2.0f;
        
        // Clamp to ensure we don't exceed bounds
        if (output > 1.0f) output = 1.0f;
        if (output < -1.0f) output = -1.0f;

        hid_interface_send_gamepad_report(output, 0);
        usleep(5000);  // ~5ms = 200 Hz
    }

    printf("HIDInterface: Gamepad report thread stopped\n");
    return NULL;
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
    // Send commands
    if (pthread_create(&gamepad_report_thread, NULL, _gamepad_report_loop, NULL) != 0) {
        perror("HIDInterface: Failed to create gamepad report thread");
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
 * @brief Sends a gamepad report with improved error handling.
 * @param position The current position of the steering wheel.
 * @param buttons The state of the buttons.
 */
void hid_interface_send_gamepad_report(float position, unsigned int buttons) {
    if (!hid_running) return;

    // Rate limiting
    if (rate_limit_enabled) {
        struct timespec current_time;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        
        if (last_send_time.tv_sec != 0 || last_send_time.tv_nsec != 0) {
            long elapsed_ms = timespec_diff_ms(&last_send_time, &current_time);
            if (elapsed_ms < HID_SEND_INTERVAL_MS) {
                return;
            }
        }
        last_send_time = current_time;
    }

    // Try to reconnect if needed
    if (!usb_connected && should_attempt_reconnect()) {
        try_open_hid_device();
    }

    // Only send if connected
    if (!usb_connected) {
        return; // Removed debug print to reduce spam
    }

    pthread_mutex_lock(&hidg_fd_mutex);
    int fd = hidg_fd;
    pthread_mutex_unlock(&hidg_fd_mutex);

    if (fd < 0) {
        return; // Removed debug print to reduce spam
    }

    // Clear the entire report buffer
    uint8_t report[HID_REPORT_SIZE];
    memset(report, 0, HID_REPORT_SIZE);
    
    // Set Report ID
    report[0] = 0x01;
    
    // Convert position to 16-bit signed value using FULL range
    // Clamp position to [-1.0, 1.0] range
    if (position > 1.0f) position = 1.0f;
    if (position < -1.0f) position = -1.0f;
    
    // Use full signed 16-bit range: -32768 to +32767
    int16_t scaled_pos;
    if (position >= 0.0f) {
        scaled_pos = (int16_t)(position * 32767.0f);
    } else {
        scaled_pos = (int16_t)(position * 32768.0f);
    }
    
    // Pack position as little-endian 16-bit
    report[1] = scaled_pos & 0xFF;        // Low byte
    report[2] = (scaled_pos >> 8) & 0xFF; // High byte
    
    // Pack buttons as little-endian 16-bit
    report[3] = buttons & 0xFF;           // Low byte of buttons
    report[4] = (buttons >> 8) & 0xFF;    // High byte of buttons
    
    // Debug output (reduced frequency to avoid spam)
    static int debug_counter = 0;
    if (++debug_counter % 200 == 0) { // Print every 200 reports (about once per second)
        printf("HID Report: Pos=%.3f->%d (0x%04X), Buttons=0x%04X\n", 
               position, scaled_pos, (uint16_t)scaled_pos, buttons);
    }

    // Send the report
    ssize_t bytes_written = write(fd, report, HID_REPORT_SIZE);
    if (bytes_written < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            static int eagain_count = 0;
            if (++eagain_count % 1000 == 0) {
                printf("HID: Device buffer full (%d times)\n", eagain_count);
            }
        } else if (errno == EPIPE || errno == ECONNRESET || errno == ESHUTDOWN) {
            consecutive_write_failures++;
            total_write_errors++;
            
            if (consecutive_write_failures > 5) {
                printf("HID: USB connection lost (consecutive failures: %d)\n", consecutive_write_failures);
                safe_close_hid_device();
            }
        } else {
            total_write_errors++;
            printf("HID: Write error: %s (errno=%d)\n", strerror(errno), errno);
        }
    } else {
        consecutive_write_failures = 0;
    }
}

// Function to recenter the wheel position
void hid_interface_recenter_wheel() {
    center_position = soem_interface_get_current_position();
    printf("HIDInterface: Wheel recentered to position: %.2f\n", center_position);
}

// Function to get current position relative to center
float hid_interface_get_relative_position() {
    if (!position_initialized) return 0.0f;
    
    float raw_pos = soem_interface_get_current_position();
    float relative_pos = raw_pos - center_position;
    float revolutions = relative_pos / 4096.0f;
    float normalized = fmodf(revolutions + 0.5f, 1.0f) - 0.5f;
    return normalized * 2.0f;
}

// Add this test function to verify your encoder values
void hid_interface_test_encoder_values() {
    printf("=== Testing Encoder Value Conversion (Full Signed Range) ===\n");
    
    float test_positions[] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};
    int num_tests = sizeof(test_positions) / sizeof(test_positions[0]);
    
    for (int i = 0; i < num_tests; i++) {
        float pos = test_positions[i];
        int16_t scaled;
        
        if (pos >= 0.0f) {
            scaled = (int16_t)(pos * 32767.0f);
        } else {
            scaled = (int16_t)(pos * 32768.0f);
        }
        
        printf("Position %.1f -> Scaled %d (0x%04X) -> Bytes [0x%02X, 0x%02X]\n",
               pos, scaled, (uint16_t)scaled, 
               scaled & 0xFF, (scaled >> 8) & 0xFF);
    }
    
    printf("\nCurrent encoder reading:\n");
    if (position_initialized) {
        float relative_pos = hid_interface_get_relative_position();
        printf("Center: %.2f, Current relative: %.3f\n", center_position, relative_pos);
        
        int16_t scaled;
        if (relative_pos >= 0.0f) {
            scaled = (int16_t)(relative_pos * 32767.0f);
        } else {
            scaled = (int16_t)(relative_pos * 32768.0f);
        }
        printf("Final scaled value: %d (0x%04X)\n", scaled, (uint16_t)scaled);
    } else {
        printf("Position not initialized yet\n");
    }
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