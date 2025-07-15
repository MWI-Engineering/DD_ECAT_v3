// hid_interface.c - Fixed version with rate limiting and error handling
#include "hid_interface.h"
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

#define HID_DEVICE_PATH "/dev/hidg0"
#define HID_REPORT_SIZE 64
#define HID_SEND_INTERVAL_MS 8  // Send reports every 8ms (125Hz) - typical for gaming controllers

static int hidg_fd = -1;

// Rate limiting for gamepad reports
static struct timespec last_send_time = {0, 0};
static int rate_limit_enabled = 1;

// Add this helper function for time comparison
static long timespec_diff_ms(struct timespec *start, struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000 + (end->tv_nsec - start->tv_nsec) / 1000000;
}
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

typedef struct {
    uint8_t report_id;
    uint8_t effect_id;
    uint8_t waveform;          // 0=square, 1=sine, 2=triangle, 3=sawtooth up, 4=sawtooth down
    uint8_t magnitude;         // Peak amplitude
    uint8_t offset;            // DC offset
    uint8_t frequency;         // Frequency in Hz
    uint8_t phase;             // Phase shift
    uint8_t reserved;
} ffb_periodic_report_t;

typedef struct {
    uint8_t report_id;
    uint8_t effect_id;
    uint8_t start_magnitude;   // Starting magnitude
    uint8_t end_magnitude;     // Ending magnitude
    uint8_t duration;          // Duration of ramp
    uint8_t reserved[3];
} ffb_ramp_report_t;

typedef struct {
    uint8_t report_id;
    uint8_t effect_id;
    uint8_t parameter_type;    // 0=gain, 1=sample_rate, 2=trigger_button, etc.
    uint8_t value;
    uint8_t reserved[4];
} ffb_effect_param_report_t;

typedef struct {
    uint8_t report_id;
    uint8_t effect_id;
    uint8_t attack_level;      // Attack level (0-255)
    uint8_t attack_time;       // Attack time
    uint8_t fade_level;        // Fade level (0-255)
    uint8_t fade_time;         // Fade time
    uint8_t reserved[2];
} ffb_envelope_report_t;

typedef struct {
    uint8_t report_id;
    uint8_t control_type;      // 0=enable actuators, 1=disable actuators, 2=stop all effects, 3=device reset, 4=device pause, 5=device continue
    uint8_t reserved[6];
} ffb_device_control_report_t;

// Parse FFB reports according to your HID descriptor
static int parse_ffb_report(uint8_t *report, size_t len, ffb_effect_t *effect) {
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
                effect->start_delay = cf->start_delay * 10; // Convert to milliseconds
                printf("FFB: Constant Force - ID:%d, Mag:%.2f, Dir:%.1f°, Dur:%dms\n", 
                       effect->id, effect->magnitude, effect->direction, effect->duration);
                return 1;
            }
            break;
            
        case 0x03: // Spring/Damper/Inertia Effects
            if (len >= sizeof(ffb_condition_report_t)) {
                ffb_condition_report_t *cond = (ffb_condition_report_t *)report;
                effect->id = cond->effect_id;
                
                // Store all coefficients
                effect->spring_coefficient = cond->spring_coefficient / 255.0f;
                effect->damper_coefficient = cond->damper_coefficient / 255.0f;
                effect->inertia_coefficient = cond->inertia_coefficient / 255.0f;
                effect->friction_coefficient = cond->friction_coefficient / 255.0f;
                
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
                
                printf("FFB: Condition Effect - ID:%d, Type:%d, Mag:%.2f, Spring:%.2f, Damper:%.2f\n",
                       effect->id, effect->type, effect->magnitude, effect->spring_coefficient, effect->damper_coefficient);
                return 1;
            }
            break;
            
        case 0x04: // Periodic Effects (Sine, Square, etc.)
            if (len >= sizeof(ffb_periodic_report_t)) {
                ffb_periodic_report_t *per = (ffb_periodic_report_t *)report;
                effect->type = FFB_EFFECT_PERIODIC;
                effect->id = per->effect_id;
                effect->magnitude = per->magnitude / 255.0f;
                effect->direction = 0.0f; // Periodic effects don't have direction in same way
                effect->duration = 0; // Periodic effects are typically continuous
                
                // Store periodic-specific parameters in unused fields
                effect->start_delay = per->waveform; // Waveform type
                effect->timestamp = (per->frequency << 16) | (per->phase << 8) | per->offset; // Pack frequency, phase, offset
                
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
                effect->direction = ramp->end_magnitude / 255.0f; // Reuse direction field for end magnitude
                effect->duration = ramp->duration * 10; // Convert to milliseconds
                
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
                
                // Could store parameter changes but for now just log
                // In a full implementation, you might update existing effects in the queue
                return 0; // Not a new effect, just parameters
            }
            break;
            
        case 0x07: // Envelope Parameters
            if (len >= sizeof(ffb_envelope_report_t)) {
                ffb_envelope_report_t *env = (ffb_envelope_report_t *)report;
                printf("FFB: Envelope Parameters - ID:%d, Attack:%d/%d, Fade:%d/%d\n",
                       env->effect_id, env->attack_level, env->attack_time, 
                       env->fade_level, env->fade_time);
                
                // Could store envelope parameters but for now just log
                // In a full implementation, you might update existing effects in the queue
                return 0; // Not a new effect, just envelope
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
                        // Clear the effect queue
                        pthread_mutex_lock(&queue_mutex);
                        queue_head = queue_tail = queue_count = 0;
                        pthread_mutex_unlock(&queue_mutex);
                        break;
                    case 3:
                        printf("(Device Reset)\n");
                        // Clear the effect queue
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
                return 0; // Not a force effect
            }
            break;
            
        default:
            printf("FFB: Unknown report ID: 0x%02X\n", report_id);
            return 0;
    }
    
    return 0;
}

// --- Private function for FFB reception ---
static void* _usb_ffb_reception_thread(void* arg) {
    (void)arg; // Suppress unused parameter warning
    uint8_t report[HID_REPORT_SIZE];

    printf("FFB: Reception thread started\n");
    
    while (hid_running) {
        ssize_t len = read(hidg_fd, report, HID_REPORT_SIZE);
        if (len > 0) {
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

    // Rate limiting: only send reports every HID_SEND_INTERVAL_MS milliseconds
    if (rate_limit_enabled) {
        struct timespec current_time;
        clock_gettime(CLOCK_MONOTONIC, &current_time);
        
        if (last_send_time.tv_sec != 0 || last_send_time.tv_nsec != 0) {
            long elapsed_ms = timespec_diff_ms(&last_send_time, &current_time);
            if (elapsed_ms < HID_SEND_INTERVAL_MS) {
                return; // Skip this report to avoid overwhelming the HID device
            }
        }
        last_send_time = current_time;
    }

    uint8_t report[HID_REPORT_SIZE] = {0};

    // Format matching your HID descriptor:
    // Bytes 0-1: Steering wheel position (int16_t, scaled from -32767 to 32767)
    // Bytes 2-3: Button state (16 buttons, bitmask)
    
    int16_t scaled_pos = (int16_t)(position * 32767.0f);  // Assumes position in [-1.0, 1.0]
    report[0] = scaled_pos & 0xFF;
    report[1] = (scaled_pos >> 8) & 0xFF;
    report[2] = buttons & 0xFF;
    report[3] = (buttons >> 8) & 0xFF;

    ssize_t bytes_written = write(hidg_fd, report, HID_REPORT_SIZE);
    if (bytes_written < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            // Device buffer is full, this is normal - just skip this report
            // Don't spam the console with these errors
            static int eagain_count = 0;
            if (++eagain_count % 100 == 0) {
                printf("HIDInterface: Device buffer full (%d times), skipping reports\n", eagain_count);
            }
        } else {
            // Other errors are more serious
            perror("HIDInterface: Failed to send gamepad report");
        }
    }
}

/**
 * @brief Enable or disable rate limiting for gamepad reports
 * @param enable 1 to enable rate limiting, 0 to disable
 */
void hid_interface_set_rate_limiting(int enable) {
    rate_limit_enabled = enable;
    if (!enable) {
        // Reset the last send time when disabling rate limiting
        last_send_time.tv_sec = 0;
        last_send_time.tv_nsec = 0;
    }
}

/**
 * @brief Alternative: Use blocking I/O for more reliable transmission
 * Call this after hid_interface_init() if you want blocking writes
 */
int hid_interface_set_blocking_mode() {
    if (hidg_fd < 0) return -1;
    
    int flags = fcntl(hidg_fd, F_GETFL);
    if (flags == -1) {
        perror("HIDInterface: Failed to get file flags");
        return -1;
    }
    
    // Remove O_NONBLOCK flag to make writes blocking
    flags &= ~O_NONBLOCK;
    if (fcntl(hidg_fd, F_SETFL, flags) == -1) {
        perror("HIDInterface: Failed to set blocking mode");
        return -1;
    }
    
    printf("HIDInterface: Switched to blocking mode\n");
    return 0;
}