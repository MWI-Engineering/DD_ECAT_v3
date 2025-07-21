// main.c - Improved version with real-time control and better error handling
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <sys/mman.h>    // For memory locking
#include <sys/resource.h> // For priority setting
#include <pthread.h>
#include <errno.h>
#include <sched.h>       // For real-time scheduling

// Application libraries
#include "hid_interface.h"
#include "ffb_calculator.h"
#include "soem_interface.h"
#include "ffb_types.h"

// Configuration constants
#define MAIN_LOOP_FREQUENCY_HZ 1000
#define CYCLE_TIME_NS (1000000000L / MAIN_LOOP_FREQUENCY_HZ)
#define MAX_STEERING_ANGLE 540.0f  // ±540 degrees (3 full turns)
#define MAX_TORQUE_LIMIT 5000.0f   // Maximum torque in appropriate units
#define STATS_PRINT_INTERVAL 1000  // Print stats every 1000 loops (1 second)
#define MAX_LATE_WARNINGS 10       // Limit timing warnings
#define EMERGENCY_STOP_THRESHOLD 10000.0f // Emergency torque threshold

// Global flags and state
static volatile int running = 1;
static volatile int emergency_stop = 0;
static volatile int pause_control = 0;

// Performance monitoring structure
typedef struct {
    long min_time_ns;
    long max_time_ns;
    long total_time_ns;
    int loop_count;
    int late_warnings;
    int emergency_stops;
    int communication_errors;
    int ffb_effects_processed;
    double avg_torque;
    double max_torque;
} performance_stats_t;

// Application state structure
typedef struct {
    // Control variables
    float current_position;
    float current_velocity;
    float desired_torque;
    float normalized_position;
    unsigned int button_states;
    
    // FFB state
    ffb_effect_t current_ffb_effect;
    int effect_available;
    
    // Communication status
    int ethercat_status;
    int hid_status;
    int last_ethercat_status;
    
    // Performance monitoring
    performance_stats_t stats;
    
    // Timing
    struct timespec loop_start_time;
    struct timespec loop_end_time;
} app_state_t;

// Function prototypes
static void setup_real_time_scheduling(void);
static void setup_signal_handlers(void);
static void cleanup_and_exit(int exit_code);
static void sigint_handler(int signum);
static void sigusr1_handler(int signum);
static void sigusr2_handler(int signum);
static float normalize_position_for_hid(float position_degrees);
static unsigned int read_button_states(void);
static void update_performance_stats(app_state_t *state);
static void print_performance_stats(const performance_stats_t *stats);
static void reset_performance_stats(performance_stats_t *stats);
static int apply_safety_checks(app_state_t *state);
static void maintain_loop_timing(const struct timespec *start_time, const struct timespec *end_time);
static long timespec_diff_ns(const struct timespec *start, const struct timespec *end);

// Signal handlers
static void sigint_handler(int signum) {
    printf("\nCaught SIGINT (Ctrl+C), initiating graceful shutdown...\n");
    running = 0;
}

static void sigusr1_handler(int signum) {
    printf("\nCaught SIGUSR1, pausing control loop...\n");
    pause_control = 1;
}

static void sigusr2_handler(int signum) {
    printf("\nCaught SIGUSR2, resuming control loop...\n");
    pause_control = 0;
}

// Setup real-time scheduling and memory locking
static void setup_real_time_scheduling(void) {
    struct sched_param param;
    int ret;

    // Lock memory to prevent page faults
    ret = mlockall(MCL_CURRENT | MCL_FUTURE);
    if (ret != 0) {
        perror("Warning: Failed to lock memory");
    }

    // Set real-time scheduling priority
    param.sched_priority = 80; // High priority but not maximum
    ret = sched_setscheduler(0, SCHED_FIFO, &param);
    if (ret != 0) {
        perror("Warning: Failed to set real-time scheduling");
        printf("Running with normal scheduling. Consider running with sudo for real-time priority.\n");
    } else {
        printf("Real-time scheduling enabled with priority %d\n", param.sched_priority);
    }

    // Set process priority
    ret = setpriority(PRIO_PROCESS, 0, -20);
    if (ret != 0) {
        perror("Warning: Failed to set process priority");
    }
}

// Setup signal handlers
static void setup_signal_handlers(void) {
    struct sigaction sa;
    
    // SIGINT handler (Ctrl+C)
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = sigint_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;
    sigaction(SIGINT, &sa, NULL);
    
    // SIGUSR1 handler (pause control)
    sa.sa_handler = sigusr1_handler;
    sigaction(SIGUSR1, &sa, NULL);
    
    // SIGUSR2 handler (resume control)
    sa.sa_handler = sigusr2_handler;
    sigaction(SIGUSR2, &sa, NULL);
    
    // Block SIGPIPE (broken pipe)
    sa.sa_handler = SIG_IGN;
    sigaction(SIGPIPE, &sa, NULL);
}

// Cleanup and exit function
static void cleanup_and_exit(int exit_code) {
    printf("\nInitiating cleanup sequence...\n");
    
    // Stop all subsystems
    running = 0;
    
    // Stop EtherCAT first to ensure safe torque shutdown
    printf("Stopping EtherCAT master...\n");
    soem_interface_stop_master();
    
    // Stop HID interface
    printf("Stopping HID interface...\n");
    hid_interface_stop();
    
    // Unlock memory
    munlockall();
    
    printf("Cleanup completed successfully.\n");
    exit(exit_code);
}

// Convert position to normalized range for HID report
static float normalize_position_for_hid(float position_degrees) {
    // Clamp to range'
    
    if (position_degrees > MAX_STEERING_ANGLE) position_degrees = MAX_STEERING_ANGLE;
    if (position_degrees < -MAX_STEERING_ANGLE) position_degrees = -MAX_STEERING_ANGLE;
    
    // Normalize to [-1.0, 1.0]
    return position_degrees / MAX_STEERING_ANGLE;
}

// Read button states (placeholder implementation)
static unsigned int read_button_states(void) {
    // TODO: Implement actual button reading
    // This could read from:
    // - GPIO pins on Raspberry Pi
    // - I2C/SPI button controller
    // - Another HID device
    // - Network-based button interface
    return 0;
}

// Time difference calculation in nanoseconds
static long timespec_diff_ns(const struct timespec *start, const struct timespec *end) {
    return (end->tv_sec - start->tv_sec) * 1000000000L + (end->tv_nsec - start->tv_nsec);
}

// Update performance statistics
static void update_performance_stats(app_state_t *state) {
    long elapsed_ns = timespec_diff_ns(&state->loop_start_time, &state->loop_end_time);
    performance_stats_t *stats = &state->stats;
    
    if (elapsed_ns < stats->min_time_ns) stats->min_time_ns = elapsed_ns;
    if (elapsed_ns > stats->max_time_ns) stats->max_time_ns = elapsed_ns;
    stats->total_time_ns += elapsed_ns;
    stats->loop_count++;
    
    // Update torque statistics
    float abs_torque = fabs(state->desired_torque);
    stats->avg_torque = (stats->avg_torque * (stats->loop_count - 1) + abs_torque) / stats->loop_count;
    if (abs_torque > stats->max_torque) stats->max_torque = abs_torque;
    
    if (state->effect_available) {
        stats->ffb_effects_processed++;
    }
}

// Print performance statistics
static void print_performance_stats(const performance_stats_t *stats) {
    if (stats->loop_count == 0) return;
    
    long avg_time_ns = stats->total_time_ns / stats->loop_count;
    
    printf("=== Performance Statistics ===\n");
    printf("Loop timing: Min=%.3fms, Max=%.3fms, Avg=%.3fms\n",
           stats->min_time_ns / 1000000.0, stats->max_time_ns / 1000000.0, avg_time_ns / 1000000.0);
    printf("Loop count: %d, Late warnings: %d, Emergency stops: %d\n",
           stats->loop_count, stats->late_warnings, stats->emergency_stops);
    printf("Communication errors: %d, FFB effects processed: %d\n",
           stats->communication_errors, stats->ffb_effects_processed);
    printf("Torque: Avg=%.1f, Max=%.1f\n", stats->avg_torque, stats->max_torque);
    
    // HID and EtherCAT statistics
    int hid_write_errors, hid_read_errors, hid_reconnects;
    hid_interface_get_stats(&hid_write_errors, &hid_read_errors, &hid_reconnects);
    printf("HID: Write errors=%d, Read errors=%d, Reconnects=%d, Connected=%s\n",
           hid_write_errors, hid_read_errors, hid_reconnects,
           hid_interface_get_connection_status() ? "Yes" : "No");
}

// Reset performance statistics
static void reset_performance_stats(performance_stats_t *stats) {
    memset(stats, 0, sizeof(performance_stats_t));
    stats->min_time_ns = LONG_MAX;
}

// Apply safety checks and emergency procedures
static int apply_safety_checks(app_state_t *state) {
    // Check for emergency stop conditions
    if (fabs(state->desired_torque) > EMERGENCY_STOP_THRESHOLD) {
        if (!emergency_stop) {
            printf("EMERGENCY STOP: Torque %.1f exceeds threshold %.1f\n",
                   state->desired_torque, EMERGENCY_STOP_THRESHOLD);
            emergency_stop = 1;
            state->stats.emergency_stops++;
        }
        state->desired_torque = 0.0f;
        return 1;
    }
    
    // Check for communication loss
    if (!state->ethercat_status) {
        static int comm_loss_count = 0;
        if (++comm_loss_count > 100) { // 100ms without communication
            printf("SAFETY: EtherCAT communication lost, setting zero torque\n");
            state->desired_torque = 0.0f;
            comm_loss_count = 0;
            return 1;
        }
    }
    
    // Apply torque limits
    if (fabs(state->desired_torque) > MAX_TORQUE_LIMIT) {
        state->desired_torque = (state->desired_torque > 0) ? MAX_TORQUE_LIMIT : -MAX_TORQUE_LIMIT;
    }
    
    return 0;
}

// Maintain loop timing
static void maintain_loop_timing(const struct timespec *start_time, const struct timespec *end_time) {
    static int late_warning_count = 0;
    
    long elapsed_ns = timespec_diff_ns(start_time, end_time);
    long sleep_ns = CYCLE_TIME_NS - elapsed_ns;
    
    if (sleep_ns > 0) {
        struct timespec sleep_time = {
            .tv_sec = sleep_ns / 1000000000L,
            .tv_nsec = sleep_ns % 1000000000L
        };
        nanosleep(&sleep_time, NULL);
    } else if (sleep_ns < -1000000 && late_warning_count < MAX_LATE_WARNINGS) {
        printf("Warning: Loop running %.3fms late (target: %.3fms, actual: %.3fms)\n",
               -sleep_ns / 1000000.0, CYCLE_TIME_NS / 1000000.0, elapsed_ns / 1000000.0);
        late_warning_count++;
    }
}

// Main function
int main(int argc, char *argv[]) {
    printf("=== Raspberry Pi FFB Steering Wheel Application ===\n");
    printf("Real-time version with improved error handling\n");
    printf("Press Ctrl+C to stop, SIGUSR1 to pause, SIGUSR2 to resume\n");
    
    // Initialize application state
    app_state_t app_state;
    memset(&app_state, 0, sizeof(app_state));
    reset_performance_stats(&app_state.stats);
    
    // Setup real-time environment
    setup_real_time_scheduling();
    setup_signal_handlers();
    
    // --- Subsystem Initialization ---
    printf("\n=== Initializing Subsystems ===\n");
    
    // Initialize HID interface
    printf("Initializing HID interface...\n");
    if (hid_interface_init() != 0) {
        fprintf(stderr, "Failed to initialize HID interface.\n");
        cleanup_and_exit(EXIT_FAILURE);
    }
    
    if (hid_interface_start() != 0) {
        fprintf(stderr, "Failed to start HID interface.\n");
        cleanup_and_exit(EXIT_FAILURE);
    }
    
    // Initialize FFB calculator
    printf("Initializing FFB calculator...\n");
    ffb_calculator_init();
    
    // Initialize EtherCAT
    const char *ethercat_ifname = (argc > 1) ? argv[1] : "eth1";
    printf("Initializing EtherCAT master on interface %s...\n", ethercat_ifname);
    
    if (soem_interface_init_enhanced(ethercat_ifname) != 0) {
        fprintf(stderr, "Failed to initialize EtherCAT master.\n");
        cleanup_and_exit(EXIT_FAILURE);
    }
    
    // Wait for EtherCAT stabilization
    printf("Waiting for EtherCAT stabilization...\n");
    sleep(2);
    
    // --- Main Control Loop ---
    printf("\n=== Starting Main Control Loop ===\n");
    printf("Loop frequency: %d Hz (%.3f ms cycle time)\n", 
           MAIN_LOOP_FREQUENCY_HZ, CYCLE_TIME_NS / 1000000.0);
    
    while (running) {
        clock_gettime(CLOCK_MONOTONIC, &app_state.loop_start_time);
        
        // Skip control if paused
        if (pause_control) {
            usleep(10000); // Sleep 10ms when paused
            continue;
        }
        
        // 1. Update communication status
        app_state.ethercat_status = soem_interface_get_communication_status();
        app_state.hid_status = hid_interface_get_connection_status();
        
        if (app_state.ethercat_status != app_state.last_ethercat_status) {
            printf("EtherCAT status changed: %s\n", 
                   app_state.ethercat_status ? "Connected" : "Disconnected");
            if (!app_state.ethercat_status) {
                app_state.stats.communication_errors++;
            }
            app_state.last_ethercat_status = app_state.ethercat_status;
        }
        
        // 2. Get current position and velocity from servo
        app_state.current_position = soem_interface_get_current_position();
        app_state.current_velocity = soem_interface_get_current_velocity();
        
        // 3. Get FFB commands from PC
        app_state.effect_available = hid_interface_get_ffb_effect(&app_state.current_ffb_effect);
        const ffb_effect_t *effect_ptr = app_state.effect_available ? &app_state.current_ffb_effect : NULL;
        
        // 4. Calculate desired torque
        app_state.desired_torque = ffb_calculator_calculate_torque(
            effect_ptr, app_state.current_position, app_state.current_velocity);
        
        // 5. Apply safety checks
        apply_safety_checks(&app_state);
        
        // 6. Send torque command to servo (only if EtherCAT is connected)
        if (app_state.ethercat_status && !emergency_stop) {
            soem_interface_send_and_receive_pdo(app_state.desired_torque);
        } else {
            // Send zero torque if communication is lost or emergency stop is active
            soem_interface_send_and_receive_pdo(0.0f);
        }
        
        // 7. Read button states
        app_state.button_states = read_button_states();
        
        // 8. Send gamepad report to PC (only if HID is connected)
        if (app_state.hid_status) {
            app_state.normalized_position = normalize_position_for_hid(app_state.current_position);
            hid_interface_send_gamepad_report(app_state.normalized_position, app_state.button_states);
        }
        
        // 9. Update performance statistics
        clock_gettime(CLOCK_MONOTONIC, &app_state.loop_end_time);
        update_performance_stats(&app_state);
        
        // 10. Print periodic statistics
        if (app_state.stats.loop_count % STATS_PRINT_INTERVAL == 0) {
            printf("Status: Pos=%.1f°, Vel=%.1f°/s, Torque=%.1f, EtherCAT=%s, HID=%s, Emergency=%s\n",
                   app_state.current_position, app_state.current_velocity, app_state.desired_torque,
                   app_state.ethercat_status ? "OK" : "LOST",
                   app_state.hid_status ? "OK" : "LOST",
                   emergency_stop ? "STOP" : "OK");
        }
        
        // 11. Maintain loop timing
        maintain_loop_timing(&app_state.loop_start_time, &app_state.loop_end_time);
        
        // Clear emergency stop if torque is back to normal
        if (emergency_stop && fabs(app_state.desired_torque) < MAX_TORQUE_LIMIT * 0.5f) {
            emergency_stop = 0;
            printf("Emergency stop cleared\n");
        }
    }
    
    // --- Final Statistics and Cleanup ---
    printf("\n=== Final Statistics ===\n");
    print_performance_stats(&app_state.stats);
    
    cleanup_and_exit(EXIT_SUCCESS);
    return EXIT_SUCCESS; // Never reached
}