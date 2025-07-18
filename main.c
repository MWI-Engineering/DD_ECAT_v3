// main.c - Updated for better integration
#include <stdio.h>
#include <stdlib.h>
#include <time.h>   // For clock_gettime
#include <signal.h> // For signal handling
#include <unistd.h> // For usleep
#include <string.h> // For memset
#include <math.h>   // For fabs()
#include <limits.h> // For LONG_MAX
// Include libaries of this application
#include "hid_interface.h"
#include "ffb_calculator.h"
#include "soem_interface.h"
#include "ffb_types.h"

// Global flag for graceful shutdown
static volatile int running = 1;

// Signal handler for Ctrl+C
void sigint_handler(int signum) {
    printf("\nCaught signal %d, shutting down...\n", signum);
    running = 0;
}

// Function to convert position to normalized range for HID report
float normalize_position_for_hid(float position_degrees) {
    // Assuming steering wheel has ±540 degrees range (3 full turns)
    const float MAX_STEERING_ANGLE = 540.0f;
    
    // Clamp to range
    if (position_degrees > MAX_STEERING_ANGLE) position_degrees = MAX_STEERING_ANGLE;
    if (position_degrees < -MAX_STEERING_ANGLE) position_degrees = -MAX_STEERING_ANGLE;
    
    // Normalize to [-1.0, 1.0]
    return position_degrees / MAX_STEERING_ANGLE;
}

// Function to read button states (placeholder - implement according to your hardware)
unsigned int read_button_states() {
    // This is a placeholder function. In a real implementation, you would:
    // - Read GPIO pins on the Raspberry Pi
    // - Read from an I2C/SPI button controller
    // - Read from another HID device
    // - etc.
    // For now, return 0 (no buttons pressed)
    return 0;
}

// Function to calculate loop statistics
void calculate_loop_stats(struct timespec *start_time, struct timespec *end_time, 
                         long *min_time, long *max_time, long *total_time, int *loop_count) {
    long elapsed_ns = (end_time->tv_sec - start_time->tv_sec) * 1000000000L + 
                      (end_time->tv_nsec - start_time->tv_nsec);
    
    if (elapsed_ns < *min_time) *min_time = elapsed_ns;
    if (elapsed_ns > *max_time) *max_time = elapsed_ns;
    *total_time += elapsed_ns;
    (*loop_count)++;
}

int main(int argc, char *argv[]) {
    printf("Starting Raspberry Pi FFB Steering Wheel Application (C version)...\n");
    printf("Press Ctrl+C to stop the application.\n");

    // Register signal handler for graceful shutdown
    signal(SIGINT, sigint_handler);

    // --- Initialization ---
    printf("Initializing HID interface...\n");
    if (hid_interface_init() != 0) {
        fprintf(stderr, "Failed to initialize HID interface.\n");
        return EXIT_FAILURE;
    }
    
    if (hid_interface_start() != 0) {
        fprintf(stderr, "Failed to start HID interface.\n");
        hid_interface_stop();
        return EXIT_FAILURE;
    }

    printf("Initializing FFB calculator...\n");
    ffb_calculator_init();

    // Default EtherCAT interface name
    const char *ethercat_ifname = "eth1";
    if (argc > 1) {
        ethercat_ifname = argv[1]; // Allow specifying interface as command-line argument
    }

    printf("Initializing EtherCAT master on interface %s...\n", ethercat_ifname);
    // Function call from soem_interface_init_master to soem_interface_init_enhanced
    if (soem_interface_init_enhanced(ethercat_ifname) != 0) {
        fprintf(stderr, "Failed to initialize EtherCAT master. Exiting.\n");
        //soem_interface_stop_master();
        hid_interface_stop();
        return EXIT_FAILURE;
    }

    // Wait a moment for EtherCAT to stabilize
    printf("Waiting for EtherCAT to stabilize...\n");
    sleep(2);

    // --- Main Loop Configuration ---
    long loop_frequency = 1000; // Hz (1ms cycle time for FFB)
    long cycle_time_ns = 1000000000L / loop_frequency; // Nanoseconds per cycle
    
    printf("Entering main loop at %ld Hz (%.1f ms cycle time)...\n", 
           loop_frequency, cycle_time_ns / 1000000.0);

    // Loop timing variables
    struct timespec start_time, end_time;
    long elapsed_ns, sleep_ns;
    
    // Performance monitoring variables
    long min_loop_time = LONG_MAX;
    long max_loop_time = 0;
    long total_loop_time = 0;
    int loop_count = 0;
    int stats_print_interval = 1000; // Print stats every 1000 loops (1 second)

    // FFB and control variables
    ffb_effect_t current_ffb_effect;
    memset(&current_ffb_effect, 0, sizeof(current_ffb_effect));
    
    int effect_available = 0;
    float current_position = 0.0f;
    float current_velocity = 0.0f;
    float desired_torque = 0.0f;
    float normalized_position = 0.0f;
    unsigned int button_states = 0;
    
    // State tracking
    int communication_errors = 0;
    int last_communication_status = 1;

    // --- Main Control Loop ---
    while (running) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // 1. Check EtherCAT communication status
        int comm_status = soem_interface_get_communication_status();
        if (comm_status != last_communication_status) {
            if (comm_status) {
                printf("EtherCAT communication restored.\n");
            } else {
                printf("EtherCAT communication lost!\n");
                communication_errors++;
            }
            last_communication_status = comm_status;
        }

        // 2. Get current position and velocity from servo (via EtherCAT)
        current_position = soem_interface_get_current_position();
        current_velocity = soem_interface_get_current_velocity();

        // 3. Get FFB commands from PC (via HID)
        effect_available = hid_interface_get_ffb_effect(&current_ffb_effect);
        const ffb_effect_t *effect_ptr = effect_available ? &current_ffb_effect : NULL;

        // 4. Calculate desired torque based on FFB effects
        desired_torque = ffb_calculator_calculate_torque(effect_ptr, current_position, current_velocity);

        // 5. Apply safety limits to torque
        const float MAX_TORQUE = 5000.0f; // Adjust based on your motor specifications
        if (fabs(desired_torque) > MAX_TORQUE) {
            desired_torque = (desired_torque > 0) ? MAX_TORQUE : -MAX_TORQUE;
        }

        // 6. Send torque command to servo (via EtherCAT)
        soem_interface_send_and_receive_pdo(desired_torque);

        // 7. Read button states
        button_states = read_button_states();

        // 8. Send gamepad report (position, buttons) to PC (via HID)
        normalized_position = normalize_position_for_hid(current_position);
        hid_interface_send_gamepad_report(normalized_position, button_states);

        // --- Performance Monitoring ---
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        calculate_loop_stats(&start_time, &end_time, &min_loop_time, &max_loop_time, 
                           &total_loop_time, &loop_count);

        // Print statistics periodically
        if (loop_count % stats_print_interval == 0) {
            long avg_loop_time = total_loop_time / loop_count;
            printf("Loop Stats: Min=%.3fms, Max=%.3fms, Avg=%.3fms, Pos=%.1f°, Vel=%.1f°/s, Torque=%.1f, CommErr=%d\n",
                   min_loop_time / 1000000.0, max_loop_time / 1000000.0, avg_loop_time / 1000000.0,
                   current_position, current_velocity, desired_torque, communication_errors);
        }

        // --- Maintain Loop Frequency ---
        elapsed_ns = (end_time.tv_sec - start_time.tv_sec) * 1000000000L + 
                     (end_time.tv_nsec - start_time.tv_nsec);
        sleep_ns = cycle_time_ns - elapsed_ns;

        if (sleep_ns > 0) {
            usleep(sleep_ns / 1000); // usleep takes microseconds
        } else if (sleep_ns < -1000000) { // More than 1ms late
            static int late_warning_count = 0;
            if (late_warning_count++ < 10) { // Limit warnings to avoid spam
                printf("Warning: Loop running %.3fms late (target: %.3fms, actual: %.3fms)\\n",
                       -sleep_ns / 1000000.0, cycle_time_ns / 1000000.0, elapsed_ns / 1000000.0);
            }
        }
    }

    // --- Final Statistics ---
    if (loop_count > 0) {
        long avg_loop_time = total_loop_time / loop_count;
        printf("\nFinal Loop Statistics:\n");
        printf("  Total loops: %d\n", loop_count);
        printf("  Min loop time: %.3f ms\n", min_loop_time / 1000000.0);
        printf("  Max loop time: %.3f ms\n", max_loop_time / 1000000.0);
        printf("  Avg loop time: %.3f ms\n", avg_loop_time / 1000000.0);
        printf("  Communication errors: %d\n", communication_errors);
    }

    // --- Cleanup ---
    printf("Cleaning up...\n");
    
    // Stop EtherCAT master first to ensure safe torque shutdown
    soem_interface_stop_master();
    
    // Stop HID interface
    hid_interface_stop();
    
    printf("Application stopped successfully.\n");
    return EXIT_SUCCESS;
}
