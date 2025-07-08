// main.c
#include <stdio.h>
#include <stdlib.h>
#include <time.h>   // For clock_gettime
#include <signal.h> // For signal handling

// Add this define before including unistd.h to help with usleep declaration
#include <unistd.h> // For usleep

#include "hid_interface.h"
#include "ffb_calculator.h"
#include "soem_interface.h"

// Global flag for graceful shutdown
static volatile int running = 1;

// Signal handler for Ctrl+C
void sigint_handler(int signum) {
    printf("\nCaught signal %d, shutting down...\n", signum);
    running = 0;
}

int main(int argc, char *argv[]) {
    printf("Starting Raspberry Pi FFB Steering Wheel Application (C version)...\n");

    // Register signal handler for graceful shutdown
    signal(SIGINT, sigint_handler);

    // --- Initialization ---
    if (hid_interface_init() != 0) {
        fprintf(stderr, "Failed to initialize HID interface.\n");
        return EXIT_FAILURE;
    }
    if (hid_interface_start() != 0) {
        fprintf(stderr, "Failed to start HID interface.\n");
        hid_interface_stop();
        return EXIT_FAILURE;
    }

    ffb_calculator_init();

    // Default EtherCAT interface name
    const char *ethercat_ifname = "eth0";
    if (argc > 1) {
        ethercat_ifname = argv[1]; // Allow specifying interface as command-line argument
    }

    if (soem_interface_init_master(ethercat_ifname) != 0) {
        fprintf(stderr, "Failed to initialize EtherCAT master. Exiting.\n");
        hid_interface_stop();
        return EXIT_FAILURE;
    }

    // --- Main Loop ---
    long loop_frequency = 1000; // Hz (1ms cycle time for FFB)
    long cycle_time_ns = 1000000000L / loop_frequency; // Nanoseconds per cycle

    printf("Entering main loop at %ld Hz...\n", loop_frequency);

    struct timespec start_time, end_time;
    long elapsed_ns, sleep_ns;

    ffb_effect_t current_ffb_effect;
    int effect_available;
    float current_position;
    float current_velocity;
    float desired_torque;
    unsigned int dummy_buttons = 0; // Placeholder for button states

    while (running) {
        clock_gettime(CLOCK_MONOTONIC, &start_time);

        // 1. Get FFB Commands from PC (via HID)
        effect_available = hid_interface_get_ffb_effect(&current_ffb_effect);
        const ffb_effect_t *effect_ptr = effect_available ? &current_ffb_effect : NULL;

        // 2. Get Current Position and Velocity from Servo (via SOEM)
        current_position = soem_interface_get_current_position();
        current_velocity = soem_interface_get_current_velocity();

        // 3. Calculate Desired Torque
        desired_torque = ffb_calculator_calculate_torque(effect_ptr, current_position, current_velocity);

        // 4. Send Torque to Servo and Receive New Position/Velocity (via SOEM)
        soem_interface_send_and_receive_pdo(desired_torque);

        // 5. Send Gamepad Report (Position, Buttons) to PC (via HID)
        // For buttons, you'd read GPIOs or other input sources on the RPi
        hid_interface_send_gamepad_report(current_position, dummy_buttons);

        // --- Maintain Loop Frequency ---
        clock_gettime(CLOCK_MONOTONIC, &end_time);
        elapsed_ns = (end_time.tv_sec - start_time.tv_sec) * 1000000000L + (end_time.tv_nsec - start_time.tv_nsec);
        sleep_ns = cycle_time_ns - elapsed_ns;

        if (sleep_ns > 0) {
            usleep(sleep_ns / 1000); // usleep takes microseconds
        } else {
            // fprintf(stderr, "Warning: Loop missed deadline by %ld ns\n", -sleep_ns);
        }
    }

    // --- Cleanup ---
    hid_interface_stop();
    soem_interface_stop_master();
    printf("Application stopped.\n");

    return EXIT_SUCCESS;
}
