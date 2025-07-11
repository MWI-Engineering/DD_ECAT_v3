// main_integration.c - Main integration file for FFB system with UDP communication
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>

// Include your existing headers
#include "hid_interface.h"
#include "udp_communication.h"
// #include "soem_interface.h"    // Your EtherCAT interface
// #include "ffb_calculator.h"    // Your FFB calculation engine
// #include "motor_control.h"     // Your motor control interface

// Global variables
static volatile bool system_running = true;
static pthread_mutex_t system_mutex = PTHREAD_MUTEX_INITIALIZER;

// System state
static float current_wheel_position = 0.0f;
static float current_wheel_speed = 0.0f;
static uint16_t current_button_state = 0;
static bool ffb_enabled = true;
static float force_multiplier = 1.0f;
static char current_effect_name[64] = "None";

// Motor status (you'll replace these with actual motor interface calls)
static float motor_current = 0.0f;
static float motor_temperature = 25.0f;
static float motor_voltage = 24.0f;
static float motor_load = 0.0f;
static bool is_calibrated = false;
static bool emergency_stop_active = false;

// Signal handler for graceful shutdown
void signal_handler(int signum) {
    printf("\nReceived signal %d, shutting down...\n", signum);
    system_running = false;
}

// Stub functions for your existing motor/wheel interfaces
// Replace these with calls to your actual motor control system

void motor_emergency_stop(void) {
    printf("MOTOR: Emergency stop activated!\n");
    pthread_mutex_lock(&system_mutex);
    emergency_stop_active = true;
    pthread_mutex_unlock(&system_mutex);
    
    // Add your actual emergency stop code here
    // - Disable motor power
    // - Set safe state
    // - Stop all FFB effects
}

void motor_calibrate(void) {
    printf("MOTOR: Starting calibration...\n");
    pthread_mutex_lock(&system_mutex);
    is_calibrated = false;
    pthread_mutex_unlock(&system_mutex);
    
    // Add your actual calibration code here
    // - Move to center position
    // - Find limits
    // - Set zero position
    
    sleep(2); // Simulate calibration time
    
    pthread_mutex_lock(&system_mutex);
    is_calibrated = true;
    current_wheel_position = 0.0f;
    pthread_mutex_unlock(&system_mutex);
    
    printf("MOTOR: Calibration complete!\n");
}

void ffb_enable(bool enable) {
    pthread_mutex_lock(&system_mutex);
    ffb_enabled = enable;
    pthread_mutex_unlock(&system_mutex);
    
    printf("FFB: %s\n", enable ? "Enabled" : "Disabled");
    
    // Add your actual FFB enable/disable code here
}

void ffb_set_force_multiplier(float multiplier) {
    pthread_mutex_lock(&system_mutex);
    force_multiplier = multiplier;
    pthread_mutex_unlock(&system_mutex);
    
    printf("FFB: Force multiplier set to %.2f\n", multiplier);
    
    // Add your actual force multiplier code here
}

void ffb_reset_all_effects(void) {
    printf("FFB: Resetting all effects\n");
    
    pthread_mutex_lock(&system_mutex);
    strcpy(current_effect_name, "None");
    pthread_mutex_unlock(&system_mutex);
    
    // Add your actual effect reset code here
}

// Getter functions for UDP communication
float motor_get_current(void) {
    pthread_mutex_lock(&system_mutex);
    float current = motor_current;
    pthread_mutex_unlock(&system_mutex);
    return current;
}

float motor_get_temperature(void) {
    pthread_mutex_lock(&system_mutex);
    float temp = motor_temperature;
    pthread_mutex_unlock(&system_mutex);
    return temp;
}

float motor_get_voltage(void) {
    pthread_mutex_lock(&system_mutex);
    float voltage = motor_voltage;
    pthread_mutex_unlock(&system_mutex);
    return voltage;
}

float motor_get_load(void) {
    pthread_mutex_lock(&system_mutex);
    float load = motor_load;
    pthread_mutex_unlock(&system_mutex);
    return load;
}

float wheel_get_position(void) {
    pthread_mutex_lock(&system_mutex);
    float position = current_wheel_position;
    pthread_mutex_unlock(&system_mutex);
    return position;
}

float wheel_get_speed(void) {
    pthread_mutex_lock(&system_mutex);
    float speed = current_wheel_speed;
    pthread_mutex_unlock(&system_mutex);
    return speed;
}

uint16_t wheel_get_button_state(void) {
    pthread_mutex_lock(&system_mutex);
    uint16_t buttons = current_button_state;
    pthread_mutex_unlock(&system_mutex);
    return buttons;
}

void wheel_get_current_ffb_effect(char* effect_name, int max_len) {
    pthread_mutex_lock(&system_mutex);
    strncpy(effect_name, current_effect_name, max_len - 1);
    effect_name[max_len - 1] = '\0';
    pthread_mutex_unlock(&system_mutex);
}

// Main system loop
void* system_loop(void* arg) {
    printf("System loop started\n");
    
    while (system_running) {
        // Process FFB effects from HID interface
        ffb_effect_t effect;
        while (hid_interface_get_ffb_effect(&effect)) {
            printf("Processing FFB effect: Type=%d, ID=%d, Magnitude=%.2f\n", 
                   effect.type, effect.id, effect.magnitude);
            
            // Update current effect name
            pthread_mutex_lock(&system_mutex);
            switch (effect.type) {
                case FFB_EFFECT_CONSTANT_FORCE:
                    strcpy(current_effect_name, "Constant Force");
                    break;
                case FFB_EFFECT_SPRING:
                    strcpy(current_effect_name, "Spring");
                    break;
                case FFB_EFFECT_DAMPER:
                    strcpy(current_effect_name, "Damper");
                    break;
                case FFB_EFFECT_PERIODIC:
                    strcpy(current_effect_name, "Periodic");
                    break;
                case FFB_EFFECT_RAMP:
                    strcpy(current_effect_name, "Ramp");
                    break;
                default:
                    strcpy(current_effect_name, "Unknown");
                    break;
            }
            pthread_mutex_unlock(&system_mutex);
            
            // Process the effect through your FFB calculator and motor control
            // ffb_calculator_process_effect(&effect);
            // motor_control_apply_force(calculated_force);
        }
        
        // Update wheel position and speed (replace with actual sensor readings)
        pthread_mutex_lock(&system_mutex);
        // current_wheel_position = get_actual_wheel_position();
        // current_wheel_speed = get_actual_wheel_speed();
        // current_button_state = get_actual_button_state();
        
        // Update motor status (replace with actual motor readings)
        // motor_current = get_actual_motor_current();
        // motor_temperature = get_actual_motor_temperature();
        // motor_voltage = get_actual_motor_voltage();
        // motor_load = get_actual_motor_load();
        pthread_mutex_unlock(&system_mutex);
        
        // Send gamepad report
        hid_interface_send_gamepad_report(current_wheel_position, current_button_state);
        
        // Sleep for a short time (adjust based on your control loop frequency)
        usleep(1000); // 1ms = 1000Hz loop
    }
    
    printf("System loop stopped\n");
    return NULL;
}

int main(int argc, char* argv[]) {
    printf("FFB Wheel System Starting...\n");
    
    // Install signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize HID interface
    if (hid_interface_init() != 0) {
        fprintf(stderr, "Failed to initialize HID interface\n");
        return 1;
    }
    
    // Initialize UDP communication
    if (udp_communication_init() != 0) {
        fprintf(stderr, "Failed to initialize UDP communication\n");
        hid_interface_stop();
        return 1;
    }
    
    // Initialize your other subsystems here
    // if (soem_interface_init() != 0) {
    //     fprintf(stderr, "Failed to initialize EtherCAT interface\n");
    //     return 1;
    // }
    
    // if (ffb_calculator_init() != 0) {
    //     fprintf(stderr, "Failed to initialize FFB calculator\n");
    //     return 1;
    // }
    
    // if (motor_control_init() != 0) {
    //     fprintf(stderr, "Failed to initialize motor control\n");
    //     return 1;
    // }
    
    // Start all subsystems
    if (hid_interface_start() != 0) {
        fprintf(stderr, "Failed to start HID interface\n");
        return 1;
    }
    
    if (udp_communication_start() != 0) {
        fprintf(stderr, "Failed to start UDP communication\n");
        hid_interface_stop();
        return 1;
    }
    
    // Start your other subsystems here
    // soem_interface_start();
    // ffb_calculator_start();
    // motor_control_start();
    
    // Create main system thread
    pthread_t system_thread;
    if (pthread_create(&system_thread, NULL, system_loop, NULL) != 0) {
        fprintf(stderr, "Failed to create system thread\n");
        return 1;
    }
    
    printf("FFB Wheel System running. Press Ctrl+C to stop.\n");
    
    // Wait for system thread to finish
    pthread_join(system_thread, NULL);
    
    // Cleanup
    printf("Shutting down subsystems...\n");
    
    udp_communication_stop();
    hid_interface_stop();
    
    // Stop your other subsystems here
    // motor_control_stop();
    // ffb_calculator_stop();
    // soem_interface_stop();
    
    printf("FFB Wheel System stopped.\n");
    return 0;
}

// Build instructions:
// gcc -o ffb_system main_integration.c udp_communication.c hid_interface.c \
//     -lpthread -ljson-c -lm
// 
// Make sure to install json-c library:
// sudo apt-get install libjson-c-dev
//
// Run with:
// sudo ./ffb_system
