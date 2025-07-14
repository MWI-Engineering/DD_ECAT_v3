// soem_interface.c - Updated for proper integration
#include "soem_interface.h"
#include <stdio.h>
#include <unistd.h> // For usleep
#include <string.h> // For memset
#include <pthread.h>
#include <stdarg.h> // For va_list in ecat_print_func
#include <math.h>   // For scaling calculations

// --- SOEM Library Includes ---
#include "ethercat.h"
#include "ethercattype.h" 

// --- SOEM Global Variables ---
char IOmap[4096]; // Global memory for EtherCAT Process Data
ec_ODlistt ODlist; // Object dictionary list
ec_groupt DCgroup; // DC group for distributed clocks (if used)
int wkc;           // Working counter
int expectedWKC;   // Expected working counter
ec_timet tmo;      // Timeout for SOEM functions

// --- PDO Structures for Synapticon ACTILINK-S (Slave 1) ---
typedef struct PACKED
{
    uint16 controlword;         // 0x6040:0x00
    int8   modes_of_operation;  // 0x6060:0x00
    int16  target_torque;       // 0x6071:0x00
    int32  target_position;     // 0x607A:0x00
    int32  target_velocity;     // 0x60FF:0x00
    int16  torque_offset;       // 0x60B2:0x00
    uint32 tuning_command;      // 0x2701:0x00
    uint32 physical_outputs;    // 0x60FE:0x01
    uint32 bit_mask;            // 0x60FE:0x02
    uint32 user_mosi;           // 0x2703:0x00
    int32  velocity_offset;     // 0x60B1:0x00
} SomanetOutputs_t;

typedef struct PACKED
{
    uint16 statusword;              // 0x6041:0x00
    int8   modes_of_operation_disp; // 0x6061:0x00
    int32  position_actual_value;   // 0x6064:0x00
    int32  velocity_actual_value;   // 0x606C:0x00
    int16  torque_actual_value;     // 0x6077:0x00
    uint16 analog_input_1;          // 0x2401:0x00
    uint16 analog_input_2;          // 0x2402:0x00
    uint16 analog_input_3;          // 0x2403:0x00
    uint16 analog_input_4;          // 0x2404:0x00
    uint32 tuning_status;           // 0x2702:0x00
    uint32 digital_inputs;          // 0x60FD:0x00
    uint32 user_miso;               // 0x2704:0x00
    uint32 timestamp;               // 0x20F0:0x00
    int32  position_demand_internal; // 0x60FC:0x00
    int32  velocity_demand_value;   // 0x606B:0x00
    int16  torque_demand;           // 0x6074:0x00
} SomanetInputs_t;

// Pointers to the PDO structures in the IOmap
static SomanetOutputs_t *somanet_outputs;
static SomanetInputs_t *somanet_inputs;

// --- Internal state variables ---
static int master_initialized = 0;
static volatile int ecat_thread_running = 0;
static pthread_t ecat_thread;
static pthread_mutex_t pdo_mutex = PTHREAD_MUTEX_INITIALIZER;

// Current values - protected by mutex
static float current_position_f = 0.0f;
static float current_velocity_f = 0.0f;
static float target_torque_f = 0.0f;
static int communication_ok = 0;

// Scaling factors - adjust these based on your motor specifications
#define POSITION_SCALE_FACTOR (360.0f / 1000000.0f)  // Convert from encoder counts to degrees
#define VELOCITY_SCALE_FACTOR (1.0f / 1000.0f)       // Convert from internal units to deg/s
#define TORQUE_SCALE_FACTOR (1000.0f)                // Convert from float torque to motor units

// SOEM hook for printing messages
void ecat_print_func(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

// EtherCAT real-time loop thread
OSAL_THREAD_FUNC ecat_loop(void *ptr)
{
    printf("EtherCAT loop thread started.\n");
    
    float local_target_torque = 0.0f;
    int consecutive_errors = 0;
    const int MAX_CONSECUTIVE_ERRORS = 10;

    while (ecat_thread_running) {
        // Get target torque from main thread
        pthread_mutex_lock(&pdo_mutex);
        local_target_torque = target_torque_f;
        pthread_mutex_unlock(&pdo_mutex);

        // Update output PDOs
        if (somanet_outputs) {
            somanet_outputs->target_torque = (int16)(local_target_torque * TORQUE_SCALE_FACTOR);
            // Keep controlword at 0x0F (operation enabled) during normal operation
            somanet_outputs->controlword = 0x0F;
        }

        // Send process data to slaves
        ec_send_processdata();
        
        // Receive process data from slaves
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc >= expectedWKC) {
            // Communication successful
            consecutive_errors = 0;
            printf("Communication is succesful");
            
            if (somanet_inputs) {
                // Update current values with thread safety
                pthread_mutex_lock(&pdo_mutex);
                current_position_f = (float)somanet_inputs->position_actual_value * POSITION_SCALE_FACTOR;
                current_velocity_f = (float)somanet_inputs->velocity_actual_value * VELOCITY_SCALE_FACTOR;
                communication_ok = 1;
                pthread_mutex_unlock(&pdo_mutex);

                // Check for drive faults
                if (somanet_inputs->statusword & 0x08) {
                    printf("EtherCAT: Drive fault detected! Statusword: 0x%04X\n", somanet_inputs->statusword);
                    // You might want to handle faults here (e.g., clear fault, stop operation)
                }
            }
        } else {
            // Communication error
            consecutive_errors++;
            pthread_mutex_lock(&pdo_mutex);
            communication_ok = 0;
            pthread_mutex_unlock(&pdo_mutex);
            
            if (consecutive_errors > MAX_CONSECUTIVE_ERRORS) {
                printf("EtherCAT: Too many consecutive communication errors (%d). Expected WKC: %d, Got: %d\n", 
                       consecutive_errors, expectedWKC, wkc);
                // Reset counter to avoid spam
                consecutive_errors = 0;
            }
        }

        // 1ms cycle time
        usleep(1000);
    }

    printf("EtherCAT loop thread stopped.\n");
    return NULL;
}

/**
 * @brief Initializes the SOEM master and discovers slaves.
 * @param ifname The network interface name (e.g., "eth0").
 * @return 0 on success, -1 on failure.
 */
int soem_interface_init_master(const char *ifname) {
    printf("SOEM_Interface: Initializing EtherCAT master for interface %s...\n", ifname);

    // Initialize SOEM master
    if (ec_init(ifname)) {
        printf("SOEM_Interface: ec_init success.\n");

        // Find and configure slaves
        if (ec_config_init(FALSE) > 0) {
            printf("SOEM_Interface: %d slaves found and configured.\n", ec_slavecount);
            
        //Compare Ibytes and Obytes
            for (int i = 1; i <= ec_slavecount; i++) {
                printf("Slave %d: name = %s\n", i, ec_slave[i].name);
                printf("  Input size: %d bytes\n", ec_slave[i].Ibytes);
                printf("  Output size: %d bytes\n", ec_slave[i].Obytes);
                printf("  State: 0x%02X\n", ec_slave[i].state);
            }

            printf("Expected input size: %zu, output size: %zu\n",
            sizeof(SomanetInputs_t), sizeof(SomanetOutputs_t));

        // Map PDOs for all slaves
            if (ec_slavecount >= 1) {
                ec_config_map(&IOmap[0]);
                
        // Wait until all slaves reach SAFE_OP state
             printf("SOEM_Interface: Waiting for all slaves to reach SAFE_OP...\n");

            int wait_count = 0;
            const int max_waits = 50; // e.g., wait up to 5 seconds (50 * 100ms)

            do {
                 ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);

                int all_safe_op = 1;
                for (int i = 1; i <= ec_slavecount; i++) {
                    if (ec_slave[i].state != EC_STATE_SAFE_OP) {
                   printf("  Slave %d not in SAFE_OP (state: 0x%02X)\n", i, ec_slave[i].state);
                   all_safe_op = 0;
                    }
                }

                if (all_safe_op) {
                 printf("SOEM_Interface: All slaves are in SAFE_OP.\n");
                 break;
                }

                usleep(100000); // Wait 100ms
                wait_count++;
                } while (wait_count < max_waits);

                if (wait_count >= max_waits) {
                    fprintf(stderr, "SOEM_Interface: Timeout waiting for SAFE_OP state.\n");
                    ec_close();
                    return -1;
                }

        // Calculate expected working counter
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
                printf("SOEM_Interface: Expected working counter: %d\n", expectedWKC);

        // Get pointers to PDOs for slave 1 (Synapticon ACTILINK-S)
            if (ec_slave[1].outputs && ec_slave[1].inputs) {
                somanet_outputs = (SomanetOutputs_t *)ec_slave[1].outputs;
                somanet_inputs = (SomanetInputs_t *)ec_slave[1].inputs;
                    
                printf("SOEM_Interface: PDO mapping complete. Slave 1 - Output size: %d bytes, Input size: %d bytes.\n",
                           ec_slave[1].Obytes, ec_slave[1].Ibytes);
                    
        // Verify PDO sizes match our structures
            if (ec_slave[1].Obytes != sizeof(SomanetOutputs_t) || 
                    ec_slave[1].Ibytes != sizeof(SomanetInputs_t)) {
                    printf("SOEM_Interface: Warning - PDO size mismatch! Expected Out:%zu In:%zu, Got Out:%d In:%d\n",
                    sizeof(SomanetOutputs_t), sizeof(SomanetInputs_t),
                    ec_slave[1].Obytes, ec_slave[1].Ibytes);
                    }
                } else {
                    fprintf(stderr, "SOEM_Interface: Failed to get PDO pointers for Slave 1.\n");
                    ec_close();
                    return -1;
                }

        // Clear output PDOs initially
            memset(somanet_outputs, 0, sizeof(SomanetOutputs_t));

        // Read initial drive status before sending controlword
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            if (wkc >= expectedWKC && somanet_inputs) {
                printf("SOEM_Interface: Initial drive state before controlword sequence:\n");
                printf("  Statusword: 0x%04X\n", somanet_inputs->statusword);
                printf("  Modes of Operation Display: 0x%02X\n", somanet_inputs->modes_of_operation_disp);
                printf("  Position: %d\n", somanet_inputs->position_actual_value);
                printf("  Velocity: %d\n", somanet_inputs->velocity_actual_value);
                printf("  Torque: %d\n", somanet_inputs->torque_actual_value);
                
            } else {
                fprintf(stderr, "SOEM_Interface: Failed to read initial drive state (WKC: %d, expected: %d).\n", wkc, expectedWKC);
                }

        // Initialize drive state machine
            printf("SOEM_Interface: Initializing drive state machine...\n");               
                
        // Set Modes of Operation to Cyclic Synchronous Torque (CST)
                somanet_outputs->modes_of_operation = 0x0A; // CST mode
                
        // State machine sequence: Ready to switch on -> Switch on -> Operation enabled
            const uint16_t state_sequence[] = {0x06, 0x07, 0x0F};
            const char* state_names[] = {"Ready to switch on", "Switch on", "Operation enabled"};
                
                for (int i = 0; i < 3; i++) {
                    somanet_outputs->controlword = state_sequence[i];
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);
                    
                    if (wkc >= expectedWKC) {
                        printf("SOEM_Interface: %s - Controlword: 0x%04X, Statusword: 0x%04X\n", 
                               state_names[i], somanet_outputs->controlword, somanet_inputs->statusword);
                    } else {
                        fprintf(stderr, "SOEM_Interface: Communication error during state transition %d\n", i);
                    }
                    usleep(100000); // Wait 100ms between state transitions
                }

        // Request OP state for all slaves
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
                
        // Check if all slaves reached operational state
            int all_operational = 1;
                for (int i = 1; i <= ec_slavecount; i++) {
                    if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                        printf("SOEM_Interface: Slave %d not operational. State: %d\n", i, ec_slave[i].state);
                        all_operational = 0;
                    }
                }
                
                if (all_operational) {
                    printf("SOEM_Interface: All slaves operational. Starting EtherCAT thread...\n");
                    
            // Start EtherCAT communication thread
                    ecat_thread_running = 1;
                    if (pthread_create(&ecat_thread, NULL, ecat_loop, NULL) != 0) {
                        fprintf(stderr, "SOEM_Interface: Failed to create EtherCAT thread.\n");
                        ec_close();
                        return -1;
                    }
                    
                    master_initialized = 1;
                    printf("SOEM_Interface: EtherCAT master initialized successfully.\n");
                    return 0;
                } else {
                    fprintf(stderr, "SOEM_Interface: Not all slaves reached operational state.\n");
                    ec_close();
                    return -1;
                }
            } else {
                fprintf(stderr, "SOEM_Interface: No EtherCAT slaves found.\n");
                ec_close();
                return -1;
            }
        } else {
            fprintf(stderr, "SOEM_Interface: No slaves found on %s.\n", ifname);
            ec_close();
            return -1;
        }
    } else {
        fprintf(stderr, "SOEM_Interface: No socket connection on %s. Run with 'sudo' or set CAP_NET_RAW.\n", ifname);
        return -1;
    }
}

/**
 * @brief Sends target torque to the EtherCAT thread.
 * The actual PDO communication happens in the EtherCAT thread.
 * @param target_torque The desired torque to send to the servo motor.
 */
void soem_interface_send_and_receive_pdo(float target_torque) {
    if (!master_initialized) {
        return;
    }

// Update target torque for the EtherCAT thread
    pthread_mutex_lock(&pdo_mutex);
    target_torque_f = target_torque;
    pthread_mutex_unlock(&pdo_mutex);
}

/**
 * @brief Returns the last known position from the servo motor.
 * @return The current angular position in degrees.
 */
float soem_interface_get_current_position() {
    float position;
    pthread_mutex_lock(&pdo_mutex);
    position = current_position_f;
    pthread_mutex_unlock(&pdo_mutex);
    return position;
}

/**
 * @brief Returns the last known velocity from the servo motor.
 * @return The current angular velocity in degrees/second.
 */
float soem_interface_get_current_velocity() {
    float velocity;
    pthread_mutex_lock(&pdo_mutex);
    velocity = current_velocity_f;
    pthread_mutex_unlock(&pdo_mutex);
    return velocity;
}

/**
 * @brief Returns the communication status.
 * @return 1 if communication is OK, 0 if there are communication errors.
 */
int soem_interface_get_communication_status() {
    int status;
    pthread_mutex_lock(&pdo_mutex);
    status = communication_ok;
    pthread_mutex_unlock(&pdo_mutex);
    return status;
}

/**
 * @brief Stops the SOEM master and cleans up resources.
 */
void soem_interface_stop_master() {
    if (master_initialized) {
        printf("SOEM_Interface: Stopping EtherCAT master...\n");
        
        // Stop EtherCAT thread
        ecat_thread_running = 0;
        if (ecat_thread) {
            pthread_join(ecat_thread, NULL);
            printf("SOEM_Interface: EtherCAT thread stopped.\n");
        }
        
        // Set drive to safe state before closing
        if (somanet_outputs) {
            somanet_outputs->target_torque = 0;  // Zero torque
            somanet_outputs->controlword = 0x06; // Ready to switch on
            ec_send_processdata();
            usleep(10000); // Wait 10ms
        }
        
        // Request PREOP state for all slaves before closing
        ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
        ec_close();
        
        master_initialized = 0;
        printf("SOEM_Interface: EtherCAT master stopped.\n");
    }
}