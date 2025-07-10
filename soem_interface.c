// soem_interface.c
#include "soem_interface.h"
#include <stdio.h>
#include <unistd.h> // For usleep
#include <string.h> // For memset
#include <pthread.h>
#include <stdarg.h> // For va_list in ecat_print_func

// --- SOEM Library Includes ---
#include <ethercat.h> 
#include "ethercattype.h" 
// Try including ethercatprint.h - if this doesn't exist, we'll handle it differently
#ifdef HAVE_ETHERCATPRINT_H
#include <ethercatprint.h>
#endif

// --- SOEM Global Variables ---
char IOmap[4096]; // Global memory for EtherCAT Process Data
ec_ODlistt ODlist; // Object dictionary list
ec_groupt DCgroup; // DC group for distributed clocks (if used)
int wkc;           // Working counter
int expectedWKC;   // Expected working counter
ec_timet tmo;      // Timeout for SOEM functions

// --- PDO Structures for Synapticon ACTILINK-S (Slave 1) ---
// These structures are packed to ensure no padding bytes are added by the compiler,
// which is crucial for direct memory access to EtherCAT PDOs.

// Output PDOs (from Master to Slave) for SM2
// Based on the provided mapping:
// [0x0000.0] 0x6040:0x00 0x10 UNSIGNED16 Controlword
// [0x0002.0] 0x6060:0x00 0x08 INTEGER8 Modes of operation
// [0x0003.0] 0x6071:0x00 0x10 INTEGER16 Target Torque
// [0x0005.0] 0x607A:0x00 0x20 INTEGER32 Target position
// [0x0009.0] 0x60FF:0x00 0x20 INTEGER32 Target velocity
// [0x000D.0] 0x60B2:0x00 0x10 INTEGER16 Torque offset
// [0x000F.0] 0x2701:0x00 0x20 UNSIGNED32 Tuning command
// [0x0013.0] 0x60FE:0x01 0x20 UNSIGNED32 Physical outputs
// [0x0017.0] 0x60FE:0x02 0x20 UNSIGNED32 Bit mask
// [0x001B.0] 0x2703:0x00 0x20 UNSIGNED32 User MOSI
// [0x001F.0] 0x60B1:0x00 0x20 INTEGER32 Velocity offset
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

// Input PDOs (from Slave to Master) for SM3
// Based on the provided mapping:
// [0x0023.0] 0x6041:0x00 0x10 UNSIGNED16 Statusword
// [0x0025.0] 0x6061:0x00 0x08 INTEGER8 Modes of operation display
// [0x0026.0] 0x6064:0x00 0x20 INTEGER32 Position actual value
// [0x002A.0] 0x606C:0x00 0x20 INTEGER32 Velocity actual value
// [0x002E.0] 0x6077:0x00 0x10 INTEGER16 Torque actual value
// [0x0030.0] 0x2401:0x00 0x10 UNSIGNED16 Analog input 1
// [0x0032.0] 0x2402:0x00 0x10 UNSIGNED16 Analog input 2
// [0x0034.0] 0x2403:0x00 0x10 UNSIGNED16 Analog input 3
// [0x0036.0] 0x2404:0x00 0x10 UNSIGNED16 Analog input 4
// [0x0038.0] 0x2702:0x00 0x20 UNSIGNED32 Tuning status
// [0x003C.0] 0x60FD:0x00 0x20 UNSIGNED32 Digital inputs
// [0x0040.0] 0x2704:0x00 0x20 UNSIGNED32 User MISO
// [0x0044.0] 0x20F0:0x00 0x20 UNSIGNED32 Timestamp
// [0x0048.0] 0x60FC:0x00 0x20 INTEGER32 Position demand internal value
// [0x004C.0] 0x606B:0x00 0x20 INTEGER32 Velocity demand value
// [0x0050.0] 0x6074:0x00 0x10 INTEGER16 Torque demand
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
static float current_position_f = 0.0f; // Floating point position
static float current_velocity_f = 0.0f; // Floating point velocity

// --- SOEM Utility Functions ---
// For printing SOEM messages
// Corrected ecat_loop: OSAL_THREAD_FUNC is typically void, so no return value.
// Add these function implementations somewhere after your global variables or other functions
void soem_interface_read_inputs(void)
{
    // For now, leave empty or add a printf for testing
    //printf("Reading inputs current_position_f\n");
    // Example: You would copy data from somanet_inputs to your application variables here
    // current_position_f = (float)somanet_inputs->position_actual_value;
}

void soem_interface_write_outputs(void)
{
    // For now, leave empty or add a printf for testing
    // printf("Writing outputs...\n");
    // Example: You would copy data from your application variables to somanet_outputs here
    // somanet_outputs->target_position = (int32)desired_position;
}

OSAL_THREAD_FUNC ecat_loop(void *ptr)
{
    // A flag to control the loop's execution, typically set to false to stop the thread
    // This 'run_thread' variable would be a global or static variable,
    // and your main application would set it to false when it wants to shut down.
    int *run_thread = (int *)ptr; // Assuming 'ptr' is a pointer to an integer flag

    printf("EtherCAT loop thread started.\n");

    // Main EtherCAT loop
    while (*run_thread) // Loop continues as long as the run_thread flag is true
    {
        // 1. Send process data (outputs) to slaves
        //   ec_send_processdata() sends the data from IOmap to the slaves.
        ec_send_processdata();

        // 2. Receive process data (inputs) from slaves
        //   ec_receive_processdata(EC_TIMEOUTRET) waits for received data up to a timeout.
        //   It returns the working counter (wkc) for the received data.
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        // 3. Process the received data (check working counter, update inputs)
        if (wkc >= expectedWKC)
        {
            // Data received successfully.
            // You can now access the input PDOs through 'somanet_inputs'.
            // Example:
            printf("Slave 1 Statusword: 0x%04X\n", somanet_inputs->statusword);
            printf("Slave 1 Position: %d\n", somanet_inputs->position_actual_value);

            // Call a function to read/update your application's input values
            // based on the received EtherCAT data.
            soem_interface_read_inputs(); // You would implement this function

            // Call a function to update the output values that will be sent in the next cycle.
            // This function would typically take control commands from your application.
            soem_interface_write_outputs(); // You would implement this function
        }
        else
        {
            // Working counter mismatch or timeout. This indicates a problem
            // (e.g., slave not responding, communication issue).
            // You might want to handle errors, try to recover, or log this.
            printf("EtherCAT loop: WKC mismatch. Expected %d, Got %d\n", expectedWKC, wkc);
            // Optionally, try to recover or re-initialize:
            // ec_readstate();
            // Check slave states, try to bring them back to OP.
        }

        // 4. Manage distributed clocks (if using DC)
        // If your system relies on Distributed Clocks for synchronization,
        // you would periodically update them here.
        // if (ec_configdc()) {
        //     // DC configured successfully
        // }

        // 5. Short delay for cycle time
        // This usleep defines the cycle time of your EtherCAT loop.
        // For a 1ms cycle, it would be 1000 microseconds.
        // Adjust this based on your application's requirements and slave capabilities.
        usleep(1000); // Example: 1ms cycle time (1000 microseconds)
    }

    printf("EtherCAT loop thread stopped.\n");
    // As OSAL_THREAD_FUNC is void, no return statement is needed.
    // The thread simply exits when *run_thread becomes false.
}

// SOEM hook for printing messages (optional, but good for debugging)
void ecat_print_func(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

/**
 * @brief Initializes the SOEM master and discovers slaves.
 * @param ifname The network interface name (e.g., "eth0").
 * @return 0 on success, -1 on failure.
 */
int soem_interface_init_master(const char *ifname) {
    printf("SOEM_Interface: Initializing EtherCAT master for interface %s...\n", ifname);

    // Try to set SOEM print function for debugging - handle different SOEM versions
    #ifdef HAVE_ETHERCATPRINT_H
        // If ethercatprint.h exists, try ec_set_print_func
        #ifdef ec_set_print_func
            ec_set_print_func(ecat_print_func);
        #endif
    #else
        // Alternative: Some SOEM versions might have this function directly in ethercat.h
        // or it might be called differently. Let's try a few common variations:
        #ifdef ec_printHook
            ec_printHook = ecat_print_func;
        #elif defined(ec_set_print_hook)
            ec_set_print_hook(ecat_print_func);
        #endif
    #endif
    
    // If none of the above work, you can comment out the print function setup
    // and SOEM will use its default printing behavior
    
    // Initialize SOEM master
    if (ec_init(ifname)) {
        printf("SOEM_Interface: ec_init success.\n");

        // Find and configure slaves
        if (ec_config_init(FALSE) > 0) {
            printf("SOEM_Interface: %d slaves found and configured.\n", ec_slavecount);

            if (ec_slavecount >= 1) {
                // Assuming Synapticon ACTILINK-S is Slave 1
                int slave_idx = 1;

                // Check if slave 1 is the expected Synapticon device
                // You might want to check Vendor ID (Man) and Product Code (ID) here
                // For example: if (ec_slave[slave_idx].eep_man == 0x000022d2 && ec_slave[slave_idx].eep_id == 0x00000401)
                printf("SOEM_Interface: Configuring PDOs for Synapticon ACTILINK-S (Slave %d)....\n", slave_idx);

                // Map PDOs for the specific slave
                ec_config_map_group(&IOmap[0], 0);
                // ec_config_map_v2(&IOmap[0], 0); // Map all slaves, starting from IOmap[0]

                // Get pointers to the PDOs for slave 1
                somanet_outputs = (SomanetOutputs_t *)ec_slave[slave_idx].outputs;
                somanet_inputs = (SomanetInputs_t *)ec_slave[slave_idx].inputs;

                if (somanet_outputs == NULL || somanet_inputs == NULL) {
                    fprintf(stderr, "SOEM_Interface: Failed to get PDO pointers for Slave %d.\n", slave_idx);
                    ec_close();
                    return -1;
                }

                printf("SOEM_Interface: PDO mapping complete. Output size: %d bytes, Input size: %d bytes.\n",
                       ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Ibytes);

                // Clear output PDOs initially
                memset(somanet_outputs, 0, sizeof(SomanetOutputs_t));

                // --- Set up initial drive state ---
                // Set Modes of Operation to Cyclic Synchronous Torque (CST) - usually 0x0A for CiA 402
                // Check Synapticon documentation for exact value if different.
                somanet_outputs->modes_of_operation = 0x0A; // CST mode

                // Set Controlword to "Ready to switch on" (0x06)
                somanet_outputs->controlword = 0x06;
                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);
                printf("SOEM_Interface: Sent Controlword 0x06, Statusword: 0x%X\n", somanet_inputs->statusword);
                usleep(10000); // Wait 10ms

                // Set Controlword to "Switch on" (0x07)
                somanet_outputs->controlword = 0x07;
                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);
                printf("SOEM_Interface: Sent Controlword 0x07, Statusword: 0x%X\n", somanet_inputs->statusword);
                usleep(10000); // Wait 10ms

                // Set Controlword to "Operation enabled" (0x0F)
                somanet_outputs->controlword = 0x0F;
                ec_send_processdata();
                wkc = ec_receive_processdata(EC_TIMEOUTRET);
                printf("SOEM_Interface: Sent Controlword 0x0F, Statusword: 0x%X\n", somanet_inputs->statusword);
                usleep(10000); // Wait 10ms

                // Request OP state for all slaves
                ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
                if (ec_slave[slave_idx].state == EC_STATE_OPERATIONAL) {
                    printf("SOEM_Interface: Synapticon ACTILINK-S (Slave %d) is in OPERATIONAL state.\n", slave_idx);
                    master_initialized = 1;
                    return 0; // Success
                } else {
                    fprintf(stderr, "SOEM_Interface: Failed to bring Synapticon ACTILINK-S (Slave %d) to OPERATIONAL state. Current state: %d\n", slave_idx, ec_slave[slave_idx].state);
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
 * @brief Sends process data (e.g., target torque) and receives process data (e.g., actual position).
 * This is the core real-time loop function.
 * @param target_torque The desired torque to send to the servo motor.
 */
void soem_interface_send_and_receive_pdo(float target_torque) {
    if (!master_initialized) {
        // printf("SOEM_Interface: Master not initialized. Cannot send/receive PDOs.\n");
        return;
    }

    // Write target_torque to the PDO.
    // The Synapticon expects INTEGER16 for Target Torque.
    // You'll need to scale your float torque value to the appropriate range for the motor.
    // For example, if your motor's max torque corresponds to 32767 (max for INTEGER16),
    // and your FFB calculator outputs a value from -5000 to 5000, you'd scale it.
    // Example scaling: (target_torque / 5000.0f) * 32767.0f
    somanet_outputs->target_torque = (int16)target_torque; // Simple cast for now, adjust scaling as needed

    // Send process data
    ec_send_processdata();
    wkc = ec_receive_processdata(EC_TIMEOUTRET); // Receive process data

    if (wkc >= expectedWKC) { // Check if all expected datagrams were received
        // Read actual position and velocity from input PDOs
        current_position_f = (float)somanet_inputs->position_actual_value;
        current_velocity_f = (float)somanet_inputs->velocity_actual_value;

        // You might want to add scaling for these values as well,
        // depending on the units reported by the Synapticon drive.
        // For example, if position is in encoder counts, convert to degrees/radians.

        // Optional: Check statusword for errors or state changes
        // if (somanet_inputs->statusword & 0x08) { // Example: Check for fault bit
        //     fprintf(stderr, "SOEM_Interface: Drive fault detected! Statusword: 0x%X\n", somanet_inputs->statusword);
        // }
    } else {
        // Handle missed working counter (communication error)
        // fprintf(stderr, "SOEM_Interface: Working counter miss! WKC: %d, Expected: %d\n", wkc, expectedWKC);
    }
}

/**
 * @brief Returns the last known position from the servo motor.
 * @return The current angular position.
 */
float soem_interface_get_current_position() {
    return current_position_f;
}

/**
 * @brief Returns the last known velocity from the servo motor.
 * @return The current angular velocity.
 */
float soem_interface_get_current_velocity() {
    return current_velocity_f;
}

/**
 * @brief Stops the SOEM master and cleans up resources.
 */
void soem_interface_stop_master() {
    if (master_initialized) {
        // Request PREOP state for all slaves before closing
        ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
        ec_close();
    }
    printf("SOEM_Interface: Stopped EtherCAT master.\n");
    master_initialized = 0;
}