// soem_interface.c - Improved version with CiA 402 state machine and proper initialization
#include "soem_interface.h" // This now contains the PDO struct definitions
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdarg.h>
#include <math.h>
#include <stdbool.h>

// --- SOEM Library Includes ---
#include "ethercat.h"
#include "ethercattype.h"

// Note: PACKED is now typically defined in your compiler or other headers.
// If you still get warnings/errors about PACKED, you might need to ensure it's defined
// globally or add it back with a proper guard, but usually it's handled.
// #ifndef PACKED
// #define PACKED __attribute__((__packed__))
// #endif

// --- CiA 402 State Machine Definitions ---
#define CIA402_STATUSWORD_RTSO          0x0001  // Ready to switch on
#define CIA402_STATUSWORD_SO            0x0002  // Switched on
#define CIA402_STATUSWORD_OE            0x0004  // Operation enabled
#define CIA402_STATUSWORD_FAULT         0x0008  // Fault
#define CIA402_STATUSWORD_VE            0x0010  // Voltage enabled
#define CIA402_STATUSWORD_QS            0x0020  // Quick stop
#define CIA402_STATUSWORD_SOD           0x0040  // Switch on disabled
#define CIA402_STATUSWORD_WARNING       0x0080  // Warning
#define CIA402_STATUSWORD_REMOTE        0x0200  // Remote
#define CIA402_STATUSWORD_TARGET        0x0400  // Target reached
#define CIA402_STATUSWORD_INTERNAL      0x0800  // Internal limit active
// --- Controlwords
#define CIA402_CONTROLWORD_SO           0x0001  // Switch on
#define CIA402_CONTROLWORD_EV           0x0002  // Enable voltage
#define CIA402_CONTROLWORD_QS           0x0004  // Quick stop
#define CIA402_CONTROLWORD_EO           0x0008  // Enable operation
#define CIA402_CONTROLWORD_FAULT_RESET  0x0080  // Fault reset

// --- SOEM Global Variables ---
char IOmap[4096];
ec_ODlistt ODlist;
ec_groupt DCgroup;
int wkc;
int expectedWKC;
ec_timet tmo;

// --- PDO Structures for Synapticon ACTILINK-S (Slave 1) ---
// These structures are now defined ONLY in soem_interface.h to avoid conflicts.
// We only need to declare the pointers here.
somanet_rx_pdo_enhanced_t *somanet_outputs;
somanet_tx_pdo_enhanced_t *somanet_inputs;

// Mutex for protecting PDO data access
static pthread_mutex_t pdo_mutex = PTHREAD_MUTEX_INITIALIZER;

// Thread for EtherCAT communication
static pthread_t ecat_thread;
static volatile int ecat_thread_running = 0;
static volatile int master_initialized = 0;
static volatile int communication_ok = 0;

// Global variables to hold current PDO values
static float target_torque_f = 0.0f;
static float current_position_f = 0.0f;
static float current_velocity_f = 0.0f;
static cia402_state_t current_cia402_state = CIA402_STATE_NOT_READY;
static uint16_t current_statusword = 0;
static uint16_t current_controlword = 0;

// --- CiA 402 State Machine Functions ---
cia402_state_t get_cia402_state(uint16_t statusword) {
    // Extract relevant bits for state determination
    uint16_t state_bits = statusword & 0x006F;
    
    switch (state_bits) {
        case 0x0000: return CIA402_STATE_NOT_READY;
        case 0x0040: return CIA402_STATE_SWITCH_ON_DISABLED;
        case 0x0021: return CIA402_STATE_READY_TO_SWITCH_ON;
        case 0x0023: return CIA402_STATE_SWITCHED_ON;
        case 0x0027: return CIA402_STATE_OPERATION_ENABLED;
        case 0x0007: return CIA402_STATE_QUICK_STOP_ACTIVE;
        case 0x000F: return CIA402_STATE_FAULT_REACTION_ACTIVE;
        case 0x0008: return CIA402_STATE_FAULT;
        default: 
            // Check for fault bit
            if (statusword & CIA402_STATUSWORD_FAULT) {
                return CIA402_STATE_FAULT;
            }
            return CIA402_STATE_NOT_READY;
    }
}

const char* get_cia402_state_name(cia402_state_t state) {
    switch (state) {
        case CIA402_STATE_NOT_READY: return "NOT_READY";
        case CIA402_STATE_SWITCH_ON_DISABLED: return "SWITCH_ON_DISABLED";
        case CIA402_STATE_READY_TO_SWITCH_ON: return "READY_TO_SWITCH_ON";
        case CIA402_STATE_SWITCHED_ON: return "SWITCHED_ON";
        case CIA402_STATE_OPERATION_ENABLED: return "OPERATION_ENABLED";
        case CIA402_STATE_QUICK_STOP_ACTIVE: return "QUICK_STOP_ACTIVE";
        case CIA402_STATE_FAULT_REACTION_ACTIVE: return "FAULT_REACTION_ACTIVE";
        case CIA402_STATE_FAULT: return "FAULT";
        default: return "UNKNOWN";
    }
}

uint16_t get_cia402_controlword_for_transition(cia402_state_t current_state, cia402_state_t target_state) {
    switch (current_state) {
        case CIA402_STATE_NOT_READY:
        case CIA402_STATE_SWITCH_ON_DISABLED:
            if (target_state == CIA402_STATE_READY_TO_SWITCH_ON) {
                return 0x0006; // Shutdown: Enable voltage + Quick stop
            }
            break;
            
        case CIA402_STATE_READY_TO_SWITCH_ON:
            if (target_state == CIA402_STATE_SWITCHED_ON) {
                return 0x0007; // Switch on: Enable voltage + Quick stop + Switch on
            }
            break;
            
        case CIA402_STATE_SWITCHED_ON:
            if (target_state == CIA402_STATE_OPERATION_ENABLED) {
                return 0x000F; // Enable operation: All control bits set
            }
            break;
            
        case CIA402_STATE_FAULT:
            return 0x0080; // Fault reset
            
        default:
            break;
    }
    
    // Default: maintain current state
    return 0x0006;
}

// Function to initialize CiA 402 parameters via SDO
int initialize_cia402_parameters(uint16_t slave_idx) {
    printf("SOEM_Interface: Initializing CiA 402 parameters for slave %u...\n", slave_idx);
    
    // Set modes of operation to torque mode (4)
    int8_t torque_mode = 4;
    if (soem_interface_write_sdo(slave_idx, 0x6060, 0x00, sizeof(torque_mode), &torque_mode) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set modes of operation to torque mode\n");
        return -1;
    }
    printf("SOEM_Interface: Set modes of operation to torque mode (4)\n");
    
    // Set motor rated current (example: 3000 mA = 3A)
    // Note: Adjust this value according to your motor specifications
    uint32_t motor_rated_current = 3000; // mA
    if (soem_interface_write_sdo(slave_idx, 0x6075, 0x00, sizeof(motor_rated_current), &motor_rated_current) != 0) {
        printf("SOEM_Interface: Warning: Failed to set motor rated current (may not be supported)\n");
    } else {
        printf("SOEM_Interface: Set motor rated current to %u mA\n", motor_rated_current);
    }
    
    // Set max torque (example: 1000 per mille = 100% of rated torque)
    uint16_t max_torque = 1000; // per mille
    if (soem_interface_write_sdo(slave_idx, 0x6072, 0x00, sizeof(max_torque), &max_torque) != 0) {
        printf("SOEM_Interface: Warning: Failed to set max torque\n");
    } else {
        printf("SOEM_Interface: Set max torque to %u per mille\n", max_torque);
    }
    
    // Set torque slope (acceleration/deceleration limit)
    // Example: 10000 per mille/s (adjust based on your application needs)
    uint32_t torque_slope = 10000;
    if (soem_interface_write_sdo(slave_idx, 0x6087, 0x00, sizeof(torque_slope), &torque_slope) != 0) {
        printf("SOEM_Interface: Warning: Failed to set torque slope\n");
    } else {
        printf("SOEM_Interface: Set torque slope to %u per mille/s\n", torque_slope);
    }
    
    // Set interpolation time period (optional, for smoother operation)
    // Example: 1000 microseconds = 1ms
    uint8_t interpolation_time_period = 1; // 1ms
    int8_t interpolation_time_index = -3;  // 10^-3 seconds
    if (soem_interface_write_sdo(slave_idx, 0x60C2, 0x01, sizeof(interpolation_time_period), &interpolation_time_period) != 0) {
        printf("SOEM_Interface: Warning: Failed to set interpolation time period\n");
    } else {
        printf("SOEM_Interface: Set interpolation time period to %u ms\n", interpolation_time_period);
    }
    
    if (soem_interface_write_sdo(slave_idx, 0x60C2, 0x02, sizeof(interpolation_time_index), &interpolation_time_index) != 0) {
        printf("SOEM_Interface: Warning: Failed to set interpolation time index\n");
    } else {
        printf("SOEM_Interface: Set interpolation time index to %d\n", interpolation_time_index);
    }
    
    // Set position encoder resolution (if available)
    // This is device-specific and may not be needed for all drives
    uint32_t encoder_increments = 4096; // Example: 4096 increments per revolution
    if (soem_interface_write_sdo(slave_idx, 0x608F, 0x01, sizeof(encoder_increments), &encoder_increments) != 0) {
        printf("SOEM_Interface: Info: Encoder increments setting not available (normal for some drives)\n");
    } else {
        printf("SOEM_Interface: Set encoder increments to %u per revolution\n", encoder_increments);
    }
    
    // Set gear ratio (if applicable)
    uint32_t gear_ratio_num = 1;   // Numerator
    uint32_t gear_ratio_den = 1;   // Denominator
    if (soem_interface_write_sdo(slave_idx, 0x608F, 0x02, sizeof(gear_ratio_num), &gear_ratio_num) != 0) {
        printf("SOEM_Interface: Info: Gear ratio setting not available (normal for direct drive)\n");
    } else {
        printf("SOEM_Interface: Set gear ratio to %u/%u\n", gear_ratio_num, gear_ratio_den);
    }
    
    // Wait for parameters to be processed
    usleep(100000); // 100ms delay
    
    // Verify modes of operation was set correctly
    int8_t current_mode = 0;
    if (soem_interface_read_sdo(slave_idx, 0x6061, 0x00, sizeof(current_mode), &current_mode) == 0) {
        printf("SOEM_Interface: Current modes of operation display: %d\n", current_mode);
        if (current_mode == torque_mode) {
            printf("SOEM_Interface: Mode verification successful\n");
        } else {
            printf("SOEM_Interface: Warning: Mode not yet active (expected %d, got %d)\n", torque_mode, current_mode);
        }
    } else {
        printf("SOEM_Interface: Warning: Could not verify modes of operation\n");
    }
    
    printf("SOEM_Interface: CiA 402 parameter initialization completed\n");
    return 0;
}

// --- Helper function to check if slave is in operational state ---
int is_slave_operational(int slave_idx) {
    uint16 actual_state = ec_slave[slave_idx].state & 0x0F;
    return actual_state == EC_STATE_OPERATIONAL;
}

// --- Helper function to get readable state name ---
const char* get_state_name(uint16 state) {
    uint16 actual_state = state & 0x0F;
    switch(actual_state) {
        case EC_STATE_INIT: return "INIT";
        case EC_STATE_PRE_OP: return "PRE_OP";
        case EC_STATE_BOOT: return "BOOT";
        case EC_STATE_SAFE_OP: return "SAFE_OP";
        case EC_STATE_OPERATIONAL: return "OPERATIONAL";
        default: return "UNKNOWN";
    }
}

// --- Helper functions for SDO operations ---
int soem_interface_write_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex, uint16_t data_size, void *data) {
    int wkc_sdo;
    wkc_sdo = ec_SDOwrite(slave_idx, index, subindex, FALSE, data_size, data, EC_TIMEOUTRXM);
    if (wkc_sdo == 0) {
        fprintf(stderr, "SOEM_Interface: SDO write failed for slave %u, index 0x%04X:%02X\n", slave_idx, index, subindex);
        return -1;
    }
    return 0;
}

int soem_interface_read_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex, uint16_t data_size, void *data) {
    int wkc_sdo;
    int actual_size = data_size;
    wkc_sdo = ec_SDOread(slave_idx, index, subindex, FALSE, &actual_size, data, EC_TIMEOUTRXM);
    if (wkc_sdo == 0) {
        fprintf(stderr, "SOEM_Interface: SDO read failed for slave %u, index 0x%04X:%02X\n", slave_idx, index, subindex);
        return -1;
    }
    return 0;
}

// --- Function to set EtherCAT slave state ---
int soem_interface_set_ethercat_state(uint16_t slave_idx, ec_state desired_state) {
    int max_retries = 5;
    int retry_count = 0;
    
    while (retry_count < max_retries) {
        printf("SOEM_Interface: Attempt %d/%d - Setting slave %u to state %s...\n", 
               retry_count + 1, max_retries, slave_idx, get_state_name(desired_state));
        
        // Clear any existing AL status codes
        ec_slave[slave_idx].ALstatuscode = 0;
        
        // Set the desired state
        ec_slave[slave_idx].state = desired_state;
        ec_writestate(slave_idx);
        
        // Wait longer for state transition
        usleep(200000); // 200ms delay
        
        // Check the state with extended timeout
        int wkc_state = ec_statecheck(slave_idx, desired_state, EC_TIMEOUTSTATE * 2);
        
        if (wkc_state > 0 && (ec_slave[slave_idx].state & 0x0F) == desired_state) {
            printf("SOEM_Interface: Slave %u successfully transitioned to state %s\n", 
                   slave_idx, get_state_name(ec_slave[slave_idx].state));
            return 0;
        }
        
        // Print detailed error information
        printf("SOEM_Interface: State transition failed - Current state: %s, ALstatuscode: 0x%04X\n",
               get_state_name(ec_slave[slave_idx].state), ec_slave[slave_idx].ALstatuscode);
        
        // If we're trying to go to OPERATIONAL and stuck in PRE_OP, try intermediate states
        if (desired_state == EC_STATE_OPERATIONAL && (ec_slave[slave_idx].state & 0x0F) == EC_STATE_PRE_OP) {
            printf("SOEM_Interface: Slave stuck in PRE_OP, trying intermediate SAFE_OP transition...\n");
            
            // Force to SAFE_OP first
            ec_slave[slave_idx].state = EC_STATE_SAFE_OP;
            ec_writestate(slave_idx);
            usleep(200000);
            
            if (ec_statecheck(slave_idx, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 2) > 0) {
                printf("SOEM_Interface: Intermediate SAFE_OP transition successful\n");
                // Now try OPERATIONAL again
                ec_slave[slave_idx].state = EC_STATE_OPERATIONAL;
                ec_writestate(slave_idx);
                usleep(200000);
                
                if (ec_statecheck(slave_idx, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 2) > 0) {
                    printf("SOEM_Interface: Final OPERATIONAL transition successful\n");
                    return 0;
                }
            }
        }
        
        retry_count++;
        if (retry_count < max_retries) {
            printf("SOEM_Interface: Retrying in 500ms...\n");
            usleep(500000); // 500ms delay before retry
        }
    }
    
    fprintf(stderr, "SOEM_Interface: Failed to set slave %u to state %s after %d attempts\n",
            slave_idx, get_state_name(desired_state), max_retries);
    return -1;
}

// --- Function to perform CiA 402 state machine transition ---
int perform_cia402_transition_to_operational(uint16_t slave_idx) {
    printf("SOEM_Interface: Starting CiA 402 state machine transition to operational...\n");
    
    int max_attempts = 50;
    int attempt = 0;
    
    while (attempt < max_attempts) {
        // Read current statusword
        if (somanet_inputs) {
            current_statusword = somanet_inputs->statusword;
        } else {
            fprintf(stderr, "SOEM_Interface: somanet_inputs not available for status reading.\n");
            return -1;
        }
        
        current_cia402_state = get_cia402_state(current_statusword);
        
        printf("SOEM_Interface: Attempt %d - Current CiA 402 state: %s (statusword: 0x%04X)\n", 
               attempt + 1, get_cia402_state_name(current_cia402_state), current_statusword);
        
        // Check if we're in operational state
        if (current_cia402_state == CIA402_STATE_OPERATION_ENABLED) {
            printf("SOEM_Interface: Successfully reached Operation Enabled state!\n");
            return 0;
        }
        
        // Handle fault state
        if (current_cia402_state == CIA402_STATE_FAULT) {
            printf("SOEM_Interface: Device in fault state. Attempting fault reset...\n");
            current_controlword = CIA402_CONTROLWORD_FAULT_RESET;
        } else {
            // Determine next transition
            switch (current_cia402_state) {
                case CIA402_STATE_NOT_READY:
                case CIA402_STATE_SWITCH_ON_DISABLED:
                    current_controlword = 0x0006; // Shutdown
                    break;
                case CIA402_STATE_READY_TO_SWITCH_ON:
                    current_controlword = 0x0007; // Switch on
                    break;
                case CIA402_STATE_SWITCHED_ON:
                    current_controlword = 0x000F; // Enable operation
                    break;
                case CIA402_STATE_QUICK_STOP_ACTIVE:
                    current_controlword = 0x0006; // Shutdown
                    break;
                default:
                    current_controlword = 0x0006; // Default to shutdown
                    break;
            }
        }
        
        // Apply controlword
        if (somanet_outputs) {
            somanet_outputs->controlword = current_controlword;
        }
        
        // Send PDO data
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        
        printf("SOEM_Interface: Applied controlword: 0x%04X\n", current_controlword);
        
        attempt++;
        usleep(100000); // 100ms delay between attempts
    }
    
    fprintf(stderr, "SOEM_Interface: Failed to reach operational state after %d attempts.\n", max_attempts);
    return -1;
}

// --- SOEM Thread Function ---
void *ecat_loop(void *ptr) {
    int slave_idx = 1;
    int state_machine_initialized = 0;

    printf("SOEM_Interface: EtherCAT thread started.\n");

    while (!master_initialized && ecat_thread_running) {
        usleep(10000);
    }

    if (!master_initialized) {
        printf("SOEM_Interface: Master not initialized, exiting thread.\n");
        return NULL;
    }

    printf("SOEM_Interface: Entering EtherCAT cyclic loop.\n");

    while (ecat_thread_running) {
        // Update output PDO data
        pthread_mutex_lock(&pdo_mutex);
        if (somanet_outputs) {
            // Only update torque if we're in operational state
            if (current_cia402_state == CIA402_STATE_OPERATION_ENABLED) {
                somanet_outputs->target_torque = (int16_t)(target_torque_f * 1000.0f);
            } else {
                somanet_outputs->target_torque = 0; // Safe value
            }
            
            // Keep controlword updated for state machine
            somanet_outputs->controlword = current_controlword;
            somanet_outputs->modes_of_operation = 4; // Torque mode
        }
        pthread_mutex_unlock(&pdo_mutex);

        // Exchange process data
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc >= expectedWKC) {
            communication_ok = 1;
            
            // Update input PDO data
            pthread_mutex_lock(&pdo_mutex);
            if (somanet_inputs) {
                current_statusword = somanet_inputs->statusword;
                current_cia402_state = get_cia402_state(current_statusword);
                current_position_f = (float)somanet_inputs->position_actual_value;
                current_velocity_f = (float)somanet_inputs->velocity_actual_value;
            }
            pthread_mutex_unlock(&pdo_mutex);
            
            // Initialize state machine once
            if (!state_machine_initialized && somanet_inputs && somanet_outputs) {
                if (perform_cia402_transition_to_operational(slave_idx) == 0) {
                    state_machine_initialized = 1;
                    printf("SOEM_Interface: CiA 402 state machine initialized successfully.\n");
                } else {
                    fprintf(stderr, "SOEM_Interface: Failed to initialize CiA 402 state machine.\n");
                }
            }
        } else {
            communication_ok = 0;
        }

        // Check EtherCAT slave state
        if (!is_slave_operational(slave_idx)) {
            ec_statecheck(slave_idx, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
        }

        usleep(1000); // 1ms cycle time
    }

    printf("SOEM_Interface: EtherCAT thread stopping.\n");
    return NULL;
}

// Function to check if PDO mapping is needed by comparing current mapping
int check_pdo_mapping_needed(uint16_t slave_idx) {
    uint8_t num_mapped_objects = 0;
    uint32_t mapped_object = 0;
    
    // Check RxPDO mapping (0x1600)
    if (soem_interface_read_sdo(slave_idx, 0x1600, 0x00, sizeof(num_mapped_objects), &num_mapped_objects) == 0) {
        printf("SOEM_Interface: Current RxPDO mapping has %d objects\n", num_mapped_objects);
        
        // Read first few mapped objects to see if they match our expected mapping
        for (int i = 1; i <= num_mapped_objects && i <= 3; i++) {
            if (soem_interface_read_sdo(slave_idx, 0x1600, i, sizeof(mapped_object), &mapped_object) == 0) {
                printf("SOEM_Interface: RxPDO[%d] = 0x%08X\n", i, mapped_object);
            }
        }
    }
    
    // Check TxPDO mapping (0x1A00)
    if (soem_interface_read_sdo(slave_idx, 0x1A00, 0x00, sizeof(num_mapped_objects), &num_mapped_objects) == 0) {
        printf("SOEM_Interface: Current TxPDO mapping has %d objects\n", num_mapped_objects);
        
        // Read first few mapped objects
        for (int i = 1; i <= num_mapped_objects && i <= 3; i++) {
            if (soem_interface_read_sdo(slave_idx, 0x1A00, i, sizeof(mapped_object), &mapped_object) == 0) {
                printf("SOEM_Interface: TxPDO[%d] = 0x%08X\n", i, mapped_object);
            }
        }
    }
    
    return 1; // For now, always configure - you can add logic to check if current mapping matches expected
}

// Enhanced PDO mapping configuration with proper error handling and validation
int soem_interface_configure_pdo_mapping_enhanced(uint16_t slave_idx, uint16_t pdo_assign_idx, 
                                                  uint16_t pdo_map_idx, uint32_t *mapped_objects, 
                                                  uint8_t num_mapped_objects) {
    uint8_t zero_val = 0;
    uint8_t current_num_objects = 0;
    uint32_t current_object = 0;
    
    printf("SOEM_Interface: Configuring PDO mapping for slave %u (PDO 0x%04X)...\n", slave_idx, pdo_map_idx);

    // Ensure we're in Pre-operational state as required by Synapticon
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_PRE_OP) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set slave to Pre-Op for PDO configuration.\n");
        return -1;
    }
    usleep(100000); // 100ms delay for state stabilization

    // Step 1: Read current mapping to see if reconfiguration is needed
    printf("SOEM_Interface: Reading current PDO mapping...\n");
    if (soem_interface_read_sdo(slave_idx, pdo_map_idx, 0x00, sizeof(current_num_objects), &current_num_objects) == 0) {
        printf("SOEM_Interface: Current mapping has %d objects\n", current_num_objects);
        
        // Check if current mapping matches desired mapping
        bool mapping_matches = (current_num_objects == num_mapped_objects);
        if (mapping_matches) {
            for (uint8_t i = 0; i < current_num_objects; i++) {
                if (soem_interface_read_sdo(slave_idx, pdo_map_idx, i + 1, sizeof(current_object), &current_object) == 0) {
                    if (current_object != mapped_objects[i]) {
                        mapping_matches = false;
                        break;
                    }
                } else {
                    mapping_matches = false;
                    break;
                }
            }
        }
        
        if (mapping_matches) {
            printf("SOEM_Interface: Current PDO mapping already matches desired configuration. Skipping write.\n");
            return 0; // No reconfiguration needed
        }
    }

    // Step 2: Disable PDO assignment first (as per Synapticon documentation)
    // Set subindex 0 of PDO assign to 0 to disable assignment
    printf("SOEM_Interface: Disabling PDO assignment (0x%04X:0x00 = 0)...\n", pdo_assign_idx);
    if (soem_interface_write_sdo(slave_idx, pdo_assign_idx, 0x00, sizeof(zero_val), &zero_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to disable PDO assignment.\n");
        return -1;
    }
    usleep(50000); // 50ms delay

    // Step 3: Disable the PDO mapping object (set subindex 0 to 0)
    printf("SOEM_Interface: Disabling PDO mapping object (0x%04X:0x00 = 0)...\n", pdo_map_idx);
    if (soem_interface_write_sdo(slave_idx, pdo_map_idx, 0x00, sizeof(zero_val), &zero_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to disable PDO mapping.\n");
        return -1;
    }
    usleep(50000); // 50ms delay

    // Step 4: Write new mapping objects (subindices 1 to n)
    printf("SOEM_Interface: Writing %d new mapping objects to 0x%04X...\n", num_mapped_objects, pdo_map_idx);
    for (uint8_t i = 0; i < num_mapped_objects; i++) {
        printf("SOEM_Interface: Writing object %d: 0x%08X\n", i + 1, mapped_objects[i]);
        if (soem_interface_write_sdo(slave_idx, pdo_map_idx, i + 1, sizeof(uint32_t), &mapped_objects[i]) != 0) {
            fprintf(stderr, "SOEM_Interface: SDO write failed for slave %u, index 0x%04X:%02X\n", slave_idx, pdo_map_idx, i + 1);
            fprintf(stderr, "SOEM_Interface: Failed to write mapping object %d.\n", i + 1);
            return -1;
        }
        usleep(20000); // 20ms delay between writes
    }

    // Step 5: Enable the PDO mapping by setting subindex 0 to number of objects
    printf("SOEM_Interface: Enabling PDO mapping (0x%04X:0x00 = %d objects)...\n", pdo_map_idx, num_mapped_objects);
    if (soem_interface_write_sdo(slave_idx, pdo_map_idx, 0x00, sizeof(num_mapped_objects), &num_mapped_objects) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to enable PDO mapping.\n");
        return -1;
    }
    usleep(50000); // 50ms delay

    // Step 6: Re-enable PDO assignment
    // This is the corrected logic:
    // First, assign the PDO map index to subindex 1 of the PDO assign object.
    // Then, set subindex 0 of the PDO assign object to the number of assigned PDOs (which is 1 in this case).
    uint16_t pdo_map_idx_u16 = pdo_map_idx; // Cast to uint16_t for the SDO write
    uint8_t num_assigned_pdo = 1; // We are assigning one PDO (0x1600 or 0x1A00)

    printf("SOEM_Interface: Assigning PDO 0x%04X to 0x%04X:0x01...\n", pdo_map_idx_u16, pdo_assign_idx);
    if (soem_interface_write_sdo(slave_idx, pdo_assign_idx, 0x01, sizeof(pdo_map_idx_u16), &pdo_map_idx_u16) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to assign PDO 0x%04X to 0x%04X:0x01.\n", pdo_map_idx_u16, pdo_assign_idx);
        return -1;
    }
    usleep(50000); // 50ms delay

    printf("SOEM_Interface: Setting number of assigned PDOs for 0x%04X:0x00 to %d...\n", pdo_assign_idx, num_assigned_pdo);
    if (soem_interface_write_sdo(slave_idx, pdo_assign_idx, 0x00, sizeof(num_assigned_pdo), &num_assigned_pdo) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set number of assigned PDOs for 0x%04X:0x00.\n", pdo_assign_idx);
        return -1;
    }
    usleep(50000); // 50ms delay


    // Step 7: Verify the configuration
    printf("SOEM_Interface: Verifying PDO configuration...\n");
    uint8_t verified_num_objects = 0;
    if (soem_interface_read_sdo(slave_idx, pdo_map_idx, 0x00, sizeof(verified_num_objects), &verified_num_objects) == 0) {
        if (verified_num_objects == num_mapped_objects) {
            printf("SOEM_Interface: PDO mapping verification successful (%d objects).\n", verified_num_objects);
        } else {
            fprintf(stderr, "SOEM_Interface: PDO mapping verification failed. Expected %d, got %d objects.\n", 
                    num_mapped_objects, verified_num_objects);
            return -1;
        }
    } else {
        fprintf(stderr, "SOEM_Interface: Failed to verify PDO mapping.\n");
        return -1;
    }

    return 0;
}

// Function to validate PDO configuration after mapping
int validate_pdo_configuration(uint16_t slave_idx) {
    printf("SOEM_Interface: Validating PDO configuration...\n");
    
    // Validate RxPDO configuration
    uint8_t num_rx_objects = 0;
    if (soem_interface_read_sdo(slave_idx, 0x1600, 0x00, sizeof(num_rx_objects), &num_rx_objects) == 0) {
        printf("SOEM_Interface: RxPDO has %d mapped objects:\n", num_rx_objects);
        for (int i = 1; i <= num_rx_objects; i++) {
            uint32_t mapped_object = 0;
            if (soem_interface_read_sdo(slave_idx, 0x1600, i, sizeof(mapped_object), &mapped_object) == 0) {
                uint16_t index = (mapped_object >> 16) & 0xFFFF;
                uint8_t subindex = (mapped_object >> 8) & 0xFF;
                uint8_t bit_length = mapped_object & 0xFF;
                printf("SOEM_Interface: RxPDO[%d]: 0x%04X:%02X (%d bits)\n", 
                       i, index, subindex, bit_length);
            }
        }
    }
    
    // Validate TxPDO configuration
    uint8_t num_tx_objects = 0;
    if (soem_interface_read_sdo(slave_idx, 0x1A00, 0x00, sizeof(num_tx_objects), &num_tx_objects) == 0) {
        printf("SOEM_Interface: TxPDO has %d mapped objects:\n", num_tx_objects);
        for (int i = 1; i <= num_tx_objects; i++) {
            uint32_t mapped_object = 0;
            if (soem_interface_read_sdo(slave_idx, 0x1A00, i, sizeof(mapped_object), &mapped_object) == 0) {
                uint16_t index = (mapped_object >> 16) & 0xFFFF;
                uint8_t subindex = (mapped_object >> 8) & 0xFF;
                uint8_t bit_length = mapped_object & 0xFF;
                printf("SOEM_Interface: TxPDO[%d]: 0x%04X:%02X (%d bits)\n", 
                       i, index, subindex, bit_length);
            }
        }
    }
    
    // Validate PDO assignments
    uint8_t num_assigned_rx = 0;
    if (soem_interface_read_sdo(slave_idx, 0x1C12, 0x00, sizeof(num_assigned_rx), &num_assigned_rx) == 0) {
        printf("SOEM_Interface: RxPDO assignment has %d entries\n", num_assigned_rx);
    }
    
    uint8_t num_assigned_tx = 0;
    if (soem_interface_read_sdo(slave_idx, 0x1C13, 0x00, sizeof(num_assigned_tx), &num_assigned_tx) == 0) {
        printf("SOEM_Interface: TxPDO assignment has %d entries\n", num_assigned_tx);
    }
    
    return 0;
}

// Enhanced SOMANET PDO configuration with multiple mapping support
int configure_somanet_pdo_mapping_enhanced(uint16_t slave_idx) {
     printf("SOEM_Interface: Starting robust PDO mapping configuration for slave %u...\n", slave_idx);
    
    // First, ensure we're in INIT state for clean configuration
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_INIT) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set slave to INIT state\n");
        return -1;
    }
    
    // Move to PRE_OP for PDO configuration
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_PRE_OP) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set slave to PRE_OP state\n");
        return -1;
    }
    
    // Check if device supports PDO mapping at all
    uint8_t test_val = 0;
    if (soem_interface_read_sdo(slave_idx, 0x1C12, 0x00, sizeof(test_val), &test_val) != 0) {
        printf("SOEM_Interface: Device may not support PDO assignment modification, using default mapping\n");
        return 0; // Continue with default mapping
    }
    
    // Define the full RxPDO mapping based on PDO_mapping.md (SM2 outputs)
    // REDUCED FOR DEBUGGING: Only essential objects for basic control
    uint32_t rxpdo_mapping[] = {
        0x60400010,  // Controlword (16 bits)
        0x60600008,  // Modes of operation (8 bits)
        0x60710010   // Target Torque (16 bits)
        // Removed: Target position, Target velocity, Torque offset, Tuning command, Physical outputs, Bit mask, User MOSI, Velocity offset
    };
    
    // Define the full TxPDO mapping based on PDO_mapping.md (SM3 inputs)
    uint32_t txpdo_mapping[] = {
        0x60410010,  // Statusword (16 bits)
        0x60610008,  // Modes of operation display (8 bits)
        0x60640020,  // Position actual value (32 bits)
        0x606C0020,  // Velocity actual value (32 bits)
        0x60770010   // Torque actual value (16 bits)
        // Removed: Analog inputs, Tuning status, Digital inputs, User MISO, Timestamp, Position demand internal value, Velocity demand value, Torque demand
    };
    
    // Calculate and verify sizes (for logging purposes)
    uint16_t rxpdo_size_bits = 0;
    for (int i = 0; i < sizeof(rxpdo_mapping)/sizeof(uint32_t); i++) {
        rxpdo_size_bits += (rxpdo_mapping[i] & 0xFF);
    }
    uint16_t txpdo_size_bits = 0;
    for (int i = 0; i < sizeof(txpdo_mapping)/sizeof(uint32_t); i++) {
        txpdo_size_bits += (txpdo_mapping[i] & 0xFF);
    }
    
    printf("SOEM_Interface: Full PDO mapping - RxPDO: %d bits (%d bytes), TxPDO: %d bits (%d bytes)\n", 
           rxpdo_size_bits, rxpdo_size_bits/8, txpdo_size_bits, txpdo_size_bits/8);
    
    // Try to configure with full mapping
    if (soem_interface_configure_pdo_mapping_enhanced(slave_idx, 0x1C12, 0x1600, 
                                                     rxpdo_mapping, sizeof(rxpdo_mapping)/sizeof(uint32_t)) != 0) {
        fprintf(stderr, "SOEM_Interface: RxPDO mapping failed, cannot proceed.\n");
        return -1; // Critical error, cannot continue without correct RxPDO
    }
    
    if (soem_interface_configure_pdo_mapping_enhanced(slave_idx, 0x1C13, 0x1A00, 
                                                     txpdo_mapping, sizeof(txpdo_mapping)/sizeof(uint32_t)) != 0) {
        fprintf(stderr, "SOEM_Interface: TxPDO mapping failed, cannot proceed.\n");
        return -1; // Critical error, cannot continue without correct TxPDO
    }
    
    printf("SOEM_Interface: PDO mapping configuration completed\n");
    return 0;
}

// Enhanced initialization with better error recovery
int soem_interface_init_enhanced(const char *ifname) {
    int i;
    int slave_idx = 1;

    printf("SOEM_Interface: Enhanced initialization starting on %s...\n", ifname);

    if (!ec_init(ifname)) {
        fprintf(stderr, "SOEM_Interface: ec_init failed on %s\n", ifname);
        return -1;
    }

    printf("SOEM_Interface: ec_init succeeded\n");

    if (ec_config_init(FALSE) <= 0) {
        fprintf(stderr, "SOEM_Interface: No slaves found during config_init\n");
        return -1;
    }

    printf("SOEM_Interface: Found %d slaves\n", ec_slavecount);

    if (ec_slavecount == 0) {
        fprintf(stderr, "SOEM_Interface: No EtherCAT slaves found!\n");
        return -1;
    }

    // Print detailed slave information
    for (i = 1; i <= ec_slavecount; i++) {
        printf("SOEM_Interface: Slave %d: %s\n", i, ec_slave[i].name);
        printf("  - Vendor ID: 0x%08X, Product Code: 0x%08X\n", 
               ec_slave[i].eep_id, ec_slave[i].eep_pdi);
        printf("  - Output: %d bits (%d bytes), Input: %d bits (%d bytes)\n",
               ec_slave[i].Obits, ec_slave[i].Obits/8, ec_slave[i].Ibits, ec_slave[i].Ibits/8);
        printf("  - State: %s, ALstatuscode: 0x%04X\n", 
               get_state_name(ec_slave[i].state), ec_slave[i].ALstatuscode);
    }

    // Configure PDO mapping with improved robustness
    if (configure_somanet_pdo_mapping_enhanced(slave_idx) != 0) {
        fprintf(stderr, "SOEM_Interface: Critical: PDO mapping configuration failed, cannot proceed.\n");
        return -1;
    }

    // Configure distributed clocks
    ec_configdc();

    // Map the IO
    printf("SOEM_Interface: Mapping IO...\n");
    if (ec_config_map(&IOmap) == 0) {
        fprintf(stderr, "SOEM_Interface: ec_config_map failed\n");
        return -1;
    }

    // Print actual mapped sizes
    printf("SOEM_Interface: IO mapping completed\n");
    for (i = 1; i <= ec_slavecount; i++) {
        printf("SOEM_Interface: Slave %d mapped - Output: %d bytes, Input: %d bytes\n",
               i, ec_slave[i].Obits/8, ec_slave[i].Ibits/8);
    }

    // Assign PDO pointers with size validation
    if (ec_slave[slave_idx].outputs > 0) {
        int output_size = ec_slave[slave_idx].Obits / 8;
        // Adjust this check to match the *reduced* RxPDO size for debugging
        if (output_size >= (sizeof(uint16_t) + sizeof(int8_t) + sizeof(int16_t))) { // Controlword, Modes of operation, Target Torque
            somanet_outputs = (somanet_rx_pdo_enhanced_t *)(ec_slave[slave_idx].outputs);
            printf("SOEM_Interface: somanet_outputs mapped successfully (%d bytes available)\n", output_size);
        } else {
            fprintf(stderr, "SOEM_Interface: Output PDO size mismatch: need at least %zu bytes, have %d bytes\n",
                    (sizeof(uint16_t) + sizeof(int8_t) + sizeof(int16_t)), output_size);
            return -1;
        }
    } else {
        fprintf(stderr, "SOEM_Interface: No output PDO data available!\n");
        return -1;
    }
    
    if (ec_slave[slave_idx].inputs > 0) {
        int input_size = ec_slave[slave_idx].Ibits / 8;
        if (input_size >= sizeof(somanet_tx_pdo_enhanced_t)) {
            somanet_inputs = (somanet_tx_pdo_enhanced_t *)(ec_slave[slave_idx].inputs);
            printf("SOEM_Interface: somanet_inputs mapped successfully (%d bytes available)\n", input_size);
        } else {
            fprintf(stderr, "SOEM_Interface: Input PDO size mismatch: need %zu bytes, have %d bytes\n",
                    sizeof(somanet_tx_pdo_enhanced_t), input_size);
            return -1;
        }
    } else {
        fprintf(stderr, "SOEM_Interface: No input PDO data available!\n");
        return -1;
    }

    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("SOEM_Interface: Expected WKC: %d\n", expectedWKC);

    // Initialize CiA 402 parameters in PRE_OP
    if (initialize_cia402_parameters(slave_idx) != 0) {
        printf("SOEM_Interface: CiA 402 initialization had issues, continuing anyway\n");
    }
    
    // Initialize safe values
    if (somanet_outputs) {
        memset(somanet_outputs, 0, sizeof(somanet_rx_pdo_enhanced_t));
        somanet_outputs->controlword = 0x0006; // Shutdown
        somanet_outputs->modes_of_operation = 4; // Torque mode
    }

    // Use enhanced state transition function
    printf("SOEM_Interface: Transitioning to Safe-Operational...\n");
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_SAFE_OP) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to reach Safe-Operational state\n");
        return -1;
    }

    // Send initial PDO data
    ec_send_processdata();
    usleep(50000); // 50ms delay

    // Transition to Operational with enhanced function
    printf("SOEM_Interface: Transitioning to Operational...\n");
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_OPERATIONAL) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to reach Operational state\n");
        return -1;
    }

    // Verify all slaves are operational
    for (i = 1; i <= ec_slavecount; i++) {
        if (!is_slave_operational(i)) {
            fprintf(stderr, "SOEM_Interface: Slave %d not operational after transition\n", i);
            return -1;
        }
    }

    printf("SOEM_Interface: All slaves operational, starting communication thread...\n");
    master_initialized = 1;
    ecat_thread_running = 1;
    
    if (pthread_create(&ecat_thread, NULL, ecat_loop, NULL) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to create EtherCAT thread\n");
        return -1;
    }
    
    return 0;
}

// --- External Interface Functions ---
void soem_interface_send_and_receive_pdo(float target_torque) {
    if (!master_initialized) return;
    
    pthread_mutex_lock(&pdo_mutex);
    target_torque_f = target_torque;
    pthread_mutex_unlock(&pdo_mutex);
}

float soem_interface_get_current_position() {
    float position;
    pthread_mutex_lock(&pdo_mutex);
    position = current_position_f;
    pthread_mutex_unlock(&pdo_mutex);
    return position;
}

float soem_interface_get_current_velocity() {
    float velocity;
    pthread_mutex_lock(&pdo_mutex);
    velocity = current_velocity_f;
    pthread_mutex_unlock(&pdo_mutex);
    return velocity;
}

int soem_interface_get_communication_status() {
    return communication_ok;
}

cia402_state_t soem_interface_get_cia402_state() {
    return current_cia402_state;
}

uint16_t soem_interface_get_statusword() {
    return current_statusword;
}

void soem_interface_stop_master() {
    if (master_initialized) {
        printf("SOEM_Interface: Stopping EtherCAT master...\n");

        ecat_thread_running = 0;
        if (ecat_thread) {
            pthread_join(ecat_thread, NULL);
        }

        // Safe shutdown: disable operation
        if (somanet_outputs) {
            somanet_outputs->target_torque = 0;
            somanet_outputs->controlword = 0x0006; // Shutdown
        }
        ec_send_processdata();
        
        usleep(100000); // Wait 100ms

        // Transition to Safe-Op then Init
        soem_interface_set_ethercat_state(0, EC_STATE_SAFE_OP);
        soem_interface_set_ethercat_state(0, EC_STATE_INIT);

        ec_close();
        master_initialized = 0;
        printf("SOEM_Interface: EtherCAT master stopped.\n");
    }
}
