// soem_interface.c - Improved version with CiA 402 state machine and proper initialization
#include "soem_interface.h"
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <stdarg.h>
#include <math.h>

// --- SOEM Library Includes ---
#include "ethercat.h"
#include "ethercattype.h"

#ifndef PACKED
#define PACKED __attribute__((__packed__))
#endif

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
    uint32 bit_mask;            // Padding for alignment
} somanet_rx_pdo_t;

typedef struct PACKED
{
    uint16 statusword;                  // 0x6041:0x00
    int8   modes_of_operation_display;  // 0x6061:0x00
    int32  position_actual_value;       // 0x6064:0x00
    int32  velocity_actual_value;       // 0x6069:0x00
    int16  torque_actual_value;         // 0x6077:0x00
    int16  current_actual_value;        // 0x6078:0x00
    uint32 physical_inputs;             // 0x60FD:0x01
    uint32 tuning_status;               // 0x2702:0x00
} somanet_tx_pdo_t;

// Pointers to the PDO data in the IOmap
somanet_rx_pdo_t *somanet_outputs = NULL; // Initialize to NULL, remove if it causes errors
somanet_tx_pdo_t *somanet_inputs = NULL; // Initialize to NULL, remove if it causes errors

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
    int wkc_state;
    printf("SOEM_Interface: Attempting to set slave %u to state %s (%d)...\n", 
           slave_idx, get_state_name(desired_state), desired_state);
    
    ec_slave[slave_idx].state = desired_state;
    ec_writestate(slave_idx);
    
    wkc_state = ec_statecheck(slave_idx, desired_state, EC_TIMEOUTSTATE);
    
    if (wkc_state == 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set slave %u to state %s (%d). Current state: %s (%d)\n",
                slave_idx, get_state_name(desired_state), desired_state, 
                get_state_name(ec_slave[slave_idx].state), ec_slave[slave_idx].state);
        return -1;
    }
    
    printf("SOEM_Interface: Slave %u successfully transitioned to state %s (%d).\n", 
           slave_idx, get_state_name(ec_slave[slave_idx].state), ec_slave[slave_idx].state);
    return 0;
}

// --- Function to initialize CiA 402 motor parameters ---
int initialize_cia402_parameters(uint16_t slave_idx) {
    printf("SOEM_Interface: Initializing CiA 402 parameters for slave %u...\n", slave_idx);
    
    // Set modes of operation to torque mode (4)
    int8_t torque_mode = 4;
    if (soem_interface_write_sdo(slave_idx, 0x6060, 0x00, sizeof(torque_mode), &torque_mode) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set modes of operation to torque mode.\n");
        return -1;
    }
    
    // Set reasonable torque limits (adjust based on your motor specifications)
    uint16_t max_torque = 13000; // 13 Nm, adjust as needed
    if (soem_interface_write_sdo(slave_idx, 0x6072, 0x00, sizeof(max_torque), &max_torque) != 0) {
        printf("SOEM_Interface: Warning: Could not set max torque limit.\n");
    }
    
    // Read and display some basic motor parameters
    uint32_t motor_rated_current = 0;
    if (soem_interface_read_sdo(slave_idx, 0x6075, 0x00, sizeof(motor_rated_current), &motor_rated_current) == 0) {
        printf("SOEM_Interface: Motor rated current: %u mA\n", motor_rated_current);
    }
    
    // Try to set option codes, but don't fail if they're not supported
    int16_t quick_stop_option = 5;
    if (soem_interface_write_sdo(slave_idx, 0x605A, 0x00, sizeof(quick_stop_option), &quick_stop_option) != 0) {
        printf("SOEM_Interface: Note: Quick stop option code not supported (this is normal for some devices).\n");
    }
    
    int16_t shutdown_option = 0;
    if (soem_interface_write_sdo(slave_idx, 0x605B, 0x00, sizeof(shutdown_option), &shutdown_option) != 0) {
        printf("SOEM_Interface: Note: Shutdown option code not supported (this is normal for some devices).\n");
    }
    
    int16_t disable_operation_option = 1;
    if (soem_interface_write_sdo(slave_idx, 0x605C, 0x00, sizeof(disable_operation_option), &disable_operation_option) != 0) {
        printf("SOEM_Interface: Note: Disable operation option code not supported (this is normal for some devices).\n");
    }
    
    printf("SOEM_Interface: CiA 402 parameters initialization completed.\n");
    return 0;
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

// --- PDO Configuration Functions ---
int soem_interface_configure_pdo_mapping(uint16_t slave_idx, uint16_t pdo_assign_idx, uint16_t pdo_map_idx, uint32_t *mapped_objects, uint8_t num_mapped_objects) {
    uint8_t zero_val = 0;
    uint8_t original_assign_val = 1;
    
    printf("SOEM_Interface: Configuring PDO mapping for slave %u...\n", slave_idx);

    // Set to Pre-operational
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_PRE_OP) != 0) {
        return -1;
    }
    usleep(50000);

    // Disable PDO
    if (soem_interface_write_sdo(slave_idx, pdo_assign_idx, 0x00, sizeof(zero_val), &zero_val) != 0) {
        return -1;
    }
    usleep(20000);

    // Clear mapping
    if (soem_interface_write_sdo(slave_idx, pdo_map_idx, 0x00, sizeof(zero_val), &zero_val) != 0) {
        return -1;
    }
    usleep(20000);

    // Write new mapping
    for (uint8_t i = 0; i < num_mapped_objects; i++) {
        if (soem_interface_write_sdo(slave_idx, pdo_map_idx, i + 1, sizeof(uint32_t), &mapped_objects[i]) != 0) {
            return -1;
        }
        usleep(5000);
    }

    // Set number of mapped objects
    if (soem_interface_write_sdo(slave_idx, pdo_map_idx, 0x00, sizeof(num_mapped_objects), &num_mapped_objects) != 0) {
        return -1;
    }
    usleep(20000);

    // Re-enable PDO
    if (soem_interface_write_sdo(slave_idx, pdo_assign_idx, 0x00, sizeof(original_assign_val), &original_assign_val) != 0) {
        return -1;
    }
    usleep(20000);

    return 0;
}

int configure_somanet_pdo_mapping(uint16_t slave_idx) {
    printf("SOEM_Interface: Configuring SOMANET PDO mapping for slave %u...\n", slave_idx);
    
    // Ensure we're in Pre-operational state
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_PRE_OP) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set slave to Pre-Op for PDO configuration.\n");
        return -1;
    }
    usleep(100000); // 100ms delay
    
    // Verify we're in Pre-Op
    if ((ec_slave[slave_idx].state & 0x0F) != EC_STATE_PRE_OP) {
        fprintf(stderr, "SOEM_Interface: Slave %d not in Pre-Op state for PDO configuration. Current state: %s\n", 
                slave_idx, get_state_name(ec_slave[slave_idx].state));
        return -1;
    }

    // Read current PDO assignment to understand the device configuration
    uint8_t num_assigned_pdos = 0;
    uint16_t assigned_pdo = 0;
    
    // Check RxPDO assignment (0x1C12)
    printf("SOEM_Interface: Reading current RxPDO assignment...\n");
    if (soem_interface_read_sdo(slave_idx, 0x1C12, 0x00, sizeof(num_assigned_pdos), &num_assigned_pdos) == 0) {
        printf("SOEM_Interface: RxPDO assignment has %d entries\n", num_assigned_pdos);
        for (int i = 1; i <= num_assigned_pdos; i++) {
            if (soem_interface_read_sdo(slave_idx, 0x1C12, i, sizeof(assigned_pdo), &assigned_pdo) == 0) {
                printf("SOEM_Interface: RxPDO assignment[%d] = 0x%04X\n", i, assigned_pdo);
            }
        }
    } else {
        printf("SOEM_Interface: Could not read RxPDO assignment - will configure manually\n");
    }
    
    // Check TxPDO assignment (0x1C13)
    printf("SOEM_Interface: Reading current TxPDO assignment...\n");
    if (soem_interface_read_sdo(slave_idx, 0x1C13, 0x00, sizeof(num_assigned_pdos), &num_assigned_pdos) == 0) {
        printf("SOEM_Interface: TxPDO assignment has %d entries\n", num_assigned_pdos);
        for (int i = 1; i <= num_assigned_pdos; i++) {
            if (soem_interface_read_sdo(slave_idx, 0x1C13, i, sizeof(assigned_pdo), &assigned_pdo) == 0) {
                printf("SOEM_Interface: TxPDO assignment[%d] = 0x%04X\n", i, assigned_pdo);
            }
        }
    } else {
        printf("SOEM_Interface: Could not read TxPDO assignment - will configure manually\n");
    }
    
    // Since OutputSize=0, we need to actually configure the PDO mapping
    printf("SOEM_Interface: Configuring PDO mapping manually...\n");
    
    // Configure RxPDO (Master to Slave) - Use essential objects only
    uint32_t rxpdo_mapping[] = {
        0x60400010,  // 0x6040:0x00 Controlword (16-bit)
        0x60600008,  // 0x6060:0x00 Modes of operation (8-bit)
        0x60710010,  // 0x6071:0x00 Target torque (16-bit)
        0x607A0020   // 0x607A:0x00 Target position (32-bit)
    };
    
    // Configure TxPDO (Slave to Master) - Use essential objects only  
    uint32_t txpdo_mapping[] = {
        0x60410010,  // 0x6041:0x00 Statusword (16-bit)
        0x60610008,  // 0x6061:0x00 Modes of operation display (8-bit)
        0x60640020,  // 0x6064:0x00 Position actual value (32-bit)
        0x606C0020   // 0x606C:0x00 Velocity actual value (32-bit)
    };
    
    // Configure RxPDO mapping (0x1600)
    printf("SOEM_Interface: Configuring RxPDO mapping...\n");
    if (soem_interface_configure_pdo_mapping(slave_idx, 0x1C12, 0x1600, 
                                           rxpdo_mapping, sizeof(rxpdo_mapping)/sizeof(uint32_t)) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to configure RxPDO mapping.\n");
        return -1;
    }
    
    // Configure TxPDO mapping (0x1A00)
    printf("SOEM_Interface: Configuring TxPDO mapping...\n");
    if (soem_interface_configure_pdo_mapping(slave_idx, 0x1C13, 0x1A00, 
                                           txpdo_mapping, sizeof(txpdo_mapping)/sizeof(uint32_t)) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to configure TxPDO mapping.\n");
        return -1;
    }
    
    printf("SOEM_Interface: PDO mapping configuration completed successfully.\n");
    return 0;
}

// --- Main Interface Functions ---
int soem_interface_init(const char *ifname) {
    int i;
    int slave_idx = 1;

    printf("SOEM_Interface: Initializing EtherCAT master on %s...\n", ifname);

    if (ec_init(ifname)) {
        printf("SOEM_Interface: ec_init on %s succeeded.\n", ifname);

        if (ec_config_init(FALSE) > 0) {
            printf("SOEM_Interface: %d slaves found and configured.\n", ec_slavecount);

            if (ec_slavecount == 0) {
                fprintf(stderr, "SOEM_Interface: No EtherCAT slaves found!\n");
                return -1;
            }

            // Print slave information before configuration
            for (i = 1; i <= ec_slavecount; i++) {
                printf("SOEM_Interface: Slave %d: Name=%s, OutputSize=%dbytes, InputSize=%dbytes, State=%s\n",
                       i, ec_slave[i].name, ec_slave[i].Obits / 8, ec_slave[i].Ibits / 8, 
                       get_state_name(ec_slave[i].state));
                
                // Print AL status code if there's an error
                if (ec_slave[i].ALstatuscode != 0) {
                    printf("SOEM_Interface: Slave %d ALstatuscode: 0x%04x\n", i, ec_slave[i].ALstatuscode);
                }
            }

            // Configure PDO mapping for SOMANET devices BEFORE ec_config_map
            if (ec_slavecount >= slave_idx) {
                printf("SOEM_Interface: Configuring PDO mapping for slave %d...\n", slave_idx);
                if (configure_somanet_pdo_mapping(slave_idx) != 0) {
                    fprintf(stderr, "SOEM_Interface: Failed to configure PDO mapping.\n");
                    return -1;
                }
                
                // Brief delay after PDO configuration
                usleep(100000);
            }

            // Configure distributed clocks
            ec_configdc();

            // Map the IO after PDO configuration
            printf("SOEM_Interface: Mapping IO...\n");
            if (ec_config_map(&IOmap) == 0) {
                fprintf(stderr, "SOEM_Interface: ec_config_map failed\n");
                return -1;
            }

            // Assign PDO pointers
            if (ec_slave[slave_idx].outputs > 0) {
                somanet_outputs = (somanet_rx_pdo_t *)(ec_slave[slave_idx].outputs);
                printf("SOEM_Interface: somanet_outputs mapped at %p (size: %d bytes)\n", 
                       (void*)somanet_outputs, ec_slave[slave_idx].Obits / 8);
            } else {
                fprintf(stderr, "SOEM_Interface: No output PDO data available!\n");
                return -1;
            }
            
            if (ec_slave[slave_idx].inputs > 0) {
                somanet_inputs = (somanet_tx_pdo_t *)(ec_slave[slave_idx].inputs);
                printf("SOEM_Interface: somanet_inputs mapped at %p (size: %d bytes)\n", 
                       (void*)somanet_inputs, ec_slave[slave_idx].Ibits / 8);
            } else {
                fprintf(stderr, "SOEM_Interface: No input PDO data available!\n");
                return -1;
            }

            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("SOEM_Interface: Expected WKC: %d\n", expectedWKC);

            // Initialize CiA 402 parameters while in Pre-Op
            if (initialize_cia402_parameters(slave_idx) != 0) {
                fprintf(stderr, "SOEM_Interface: Failed to initialize CiA 402 parameters.\n");
                return -1;
            }

            // Transition to Safe-Operational with proper verification
            printf("SOEM_Interface: Requesting Safe-Operational state...\n");
            ec_slave[0].state = EC_STATE_SAFE_OP;
            ec_writestate(0);
            
            int wkc_state = ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
            
            // Detailed state verification
            if (wkc_state == 0 || (ec_slave[0].state & 0x0F) != EC_STATE_SAFE_OP) {
                fprintf(stderr, "SOEM_Interface: Failed to reach Safe-Operational state.\n");
                fprintf(stderr, "SOEM_Interface: Master state: %s, WKC: %d\n", 
                        get_state_name(ec_slave[0].state), wkc_state);
                
                // Print individual slave states and errors
                for (i = 1; i <= ec_slavecount; i++) {
                    printf("SOEM_Interface: Slave %d - State: %s, ALstatuscode: 0x%04x\n", 
                           i, get_state_name(ec_slave[i].state), ec_slave[i].ALstatuscode);
                    
                    // Decode common AL status codes
                    if (ec_slave[i].ALstatuscode != 0) {
                        switch (ec_slave[i].ALstatuscode) {
                            case 0x0001:
                                printf("  -> Unspecified error\n");
                                break;
                            case 0x0002:
                                printf("  -> No memory\n");
                                break;
                            case 0x0011:
                                printf("  -> Invalid requested state change\n");
                                break;
                            case 0x0012:
                                printf("  -> Unknown requested state\n");
                                break;
                            case 0x0013:
                                printf("  -> Bootstrap not supported\n");
                                break;
                            case 0x0014:
                                printf("  -> No valid firmware\n");
                                break;
                            case 0x0015:
                                printf("  -> Invalid mailbox configuration\n");
                                break;
                            case 0x0016:
                                printf("  -> Invalid mailbox configuration\n");
                                break;
                            case 0x0017:
                                printf("  -> Invalid sync manager configuration\n");
                                break;
                            case 0x0018:
                                printf("  -> No valid inputs available\n");
                                break;
                            case 0x0019:
                                printf("  -> No valid outputs available\n");
                                break;
                            case 0x001A:
                                printf("  -> Synchronization error\n");
                                break;
                            case 0x001B:
                                printf("  -> Sync manager watchdog\n");
                                break;
                            case 0x001C:
                                printf("  -> Invalid sync manager types\n");
                                break;
                            case 0x001D:
                                printf("  -> Invalid output configuration\n");
                                break;
                            case 0x001E:
                                printf("  -> Invalid input configuration\n");
                                break;
                            case 0x001F:
                                printf("  -> Invalid watchdog configuration\n");
                                break;
                            case 0x0020:
                                printf("  -> Slave needs cold start\n");
                                break;
                            case 0x0021:
                                printf("  -> Slave needs INIT\n");
                                break;
                            case 0x0022:
                                printf("  -> Slave needs PREOP\n");
                                break;
                            case 0x0023:
                                printf("  -> Slave needs SAFEOP\n");
                                break;
                            case 0x002C:
                                printf("  -> Invalid input mapping\n");
                                break;
                            case 0x002D:
                                printf("  -> Invalid output mapping\n");
                                break;
                            case 0x002E:
                                printf("  -> Inconsistent settings\n");
                                break;
                            case 0x002F:
                                printf("  -> Freerun not supported\n");
                                break;
                            case 0x0030:
                                printf("  -> Synchronization not supported\n");
                                break;
                            case 0x0031:
                                printf("  -> Freerun needs 3 buffer mode\n");
                                break;
                            case 0x0032:
                                printf("  -> Background watchdog\n");
                                break;
                            case 0x0033:
                                printf("  -> No valid inputs and outputs\n");
                                break;
                            case 0x0034:
                                printf("  -> Fatal sync error\n");
                                break;
                            case 0x0035:
                                printf("  -> No sync error\n");
                                break;
                            case 0x0036:
                                printf("  -> Cycle time too small\n");
                                break;
                            default:
                                printf("  -> Unknown AL status code: 0x%04x\n", ec_slave[i].ALstatuscode);
                                break;
                        }
                    }
                }
                return -1;
            }

            printf("SOEM_Interface: Successfully reached Safe-Operational state.\n");

            // Initialize safe values in outputs before going operational
            if (somanet_outputs) {
                memset(somanet_outputs, 0, sizeof(somanet_rx_pdo_t));
                somanet_outputs->controlword = 0x0006; // Shutdown
                somanet_outputs->modes_of_operation = 4; // Torque mode
            }

            // Send initial PDO data
            ec_send_processdata();
            usleep(10000);

            // Transition to Operational
            printf("SOEM_Interface: Requesting Operational state...\n");
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            ec_writestate(0);
            
            wkc_state = ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 4);

            // Verify all slaves are operational
            int all_operational = 1;
            for (i = 1; i <= ec_slavecount; i++) {
                if (!is_slave_operational(i)) {
                    printf("SOEM_Interface: Slave %d not operational. State: %s, ALstatuscode: 0x%04x\n", 
                           i, get_state_name(ec_slave[i].state), ec_slave[i].ALstatuscode);
                    all_operational = 0;
                }
            }

            if (all_operational) {
                printf("SOEM_Interface: All slaves operational. Starting communication thread...\n");
                master_initialized = 1;
                ecat_thread_running = 1;
                
                if (pthread_create(&ecat_thread, NULL, ecat_loop, NULL) != 0) {
                    fprintf(stderr, "SOEM_Interface: Failed to create EtherCAT thread.\n");
                    return -1;
                }
                
                return 0;
            } else {
                fprintf(stderr, "SOEM_Interface: Not all slaves operational.\n");
                return -1;
            }
        } else {
            fprintf(stderr, "SOEM_Interface: No slaves found.\n");
            return -1;
        }
    } else {
        fprintf(stderr, "SOEM_Interface: No EtherCAT master found.\n");
        return -1;
    }
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