// soem_interface.c - Fixed version with correct state checking and improved PDO configuration
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

// --- SOEM Global Variables ---
char IOmap[4096];
ec_ODlistt ODlist;
ec_groupt DCgroup;
int wkc;
int expectedWKC;
ec_timet tmo;

// --- PDO Structures for Synapticon ACTILINK-S (Slave 1) ---
// These structures define the expected PDO layout.
// They are used for mapping the received/sent data to C variables.
// The actual PDO mapping configuration will be done dynamically via SDOs.
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
    uint32 bit_mask;            // Example for padding/alignment if needed
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
somanet_rx_pdo_t *somanet_outputs;
somanet_tx_pdo_t *somanet_inputs;

// Mutex for protecting PDO data access
static pthread_mutex_t pdo_mutex = PTHREAD_MUTEX_INITIALIZER;

// Thread for EtherCAT communication
static pthread_t ecat_thread;
static volatile int ecat_thread_running = 0;
static volatile int master_initialized = 0;
static volatile int communication_ok = 0;

// Global variables to hold current PDO values (for external access)
static float target_torque_f = 0.0f;
static float current_position_f = 0.0f;
static float current_velocity_f = 0.0f;

// --- Helper function to check if slave is in operational state ---
// Fixed: EtherCAT state can include ACK bit (0x10), so we need to mask it out
int is_slave_operational(int slave_idx) {
    // Extract the actual state by masking out the ACK bit
    uint16 actual_state = ec_slave[slave_idx].state & 0x0F;
    // Check if the actual state is operational (8)
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

// --- SOEM Thread Function ---
void *ecat_loop(void *ptr) {
    int slave_idx = 1; // Assuming slave 1 is the Synapticon device

    printf("SOEM_Interface: EtherCAT thread started.\n");

    // Wait for master initialization to complete
    while (!master_initialized && ecat_thread_running) {
        usleep(10000); // Wait 10ms
    }

    if (!master_initialized) {
        printf("SOEM_Interface: Master not initialized, exiting thread.\n");
        return NULL;
    }

    printf("SOEM_Interface: Entering EtherCAT cyclic loop.\n");

    /*
    // If statusword indicates a fault (e.g., bit 3 set)
    uint16_t error_register_val;
    if (soem_interface_read_sdo(slave_idx, 0x603F, 0x00, sizeof(error_register_val), &error_register_val) == 0) {
        printf("SOEM_Interface: Error Register (0x603F:00): 0x%04X\n", error_register_val);
    // Decode error_register_val based on CiA 402 or Synapticon specific definitions
    }

    uint32_t error_history_entry; // Typically 0x1003:01 is the last error
    if (soem_interface_read_sdo(slave_idx, 0x1003, 0x01, sizeof(error_history_entry), &error_history_entry) == 0) {
        printf("SOEM_Interface: Last Error Code (0x1003:01): 0x%08X\n", error_history_entry);
    // The top 16 bits are the error code, lower 16 bits might be additional info
    }
    */

    while (ecat_thread_running) {
        // Send process data
        pthread_mutex_lock(&pdo_mutex);
        if (somanet_outputs) {
            somanet_outputs->target_torque = (int16_t)(target_torque_f * 1000.0f); // Assuming torque is in mNm
            // Other outputs can be set here if needed
        }
        pthread_mutex_unlock(&pdo_mutex);

        // Exchange process data
        ec_send_processdata();
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc >= expectedWKC) {
            communication_ok = 1;
            pthread_mutex_lock(&pdo_mutex);
            if (somanet_inputs) {
                current_position_f = (float)somanet_inputs->position_actual_value;
                current_velocity_f = (float)somanet_inputs->velocity_actual_value;
                // Read other inputs if needed
            }
            pthread_mutex_unlock(&pdo_mutex);
        } else {
            communication_ok = 0;
            //printf("SOEM_Interface: WKC mismatch: %d/%d\n", wkc, expectedWKC);
        }

        // Check slave state - use helper function
        if (!is_slave_operational(slave_idx)) {
            // printf("SOEM_Interface: Slave %d not in OP state, current state: %s (%d)\n", 
            //        slave_idx, get_state_name(ec_slave[slave_idx].state), ec_slave[slave_idx].state);
            ec_statecheck(slave_idx, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
        }

        usleep(1000); // Cycle time: 1ms (1000us)
    }

    printf("SOEM_Interface: EtherCAT thread stopping.\n");
    return NULL;
}

// --- Helper function for SDO writes ---
// This function performs an SDO write operation to a specific slave.
// It uses ec_SDOwrite from the SOEM library.
// @param slave_idx The index of the slave (1-based).
// @param index The 16-bit object dictionary index.
// @param subindex The 8-bit object dictionary subindex.
// @param data_size The size of the data to write in bytes.
// @param data Pointer to the data to write.
// @return 0 on success, -1 on failure.
int soem_interface_write_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex, uint16_t data_size, void *data) {
    int wkc_sdo;
    wkc_sdo = ec_SDOwrite(slave_idx, index, subindex, FALSE, data_size, data, EC_TIMEOUTRXM);
    if (wkc_sdo == 0) {
        fprintf(stderr, "SOEM_Interface: SDO write failed for slave %u, index 0x%04X:%02X\n", slave_idx, index, subindex);
        return -1;
    }
    return 0;
}

// --- Helper function for SDO reads ---
int soem_interface_read_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex, uint16_t data_size, void *data) {
    int wkc_sdo;
    wkc_sdo = ec_SDOread(slave_idx, index, subindex, FALSE, &data_size, data, EC_TIMEOUTRXM);
    if (wkc_sdo == 0) {
        fprintf(stderr, "SOEM_Interface: SDO read failed for slave %u, index 0x%04X:%02X\n", slave_idx, index, subindex);
        return -1;
    }
    return 0;
}

// --- Function to set EtherCAT slave state ---
// This function attempts to set a specific EtherCAT slave to a desired state.
// @param slave_idx The index of the slave (1-based).
// @param desired_state The target EtherCAT state (e.g., EC_STATE_PRE_OP, EC_STATE_OPERATIONAL).
// @return 0 on success, -1 on failure.
int soem_interface_set_ethercat_state(uint16_t slave_idx, ec_state desired_state) {
    int wkc_state;
    printf("SOEM_Interface: Attempting to set slave %u to state %s (%d)...\n", 
           slave_idx, get_state_name(desired_state), desired_state);
    
    ec_slave[slave_idx].state = desired_state;
    ec_writestate(slave_idx);
    
    // Wait for state transition with timeout
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

// --- Function to configure PDO mapping dynamically ---
// This function implements the steps for dynamic PDO remapping as per Synapticon documentation.
// @param slave_idx The index of the slave (1-based).
// @param pdo_assign_idx The index of the PDO Assign object (e.g., 0x1C12 for RxPDO, 0x1C13 for TxPDO).
// @param pdo_map_idx The index of the PDO Mapping object (e.g., 0x1600 for RxPDO, 0x1A00 for TxPDO).
// @param mapped_objects An array of 32-bit values, where each value represents an object to map:
//                       (Index << 16 | Subindex << 8 | LengthInBits).
// @param num_mapped_objects The number of objects in the mapped_objects array.
// @return 0 on success, -1 on failure.
int soem_interface_configure_pdo_mapping(uint16_t slave_idx, uint16_t pdo_assign_idx, uint16_t pdo_map_idx, uint32_t *mapped_objects, uint8_t num_mapped_objects) {
    uint8_t zero_val = 0;
    uint8_t original_assign_val;
    uint8_t current_assign_val;
    int ret = 0;

    printf("SOEM_Interface: Starting PDO mapping configuration for slave %u...\n", slave_idx);

    // 1. Ensure that the desired object is PDO mappable (checked by design/documentation)
    //    This step is assumed to be verified during the design phase based on the device's object dictionary.
    //    The 'mapped_objects' array should only contain objects confirmed to be PDO mappable.

    // 2. Switch EtherCAT state to Pre-operational
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_PRE_OP) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to transition to Pre-operational state.\n");
        return -1;
    }
    usleep(50000); // Give slave more time to settle (50ms)

    // Read current PDO assign value before disabling to restore later
    uint16_t read_size = sizeof(original_assign_val);
    if (soem_interface_read_sdo(slave_idx, pdo_assign_idx, 0x00, read_size, &original_assign_val) != 0) {
        // If read fails, use defaults as per Synapticon documentation
        printf("SOEM_Interface: Could not read current PDO assign value, using defaults.\n");
        if (pdo_assign_idx == 0x1C12) { // RxPDO assign
            original_assign_val = 3;
        } else if (pdo_assign_idx == 0x1C13) { // TxPDO assign
            original_assign_val = 4;
        } else {
            fprintf(stderr, "SOEM_Interface: Unknown PDO assign index 0x%04X.\n", pdo_assign_idx);
            return -1;
        }
    }
    printf("SOEM_Interface: Original PDO assign value for 0x%04X:00 is %u\n", pdo_assign_idx, original_assign_val);

    // 3. Disable PDO distribution by setting 0 to 0x1C12:00 (for RxPDO) or setting 0 to 0x1C13:00 (for TxPDO)
    printf("SOEM_Interface: Disabling PDO distribution for 0x%04X:00...\n", pdo_assign_idx);
    if (soem_interface_write_sdo(slave_idx, pdo_assign_idx, 0x00, sizeof(zero_val), &zero_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to disable PDO distribution.\n");
        ret = -1; goto cleanup;
    }
    usleep(20000); // 20ms delay

    // 4. Disable one of the PDO mapping objects by setting its subindex 0 to 0
    printf("SOEM_Interface: Disabling PDO mapping object 0x%04X:00...\n", pdo_map_idx);
    if (soem_interface_write_sdo(slave_idx, pdo_map_idx, 0x00, sizeof(zero_val), &zero_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to disable PDO mapping object.\n");
        ret = -1; goto cleanup;
    }
    usleep(20000); // 20ms delay

    // 5. Enter the objects that should be mapped to the mapping entry subindices 1 to n
    printf("SOEM_Interface: Writing %u mapped objects to 0x%04X...\n", num_mapped_objects, pdo_map_idx);
    for (uint8_t i = 0; i < num_mapped_objects; i++) {
        uint8_t subindex = i + 1;
        printf("  Mapping object %u: 0x%08X to 0x%04X:%02X\n", i + 1, mapped_objects[i], pdo_map_idx, subindex);
        if (soem_interface_write_sdo(slave_idx, pdo_map_idx, subindex, sizeof(uint32_t), &mapped_objects[i]) != 0) {
            fprintf(stderr, "SOEM_Interface: Failed to write mapped object %u.\n", i + 1);
            ret = -1; goto cleanup;
        }
        usleep(5000); // 5ms delay between SDO writes
    }

    // 6. Set subindex 0 to the number n of used entries
    printf("SOEM_Interface: Setting number of mapped objects for 0x%04X:00 to %u...\n", pdo_map_idx, num_mapped_objects);
    if (soem_interface_write_sdo(slave_idx, pdo_map_idx, 0x00, sizeof(num_mapped_objects), &num_mapped_objects) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set number of mapped objects.\n");
        ret = -1; goto cleanup;
    }
    usleep(20000); // 20ms delay

    // 7. Enable PDO distribution by setting 0x1C12:00 or 0x1C13:00 to their original values
    printf("SOEM_Interface: Re-enabling PDO distribution for 0x%04X:00 with value %u...\n", pdo_assign_idx, original_assign_val);
    if (soem_interface_write_sdo(slave_idx, pdo_assign_idx, 0x00, sizeof(original_assign_val), &original_assign_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to re-enable PDO distribution.\n");
        ret = -1; goto cleanup;
    }
    usleep(20000); // 20ms delay

    printf("SOEM_Interface: PDO mapping configuration completed for slave %u.\n", slave_idx);

cleanup:
    // Always attempt to transition back to Safe-Operational first, then Operational
    printf("SOEM_Interface: Attempting to transition back to Safe-Operational state...\n");
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_SAFE_OP) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to transition to Safe-Operational state.\n");
        ret = -1;
    }
    usleep(50000); // 50ms delay
    
    printf("SOEM_Interface: Attempting to transition back to Operational state...\n");
    if (soem_interface_set_ethercat_state(slave_idx, EC_STATE_OPERATIONAL) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to transition back to Operational state.\n");
        ret = -1;
    }
    return ret;
}

// --- SOEM Interface Functions ---

// Initialize the EtherCAT master and slaves
int soem_interface_init(const char *ifname) {
    int i, chk;
    int slave_idx = 1; // Assuming slave 1 is the Synapticon device

    printf("SOEM_Interface: Initializing EtherCAT master on %s...\n", ifname);

    // Initialize SOEM
    if (ec_init(ifname)) {
        printf("SOEM_Interface: ec_init on %s succeeded.\n", ifname);

        // Find slaves
        if (ec_config_init(FALSE) > 0) {
            printf("SOEM_Interface: %d slaves found and configured.\n", ec_slavecount);

            if (ec_slavecount == 0) {
                fprintf(stderr, "SOEM_Interface: No EtherCAT slaves found!\n");
                return -1;
            }

            // Configure distributed clocks (if supported/needed)
            ec_configdc();

            // Configure process data (PDOs)
            // This is where SOEM's auto-configuration happens.
            // We will override this later with dynamic PDO mapping.
            ec_config_map(&IOmap);

            // Print slave information
            for (i = 1; i <= ec_slavecount; i++) {
                printf("SOEM_Interface: Slave %d: Name=%s, OutputSize=%dbytes, InputSize=%dbytes, State=%s (%d)\n",
                       i, ec_slave[i].name, ec_slave[i].Obits / 8, ec_slave[i].Ibits / 8, 
                       get_state_name(ec_slave[i].state), ec_slave[i].state);
            }

            // Check for Synapticon slave (by name or vendor ID/product code)
            // For now, we assume slave 1 is the Synapticon device.
            if (ec_slavecount >= slave_idx) {
                printf("SOEM_Interface: Mapping PDOs for Synapticon slave (index %d)...\n", slave_idx);

                // Define custom RxPDO mapping for Synapticon ACTILINK-S ---
                // Objects are (Index << 16 | Subindex << 8 | LengthInBits)
                // Based on Synapticon documentation, use standard CiA 402 objects
                uint32_t custom_rx_mapped_objects[] = {
                    0x60400010, // 0x6040:00 Controlword (16 bits)
                    0x60600008, // 0x6060:00 Modes of operation (8 bits)
                    0x60710010, // 0x6071:00 Target torque (16 bits)
                    0x607A0020, // 0x607A:00 Target position (32 bits)
                    0x60FF0020, // 0x60FF:00 Target velocity (32 bits)
                    0x60B20010, // 0x60B2:00 Torque offset (16 bits)
                };
                uint8_t num_rx_mapped_objects = sizeof(custom_rx_mapped_objects) / sizeof(custom_rx_mapped_objects[0]);

                // Configure RxPDO mapping (0x1600 is RxPDO mapping 1)
                if (soem_interface_configure_pdo_mapping(slave_idx, 0x1C12, 0x1600, custom_rx_mapped_objects, num_rx_mapped_objects) != 0) {
                    fprintf(stderr, "SOEM_Interface: Failed to configure RxPDO mapping.\n");
                    return -1;
                }
                printf("SOEM_Interface: RxPDO mapping configured.\n");

                // Define custom TxPDO mapping for Synapticon ACTILINK-S ---
                uint32_t custom_tx_mapped_objects[] = {
                    0x60410010, // 0x6041:00 Statusword (16 bits)
                    0x60610008, // 0x6061:00 Modes of operation display (8 bits)
                    0x60640020, // 0x6064:00 Position actual value (32 bits)
                    0x60690020, // 0x6069:00 Velocity actual value (32 bits)
                    0x60770010, // 0x6077:00 Torque actual value (16 bits)
                    0x60780010, // 0x6078:00 Current actual value (16 bits)
                };
                uint8_t num_tx_mapped_objects = sizeof(custom_tx_mapped_objects) / sizeof(custom_tx_mapped_objects[0]);

                // Configure TxPDO mapping (0x1A00 is TxPDO mapping 1)
                if (soem_interface_configure_pdo_mapping(slave_idx, 0x1C13, 0x1A00, custom_tx_mapped_objects, num_tx_mapped_objects) != 0) {
                    fprintf(stderr, "SOEM_Interface: Failed to configure TxPDO mapping.\n");
                    return -1;
                }
                printf("SOEM_Interface: TxPDO mapping configured.\n");

                // Re-map the IOmap after custom PDO configuration
                ec_config_map(&IOmap);

                // Re-calculate expected WKC
                expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
                printf("SOEM_Interface: Calculated expected WKC: %d\n", expectedWKC);

                // Assign PDO pointers based on the new IOmap
                // Ensure these offsets match the configured PDOs
                if (ec_slave[slave_idx].outputs > 0) {
                    somanet_outputs = (somanet_rx_pdo_t *)(ec_slave[slave_idx].outputs);
                    printf("SOEM_Interface: somanet_outputs mapped at %p\n", (void*)somanet_outputs);
                } else {
                    fprintf(stderr, "SOEM_Interface: No output PDOs found for slave %d.\n", slave_idx);
                    return -1;
                }
                if (ec_slave[slave_idx].inputs > 0) {
                    somanet_inputs = (somanet_tx_pdo_t *)(ec_slave[slave_idx].inputs);
                    printf("SOEM_Interface: somanet_inputs mapped at %p\n", (void*)somanet_inputs);
                } else {
                    fprintf(stderr, "SOEM_Interface: No input PDOs found for slave %d.\n", slave_idx);
                    return -1;
                }

            } else {
                fprintf(stderr, "SOEM_Interface: Synapticon slave (index %d) not found.\n", slave_idx);
                return -1;
            }

            // Go to Operational state
            printf("SOEM_Interface: Requesting Operational state for all slaves...\n");
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 4); // Wait longer for all slaves

            // Check if all slaves are operational with improved state checking
            int all_slaves_operational = 1;
            for (i = 1; i <= ec_slavecount; i++) {
                if (!is_slave_operational(i)) {
                    printf("SOEM_Interface: Slave %d not operational. Current state: %s (%d)\n", 
                           i, get_state_name(ec_slave[i].state), ec_slave[i].state);
                    all_slaves_operational = 0;
                } else {
                    printf("SOEM_Interface: Slave %d is operational. Current state: %s (%d)\n", 
                           i, get_state_name(ec_slave[i].state), ec_slave[i].state);
                }
            }

            if (all_slaves_operational) {
                printf("SOEM_Interface: All slaves are in Operational state.\n");
                master_initialized = 1;
                ecat_thread_running = 1;
                // Start EtherCAT cyclic thread
                if (pthread_create(&ecat_thread, NULL, ecat_loop, NULL) != 0) {
                    fprintf(stderr, "SOEM_Interface: Failed to create EtherCAT thread.\n");
                    master_initialized = 0;
                    ecat_thread_running = 0;
                    return -1;
                }
            } else {
                fprintf(stderr, "SOEM_Interface: Not all slaves reached Operational state.\n");
                for (i = 1; i <= ec_slavecount; i++) {
                    printf("SOEM_Interface: Slave %d: Current State=%s (%d) (Operational=%s)\n", 
                           i, get_state_name(ec_slave[i].state), ec_slave[i].state, 
                           is_slave_operational(i) ? "YES" : "NO");
                }
                return -1;
            }
        } else {
            fprintf(stderr, "SOEM_Interface: No slaves found on %s.\n", ifname);
            return -1;
        }
    } else {
        fprintf(stderr, "SOEM_Interface: No EtherCAT master found on %s.\n", ifname);
        return -1;
    }

    return 0;
}

// Send target torque to the slave
void soem_interface_send_and_receive_pdo(float target_torque) {
    if (!master_initialized) {
        return;
    }
    pthread_mutex_lock(&pdo_mutex);
    target_torque_f = target_torque;
    pthread_mutex_unlock(&pdo_mutex);
}

// Get current position from the slave
float soem_interface_get_current_position() {
    float position;
    pthread_mutex_lock(&pdo_mutex);
    position = current_position_f;
    pthread_mutex_unlock(&pdo_mutex);
    return position;
}

// Get current velocity from the slave
float soem_interface_get_current_velocity() {
    float velocity;
    pthread_mutex_lock(&pdo_mutex);
    velocity = current_velocity_f;
    pthread_mutex_unlock(&pdo_mutex);
    return velocity;
}

// Get communication status
int soem_interface_get_communication_status() {
    int status;
    pthread_mutex_lock(&pdo_mutex);
    status = communication_ok;
    pthread_mutex_unlock(&pdo_mutex);
    return status;
}

// Stop the EtherCAT master
void soem_interface_stop_master() {
    if (master_initialized) {
        printf("SOEM_Interface: Stopping EtherCAT master...\n");

        ecat_thread_running = 0;
        if (ecat_thread) {
            pthread_join(ecat_thread, NULL);
        }

        // Set target torque and controlword to safe values before stopping
        if (somanet_outputs) {
            somanet_outputs->target_torque = 0;
            somanet_outputs->controlword = 0x06; // Shutdown state
        }
        ec_send_processdata(); // Send one last time

        // Go to Safe-Operational then Init state
        soem_interface_set_ethercat_state(0, EC_STATE_SAFE_OP); // All slaves
        soem_interface_set_ethercat_state(0, EC_STATE_INIT);    // All slaves

        ec_close();
        master_initialized = 0;
        printf("SOEM_Interface: EtherCAT master stopped.\n");
    }
}