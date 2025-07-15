// soem_interface.c - Fixed version with proper PDO configuration and dynamic PDO mapping
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
int is_slave_operational(int slave_idx) {
    // Check if slave is in operational state (bit 3 set)
    // State 18 (0x12) = EC_STATE_OPERATIONAL (8) + EC_STATE_ACK (16)
    // A more robust check might involve checking for EC_STATE_OPERATIONAL only,
    // and then verifying the statusword for drive readiness.
    return (ec_slave[slave_idx].state & EC_STATE_OPERATIONAL) == EC_STATE_OPERATIONAL;
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

    while (ecat_thread_running) {
        // Send process data
        pthread_mutex_lock(&pdo_mutex);
        if (somanet_outputs) {
            // Continuously set controlword to enable operation (0x0F) if in operational state
            // This ensures the drive remains active.
            if (is_slave_operational(slave_idx)) {
                somanet_outputs->controlword = 0x0F; // Enable Operation
            } else {
                // If not operational, you might want to set a safe controlword or try to transition
                somanet_outputs->controlword = 0x06; // Shutdown state
            }
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
             // printf("SOEM_Interface: Slave %d not in OP state, current state: %d\n", slave_idx, ec_slave[slave_idx].state);
            // Attempt to transition to operational if not already there.
            // This might be redundant if the main init handles it, but good for robustness.
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
// This function performs an SDO read operation from a specific slave.
// @param slave_idx The index of the slave (1-based).
// @param index The 16-bit object dictionary index.
// @param subindex The 8-bit object dictionary subindex.
// @param expected_data_size The expected size of the data to read in bytes.
// @param out_data_buffer Pointer to the buffer to store the read data.
// @return 0 on success, -1 on failure.
int soem_interface_read_sdo(uint16_t slave_idx, uint16_t index, uint8_t subindex, uint16_t expected_data_size, void *out_data_buffer) {
    int wkc_sdo;
    int actual_size = expected_data_size; // 'actual_size' will hold the size read by SOEM
    void *temp_buffer_ptr = out_data_buffer; // A temporary void* that points to the user's buffer

    // Call ec_SDOread.
    // For p_size: Pass the address of 'actual_size' (which is an int).
    // For p_data: Pass the address of 'temp_buffer_ptr' (which is a void*).
    // The ec_SDOread function will then write the data into the memory pointed to by *temp_buffer_ptr.
    wkc_sdo = ec_SDOread(slave_idx, index, subindex, FALSE, &actual_size, &temp_buffer_ptr, EC_TIMEOUTRXM);

    if (wkc_sdo == 0) {
        fprintf(stderr, "SOEM_Interface: SDO read failed for slave %u, index 0x%04X:%02X\n", slave_idx, index, subindex);
        return -1;
    }

    // Optional: Verify if the actual size read matches the expected size.
    if (actual_size != expected_data_size) {
        fprintf(stderr, "SOEM_Interface: SDO read size mismatch for slave %u, index 0x%04X:%02X. Expected %u bytes, got %d bytes.\n",
                slave_idx, index, subindex, expected_data_size, actual_size);
    }

    return 0;
}

// --- Function to set EtherCAT slave state ---
// This function attempts to set a specific EtherCAT slave to a desired state.
// @param slave_idx The index of the slave (1-based).
// @param desired_state The target EtherCAT state (e.g., EC_STATE_PRE_OP, EC_STATE_OPERATIONAL).
// @return 0 on success, -1 on failure.
int soem_interface_set_ethercat_state(uint16_t slave_idx, ec_state desired_state) { // Changed ec_state_t to ec_state
    int wkc_state;
    printf("SOEM_Interface: Attempting to set slave %u to state %d...\n", slave_idx, desired_state);
    ec_slave[slave_idx].state = desired_state;
    ec_writestate(slave_idx);
    wkc_state = ec_statecheck(slave_idx, desired_state, EC_TIMEOUTSTATE);
    if (wkc_state == 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set slave %u to state %d. Current state: %d\n",
                slave_idx, desired_state, ec_slave[slave_idx].state);
        return -1;
    }
    printf("SOEM_Interface: Slave %u successfully transitioned to state %d.\n", slave_idx, ec_slave[slave_idx].state);
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
    uint8_t original_rx_assign_val = 3; // Default for 0x1C12:00
    uint8_t original_tx_assign_val = 4; // Default for 0x1C13:00
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
    usleep(10000); // Give slave time to settle

    // Read current PDO assign value before disabling to restore later
    // Note: ec_SDOread is needed here, but SOEM's ec_SDOread returns wkc, not data directly.
    // For simplicity, we'll assume default values for restoration.
    // In a robust application, you'd read the current value first.
    // For now, we'll use the default values as per Synapticon documentation.

    // 3. Disable PDO distribution by setting 0 to 0x1C12:00 (for RxPDO) or setting 0 to 0x1C13:00 (for TxPDO)
    printf("SOEM_Interface: Disabling PDO distribution for 0x%04X:00...\n", pdo_assign_idx);
    if (soem_interface_write_sdo(slave_idx, pdo_assign_idx, 0x00, sizeof(zero_val), &zero_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to disable PDO distribution.\n");
        ret = -1; goto cleanup;
    }
    usleep(10000);

    // 4. Disable one of the PDO mapping objects by setting its subindex 0 to 0
    printf("SOEM_Interface: Disabling PDO mapping object 0x%04X:00...\n", pdo_map_idx);
    if (soem_interface_write_sdo(slave_idx, pdo_map_idx, 0x00, sizeof(zero_val), &zero_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to disable PDO mapping object.\n");
        ret = -1; goto cleanup;
    }
    usleep(10000);

    // 5. Enter the objects that should be mapped to the mapping entry subindices 1 to n
    printf("SOEM_Interface: Writing %u mapped objects to 0x%04X...\n", num_mapped_objects, pdo_map_idx);
    for (uint8_t i = 0; i < num_mapped_objects; i++) {
        uint8_t subindex = i + 1;
        printf("  Mapping object %u: 0x%08X to 0x%04X:%02X\n", i + 1, mapped_objects[i], pdo_map_idx, subindex);
        if (soem_interface_write_sdo(slave_idx, pdo_map_idx, subindex, sizeof(uint32_t), &mapped_objects[i]) != 0) {
            fprintf(stderr, "SOEM_Interface: Failed to write mapped object %u.\n", i + 1);
            ret = -1; goto cleanup;
        }
        usleep(1000); // Small delay between SDO writes
    }

    // 6. Set subindex 0 to the number n of used entries
    printf("SOEM_Interface: Setting number of mapped objects for 0x%04X:00 to %u...\n", pdo_map_idx, num_mapped_objects);
    if (soem_interface_write_sdo(slave_idx, pdo_map_idx, 0x00, sizeof(num_mapped_objects), &num_mapped_objects) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set number of mapped objects.\n");
        ret = -1; goto cleanup;
    }
    usleep(10000);

    // 7. Enable PDO distribution by setting 0x1C12:00 or 0x1C13:00 to their original values, by default 3 and 4 respectively
    printf("SOEM_Interface: Re-enabling PDO distribution for 0x%04X:00...\n", pdo_assign_idx);
    if (pdo_assign_idx == 0x1C12) { // RxPDO assign
        current_assign_val = original_rx_assign_val;
    } else if (pdo_assign_idx == 0x1C13) { // TxPDO assign
        current_assign_val = original_tx_assign_val;
    } else {
        fprintf(stderr, "SOEM_Interface: Unknown PDO assign index 0x%04X.\n", pdo_assign_idx);
        ret = -1; goto cleanup;
    }

    if (soem_interface_write_sdo(slave_idx, pdo_assign_idx, 0x00, sizeof(current_assign_val), &current_assign_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to re-enable PDO distribution.\n");
        ret = -1; goto cleanup;
    }
    usleep(10000);

    printf("SOEM_Interface: PDO mapping configuration completed for slave %u.\n", slave_idx);

cleanup:
    // Transition back to Operational state
    printf("SOEM_Interface: Attempting to transition back to Operational state...\n");
    // This transition will be handled by the main init function after drive configuration
    return ret;
}

// --- Function to configure drive modes and controlword via SDOs ---
// This function sets the Modes of Operation and Controlword to bring the drive
// to a state where it can accept operational commands.
// @param slave_idx The index of the slave (1-based).
// @param mode_of_operation The desired mode of operation (e.g., 8 for CSP, 1 for PPM).
// @return 0 on success, -1 on failure.
int soem_interface_configure_drive_modes(uint16_t slave_idx, int8_t mode_of_operation) {
    uint16_t controlword_val;
    uint16_t statusword_val = 0;
    int ret = 0;
    int attempts = 0;
    const int max_attempts = 100; // Max attempts for polling (10ms * 100 = 1 second)

    printf("SOEM_Interface: Configuring drive modes for slave %u...\n", slave_idx);

    // --- Add Fault Reset ---
    // This is crucial if the drive is in a fault state from a previous run or power cycle.
    controlword_val = 0x80; // Fault Reset
    printf("SOEM_Interface: Setting Controlword (0x6040:00) to 0x%04X (Fault Reset)...\n", controlword_val);
    if (soem_interface_write_sdo(slave_idx, 0x6040, 0x00, sizeof(controlword_val), &controlword_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to send Fault Reset command.\n");
        return -1;
    }
    usleep(20000); // Give drive time to process fault reset (20ms)

    // Poll Statusword to ensure fault is cleared (bit 3 should be 0)
    attempts = 0;
    do {
        if (soem_interface_read_sdo(slave_idx, 0x6041, 0x00, sizeof(statusword_val), &statusword_val) != 0) {
            fprintf(stderr, "SOEM_Interface: Failed to read Statusword after Fault Reset.\n");
            return -1;
        }
        printf("SOEM_Interface: Slave %u Statusword: 0x%04X (Fault bit: %d)\n", slave_idx, statusword_val, (statusword_val >> 3) & 0x01);
        if (!((statusword_val >> 3) & 0x01)) { // Check if Fault bit (bit 3) is 0
            printf("SOEM_Interface: Slave %u Fault cleared.\n", slave_idx);
            break;
        }
        usleep(10000); // Wait 10ms before retrying
        attempts++;
    } while (attempts < max_attempts);

    if (attempts >= max_attempts) {
        fprintf(stderr, "SOEM_Interface: Slave %u failed to clear fault (Statusword: 0x%04X).\n", slave_idx, statusword_val);
        return -1;
    }

    // 1. Set Modes of Operation (0x6060:00)
    printf("SOEM_Interface: Setting Modes of Operation (0x6060:00) to %d...\n", mode_of_operation);
    if (soem_interface_write_sdo(slave_idx, 0x6060, 0x00, sizeof(mode_of_operation), &mode_of_operation) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set Modes of Operation.\n");
        return -1;
    }
    usleep(10000); // Give drive time to process

    // 2. Controlword sequence to "Shutdown" (0x06)
    controlword_val = 0x06; // Shutdown
    printf("SOEM_Interface: Setting Controlword (0x6040:00) to 0x%04X (Shutdown)...\n", controlword_val);
    if (soem_interface_write_sdo(slave_idx, 0x6040, 0x00, sizeof(controlword_val), &controlword_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set Controlword to Shutdown.\n");
        return -1;
    }
    usleep(20000); // Added a slightly longer delay here before first status read

    // Poll Statusword until "Ready to Switch On" (0x21)
    // CiA 402 Statusword bits for "Ready to Switch On":
    // Bit 0 (Ready to switch on) = 1
    // Bit 1 (Switched on) = 0
    // Bit 2 (Operation enabled) = 0
    // Bit 3 (Fault) = 0
    // Bit 4 (Voltage enabled) = 1
    // Bit 5 (Quick stop) = 1 (usually active by default)
    // Bit 6 (Switch on disabled) = 0
    // Expected pattern for Ready to Switch On: Bit 0 (Ready to switch on) = 1, Bit 4 (Voltage Enabled) = 1, Bit 5 (Quick Stop) = 1
    // Other bits should be 0 for this state.
    attempts = 0;
    do {
        if (soem_interface_read_sdo(slave_idx, 0x6041, 0x00, sizeof(statusword_val), &statusword_val) != 0) {
            fprintf(stderr, "SOEM_Interface: Failed to read Statusword during Ready to Switch On check.\n");
            return -1;
        }
        printf("SOEM_Interface: Slave %u Statusword: 0x%04X (Expected Ready to Switch On: 0x21)\n", slave_idx, statusword_val);
        // Check for specific bits: Ready to switch on (0), Voltage enabled (4), Quick stop (5)
        // And ensure Switched on (1), Operation enabled (2), Fault (3), Switch on disabled (6) are 0
        // Simplified check: Focus on bits 0, 1, 2, 3, 4, 5, 6
        // Expected for Ready to Switch On: Bit 0=1, Bit 1=0, Bit 2=0, Bit 3=0, Bit 4=1, Bit 5=1, Bit 6=0
        // Mask 0x7F (bits 0-6)
        // Expected value for 0x21 (0b00100001) would be 0x21. If Quick Stop is also active (bit 5), it's 0x21 | 0x20 = 0x31.
        // Let's check for the *minimum* required bits for "Ready to Switch On" (bit 0 and bit 4) and ensure no fault (bit 3).
        if ((statusword_val & 0x01) && // Bit 0 (Ready to switch on) is 1
            ((statusword_val >> 4) & 0x01) && // Bit 4 (Voltage enabled) is 1
            !((statusword_val >> 3) & 0x01)) // Bit 3 (Fault) is 0
        {
            printf("SOEM_Interface: Slave %u is Ready to Switch On.\n", slave_idx);
            break;
        }
        usleep(10000); // Wait 10ms before retrying
        attempts++;
    } while (attempts < max_attempts);

    if (attempts >= max_attempts) {
        fprintf(stderr, "SOEM_Interface: Slave %u failed to reach Ready to Switch On state (Statusword: 0x%04X).\n", slave_idx, statusword_val);
        return -1;
    }

    // 3. Controlword sequence to "Switch On" (0x07)
    controlword_val = 0x07; // Switch On
    printf("SOEM_Interface: Setting Controlword (0x6040:00) to 0x%04X (Switch On)...\n", controlword_val);
    if (soem_interface_write_sdo(slave_idx, 0x6040, 0x00, sizeof(controlword_val), &controlword_val) != 0) {
        fprintf(stderr, "SOEM_Interface: Failed to set Controlword to Switch On.\n");
        return -1;
    }
    usleep(10000); // Give drive time to process

    // Poll Statusword until "Switched On" (0x23)
    // CiA 402 Statusword bits for "Switched On":
    // Bit 0 (Ready to switch on) = 1
    // Bit 1 (Switched on) = 1
    // Bit 2 (Operation enabled) = 0
    // Bit 3 (Fault) = 0
    // Bit 4 (Voltage enabled) = 1
    // Bit 5 (Quick stop) = 1 (usually active by default)
    // Bit 6 (Switch on disabled) = 0
    // Expected pattern for Switched On: Bit 0=1, Bit 1=1, Bit 4=1, Bit 5=1
    // Other bits should be 0 for this state.
    attempts = 0;
    do {
        if (soem_interface_read_sdo(slave_idx, 0x6041, 0x00, sizeof(statusword_val), &statusword_val) != 0) {
            fprintf(stderr, "SOEM_Interface: Failed to read Statusword during Switch On check.\n");
            return -1;
        }
        printf("SOEM_Interface: Slave %u Statusword: 0x%04X (Expected Switched On: 0x23)\n", slave_idx, statusword_val);
        // Check for specific bits: Ready to switch on (0), Switched on (1), Voltage enabled (4), Quick stop (5)
        // And ensure Operation enabled (2), Fault (3), Switch on disabled (6) are 0
        // Simplified check: Focus on bits 0, 1, 3, 4.
        // Expected for 0x23 (0b00100011) would be 0x23. If Quick Stop is also active (bit 5), it's 0x23 | 0x20 = 0x33.
        if ((statusword_val & 0x01) && // Bit 0 (Ready to switch on) is 1
            ((statusword_val >> 1) & 0x01) && // Bit 1 (Switched on) is 1
            ((statusword_val >> 4) & 0x01) && // Bit 4 (Voltage enabled) is 1
            !((statusword_val >> 3) & 0x01)) // Bit 3 (Fault) is 0
        {
            printf("SOEM_Interface: Slave %u is Switched On.\n", slave_idx);
            break;
        }
        usleep(10000); // Wait 10ms before retrying
        attempts++;
    } while (attempts < max_attempts);

    if (attempts >= max_attempts) {
        fprintf(stderr, "SOEM_Interface: Slave %u failed to reach Switched On state (Statusword: 0x%04X).\n", slave_idx, statusword_val);
        return -1;
    }

    printf("SOEM_Interface: Drive modes configured for slave %u.\n", slave_idx);
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
        int slaves_found = ec_config_init(FALSE);
        if (slaves_found > 0) {
            printf("SOEM_Interface: %d slaves found and configured by SOEM.\n", ec_slavecount);

            if (ec_slavecount == 0) {
                fprintf(stderr, "SOEM_Interface: No EtherCAT slaves found after ec_config_init!\n");
                return -1;
            }

            // Configure distributed clocks (if supported/needed)
            ec_configdc();

            // Configure process data (PDOs)
            // This is where SOEM's auto-configuration happens.
            // We will override this later with dynamic PDO mapping.
            ec_config_map(&IOmap);
            // Corrected: Use Obytes and Ibytes instead of Obits and Ibits
            printf("SOEM_Interface: IOmap configured. Output bytes: %d, Input bytes: %d.\n", ec_group[0].Obytes, ec_group[0].Ibytes);


            // Print slave information
            for (i = 1; i <= ec_slavecount; i++) {
                printf("SOEM_Interface: Slave %d: Name=%s, OutputSize=%dbytes, InputSize=%dbytes, State=%d\n",
                       i, ec_slave[i].name, ec_slave[i].Obits / 8, ec_slave[i].Ibits / 8, ec_slave[i].state);
            }

            // Check for Synapticon slave (by name or vendor ID/product code)
            // For now, we assume slave 1 is the Synapticon device.
            if (ec_slavecount >= slave_idx) {
                printf("SOEM_Interface: Mapping PDOs for Synapticon slave (index %d)...\n", slave_idx);

                // Define custom RxPDO mapping for Synapticon ACTILINK-S ---
                // Objects are (Index << 16 | Subindex << 8 | LengthInBits)
                // Example objects (adjust based on your exact needs and device object dictionary)
                uint32_t custom_rx_mapped_objects[] = {
                    0x60400010, // 0x6040:00 Controlword (16 bits)
                    0x60600008, // 0x6060:00 Modes of operation (8 bits)
                    0x60710010, // 0x6071:00 Target torque (16 bits)
                    0x607A0020, // 0x607A:00 Target position (32 bits)
                    0x60FF0020, // 0x60FF:0x00 Target velocity (32 bits)
                    0x60B20010, // 0x60B2:0x00 Torque offset (16 bits)
                    // Add padding if required by LAN9252 (e.g., 0x00000020 for 32-bit dummy)
                    // 0x00000020 // Example padding (Index 0x0000, Subindex 0x00, Length 32 bits)
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
                    0x60770010, // 0x6077:0x00 Torque actual value (16 bits)
                    0x60780010, // 0x6078:0x00 Current actual value (16 bits)
                    // Add padding if required by LAN9252
                    // 0x00000020 // Example padding
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
                // Corrected: Use Obytes and Ibytes instead of Obits and Ibits
                printf("SOEM_Interface: IOmap re-configured after custom PDOs. New Output bytes: %d, Input bytes: %d.\n", ec_group[0].Obytes, ec_group[0].Ibytes);


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

                // --- IMPORTANT: Configure drive modes and controlword after PDO mapping ---
                // Assuming "Cyclic Synchronous Position Mode" (CSP) as an example (mode 8).
                // Adjust this mode based on your Synapticon drive's documentation and desired operation.
                if (soem_interface_configure_drive_modes(slave_idx, 8) != 0) {
                    fprintf(stderr, "SOEM_Interface: Failed to configure drive modes.\n");
                    return -1;
                }

            } else {
                fprintf(stderr, "SOEM_Interface: Synapticon slave (index %d) not found.\n", slave_idx);
                return -1;
            }

            // Go to Operational state
            printf("SOEM_Interface: Requesting Operational state for all slaves...\n");
            // Increased timeout as drive state transitions can take longer
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 5);

            // Check if all slaves are operational
            int all_slaves_operational = 1;
            for (i = 1; i <= ec_slavecount; i++) {
                if (!is_slave_operational(i)) {
                    printf("SOEM_Interface: Slave %d not operational. Current state: %d\n", i, ec_slave[i].state);
                    all_slaves_operational = 0;
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
                    printf("SOEM_Interface: Slave %d: Current State=%d (Operational=%s)\n",
                           i, ec_slave[i].state, is_slave_operational(i) ? "YES" : "NO");
                }
                return -1;
            }
        } else {
            fprintf(stderr, "SOEM_Interface: No slaves found on %s after ec_config_init.\n", ifname);
            return -1;
        }
    } else {
        fprintf(stderr, "SOEM_Interface: No EtherCAT master found on %s (ec_init failed).\n", ifname);
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
