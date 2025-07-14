// soem_interface.c - Fixed version with proper PDO configuration
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

// Scaling factors
#define POSITION_SCALE_FACTOR (360.0f / 1000000.0f)
#define VELOCITY_SCALE_FACTOR (1.0f / 1000.0f)
#define TORQUE_SCALE_FACTOR (1000.0f)

// SOEM hook for printing messages
void ecat_print_func(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}

// PDO configuration function for SOMANET
int configure_somanet_pdo(uint16 slave)
{
    int retval = 0;
    uint16 u16val;
    uint32 u32val;
    int wkc_result;
    
    printf("SOEM_Interface: Configuring PDO for SOMANET slave %d...\n", slave);
    
    // --- Start SDO Writes for Custom PDO Configuration (in Pre-Operational State) ---
    // Slave 1 is the Synapticon ACTILINK-S
    int slave_id = 1; // IMPORTANT: Adjust this if your Synapticon drive is a different slave ID

    printf("SOEM_Interface: Configuring custom PDO mappings for Slave %d...\n", slave_id);
    int sdo_wkc; // Working counter for SDO writes

    // 1. Clear existing RxPDO assignments for Sync Manager 2 (0x1C12)
    // Writing 0 to subindex 0 clears all entries in the assignment object.
    sdo_wkc = ec_SDOwrite(slave_id, 0x1C12, 0x00, FALSE, 1, &ov0, EC_TIMEOUTRXM);
    if (sdo_wkc == 0) {
        printf("SOEM_Interface: ERROR - Failed to clear RxPDO assignments (0x1C12:00). sdo_wkc=%d\n", sdo_wkc);
        // Consider adding more robust error handling here, e.g., return an error code or exit.
    } else {
        printf("SOEM_Interface: RxPDO assignments cleared.\n");
    }

    // 2. Clear existing TxPDO assignments for Sync Manager 3 (0x1C13)
    sdo_wkc = ec_SDOwrite(slave_id, 0x1C13, 0x00, FALSE, 1, &ov0, EC_TIMEOUTRXM);
    if (sdo_wkc == 0) {
        printf("SOEM_Interface: ERROR - Failed to clear TxPDO assignments (0x1C13:00). sdo_wkc=%d\n", sdo_wkc);
        // Handle error
    } else {
        printf("SOEM_Interface: TxPDO assignments cleared.\n");
    }

    // 3. Configure RxPDO Mapping (0x1600)
    // Clear existing mapping for PDO 0x1600 before writing new entries.
    sdo_wkc = ec_SDOwrite(slave_id, 0x1600, 0x00, FALSE, 1, &ov0, EC_TIMEOUTRXM);
    if (sdo_wkc == 0) {
        printf("SOEM_Interface: ERROR - Failed to clear RxPDO mapping (0x1600:00). sdo_wkc=%d\n", sdo_wkc);
        // Handle error
    }

    // Write each entry to RxPDO 0x1600 subindices (1 to RX_PDO_MAP_COUNT)
    for (int i = 0; i < RX_PDO_MAP_COUNT; i++) {
        sdo_wkc = ec_SDOwrite(slave_id, 0x1600, i + 1, FALSE, 4, &rx_pdo_mapping[i], EC_TIMEOUTRXM);
        if (sdo_wkc == 0) {
            printf("SOEM_Interface: ERROR - Failed to write RxPDO map entry %d (0x1600:%02X). sdo_wkc=%d\n", i + 1, i + 1, sdo_wkc);
            // Handle error
        }
    }
    // Set the number of mapped objects for RxPDO 0x1600 (subindex 0)
    uint8_t rx_map_count_u8 = RX_PDO_MAP_COUNT;
    sdo_wkc = ec_SDOwrite(slave_id, 0x1600, 0x00, FALSE, 1, &rx_map_count_u8, EC_TIMEOUTRXM);
    if (sdo_wkc == 0) {
        printf("SOEM_Interface: ERROR - Failed to set RxPDO map count (0x1600:00). sdo_wkc=%d\n", sdo_wkc);
        // Handle error
    } else {
        printf("SOEM_Interface: RxPDO mapping configured.\n");
    }

    // 4. Configure TxPDO Mapping (0x1A00)
    // Clear existing mapping for PDO 0x1A00 before writing new entries.
    sdo_wkc = ec_SDOwrite(slave_id, 0x1A00, 0x00, FALSE, 1, &ov0, EC_TIMEOUTRXM);
    if (sdo_wkc == 0) {
        printf("SOEM_Interface: ERROR - Failed to clear TxPDO mapping (0x1A00:00). sdo_wkc=%d\n", sdo_wkc);
        // Handle error
    }

    // Write each entry to TxPDO 0x1A00 subindices (1 to TX_PDO_MAP_COUNT)
    for (int i = 0; i < TX_PDO_MAP_COUNT; i++) {
        sdo_wkc = ec_SDOwrite(slave_id, 0x1A00, i + 1, FALSE, 4, &tx_pdo_mapping[i], EC_TIMEOUTRXM);
        if (sdo_wkc == 0) {
            printf("SOEM_Interface: ERROR - Failed to write TxPDO map entry %d (0x1A00:%02X). sdo_wkc=%d\n", i + 1, i + 1, sdo_wkc);
            // Handle error
        }
    }
    // Set the number of mapped objects for TxPDO 0x1A00 (subindex 0)
    uint8_t tx_map_count_u8 = TX_PDO_MAP_COUNT;
    sdo_wkc = ec_SDOwrite(slave_id, 0x1A00, 0x00, FALSE, 1, &tx_map_count_u8, EC_TIMEOUTRXM);
    if (sdo_wkc == 0) {
        printf("SOEM_Interface: ERROR - Failed to set TxPDO map count (0x1A00:00). sdo_wkc=%d\n", sdo_wkc);
        // Handle error
    } else {
        printf("SOEM_Interface: TxPDO mapping configured.\n");
    }

    // 5. Assign RxPDO 0x1600 to Sync Manager 2 (0x1C12)
    uint16_t pdo1600 = 0x1600; // The PDO index to assign
    sdo_wkc = ec_SDOwrite(slave_id, 0x1C12, 0x01, FALSE, 2, &pdo1600, EC_TIMEOUTRXM); // Assign 0x1600 to subindex 1
    if (sdo_wkc == 0) {
        printf("SOEM_Interface: ERROR - Failed to assign RxPDO 0x1600 to SM2 (0x1C12:01). sdo_wkc=%d\n", sdo_wkc);
        // Handle error
    }
    uint8_t sm2_assigned_count = 1; // Only one PDO (0x1600) assigned to SM2
    sdo_wkc = ec_SDOwrite(slave_id, 0x1C12, 0x00, FALSE, 1, &sm2_assigned_count, EC_TIMEOUTRXM); // Set count of assigned PDOs
    if (sdo_wkc == 0) {
        printf("SOEM_Interface: ERROR - Failed to set SM2 assigned count (0x1C12:00). sdo_wkc=%d\n", sdo_wkc);
        // Handle error
    } else {
        printf("SOEM_Interface: RxPDO 0x1600 assigned to SM2.\n");
    }

    // 6. Assign TxPDO 0x1A00 to Sync Manager 3 (0x1C13)
    uint16_t pdo1a00 = 0x1A00; // The PDO index to assign
    sdo_wkc = ec_SDOwrite(slave_id, 0x1C13, 0x01, FALSE, 2, &pdo1a00, EC_TIMEOUTRXM); // Assign 0x1A00 to subindex 1
    if (sdo_wkc == 0) {
        printf("SOEM_Interface: ERROR - Failed to assign TxPDO 0x1A00 to SM3 (0x1C13:01). sdo_wkc=%d\n", sdo_wkc);
        // Handle error
    }
    uint8_t sm3_assigned_count = 1; // Only one PDO (0x1A00) assigned to SM3
    sdo_wkc = ec_SDOwrite(slave_id, 0x1C13, 0x00, FALSE, 1, &sm3_assigned_count, EC_TIMEOUTRXM); // Set count of assigned PDOs
    if (sdo_wkc == 0) {
        printf("SOEM_Interface: ERROR - Failed to set SM3 assigned count (0x1C13:00). sdo_wkc=%d\n", sdo_wkc);
        // Handle error
    } else {
        printf("SOEM_Interface: TxPDO 0x1A00 assigned to SM3.\n");
    }

    printf("SOEM_Interface: Custom PDO configuration complete.\n");
    // --- End SDO Writes ---

    // Set slave to PREOP state for configuration
    ec_statecheck(slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
    

    /*
    // --- Configure RxPDO (Outputs from Master to Slave) ---
    
    // Clear existing RxPDO assignments
    u16val = 0;
    wkc_result = ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    if (wkc_result <= 0) {
        printf("SOEM_Interface: Failed to clear RxPDO assignments\n");
        return -1;
    }
    
    // Configure RxPDO 0x1600 mapping
    u16val = 0; // Clear subindex 0 (number of mapped objects)
    wkc_result = ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    if (wkc_result <= 0) {
        printf("SOEM_Interface: Failed to clear RxPDO 0x1600 mapping\n");
        return -1;
    }
    
    // Map RxPDO objects
    uint32 rxpdo_mapping[] = {
        0x60400010, // 0x6040:0x00 - Controlword (16 bits)
        0x60600008, // 0x6060:0x00 - Modes of operation (8 bits)
        0x60710010, // 0x6071:0x00 - Target torque (16 bits)
        0x607A0020, // 0x607A:0x00 - Target position (32 bits)
        0x60FF0020, // 0x60FF:0x00 - Target velocity (32 bits)
        0x60B20010, // 0x60B2:0x00 - Torque offset (16 bits)
        0x27010020, // 0x2701:0x00 - Tuning command (32 bits)
        0x60FE0120, // 0x60FE:0x01 - Physical outputs (32 bits)
        0x60FE0220, // 0x60FE:0x02 - Bit mask (32 bits)
        0x27030020, // 0x2703:0x00 - User MOSI (32 bits)
        0x60B10020  // 0x60B1:0x00 - Velocity offset (32 bits)
    };
    
    for (int i = 0; i < sizeof(rxpdo_mapping)/sizeof(uint32); i++) {
        wkc_result = ec_SDOwrite(slave, 0x1600, i+1, FALSE, sizeof(uint32), &rxpdo_mapping[i], EC_TIMEOUTRXM);
        if (wkc_result <= 0) {
            printf("SOEM_Interface: Failed to map RxPDO object %d (0x%08X)\n", i+1, rxpdo_mapping[i]);
            return -1;
        }
    }
    
    // Set number of mapped objects
    u16val = sizeof(rxpdo_mapping)/sizeof(uint32);
    wkc_result = ec_SDOwrite(slave, 0x1600, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    if (wkc_result <= 0) {
        printf("SOEM_Interface: Failed to set RxPDO mapping count\n");
        return -1;
    }
    
    // --- Configure TxPDO (Inputs from Slave to Master) ---
    
    // Clear existing TxPDO assignments
    u16val = 0;
    wkc_result = ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    if (wkc_result <= 0) {
        printf("SOEM_Interface: Failed to clear TxPDO assignments\n");
        return -1;
    }
    
    // Configure TxPDO 0x1A00 mapping
    u16val = 0; // Clear subindex 0
    wkc_result = ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    if (wkc_result <= 0) {
        printf("SOEM_Interface: Failed to clear TxPDO 0x1A00 mapping\n");
        return -1;
    }
    
    // Map TxPDO objects
    uint32 txpdo_mapping[] = {
        0x60410010, // 0x6041:0x00 - Statusword (16 bits)
        0x60610008, // 0x6061:0x00 - Modes of operation display (8 bits)
        0x60640020, // 0x6064:0x00 - Position actual value (32 bits)
        0x606C0020, // 0x606C:0x00 - Velocity actual value (32 bits)
        0x60770010, // 0x6077:0x00 - Torque actual value (16 bits)
        0x24010010, // 0x2401:0x00 - Analog input 1 (16 bits)
        0x24020010, // 0x2402:0x00 - Analog input 2 (16 bits)
        0x24030010, // 0x2403:0x00 - Analog input 3 (16 bits)
        0x24040010, // 0x2404:0x00 - Analog input 4 (16 bits)
        0x27020020, // 0x2702:0x00 - Tuning status (32 bits)
        0x60FD0020, // 0x60FD:0x00 - Digital inputs (32 bits)
        0x27040020, // 0x2704:0x00 - User MISO (32 bits)
        0x20F00020, // 0x20F0:0x00 - Timestamp (32 bits)
        0x60FC0020, // 0x60FC:0x00 - Position demand internal (32 bits)
        0x606B0020, // 0x606B:0x00 - Velocity demand value (32 bits)
        0x60740010  // 0x6074:0x00 - Torque demand (16 bits)
    };
    
    for (int i = 0; i < sizeof(txpdo_mapping)/sizeof(uint32); i++) {
        wkc_result = ec_SDOwrite(slave, 0x1A00, i+1, FALSE, sizeof(uint32), &txpdo_mapping[i], EC_TIMEOUTRXM);
        if (wkc_result <= 0) {
            printf("SOEM_Interface: Failed to map TxPDO object %d (0x%08X)\n", i+1, txpdo_mapping[i]);
            return -1;
        }
    }
    
    // Set number of mapped objects
    u16val = sizeof(txpdo_mapping)/sizeof(uint32);
    wkc_result = ec_SDOwrite(slave, 0x1A00, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    if (wkc_result <= 0) {
        printf("SOEM_Interface: Failed to set TxPDO mapping count\n");
        return -1;
    }
    */
    
    // --- Assign PDO to sync managers ---
    
    // Assign RxPDO 0x1600 to sync manager 2
    u16val = 0x1600;
    wkc_result = ec_SDOwrite(slave, 0x1C12, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    if (wkc_result <= 0) {
        printf("SOEM_Interface: Failed to assign RxPDO to SM2\n");
        return -1;
    }
    
    // Enable RxPDO assignment
    u16val = 1;
    wkc_result = ec_SDOwrite(slave, 0x1C12, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    if (wkc_result <= 0) {
        printf("SOEM_Interface: Failed to enable RxPDO assignment\n");
        return -1;
    }
    
    // Assign TxPDO 0x1A00 to sync manager 3
    u16val = 0x1A00;
    wkc_result = ec_SDOwrite(slave, 0x1C13, 0x01, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    if (wkc_result <= 0) {
        printf("SOEM_Interface: Failed to assign TxPDO to SM3\n");
        return -1;
    }
    
    // Enable TxPDO assignment
    u16val = 1;
    wkc_result = ec_SDOwrite(slave, 0x1C13, 0x00, FALSE, sizeof(u16val), &u16val, EC_TIMEOUTRXM);
    if (wkc_result <= 0) {
        printf("SOEM_Interface: Failed to enable TxPDO assignment\n");
        return -1;
    }
    
    printf("SOEM_Interface: PDO configuration completed successfully\n");
    return 0;
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
            somanet_outputs->controlword = 0x0F;
        }

        // Send process data to slaves
        ec_send_processdata();
        
        // Receive process data from slaves
        wkc = ec_receive_processdata(EC_TIMEOUTRET);

        if (wkc >= expectedWKC) {
            consecutive_errors = 0;
            
            if (somanet_inputs) {
                pthread_mutex_lock(&pdo_mutex);
                current_position_f = (float)somanet_inputs->position_actual_value * POSITION_SCALE_FACTOR;
                current_velocity_f = (float)somanet_inputs->velocity_actual_value * VELOCITY_SCALE_FACTOR;
                communication_ok = 1;
                pthread_mutex_unlock(&pdo_mutex);

                // Debug output every 1000 cycles
                static int debug_counter = 0;
                if (++debug_counter >= 1000) {
                    printf("EtherCAT: WKC=%d, Status=0x%04X, Pos=%d, Vel=%d, Torque=%d\n",
                           wkc, somanet_inputs->statusword, 
                           somanet_inputs->position_actual_value,
                           somanet_inputs->velocity_actual_value,
                           somanet_inputs->torque_actual_value);
                    debug_counter = 0;
                }

                // Check for drive faults
                if (somanet_inputs->statusword & 0x08) {
                    printf("EtherCAT: Drive fault detected! Statusword: 0x%04X\n", somanet_inputs->statusword);
                }
            }
        } else {
            consecutive_errors++;
            pthread_mutex_lock(&pdo_mutex);
            communication_ok = 0;
            pthread_mutex_unlock(&pdo_mutex);
            
            if (consecutive_errors > MAX_CONSECUTIVE_ERRORS) {
                printf("EtherCAT: Too many consecutive communication errors (%d). Expected WKC: %d, Got: %d\n", 
                       consecutive_errors, expectedWKC, wkc);
                consecutive_errors = 0;
            }
        }

        usleep(1000); // 1ms cycle time
    }

    printf("EtherCAT loop thread stopped.\n");
    return NULL;
}

// Define the RxPDO mapping for SM2 (Master -> Slave)
// This corresponds to what the slave receives (somanet_txpdo_t in your script)
// Format: (Object Index << 16) | (Subindex << 8) | BitLength
uint32_t rx_pdo_mapping[] = {
    0x60400010, // Controlword (0x6040:00, 16 bits)
    0x60600008, // Modes of operation (0x6060:00, 8 bits)
    0x60710010, // Target Torque (0x6071:00, 16 bits)
    0x607A0020, // Target position (0x607A:00, 32 bits)
    0x60FF0020, // Target velocity (0x60FF:00, 32 bits)
    0x60B20010, // Torque offset (0x60B2:00, 16 bits)
    0x27010020, // Tuning command (0x2701:00, 32 bits)
    0x60FE0120, // Physical outputs (0x60FE:01, 32 bits)
    0x60FE0220, // Bit mask (0x60FE:02, 32 bits)
    0x27030020, // User MOSI (0x2703:00, 32 bits)
    0x60B10020  // Velocity offset (0x60B1:00, 32 bits)
};
const uint8_t RX_PDO_MAP_COUNT = sizeof(rx_pdo_mapping) / sizeof(rx_pdo_mapping[0]);

// Define the TxPDO mapping for SM3 (Slave -> Master)
// This corresponds to what the master receives (somanet_rxpdo_t in your script)
// Format: (Object Index << 16) | (Subindex << 8) | BitLength
uint32_t tx_pdo_mapping[] = {
    0x60410010, // Statusword (0x6041:00, 16 bits)
    0x60610008, // Modes of operation display (0x6061:00, 8 bits)
    0x60640020, // Position actual value (0x6064:00, 32 bits)
    0x60690020, // Velocity actual value (0x6069:00, 32 bits)
    0x60770010, // Torque actual value (0x6077:00, 16 bits)
    0x26000120, // Analog input 1 (0x2600:01, 32 bits)
    0x26000220, // Analog input 2 (0x2600:02, 32 bits)
    0x26000320, // Analog input 3 (0x2600:03, 32 bits)
    0x26000420, // Analog input 4 (0x2600:04, 32 bits)
    0x27000020, // Tuning status (0x2700:00, 32 bits)
    0x60FD0020, // Digital inputs (0x60FD:00, 32 bits)
    0x27020020, // User MISO (0x2702:00, 32 bits)
    0x28020020, // Timestamp (0x2802:00, 32 bits)
    0x60FC0020, // Position demand internal value (0x60FC:00, 32 bits)
    0x60F90020, // Velocity demand value (0x60F9:00, 32 bits)
    0x60740010  // Torque demand (0x6074:00, 16 bits)
};
const uint8_t TX_PDO_MAP_COUNT = sizeof(tx_pdo_mapping) / sizeof(tx_pdo_mapping[0]);

// Define a zero byte for SDO writes to subindex 0 for clearing purposes
uint8_t ov0 = 0;

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
            
            // Print detailed slave information
            for (int i = 1; i <= ec_slavecount; i++) {
                printf("Slave %d: name = %s\n", i, ec_slave[i].name);
                printf("  Input size: %d bytes\n", ec_slave[i].Ibytes);
                printf("  Output size: %d bytes\n", ec_slave[i].Obytes);
                printf("  State: 0x%02X\n", ec_slave[i].state);
                printf("  Vendor ID: 0x%08X\n", ec_slave[i].eep_man);
                printf("  Product Code: 0x%08X\n", ec_slave[i].eep_id);
                printf("  Has DC: %s\n", ec_slave[i].hasdc ? "Yes" : "No");
            }

            // Configure PDO for SOMANET slave
            if (configure_somanet_pdo(1) != 0) {
                fprintf(stderr, "SOEM_Interface: Failed to configure PDO for SOMANET slave\n");
                ec_close();
                return -1;
            }

            // Re-configure after PDO setup
            if (ec_config_init(FALSE) > 0) {
                printf("SOEM_Interface: Slaves reconfigured after PDO setup.\n");
                
                // Print updated slave information
                for (int i = 1; i <= ec_slavecount; i++) {
                    printf("Slave %d after PDO config:\n", i);
                    printf("  Input size: %d bytes\n", ec_slave[i].Ibytes);
                    printf("  Output size: %d bytes\n", ec_slave[i].Obytes);
                    printf("  State: 0x%02X\n", ec_slave[i].state);
                }
            }

            // Map PDOs for all slaves
            ec_config_map(&IOmap[0]);
            
            // Get pointers to PDOs for slave 1
            if (ec_slave[1].outputs && ec_slave[1].inputs) {
                somanet_outputs = (SomanetOutputs_t *)ec_slave[1].outputs;
                somanet_inputs = (SomanetInputs_t *)ec_slave[1].inputs;
                
                printf("SOEM_Interface: PDO mapping complete. Slave 1 - Output size: %d bytes, Input size: %d bytes.\n",
                       ec_slave[1].Obytes, ec_slave[1].Ibytes);
            } else {
                fprintf(stderr, "SOEM_Interface: Failed to get PDO pointers for Slave 1.\n");
                ec_close();
                return -1;
            }

            // Wait until all slaves reach SAFE_OP state
            printf("SOEM_Interface: Waiting for all slaves to reach SAFE_OP...\n");

            int wait_count = 0;
            const int max_waits = 100; // wait up to 10 seconds

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
                // Print final states for debugging
                for (int i = 1; i <= ec_slavecount; i++) {
                    printf("  Final slave %d state: 0x%02X\n", i, ec_slave[i].state);
                }
                ec_close();
                return -1;
            }

            // Calculate expected working counter
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("SOEM_Interface: Expected working counter: %d\n", expectedWKC);

            // Clear output PDOs initially
            if (somanet_outputs) {
                memset(somanet_outputs, 0, sizeof(SomanetOutputs_t));
            }

            // Initialize drive state machine
            printf("SOEM_Interface: Initializing drive state machine...\n");
            
            // Set Modes of Operation to Cyclic Synchronous Torque (CST)
            if (somanet_outputs) {
                somanet_outputs->modes_of_operation = 0x0A; // CST mode
                
                // State machine sequence
                const uint16_t state_sequence[] = {0x06, 0x07, 0x0F};
                const char* state_names[] = {"Ready to switch on", "Switch on", "Operation enabled"};
                
                for (int i = 0; i < 3; i++) {
                    somanet_outputs->controlword = state_sequence[i];
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);
                    
                    if (wkc >= expectedWKC && somanet_inputs) {
                        printf("SOEM_Interface: %s - Controlword: 0x%04X, Statusword: 0x%04X\n", 
                               state_names[i], somanet_outputs->controlword, somanet_inputs->statusword);
                    }
                    usleep(100000); // Wait 100ms
                }
            }

            // Request OP state for all slaves
            printf("SOEM_Interface: Requesting OPERATIONAL state...\n");
            ec_statecheck(0, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
            
            // Check if all slaves reached operational state
            int all_operational = 1;
            for (int i = 1; i <= ec_slavecount; i++) {
                printf("SOEM_Interface: Slave %d final state: 0x%02X\n", i, ec_slave[i].state);
                if (ec_slave[i].state != EC_STATE_OPERATIONAL) {
                    all_operational = 0;
                }
            }
            
            if (all_operational) {
                printf("SOEM_Interface: All slaves operational. Starting EtherCAT thread...\n");
            } else {
                printf("SOEM_Interface: Some slaves not operational, but continuing...\n");
            }
            
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
            fprintf(stderr, "SOEM_Interface: No slaves found on %s.\n", ifname);
            ec_close();
            return -1;
        }
    } else {
        fprintf(stderr, "SOEM_Interface: No socket connection on %s. Run with 'sudo' or set CAP_NET_RAW.\n", ifname);
        return -1;
    }
}

// Rest of the functions remain the same...
void soem_interface_send_and_receive_pdo(float target_torque) {
    if (!master_initialized) {
        return;
    }
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
    int status;
    pthread_mutex_lock(&pdo_mutex);
    status = communication_ok;
    pthread_mutex_unlock(&pdo_mutex);
    return status;
}

void soem_interface_stop_master() {
    if (master_initialized) {
        printf("SOEM_Interface: Stopping EtherCAT master...\n");
        
        ecat_thread_running = 0;
        if (ecat_thread) {
            pthread_join(ecat_thread, NULL);
        }
        
        if (somanet_outputs) {
            somanet_outputs->target_torque = 0;
            somanet_outputs->controlword = 0x06;
            ec_send_processdata();
            usleep(10000);
        }
        
        ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
        ec_close();
        
        master_initialized = 0;
        printf("SOEM_Interface: EtherCAT master stopped.\n");
    }
}