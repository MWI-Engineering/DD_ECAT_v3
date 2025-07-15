Slave:1
 Name:SOMANET
 Output size: 280bits
 Input size: 376bits
 State: 4
 Delay: 0[ns]
 Has DC: 1
 DCParentport:0
 Activeports:0.1.0.0
 Configured address: 1001
 Man: 000022d2 ID: 00000401 Rev: 0f000005
 SM0 A:1000 L: 128 F:00010026 Type:1
 SM1 A:1080 L: 128 F:00010022 Type:2
 SM2 A:1100 L:  35 F:00010064 Type:3
 SM3 A:1d00 L:  47 F:00010020 Type:4
 FMMU0 Ls:00000000 Ll:  35 Lsb:0 Leb:7 Ps:1100 Psb:0 Ty:02 Act:01
 FMMU1 Ls:00000023 Ll:  47 Lsb:0 Leb:7 Ps:1d00 Psb:0 Ty:01 Act:01
 FMMUfunc 0:1 1:2 2:3 3:0
 MBX length wr: 128 rd: 128 MBX protocols : 0e
 CoE details: 1f FoE details: 01 EoE details: 01 SoE details: 00
 Ebus current: 0[mA]
 only LRD/LWR:0
PDO mapping according to CoE :
  SM2 outputs
     addr b   index: sub bitl data_type    name
  [0x0000.0] 0x6040:0x00 0x10 UNSIGNED16   Controlword
  [0x0002.0] 0x6060:0x00 0x08 INTEGER8     Modes of operation
  [0x0003.0] 0x6071:0x00 0x10 INTEGER16    Target Torque
  [0x0005.0] 0x607A:0x00 0x20 INTEGER32    Target position
  [0x0009.0] 0x60FF:0x00 0x20 INTEGER32    Target velocity
  [0x000D.0] 0x60B2:0x00 0x10 INTEGER16    Torque offset
  [0x000F.0] 0x2701:0x00 0x20 UNSIGNED32   Tuning command
  [0x0013.0] 0x60FE:0x01 0x20 UNSIGNED32   Physical outputs
  [0x0017.0] 0x60FE:0x02 0x20 UNSIGNED32   Bit mask
  [0x001B.0] 0x2703:0x00 0x20 UNSIGNED32   User MOSI
  [0x001F.0] 0x60B1:0x00 0x20 INTEGER32    Velocity offset
  SM3 inputs
     addr b   index: sub bitl data_type    name
  [0x0023.0] 0x6041:0x00 0x10 UNSIGNED16   Statusword
  [0x0025.0] 0x6061:0x00 0x08 INTEGER8     Modes of operation display
  [0x0026.0] 0x6064:0x00 0x20 INTEGER32    Position actual value
  [0x002A.0] 0x606C:0x00 0x20 INTEGER32    Velocity actual value
  [0x002E.0] 0x6077:0x00 0x10 INTEGER16    Torque actual value
  [0x0030.0] 0x2401:0x00 0x10 UNSIGNED16   Analog input 1
  [0x0032.0] 0x2402:0x00 0x10 UNSIGNED16   Analog input 2
  [0x0034.0] 0x2403:0x00 0x10 UNSIGNED16   Analog input 3
  [0x0036.0] 0x2404:0x00 0x10 UNSIGNED16   Analog input 4
  [0x0038.0] 0x2702:0x00 0x20 UNSIGNED32   Tuning status
  [0x003C.0] 0x60FD:0x00 0x20 UNSIGNED32   Digital inputs
  [0x0040.0] 0x2704:0x00 0x20 UNSIGNED32   User MISO
  [0x0044.0] 0x20F0:0x00 0x20 UNSIGNED32   Timestamp
  [0x0048.0] 0x60FC:0x00 0x20 INTEGER32    Position demand internal value
  [0x004C.0] 0x606B:0x00 0x20 INTEGER32    Velocity demand value
  [0x0050.0] 0x6074:0x00 0x10 INTEGER16    Torque demand

## Used in soem_interface.c

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

                // --- Example: Define custom TxPDO mapping for Synapticon ACTILINK-S ---
                uint32_t custom_tx_mapped_objects[] = {
                    0x60410010, // 0x6041:00 Statusword (16 bits)
                    0x60610008, // 0x6061:00 Modes of operation display (8 bits)
                    0x60640020, // 0x6064:00 Position actual value (32 bits)
                    0x60690020, // 0x6069:00 Velocity actual value (32 bits)
                    0x60770010, // 0x6077:00 Torque actual value (16 bits)
                    0x60780010, // 0x6078:00 Current actual value (16 bits)
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
