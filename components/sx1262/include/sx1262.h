#include <string.h> // for memset()
#include "driver/spi_master.h" // for SPI
#include "freertos/FreeRTOS.h" // for vTaskDelay()
#include "freertos/task.h"
#include "freertos/queue.h" // Queues
#include "math.h" // for pow()
#include "driver/gpio.h" // For NSS or RESET

/*
 * OPCODES
 */
// Operational Modes
#define OP_SETSTANDBY                   0x80
#define OP_SETRX                        0x82
#define OP_SETTX                        0x83
#define OP_SETSLEEP                     0x84
#define OP_CALIBRATE                    0x89
#define OP_SETRXTXFALLBACKMODE          0x93
#define OP_SETRXDUTYCYCLE               0x94
#define OP_SETPACONFIG                  0x95
#define OP_SETREGULATORMODE             0x96
#define OP_CALIBRATEIMAGE               0x98
#define OP_STOPTIMERONPREAMBLE          0x9F
#define OP_SETFS                        0xC1
#define OP_SETCAD                       0xC5
#define OP_SETTXCONTINUOUSWAVE          0xD1
#define OP_SETTXINFINITEPREAMBLE        0xD2

// Register and Buffer
#define OP_WRITEREGISTER                0x0D
#define OP_WRITEBUFFER                  0x0E
#define OP_READREGISTER                 0x1D
#define OP_READBUFFER                   0x1E

// DIO and IRQ
#define OP_CLEARIRQSTATUS               0x02
#define OP_SETDIOIRQPARAMS              0x08
#define OP_GETIRQSTATUS                 0x12
#define OP_SETDIO3ASTCXOCTRL            0x97
#define OP_SETDIO2ASRFSWITCHCTRL        0x9D

// Register and Buffer
#define OP_SETRFFREQUENCY               0x86
#define OP_GETPACKETTYPE                0x11
#define OP_SETPACKETTYPE                0x8A
#define OP_SETTXPARAMS                  0x8E
#define OP_SETCADPARAMS                 0x88
#define OP_SETMODULATIONPARAMS          0x8B
#define OP_SETPACKETPARAMS              0x8C
#define OP_SETBUFFERBASEADDRESS         0x8F
#define OP_SETLORASYMBNUMTIMEOUT        0xA0

// Register and Buffer
#define OP_RESETSTATS                   0x00
#define OP_CLEARDEVICEERRORS            0x07
#define OP_GETSTATS                     0x10
#define OP_GETRXBUFFERSTATUS            0x13
#define OP_GETPACKETSTATUS              0x14
#define OP_GETRSSIINST                  0x15
#define OP_GETDEVICEERRORS              0x17
#define OP_GETSTATUS                    0xC0

/*
 * REGISTERS
 */
#define REG_HOPPINGENABLE               0x385
#define REG_PACKETLENGTH                0x386
#define REG_NBHOPPINGBLACKS             0x387
#define REG_NBSYMBOLS                   0x388
#define REG_FREQ                        0x38A
#define REG_DIOX_OUTPUT_ENABLE          0x580
#define REG_DIOX_INPUT_ENABLE           0x583
#define REG_DIOX_PULL_UP_CONTROL        0x584
#define REG_DIOX_PULL_DOWN_CONTROL      0x585
#define REG_WHITENING_INITIAL_VALUE_MSB 0x6B8
#define REG_WHITENING_INITIAL_VALUE_LSB 0x6B9
#define REG_CRC_MSB_INITIAL_VALUE       0x6BC
#define REG_CRC_LSB_INITIAL_VALUE       0x6BD
#define REG_CRC_MSB_POLYNOMIAL_VALUE    0x6BE
#define REG_CRC_LSB_POLYNOMIAL_VALUE    0x6BF
#define REG_SYNCWORD                    0x6C0
#define REG_NODE_ADDRESS                0x6CD
#define REG_BROADCAST_ADDRESS           0x6CE
#define REG_IQ_POLARITY_SETUP           0x736
#define REG_LORA_SYNCWORD_MSB           0x740
#define REG_LORA_SYNCWORD_LSB           0x741
#define REG_LORA_CODING_RATE_RX         0x749
#define REG_LORA_CRC_CONFIGURATION_RX   0x76B
#define REG_DCCCTRL                     0x805
#define REG_MIXCTRL                     0x806
#define REG_MIXMODE                     0x818
#define REG_IF_FREQ                     0x88F
#define REG_RANDOMNUMBERGEN             0x819
#define REG_TXMODULATION                0x889
#define REG_RX_GAIN                     0x8AC
#define REG_TXCLAMPCONFIG               0x8D8
#define REG_OCP_CONFIGURATION           0x8E7
#define REG_RTC_CONTROL                 0x902
#define REG_XTA_TRIM                    0x911
#define REG_XTB_TRIM                    0x912
#define REG_DIO3_OUTPUT_VOLTAGE_CONTROL 0x920
#define REG_EVENT_MASK                  0x944

/*
 * SETTING MACROS
 */
#define STDBY_RC    0
#define STDBY_XOSC  1
#define RX_SINGLE_MODE 0x000000
#define RX_CONTINUOUS_MODE 0xFFFFFF
#define REGULATOR_LDO_ONLY 0
#define REGULATOR_LDO_DC_DC 1
#define CALIBRATE_RC64K 1 << 0
#define CALIBRATE_RC13M 1 << 1
#define CALIBRATE_PLL 1 << 2
#define CALIBRATE_ADC_PULSE 1 << 3
#define CALIBRATE_ADC_BULK_N 1 << 4
#define CALIBRATE_ADC_BULK_P 1 << 5
#define CALIBRATE_IMAGE 1 << 6
#define PA_SX1261_15DBM 0x06, 0x00, 0x01, 0x01
#define PA_SX1261_14DBM 0x04, 0x00, 0x01, 0x01
#define PA_SX1261_10DBM 0x01, 0x00, 0x01, 0x01
#define PA_SX1262_22DBM 0x04, 0x07, 0x00, 0x01
#define PA_SX1262_20DBM 0x03, 0x05, 0x00, 0x01
#define PA_SX1262_17DBM 0x02, 0x03, 0x00, 0x01
#define PA_SX1262_14DBM 0x02, 0x02, 0x00, 0x01
#define FALLBACK_FS 0x40
#define FALLBACK_STDBY_XOSC 0x30
#define FALLBACK_STDBY_RC 0x20
#define TCXOVOLTAGE_1_6V 0
#define TCXOVOLTAGE_1_7V 1
#define TCXOVOLTAGE_1_8V 2
#define TCXOVOLTAGE_2_2V 3
#define TCXOVOLTAGE_2_4V 4
#define TCXOVOLTAGE_2_7V 5
#define TCXOVOLTAGE_3_0V 6
#define TCXOVOLTAGE_3_3V 7
#define PACKET_TYPE_GFSK 0x00
#define PACKET_TYPE_LORA 0x01
#define PACKET_TYPE_LR_FHSS 0x03
#define SET_RAMP_10U   0x00 
#define SET_RAMP_20U   0x01
#define SET_RAMP_40U   0x02
#define SET_RAMP_80U   0x03
#define SET_RAMP_200U  0x04
#define SET_RAMP_800U  0x05
#define SET_RAMP_1700U 0x06
#define SET_RAMP_3400U 0x07
#define PULSESHAPE_NONE 0x00
#define PULSESHAPE_BT_0_3 0x08
#define PULSESHAPE_BT_0_5 0x09
#define PULSESHAPE_BT_0_7 0x0A
#define PULSESHAPE_BT_1_0 0x0B

#define RX_BW_4800 0x1F
#define RX_BW_5800 0x17
#define RX_BW_7300 0x0F
#define RX_BW_9700 0x1E
#define RX_BW_11700 0x16
#define RX_BW_14600 0x0E
#define RX_BW_19500 0x1D
#define RX_BW_23400 0x15
#define RX_BW_29300 0x0D
#define RX_BW_39000 0x1C
#define RX_BW_46900 0x14
#define RX_BW_58600 0x0C
#define RX_BW_78200 0x1B
#define RX_BW_93800 0x13
#define RX_BW_117300 0x0B
#define RX_BW_156200 0x1A
#define RX_BW_187200 0x12
#define RX_BW_234300 0x0A
#define RX_BW_312000 0x19
#define RX_BW_373600 0x11
#define RX_BW_467000 0x09
#define LORA_SF5 0x05
#define LORA_SF6 0x06
#define LORA_SF7 0x07
#define LORA_SF8 0x08
#define LORA_SF9 0x09
#define LORA_SF10 0x0A
#define LORA_SF11 0x0B
#define LORA_SF12 0x0C

#define LORA_BW_7 0x00
#define LORA_BW_10 0x08
#define LORA_BW_15 0x01
#define LORA_BW_20 0x09
#define LORA_BW_31 0x02
#define LORA_BW_41 0x0A
#define LORA_BW_62 0x0£
#define LORA_BW_125 0x04
#define LORA_BW_250 0x05
#define LORA_BW_500 0x06

#define LORA_CR_4_5 0x01
#define LORA_CR_4_6 0x02
#define LORA_CR_4_7 0x03
#define LORA_CR_4_8 0x04
#define LORA_CR_4_5_LI 0x05
#define LORA_CR_4_6_LI 0x06
#define LORA_CR_4_8_LI 0x07

#define PREAMBLE_OFF 0x00 
#define PREAMBLE_SIZE_8 0x04
#define PREAMBLE_SIZE_16 0x05
#define PREAMBLE_SIZE_24 0x06
#define PREAMBLE_SIZE_32 0x07

#define ADDRESS_FILTER_OFF 0x00
#define ADDRESS_FILTER_NODE_ONLY 0x01
#define ADDRESS_FILTER_NODE_BROADCAST 0x02

#define LENGTH_FIXED 0x00
#define LENGTH_VARIABLE 0x01

#define CRC_OFF 0x01
#define CRC_1_BYTE 0x00
#define CRC_2_BYTE 0x02
#define CRC_1_BYTE_INV 0x04
#define CRC_2_BYTE_INV 0x06

#define WHITENING_OFF 0x00
#define WHITENING_ON 0x01

#define HEADER_VARIABLE 0x00
#define HEADER_FIXED 0x01

#define STANDARD_IQ_SETUP 0x00
#define INVERTED_IQ_SETUP 0x01

#define CAD_ON_1_SYMB 0x00
#define CAD_ON_2_SYMB 0x01
#define CAD_ON_4_SYMB 0x02
#define CAD_ON_8_SYMB 0x03
#define CAD_ON_16_SYMB 0x04

#define CAD_ONLY 0x00
#define CAD_RX 0x01

#define IRQ_TXDONE 1 << 0
#define IRQ_RXDONE 1 << 1
#define IRQ_PREAMBLEDETECTED 1 << 2
#define IRQ_SYNCWORDVALID 1 << 3
#define IRQ_HEADERVALID 1 << 4
#define IRQ_HEADERERR 1 << 5
#define IRQ_CRCERR 1 << 6
#define IRQ_CADDONE 1 << 7
#define IRQ_CADDETECTED 1 << 8
#define IRQ_TIMEOUT 1 << 9
#define IRQ_LRFHSSHOP 1 << 14

/*
 * STATUS CODES
 */
#define STATUS_DATA_AVAILABLE 0x2 
#define STATUS_COMMAND_TIMEOUT 0x3
#define STATUS_COMMAND_PROCESSING_ERROR 0x4 
#define STATUS_COMMAND_EXECUTION_FAILURE 0x5 
#define STATUS_COMMAND_TX_DONE 0x6

#define STATUS_STBY_RC 0x2
#define STATUS_STBY_XOSC 0x3 
#define STATUS_FS 0x4
#define STATUS_RX 0x5
#define STATUS_TX 0x6

#define RXSTATUS_FSK_PKT_SENT 1 << 0
#define RXSTATUS_FSK_PKT_RECEIVED 1 << 1
#define RXSTATUS_FSK_ABORT_ERR 1 << 2
#define RXSTATUS_FSK_LENGTH_ERR 1 << 3
#define RXSTATUS_FSK_CRC_ERR 1 << 4
#define RXSTATUS_FSK_ADRS_ERR 1 << 5
#define RXSTATUS_FSK_SYNC_ERR 1 << 6
#define RXSTATUS_FSK_PREAMBLE_ERR 1 << 7

#define OPERROR_RC64K_CALIB_ERR 1 << 0
#define OPERROR_RC13M_CALIB_ERR 1 << 1
#define OPERROR_PLL_CALIB_ERR 1 << 2
#define OPERROR_ADC_CALIB_ERR 1 << 3
#define OPERROR_IMG_CALIB_ERR 1 << 4
#define OPERROR_XOSC_START_ERR 1 << 5
#define OPERROR_PLL_LOCK_ERR 1 << 6
#define OPERROR_PA_RAMP_ERR 1 << 8

/**
 * SPI transaction to receive data from an address
 * 
 * @param spi SPI Handle
 * @param busy GPIO pin read to indicate if chip is busy
 * @param cmd Opcode to be executed
 * @param addrlen Length of address in bytes
 * @param addr Address to call
 * @param rx_op Flag to indicate a receive transaction, otherwise a transmit
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
uint8_t sx126x_spi_transaction(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t addrlen, uint16_t addr, bool rx_op, uint8_t len, uint8_t *buf);

/**
 * SPI transaction to execute an opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 */
void sx126x_cmd(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd);

/**
 * SPI transaction to transmit byte via an opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param buf Byte to transmit
 */
void sx126x_cmd_tx(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t buf);

/**
 * SPI transaction to transmit data via an opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_cmd_txbuf(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t len, uint8_t *buf);

/**
 * SPI transaction to receive data via an opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_cmd_rxbuf(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t len, uint8_t *buf);

/**
 * SPI transaction to transmit data to an address
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param addrlen Length of address in bytes
 * @param addr Address to call
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_cmd_addr_txbuf(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t addrlen, uint16_t addr, uint8_t len, uint8_t *buf);

/**
 * SPI transaction to receive data from an address
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param addrlen Length of address in bytes
 * @param addr Address to call
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_cmd_addr_rxbuf(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t addrlen, uint16_t addr, uint8_t len, uint8_t *buf);

/**
 * 0x02 - ClearIRQStatus - Clear IRQ Status for main IRQ
 * 
 * @param spi SPI Handle
 */
void clearIRQStatus(spi_device_handle_t spi, gpio_num_t busy);

/**
 * 0x07 - ClearDeviceErrors - Clear device errors
 * 
 * @param spi SPI Handle
 */
void clearDeviceErrors(spi_device_handle_t spi, gpio_num_t busy);

/**
 * 0x08 - SetDIOIRQParams - Set DIO IRQ Parameters
 * 
 * @param spi SPI Handle
 * @param irq_main 16-bit mask for main IRQ
 * @param irq_dio1 16-bit mask for DIO1
 * @param irq_dio2 16-bit mask for DIO2
 * @param irq_dio3 16-bit mask for DIO3
 */
void setDioIrqParams(spi_device_handle_t spi, gpio_num_t busy, uint16_t irq_main, uint16_t irq_dio1, uint16_t irq_dio2, uint16_t irq_dio3 );

/**
 * 0x0D - WriteRegister - Transmit data to a register address
 * 
 * @param spi SPI Handle
 * @param addr Register address to write to
 * @param length Length of data to be transferred
 * @param data Pointer to data storage
 */
void writeRegister(spi_device_handle_t spi, gpio_num_t busy, uint16_t addr, uint8_t length, uint8_t *data);

/**
 * 0x12 - GetIRQStatus - Get IRQ Status for main IRQ
 * 
 * @param spi SPI Handle
 * @return `uint16_t` - IRQ state
 */
uint16_t getIRQStatus(spi_device_handle_t spi, gpio_num_t busy);

/**
 * 0x13 - GetRxBufferStatus - Return the status of the buffer from an RX perspective
 * 
 * @param spi SPI Handle
 * @returns `uint8_t | uint8_t` - length, offset
 */
uint16_t getRxBufferStatus(spi_device_handle_t spi, gpio_num_t busy);

/**
 * 0x14 - GetPacketStatus - Return the status of the packet engine, but in reverse byte order
 * 
 * @param spi SPI Handle
 * @returns `uint8_t | uint8_t | uint8t` - RssiAvg, RssiSync, RxStatus
 */
uint32_t getPacketStatus(spi_device_handle_t spi, gpio_num_t busy);

/**
 * 0x1D - ReadRegister - Receive data from a register address
 * 
 * @param spi SPI Handle
 * @param addr Register address to read from
 * @param length Length of data to be transferred
 * @param data Pointer to data storage
 */
void readRegister(spi_device_handle_t spi, gpio_num_t busy, uint16_t addr, uint8_t length, uint8_t *data);

/**
 * 0x1E - ReadBuffer - Receive data from an offset in the buffer
 * 
 * @param spi SPI Handle
 * @param offset Offset from start position in buffer
 * @param length Length of data to be transferred
 * @param data Pointer to data storage
 */
void readBuffer(spi_device_handle_t spi, gpio_num_t busy, uint8_t offset, uint8_t length, uint8_t *data);

/**
 * 0x1F - SetBufferBaseAddress - Set the boundaries in the buffer for tx and rx
 * 
 * @param spi SPI Handle
 * @param tx_addr TX base address in buffer
 * @param rx_addr RX base address in buffer
 */
void setBufferBaseAddress(spi_device_handle_t spi, gpio_num_t busy, uint8_t tx_addr, uint8_t rx_addr);

/**
 * 0x80 - SetStandby - Puts the chip into a specific mode
 * 
 * @param spi SPI Handle
 * @param stdbyconfig Standby mode
 */
void setStandby(spi_device_handle_t spi, gpio_num_t busy, uint8_t stdbyconfig) ;

/**
 * 0x82 - SetRx - Put chip into receive mode
 * 
 * @param spi SPI Handle
 * @param timeout Timeout for 15.625us. 0 = wait, 0xFFFFFE = 262.1 secs, 0xFFFFFF = continuous)
 */
void setRx(spi_device_handle_t spi, gpio_num_t busy, uint32_t timeout);

/**
 * 0x86 - SetRFFrequency
 * 
 * @param spi SPI Handle
 * @param rffreq Frequency in Hz
 */
void setRfFrequency(spi_device_handle_t spi, gpio_num_t busy, uint32_t rffreq);

/**
 * 0x8A - SetPacketType - Sets the mode of operation
 * 
 * @param spi SPI Handle
 * @param packet_type Use GFSK, LoRA or Long Range FHSS
 */
void setPacketType(spi_device_handle_t spi, gpio_num_t busy, uint8_t packet_type);

/**
 * 0x8B - SetModulationParams - Set the modulation parameters
 * 
 * @param spi SPI Handle
 * @param bitrate Transmission speed in bits/sec
 * @param pulseshape Predefined transmission gaussian pulseshapes
 * @param rx_bw Predefined reception bandwidth
 * @param freqdev Frequency deviation. Usually proportional to bitrate
 */
void setModulationParams(spi_device_handle_t spi, gpio_num_t busy, uint32_t bitrate, uint8_t pulseshape, uint8_t rx_bw, uint32_t freqdev);

/**
 * 0x8C - SetDIOIRQParams - Set DIO IRQ Parameters
 * 
 * @param spi SPI Handle
 * @param tx_preamble Transmit preamble size in bits
 * @param rx_preamble Preset preamble. Must be less than syncword.
 * @param syncword_bits Syncword size in bits
 * @param addr_filter Whether to filter on node and/or broadcast address
 * @param len_type Static or variable length
 * @param len_bytes Max length of payload in bytes
 * @param crc_type Selects the CRC type
 * @param whitening Disables / Enables whitening
 */
void setPacketParams(spi_device_handle_t spi, gpio_num_t busy, uint16_t tx_preamble, uint8_t rx_preamble, uint8_t syncword_bits, uint8_t addr_filter, uint8_t len_type, uint8_t len_bytes, uint8_t crc_type, uint8_t whitening);

/**
 * 0x97 - SetDIO3AsTCXOCtrl - Use DIO3 for TCXO power
 * 
 * @param spi SPI Handle
 * @param voltage Predefined voltage profile
 * @param delay Wait for voltage increase in 15.625us increments (max 1 sec / 0xFFFF)
 */
void setDio3AsTCXOCtrl(spi_device_handle_t spi, gpio_num_t busy, uint8_t voltage, uint16_t delay);

/**
 * 0x9D - SetDIO2AsRfSwitchCtrl - DIO2 controls the RF switch for TX, RX, etc
 * 
 * @param spi SPI Handle
 * @param enable DIO2 controls RF switch operation
 */
void setDio2AsRfSwitchCtrl(spi_device_handle_t spi, gpio_num_t busy, uint8_t enable);

/**
 * 0xC0 - GetStatus - Get the status with some output
 * 
 * @param spi SPI Handle
 * @param verbose Boolean flag to ask for printf() output
 * @return `uint8_t` Status output
 */
uint8_t getStatus(spi_device_handle_t spi, gpio_num_t busy, bool verbose);