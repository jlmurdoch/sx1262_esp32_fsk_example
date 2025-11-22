#include <string.h> // for memset()
#include "driver/spi_master.h" // for SPI
#include "freertos/FreeRTOS.h" // for vTaskDelay()
#include "freertos/task.h"
#include "freertos/queue.h" // Queues
#include "math.h" // for pow()
#include "driver/gpio.h" // For NSS or RESET
#include <unistd.h>

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
#define CALIBRATE_RC64K 1
#define CALIBRATE_RC13M 1 << 1
#define CALIBRATE_PLL 1 << 2
#define CALIBRATE_ADC_PULSE 1 << 3
#define CALIBRATE_ADC_BULK_N 1 << 4
#define CALIBRATE_ADC_BULK_P 1 << 5
#define CALIBRATE_IMAGE 1 << 6
#define PA_SX1261_15DBM 0x06, 0x00, 0x01
#define PA_SX1261_14DBM 0x04, 0x00, 0x01
#define PA_SX1261_10DBM 0x01, 0x00, 0x01
#define PA_SX1262_22DBM 0x04, 0x07, 0x00
#define PA_SX1262_20DBM 0x03, 0x05, 0x00
#define PA_SX1262_17DBM 0x02, 0x03, 0x00
#define PA_SX1262_14DBM 0x02, 0x02, 0x00
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

#define PREAMBLE_RX_OFF 0x00 
#define PREAMBLE_RX_8_BITS 0x04
#define PREAMBLE_RX_16_BITS 0x05
#define PREAMBLE_RX_24_BITS 0x06
#define PREAMBLE_RX_32_BITS 0x07

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

#define IRQ_TXDONE 1
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

#define RX_GAIN_POWER_SAVE 0x94
#define RX_GAIN_POWER_BOOST 0x96

/*
 * STATUS CODES
 */
// These are compound bit usage and exclusive
#define STATUS_DATA_AVAILABLE 0x02 << 1
#define STATUS_COMMAND_TIMEOUT 0x03 << 1
#define STATUS_COMMAND_PROCESSING_ERROR 0x04 << 1
#define STATUS_COMMAND_EXECUTION_FAILURE 0x05 << 1 
#define STATUS_COMMAND_TX_DONE 0x06 << 1
#define STATUS_STBY_RC 0x2 << 4
#define STATUS_STBY_XOSC 0x3 << 4
#define STATUS_FS 0x4 << 4
#define STATUS_RX 0x5 << 4
#define STATUS_TX 0x6 << 4

#define RXSTATUS_FSK_PKT_SENT 1
#define RXSTATUS_FSK_PKT_RECEIVED 1 << 1
#define RXSTATUS_FSK_ABORT_ERR 1 << 2
#define RXSTATUS_FSK_LENGTH_ERR 1 << 3
#define RXSTATUS_FSK_CRC_ERR 1 << 4
#define RXSTATUS_FSK_ADRS_ERR 1 << 5
#define RXSTATUS_FSK_SYNC_ERR 1 << 6
#define RXSTATUS_FSK_PREAMBLE_ERR 1 << 7

#define OPERROR_RC64K_CALIB_ERR 1
#define OPERROR_RC13M_CALIB_ERR 1 << 1
#define OPERROR_PLL_CALIB_ERR 1 << 2
#define OPERROR_ADC_CALIB_ERR 1 << 3
#define OPERROR_IMG_CALIB_ERR 1 << 4
#define OPERROR_XOSC_START_ERR 1 << 5
#define OPERROR_PLL_LOCK_ERR 1 << 6
#define OPERROR_PA_RAMP_ERR 1 << 8

/**
 * Base SPI transaction function for SX126x
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin read to indicate if chip is busy_pin
 * @param cmd Opcode to be executed
 * @param addrlen Length of address in bytes
 * @param addr Address to call
 * @param rx_op Flag to indicate a receive transaction, otherwise a transmit
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_spi_transaction(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t addrlen, uint16_t addr, uint8_t rx_op, uint8_t len, uint8_t *buf);

/**
 * @brief SPI transaction wrapper to execute an Opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 */
void sx126x_cmd(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd);

/**
 * @brief SPI transaction wrapper to transmit byte via an Opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param buf Byte to transmit
 */
void sx126x_cmd_tx(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t buf);

/**
 * @brief SPI transaction wrapper to transmit data via an Opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_cmd_txbuf(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t len, uint8_t *buf);

/**
 * @brief SPI transaction wrapper to receive data via an Opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_cmd_rxbuf(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t len, uint8_t *buf);

/**
 * @brief SPI transaction wrapper to transmit data to an address
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param addrlen Length of address in bytes
 * @param addr Address to call
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_cmd_addr_txbuf(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t addrlen, uint16_t addr, uint8_t len, uint8_t *buf);

/**
 * @brief SPI transaction wrapper to receive data from an address
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param addrlen Length of address in bytes
 * @param addr Address to call
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_cmd_addr_rxbuf(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t addrlen, uint16_t addr, uint8_t len, uint8_t *buf);

/**
 * @brief Clear one of the several of the IRQs
 * 
 *- Command: `ClearIRQStatus`
 *
 *- Opcode: `0x02`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 */
void clearIRQStatus(spi_device_handle_t spi, gpio_num_t busy_pin);

/**
 * @brief Clear all the errors (on the device)
 * 
 *- Command: `ClearDeviceErrors`
 * 
 *- Opcode: `0x07` 
 * 
 * @param spi SPI Handle
 */
void clearDeviceErrors(spi_device_handle_t spi, gpio_num_t busy_pin);

/**
 * @brief Configure the IRQ and the DIOs attached to each IRQ
 * 
 *- Command: `SetDioIrqParams`
 * 
 *- Opcode: `0x08`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param irq_main 16-bit mask for main IRQ
 * @param irq_dio1 16-bit mask for DIO1
 * @param irq_dio2 16-bit mask for DIO2
 * @param irq_dio3 16-bit mask for DIO3
 */
void setDioIrqParams(spi_device_handle_t spi, gpio_num_t busy_pin, uint16_t irq_main, uint16_t irq_dio1, uint16_t irq_dio2, uint16_t irq_dio3 );

/**
 * @brief Write into one or several registers
 * 
 *- Command: `WriteRegister`
 * 
 *- Opcode: `0x0D`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param addr Register address to write to
 * @param length Length of data to be transferred
 * @param data Pointer to data storage
 */
void writeRegister(spi_device_handle_t spi, gpio_num_t busy_pin, uint16_t addr, uint8_t length, uint8_t *data);

/**
 * @brief Write data into the FIFO
 * 
 *- Command: `WriteBuffer`
 * 
 *- Opcode: `0x0E`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param offset Offset from start position in buffer
 * @param length Length of data to be transferred
 * @param data Pointer to data storage
 */
void writeBuffer(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t offset, uint8_t length, uint8_t *data);

/**
 * @brief Get the values of the triggered IRQs
 * 
 *- Command: `GetIrqStatus`
 * 
 *- Opcode: `0x12`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @return 
 * - IrqStatus (15:0)
 */
uint16_t getIRQStatus(spi_device_handle_t spi, gpio_num_t busy_pin);

/**
 * @brief   Returns PayloadLengthRx(15:8), RxBufferPointer(7:0)
 * 
 * - Command: `GetRxBufferStatus`
 * 
 * - Opcode: `0x13`
 * 
 * @param   spi SPI Handle
 * @param   busy_pin GPIO pin used to indicate the status of the internal state machine
 * 
 * @return `uint16_t` -
 * PayloadLengthRx(15:8),
 * RxBufferPointer(7:0)
 */
uint16_t getRxBufferStatus(spi_device_handle_t spi, gpio_num_t busy_pin);

/**
 * @brief   Returns RssiAvg, RssiSync, PStatus2, PStatus3, PStatus4 in FSK protocol.
 *          Returns RssiPkt, SnrPkt in LoRa protocol.
 * 
 * - Command: `GetPacketStatus`
 * 
 * - Opcode: `0x14`
 * 
 * @param   spi SPI Handle
 * @param   busy_pin GPIO pin used to indicate the status of the internal state machine
 * 
 * @return `uint32_t` -
 * Empty(31:24),
 * RxStatus(23:16), 
 * RssiSync(15:8), 
 * RssiAvg(7:0)
 */
uint32_t getPacketStatus(spi_device_handle_t spi, gpio_num_t busy_pin);

/**
 * @brief Return the error flags
 * 
 * - Command: `GetDeviceErrors`
 * 
 * - Opcode: `0x17`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * 
 * @return `uint16_t` -
 * OpError(15:0)
 */
uint16_t getDeviceErrors(spi_device_handle_t spi, gpio_num_t busy_pin);

/**
 * @brief Read one or several registers
 * 
 * - Command: `ReadRegister`
 * 
 * - Opcode: `0x1D`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param addr Register address to read from
 * @param length Length of data to be transferred
 * @param data Pointer to data storage
 */
void readRegister(spi_device_handle_t spi, gpio_num_t busy_pin, uint16_t addr, uint8_t length, uint8_t *data);

/**
 * @brief Read data from the FIFO
 * 
 * - Command: `ReadBuffer`
 * 
 * - Opcode: `0x1E`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param offset Offset from start position in buffer
 * @param length Length of data to be transferred
 * @param data Pointer to data storage
 */
void readBuffer(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t offset, uint8_t length, uint8_t *data);

/**
 * @brief Set Chip in STDBY_RC or STDBY_XOSC mode
 * 
 * - Command: `SetStandby`
 * 
 * - Opcode: `0x80` 
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param stdbyconfig Standby mode
 */
void setStandby(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t stdbyconfig) ;

/**
 * @brief Set Chip in Rx mode
 * 
 * - Command: `SetRx`
 * 
 * - Opcode: `0x82` 
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param timeout Timeout for 15.625us. 0 = wait, 0xFFFFFE = 262.1 secs, 0xFFFFFF = continuous)
 */
void setRx(spi_device_handle_t spi, gpio_num_t busy_pin, uint32_t timeout);

/**
 * @brief Set Chip in Tx mode
 * 
 * - Command: `SetTx`
 * 
 * - Opcode: `0x83` 
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param timeout Timeout for 15.625us. 0 = wait, 0xFFFFFF = 262.1 secs)
 */
void setTx(spi_device_handle_t spi, gpio_num_t busy_pin, uint32_t timeout);

/**
 * @brief Set the RF frequency of the radio
 * 
 * - Command: `SetRfFrequency`
 * 
 * - Opcode: `0x86` 
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param rffreq Frequency in Hz
 */
void setRfFrequency(spi_device_handle_t spi, gpio_num_t busy_pin, uint32_t rffreq);

/**
 * @brief Select the packet type corresponding to the modem
 * 
 * - Command: `SetPacketType`
 * 
 * - Opcode: `0x8A`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param packet_type Use GFSK, LoRA or Long Range FHSS
 */
void setPacketType(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t packet_type);

/**
 * @brief Compute and set values in selected protocol modem for given modulation parameters
 * 
 * - Command: `SetModulationParams`
 * 
 * - Opcode: `0x8B` 
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param bitrate Transmission speed in bits/sec
 * @param pulseshape Predefined transmission gaussian pulseshapes
 * @param rx_bw Predefined reception bandwidth
 * @param freqdev Frequency deviation. Usually proportional to bitrate
 */
void setModulationParams(spi_device_handle_t spi, gpio_num_t busy_pin, uint32_t bitrate, uint8_t pulseshape, uint8_t rx_bw, uint32_t freqdev);

/**
 * @brief Set values on selected protocol modem for given packet parameters
 * 
 * - Command: `SetPacketParams`
 * 
 * - Opcode: `0x8C`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param tx_preamble Transmit preamble size in bits
 * @param rx_preamble Preset preamble. Must be less than syncword.
 * @param syncword_bits Syncword size in bits
 * @param addr_filter Whether to filter on node and/or broadcast address
 * @param len_type Static or variable length
 * @param len_bytes Max length of payload in bytes
 * @param crc_type Selects the CRC type
 * @param whitening Disables / Enables whitening
 */
void setPacketParams(spi_device_handle_t spi, gpio_num_t busy_pin, uint16_t tx_preamble, uint8_t rx_preamble, uint8_t syncword_bits, uint8_t addr_filter, uint8_t len_type, uint8_t len_bytes, uint8_t crc_type, uint8_t whitening);

/**
 * @brief Set output power and ramp time for the PA
 * 
 * - Command: `SetTxParams`
 * 
 * - Opcode: `0x8E`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param power Output power in dB from -17 (0xEF) through to +14 (0x16) 
 * @param ramptime Predefined ramp time setting
 * 
 * @warning READ DATASHEET SECTIONS 13.1.14 (SetPaConfig) AND 13.4.4 (SetTxParams) FIRST!
 */
void setTxParams(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t power, uint8_t ramptime);

/**
 * @brief Store TX and RX base address in register of selected protocol modem
 * 
 * - Command: `SetBufferBaseAddress`
 * 
 * - Opcode: `0x8F`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param tx_addr TX base address in buffer
 * @param rx_addr RX base address in buffer
 */
void setBufferBaseAddress(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t tx_addr, uint8_t rx_addr);

/**
 * @brief Configure duty cycle, max output power, device for the PA for SX1261 or SX1262
 * 
 * - Command: `SetPaConfig`
 * 
 * - Opcode: `0x95`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param dutycycle Set the duty cycle of the power amplifier
 * @param hpmax Set maximum power (0x00 to 0x07 on the SX1262 / no effect on SX1261)
 * @param device Set the device: 0 = SX1262, 1 = SX1261
 * @warning READ DATASHEET SECTIONS 13.1.14 (SetPaConfig) AND 13.4.4 (SetTxParams) FIRST!
 */
void setPaConfig(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t dutycycle, uint8_t hpmax, uint8_t device);

/**
 * @brief Configure the radio to use a TCXO controlled by DIO3
 * 
 * - Command: `SetDIO3AsTcxoCtrl`
 * 
 * - Opcode: `0x97`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param voltage Predefined voltage profile
 * @param delay Wait for voltage increase in 15.625us increments (max 1 sec / 0xFFFF)
 * 
 * 
 */
void setDio3AsTCXOCtrl(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t voltage, uint16_t delay);

/**
 * @brief Configure radio to control an RF switch from DIO2
 * 
 * - Command: `SetDIO2AsRfSwitchCtrl`
 * 
 * - Opcode: `0x9D`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param enable DIO2 controls RF switch operation
 *  
 */
void setDio2AsRfSwitchCtrl(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t enable);

/**
 * @brief Returns the current status of the device
 * 
 * - Command: `GetStatus`
 * 
 * - Opcode: `0xC0`
 * 
 * @param spi SPI Handle
 * @param busy_pin GPIO pin used to indicate the status of the internal state machine
 * @param verbose Boolean flag to perform printf() output
 * @return `uint8_t` -
 * Status(7:0)
 */
uint8_t getStatus(spi_device_handle_t spi, gpio_num_t busy_pin, bool verbose);