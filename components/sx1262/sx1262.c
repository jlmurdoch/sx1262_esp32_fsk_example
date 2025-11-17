#include "sx1262.h"

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
uint8_t sx126x_spi_transaction(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t addrlen, uint16_t addr, bool rx_op, uint8_t len, uint8_t *buf) {
    spi_transaction_ext_t t;
    esp_err_t err;
    uint8_t retcode = 0;

    while( gpio_get_level(busy) != 0 ) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    
    err = spi_device_acquire_bus(spi, portMAX_DELAY);

    // Make room for the transaction
    memset(&t, 0, sizeof(t));
    t.base.cmd = cmd;
    // Indicate that both address and dummy bits are dynamic
    t.base.flags = SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY;
    
    // Address sizing
    if (addrlen == 1) {
        // For Buffer
        t.address_bits = 8;
        t.base.addr = addr & 0xFF;
    } else if (addrlen == 2) {
        // For Register
        t.address_bits = 16;
        t.base.addr = addr; 
    } else {
        // Everything else
        t.address_bits = 0;
    }

    // If we have a data transfer
    if (len > 0) {
        // If receiving
        if (rx_op) {
            // Need dummy bits for any read operation, apart from GetStatus (0xC0)
            if (cmd == 0xC0) {
                t.dummy_bits = 0;
            } else {
                t.dummy_bits = 8;
            }
            // Space for incoming data
            t.base.rxlength = 8 * len; 
            t.base.rx_buffer = buf;
        } else {
            // Space for outgoing data
            t.base.length = 8 * len;
            t.base.tx_buffer = buf;       
        }
    }

    err = spi_device_polling_transmit(spi, (spi_transaction_t*)&t);
    ESP_ERROR_CHECK(err);

    spi_device_release_bus(spi);

    return retcode;
}

/**
 * SPI transaction to execute an opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 */
void sx126x_cmd(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd) {
    // SetFs = (spi, 0xC1)
    sx126x_spi_transaction(spi, busy, cmd, 0, 0, false, 0, NULL);
    return;
}

/**
 * SPI transaction to transmit byte via an opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param buf Byte to transmit
 */
void sx126x_cmd_tx(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t buf) {
    sx126x_spi_transaction(spi, busy, cmd, 0, 0, false, 1, &buf);
    return;
}

/**
 * SPI transaction to transmit data via an opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_cmd_txbuf(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t len, uint8_t *buf) {
    sx126x_spi_transaction(spi, busy, cmd, 0, 0, false, len, buf);
    return;
}

/**
 * SPI transaction to receive data via an opcode
 * 
 * @param spi SPI Handle
 * @param cmd Opcode to be executed
 * @param len Length of data to be transferred
 * @param buf Pointer to data storage
 */
void sx126x_cmd_rxbuf(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t len, uint8_t *buf) {
    sx126x_spi_transaction(spi, busy, cmd, 0, 0, true, len, buf);
    return;
}

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
void sx126x_cmd_addr_txbuf(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t addrlen, uint16_t addr, uint8_t len, uint8_t *buf) {
    sx126x_spi_transaction(spi, busy, cmd, addrlen, addr, false, len, buf);
    return;
}

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
void sx126x_cmd_addr_rxbuf(spi_device_handle_t spi, gpio_num_t busy, uint8_t cmd, uint8_t addrlen, uint16_t addr, uint8_t len, uint8_t *buf) {
    sx126x_spi_transaction(spi, busy, cmd, addrlen, addr, true, len, buf);
    return;
}

/**
 * 0x02 - ClearIRQStatus - Clear IRQ Status for main IRQ
 * 
 * @param spi SPI Handle
 */
void clearIRQStatus(spi_device_handle_t spi, gpio_num_t busy) {
    uint16_t buf = 0x43FF;
    buf = (buf << 8) | (buf >> 8); // swap the bytes
    sx126x_cmd_txbuf(spi, busy, OP_CLEARIRQSTATUS, sizeof(buf), (uint8_t *)&buf);
    return;
}

/**
 * 0x07 - ClearDeviceErrors - Clear device errors
 * 
 * @param spi SPI Handle
 */
void clearDeviceErrors(spi_device_handle_t spi, gpio_num_t busy){
    uint16_t buf = 0;
    sx126x_cmd_txbuf(spi, busy, OP_CLEARDEVICEERRORS, 2, (uint8_t *)&buf);
}

/**
 * 0x08 - SetDIOIRQParams - Set DIO IRQ Parameters
 * 
 * @param spi SPI Handle
 * @param irq_main 16-bit mask for main IRQ
 * @param irq_dio1 16-bit mask for DIO1
 * @param irq_dio2 16-bit mask for DIO2
 * @param irq_dio3 16-bit mask for DIO3
 */
void setDioIrqParams(spi_device_handle_t spi, gpio_num_t busy, uint16_t irq_main, uint16_t irq_dio1, uint16_t irq_dio2, uint16_t irq_dio3 ){
    // Do little-endian bit-swap of each uint16_t
    uint8_t buf[8] = { 
        (irq_main >> 8) & 0xFF,
        irq_main & 0xFF,
        (irq_dio1 >> 8) & 0xFF,
        irq_dio1 & 0xFF,
        (irq_dio2 >> 8) & 0xFF,
        irq_dio2 & 0xFF,
        (irq_dio3 >> 8) & 0xFF,
        irq_dio3 & 0xFF,
    }; 
    sx126x_cmd_txbuf(spi, busy, OP_SETDIOIRQPARAMS, sizeof(buf), (uint8_t *)&buf);
}

/**
 * 0x0D - WriteRegister - Transmit data to a register address
 * 
 * @param spi SPI Handle
 * @param addr Register address to write to
 * @param length Length of data to be transferred
 * @param data Pointer to data storage
 */
void writeRegister(spi_device_handle_t spi, gpio_num_t busy, uint16_t addr, uint8_t length, uint8_t *data){
    sx126x_cmd_addr_txbuf(spi, busy, OP_WRITEREGISTER, sizeof(uint16_t), addr, length, data);
    return;
}

/**
 * 0x12 - GetIRQStatus - Get IRQ Status for main IRQ
 * 
 * @param spi SPI Handle
 * @return `uint16_t` - IRQ state
 */
uint16_t getIRQStatus(spi_device_handle_t spi, gpio_num_t busy) {
    uint16_t buf = 0;
    sx126x_cmd_rxbuf(spi, busy, OP_GETIRQSTATUS, sizeof(buf), (uint8_t *)&buf);
    return (buf << 8) | (buf >> 8); // swap the bytes
}

/**
 * 0x13 - GetRxBufferStatus - Return the status of the buffer from an RX perspective
 * 
 * @param spi SPI Handle
 * @returns `uint8_t | uint8_t` - length, offset
 */
uint16_t getRxBufferStatus(spi_device_handle_t spi, gpio_num_t busy) {
    uint16_t buf = 0;
    sx126x_cmd_rxbuf(spi, busy, OP_GETRXBUFFERSTATUS, sizeof(buf), (uint8_t *)&buf);
    return (buf << 8) | (buf >> 8); // swap the bytes
}

/**
 * 0x14 - GetPacketStatus - Return the status of the packet engine, but in reverse byte order
 * 
 * @param spi SPI Handle
 * @returns `uint8_t | uint8_t | uint8t` - RssiAvg, RssiSync, RxStatus
 */
uint32_t getPacketStatus(spi_device_handle_t spi, gpio_num_t busy) {
    uint32_t buf = 0;
    sx126x_cmd_rxbuf(spi, busy, OP_GETPACKETSTATUS, 3, (uint8_t *)&buf);
    return buf; // no need to swap the bytes, as the MSB becomes the LSB with the important packet data
}

/**
 * 0x1D - ReadRegister - Receive data from a register address
 * 
 * @param spi SPI Handle
 * @param addr Register address to read from
 * @param length Length of data to be transferred
 * @param data Pointer to data storage
 */
void readRegister(spi_device_handle_t spi, gpio_num_t busy, uint16_t addr, uint8_t length, uint8_t *data){
    sx126x_cmd_addr_rxbuf(spi, busy, OP_READREGISTER, sizeof(uint16_t), addr, length, data);
    return;
}

/**
 * 0x1E - ReadBuffer - Receive data from an offset in the buffer
 * 
 * @param spi SPI Handle
 * @param offset Offset from start position in buffer
 * @param length Length of data to be transferred
 * @param data Pointer to data storage
 */
void readBuffer(spi_device_handle_t spi, gpio_num_t busy, uint8_t offset, uint8_t length, uint8_t *data){
    // This only uses one byte address
    sx126x_cmd_addr_rxbuf(spi, busy, OP_READBUFFER, sizeof(offset), offset, length, data);
    return;
}

/**
 * 0x1F - SetBufferBaseAddress - Set the boundaries in the buffer for tx and rx
 * 
 * @param spi SPI Handle
 * @param tx_addr TX base address in buffer
 * @param rx_addr RX base address in buffer
 */
void setBufferBaseAddress(spi_device_handle_t spi, gpio_num_t busy, uint8_t tx_addr, uint8_t rx_addr){
    uint8_t buf[2] = { tx_addr, rx_addr };
    sx126x_cmd_txbuf(spi, busy, OP_SETBUFFERBASEADDRESS, sizeof(buf), buf);
}

/**
 * 0x80 - SetStandby - Puts the chip into a specific mode
 * 
 * @param spi SPI Handle
 * @param stdbyconfig Standby mode
 */
void setStandby(spi_device_handle_t spi, gpio_num_t busy, uint8_t stdbyconfig) {
    sx126x_cmd_tx(spi, busy, OP_SETSTANDBY, stdbyconfig);
    return;
}

/**
 * 0x82 - SetRx - Put chip into receive mode
 * 
 * @param spi SPI Handle
 * @param timeout Timeout for 15.625us. 0 = wait, 0xFFFFFE = 262.1 secs, 0xFFFFFF = continuous)
 */
void setRx(spi_device_handle_t spi, gpio_num_t busy, uint32_t timeout) {
    uint8_t buf[3];
    // if the timer is above 262secs, make it 262secs
    if(timeout > 0xFFFFFF) {
        timeout = 0xFFFFFE;
    }
    // Assign the uint32_t to the 3 bytes
    buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[2] = (uint8_t)(timeout & 0xFF);

    sx126x_cmd_txbuf(spi, busy, OP_SETRX, sizeof(buf), buf);
}

/**
 * 0x86 - SetRFFrequency
 * 
 * @param spi SPI Handle
 * @param rffreq Frequency in Hz
 */
void setRfFrequency(spi_device_handle_t spi, gpio_num_t busy, uint32_t rffreq) {
    uint8_t buf[4];
    // Convert frequency using the datasheet formula
    uint32_t freq = (uint32_t)(rffreq * pow(2, 25) / 32000000UL);
    buf[0] = (uint8_t)((freq >> 24) & 0xFF);
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);
    sx126x_cmd_txbuf(spi, busy, OP_SETRFFREQUENCY, sizeof(buf), buf);
}

/**
 * 0x8A - SetPacketType - Sets the mode of operation
 * 
 * @param spi SPI Handle
 * @param packet_type Use GFSK, LoRA or Long Range FHSS
 */
void setPacketType(spi_device_handle_t spi, gpio_num_t busy, uint8_t packet_type) {
    sx126x_cmd_tx(spi, busy, OP_SETPACKETTYPE, packet_type);
    return;
}

/**
 * 0x8B - SetModulationParams - Set the modulation parameters
 * 
 * @param spi SPI Handle
 * @param bitrate Transmission speed in bits/sec
 * @param pulseshape Predefined transmission gaussian pulseshapes
 * @param rx_bw Predefined reception bandwidth
 * @param freqdev Frequency deviation. Usually proportional to bitrate
 */
void setModulationParams(spi_device_handle_t spi, gpio_num_t busy, uint32_t bitrate, uint8_t pulseshape, uint8_t rx_bw, uint32_t freqdev){
    uint8_t buf[8];
    // Bit rate conversion using formula from datasheet
    uint32_t br = (uint32_t)(32.0 * 32000000UL / bitrate); // 9600bps
    // Frequency deviation conversion using formula from datasheet
    uint32_t fdev = (uint32_t)(freqdev * pow(2, 25) / 32000000UL); // half of bps
    // Spread bitrate over 3 bytes
    buf[0] = (uint8_t)((br >> 16) & 0xFF);
    buf[1] = (uint8_t)((br >> 8) & 0xFF);
    buf[2] = (uint8_t)(br & 0xFF);
    // Gaussian shaping for Tx
    buf[3] = pulseshape; // Gaussian BT = 1
    // Bandwidth setting for Rx
    buf[4] = rx_bw; // 14.6kHz
    // Spread frequency deviation over 3 bytes
    buf[5] = (uint8_t)((fdev >> 16) & 0xFF);
    buf[6] = (uint8_t)((fdev >> 8) & 0xFF);
    buf[7] = (uint8_t)(fdev & 0xFF);

    sx126x_cmd_txbuf(spi, busy, OP_SETMODULATIONPARAMS, sizeof(buf), buf);
}

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
void setPacketParams(spi_device_handle_t spi, gpio_num_t busy, uint16_t tx_preamble, uint8_t rx_preamble, uint8_t syncword_bits, uint8_t addr_filter, uint8_t len_type, uint8_t len_bytes, uint8_t crc_type, uint8_t whitening){
    uint8_t buf[9];
    buf[0] = tx_preamble >> 8 ; // for huge preamble
    buf[1] = tx_preamble & 0xFF; // tx preamble 96 bits
    buf[2] = rx_preamble; // rx preamble 8 bits - has to be smaller than the syncword
    buf[3] = syncword_bits; // syncword of 16 bits (0x2DD4)
    buf[4] = addr_filter; // addr and node filtering on (0x0b/0x8b)
    buf[5] = len_type; // dynamic length
    buf[6] = len_bytes; // one packet is 27 bytes
    buf[7] = crc_type; // Two-byte CRC
    buf[8] = whitening; // No whitening
    sx126x_cmd_txbuf(spi, busy, OP_SETPACKETPARAMS, sizeof(buf), buf);
}

/**
 * 0x97 - SetDIO3AsTCXOCtrl - Use DIO3 for TCXO power
 * 
 * @param spi SPI Handle
 * @param voltage Predefined voltage profile
 * @param delay Wait for voltage increase in 15.625us increments (max 1 sec / 0xFFFF)
 */
void setDio3AsTCXOCtrl(spi_device_handle_t spi, gpio_num_t busy, uint8_t voltage, uint16_t delay){
    uint8_t buf[3];
    buf[0] = voltage;
    buf[1] = delay >> 8 ; 
    buf[2] = delay & 0xFF; 
    sx126x_cmd_txbuf(spi, busy, OP_SETDIO3ASTCXOCTRL, sizeof(buf), buf);
}

/**
 * 0x9D - SetDIO2AsRfSwitchCtrl - DIO2 controls the RF switch for TX, RX, etc
 * 
 * @param spi SPI Handle
 * @param enable DIO2 controls RF switch operation
 */
void setDio2AsRfSwitchCtrl(spi_device_handle_t spi, gpio_num_t busy, uint8_t enable) {
    sx126x_cmd_tx(spi, busy, OP_SETDIO2ASRFSWITCHCTRL, enable);
    return;
}

/**
 * 0xC0 - GetStatus - Get the status with some output
 * 
 * @param spi SPI Handle
 * @param verbose Boolean flag to ask for printf() output
 * @return `uint8_t` Status output
 */
uint8_t getStatus(spi_device_handle_t spi, gpio_num_t busy, bool verbose) {
    uint8_t buf;
    sx126x_cmd_rxbuf(spi, busy, OP_GETSTATUS, 1, &buf);

    // Print out the GetStatus according to the datasheet descriptions
    if (verbose) {
        uint8_t command = (buf >> 1) & 0x7;
        uint8_t chipmode = (buf >> 4) & 0x7;
        printf("Status:"); 

        switch (chipmode)
        {
        case STATUS_STBY_RC:
            printf(" STBY_RC mode");
            break;
        case STATUS_STBY_XOSC:
            printf(" STBY_XOSC mode");
            break;
        case STATUS_FS:
            printf(" FS mode");
            break;
        case STATUS_RX:
            printf(" RX mode");
            break;
        case STATUS_TX:
            printf(" TX mode");
            break;

        default:
            break;
        }

        switch (command)
        {
        case STATUS_DATA_AVAILABLE:
            printf(" | Data is available to host");
            break;
        case STATUS_COMMAND_TIMEOUT:
            printf(" | Command timeout");
            break;
        case STATUS_COMMAND_PROCESSING_ERROR:
            printf(" | Command processing error");
            break;
        case STATUS_COMMAND_EXECUTION_FAILURE:
            printf(" | Failure to execute command");
            break;
        case STATUS_COMMAND_TX_DONE:
            printf(" | Command TX done");
            break;

        default:
            break;
        }
        printf("\n"); 
    }

    return buf;
}
