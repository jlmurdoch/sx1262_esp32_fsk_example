#include "sx1262.h"

void sx126x_spi_transaction(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t addrlen, uint16_t addr, uint8_t rx_op, uint8_t len, uint8_t *buf) {
    spi_transaction_ext_t t;
    esp_err_t err;
    uint8_t retcode = 0;

    while(gpio_get_level(busy_pin) != 0) {
        usleep(1);
    }
    
    err = spi_device_acquire_bus(spi, portMAX_DELAY);
    ESP_ERROR_CHECK(err);

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

    return;
}

void sx126x_cmd(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd) {
    // SetFs = (spi, 0xC1)
    sx126x_spi_transaction(spi, busy_pin, cmd, 0, 0, false, 0, NULL);
    return;
}

void sx126x_cmd_tx(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t buf) {
    sx126x_spi_transaction(spi, busy_pin, cmd, 0, 0, false, 1, &buf);
    return;
}

void sx126x_cmd_txbuf(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t len, uint8_t *buf) {
    sx126x_spi_transaction(spi, busy_pin, cmd, 0, 0, false, len, buf);
    return;
}

void sx126x_cmd_rxbuf(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t len, uint8_t *buf) {
    sx126x_spi_transaction(spi, busy_pin, cmd, 0, 0, true, len, buf);
    return;
}

void sx126x_cmd_addr_txbuf(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t addrlen, uint16_t addr, uint8_t len, uint8_t *buf) {
    sx126x_spi_transaction(spi, busy_pin, cmd, addrlen, addr, false, len, buf);
    return;
}

void sx126x_cmd_addr_rxbuf(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t cmd, uint8_t addrlen, uint16_t addr, uint8_t len, uint8_t *buf) {
    sx126x_spi_transaction(spi, busy_pin, cmd, addrlen, addr, true, len, buf);
    return;
}

void clearIRQStatus(spi_device_handle_t spi, gpio_num_t busy_pin) {
    uint8_t buf[2] = { 0x43, 0xFF }; // Just clear anything documented
    sx126x_cmd_txbuf(spi, busy_pin, OP_CLEARIRQSTATUS, sizeof(buf), buf);
    return;
}

void clearDeviceErrors(spi_device_handle_t spi, gpio_num_t busy_pin){
    uint16_t buf = 0;
    sx126x_cmd_txbuf(spi, busy_pin, OP_CLEARDEVICEERRORS, 2, (uint8_t *)&buf);
}

void setDioIrqParams(spi_device_handle_t spi, gpio_num_t busy_pin, uint16_t irq_main, uint16_t irq_dio1, uint16_t irq_dio2, uint16_t irq_dio3 ){
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
    
    sx126x_cmd_txbuf(spi, busy_pin, OP_SETDIOIRQPARAMS, sizeof(buf), (uint8_t *)&buf);
}

void writeRegister(spi_device_handle_t spi, gpio_num_t busy_pin, uint16_t addr, uint8_t length, uint8_t *data){
    sx126x_cmd_addr_txbuf(spi, busy_pin, OP_WRITEREGISTER, sizeof(addr), addr, length, data);
    return;
}

void writeBuffer(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t offset, uint8_t length, uint8_t *data){
    // This only uses one byte address
    sx126x_cmd_addr_txbuf(spi, busy_pin, OP_WRITEBUFFER, sizeof(offset), offset, length, data);
    return;
}

uint16_t getIRQStatus(spi_device_handle_t spi, gpio_num_t busy_pin) {
    uint8_t buf[2];
    sx126x_cmd_rxbuf(spi, busy_pin, OP_GETIRQSTATUS, sizeof(buf), buf);
    return (uint16_t)(buf[0] << 8 | buf[1]); // swap the bytes
}

uint16_t getRxBufferStatus(spi_device_handle_t spi, gpio_num_t busy_pin) {
    uint8_t buf[2];
    sx126x_cmd_rxbuf(spi, busy_pin, OP_GETRXBUFFERSTATUS, sizeof(buf), buf);
    return (uint16_t)(buf[0] << 8 | buf[1]); // don't swap the bytes
}

uint32_t getPacketStatus(spi_device_handle_t spi, gpio_num_t busy_pin) {
    uint8_t buf[3];
    sx126x_cmd_rxbuf(spi, busy_pin, OP_GETPACKETSTATUS, 3, (uint8_t *)&buf);
    return (uint32_t)(buf[0] << 16 | buf[1] << 8 | buf[2]);
}

uint16_t getDeviceErrors(spi_device_handle_t spi, gpio_num_t busy_pin) {
    uint8_t buf[2];
    sx126x_cmd_rxbuf(spi, busy_pin, OP_GETDEVICEERRORS, sizeof(buf), buf);
    return (uint16_t)(buf[0] << 8 | buf[1]); 
}

void readRegister(spi_device_handle_t spi, gpio_num_t busy_pin, uint16_t addr, uint8_t length, uint8_t *data){
    sx126x_cmd_addr_rxbuf(spi, busy_pin, OP_READREGISTER, sizeof(addr), addr, length, data);
    return;
}

void readBuffer(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t offset, uint8_t length, uint8_t *data){
    // This only uses one byte address
    sx126x_cmd_addr_rxbuf(spi, busy_pin, OP_READBUFFER, sizeof(offset), offset, length, data);
    return;
}

void setStandby(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t stdbyconfig) {
    sx126x_cmd_tx(spi, busy_pin, OP_SETSTANDBY, stdbyconfig);
    return;
}

void setRx(spi_device_handle_t spi, gpio_num_t busy_pin, uint32_t timeout) {
    uint8_t buf[3];
    // if the timer is above 262secs, make it 262secs
    if(timeout > 0xFFFFFF) {
        timeout = 0xFFFFFE;
    }
    // Assign the uint32_t to the 3 bytes
    buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[2] = (uint8_t)(timeout & 0xFF);

    sx126x_cmd_txbuf(spi, busy_pin, OP_SETRX, sizeof(buf), buf);
}

void setTx(spi_device_handle_t spi, gpio_num_t busy_pin, uint32_t timeout) {
    uint8_t buf[3];
    // if the timer is above 262secs, make it 262secs
    if(timeout > 0x00FFFFFF) {
        timeout = 0x00FFFFFF;
    }
    // Assign the uint32_t to the 3 bytes
    buf[0] = (uint8_t)((timeout >> 16) & 0xFF);
    buf[1] = (uint8_t)((timeout >> 8) & 0xFF);
    buf[2] = (uint8_t)(timeout & 0xFF);

    sx126x_cmd_txbuf(spi, busy_pin, OP_SETTX, sizeof(buf), buf);
}

void setRfFrequency(spi_device_handle_t spi, gpio_num_t busy_pin, uint32_t rffreq) {
    uint8_t buf[4];
    // Convert frequency using the datasheet formula
    uint32_t freq = (uint32_t)(rffreq * pow(2, 25) / 32000000UL);
    buf[0] = (uint8_t)((freq >> 24) & 0xFF);
    buf[1] = (uint8_t)((freq >> 16) & 0xFF);
    buf[2] = (uint8_t)((freq >> 8) & 0xFF);
    buf[3] = (uint8_t)(freq & 0xFF);
    sx126x_cmd_txbuf(spi, busy_pin, OP_SETRFFREQUENCY, sizeof(buf), buf);
}

void setPacketType(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t packet_type) {
    sx126x_cmd_tx(spi, busy_pin, OP_SETPACKETTYPE, packet_type);
    return;
}

void setModulationParams(spi_device_handle_t spi, gpio_num_t busy_pin, uint32_t bitrate, uint8_t pulseshape, uint8_t rx_bw, uint32_t freqdev){
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

    sx126x_cmd_txbuf(spi, busy_pin, OP_SETMODULATIONPARAMS, sizeof(buf), buf);
}

void setPacketParams(spi_device_handle_t spi, gpio_num_t busy_pin, uint16_t tx_preamble, uint8_t rx_preamble, uint8_t syncword_bits, uint8_t addr_filter, uint8_t len_type, uint8_t len_bytes, uint8_t crc_type, uint8_t whitening){
    uint8_t buf[9];
    buf[0] = tx_preamble & 0xFF; // for huge preamble
    buf[1] = tx_preamble >> 8; // tx preamble 96 bits
    buf[2] = rx_preamble; // rx preamble 8 bits - has to be smaller than the syncword
    buf[3] = syncword_bits; // syncword of 16 bits (0x2DD4)
    buf[4] = addr_filter; // addr and node filtering on (0x0b/0x8b)
    buf[5] = len_type; // dynamic length
    buf[6] = len_bytes; // one packet is 27 bytes
    buf[7] = crc_type; // Two-byte CRC
    buf[8] = whitening; // No whitening
    sx126x_cmd_txbuf(spi, busy_pin, OP_SETPACKETPARAMS, sizeof(buf), buf);
}

void setTxParams(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t power, uint8_t ramptime){
    uint8_t buf[2];
    buf[0] = power;
    buf[1] = ramptime;
    sx126x_cmd_txbuf(spi, busy_pin, OP_SETTXPARAMS, sizeof(buf), buf);
}

void setBufferBaseAddress(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t tx_addr, uint8_t rx_addr){
    uint8_t buf[2] = { tx_addr, rx_addr };
    sx126x_cmd_txbuf(spi, busy_pin, OP_SETBUFFERBASEADDRESS, sizeof(buf), buf);
}

void setPaConfig(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t dutycycle, uint8_t hpmax, uint8_t device){
    uint8_t buf[4];
    buf[0] = dutycycle;
    buf[1] = hpmax;
    buf[2] = device;
    buf[3] = 0x01; // Mandatory for paLut
    sx126x_cmd_txbuf(spi, busy_pin, OP_SETPACONFIG, sizeof(buf), buf);
}

void setDio3AsTCXOCtrl(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t voltage, uint16_t delay){
    uint8_t buf[3];
    buf[0] = voltage;
    buf[1] = delay >> 8 ; 
    buf[2] = delay & 0xFF; 
    sx126x_cmd_txbuf(spi, busy_pin, OP_SETDIO3ASTCXOCTRL, sizeof(buf), buf);
}

void setDio2AsRfSwitchCtrl(spi_device_handle_t spi, gpio_num_t busy_pin, uint8_t enable) {
    sx126x_cmd_tx(spi, busy_pin, OP_SETDIO2ASRFSWITCHCTRL, enable);
    return;
}

uint8_t getStatus(spi_device_handle_t spi, gpio_num_t busy_pin, bool verbose) {
    uint8_t buf;
    sx126x_cmd_rxbuf(spi, busy_pin, OP_GETSTATUS, 1, &buf);

    // Print out the GetStatus according to the datasheet descriptions
    if (verbose) {
        uint8_t chipmode = buf & 0x70; // Remove bit 7
        uint8_t command = buf & 0x0E; // Mask bit 0

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
