/**
 * @file    sx1262_esp32_fsk_example.c
 * 
 * @brief   SX1262 ESP32 FSK Example
 *
 * This is for a Seeed Studio XIAO ESP32-S3 Plus with a Wio-SX1262 module.
 * 
 * This is just a receive-only example for the time being.
 * 
 * Generated with the Espressif IoT Development Framework:
 *   cd esp-idf
 *   . ./export.sh
 *   idf.py create-project sx1262_esp32_fsk_example
 *   cd sx1262_esp32_fsk_example
 * 
 * To configure the architecture (i.e. XIAO ESP-S3 Plus)
 *   idf.py set-target esp32s3
 *   idf.py menuconfig
 *      Serial flasher config
 *          Flash size (8MB)
 * 
 * Edit CMakeLists.txt: 
 *   REQUIRES sx1262
 */

#include "sx1262.h"

// SPI to use
#define SPI_HOST_ID SPI2_HOST

// Pins on Seeed Studio Wio-SX1262
#define LORA_SPI_SCK 7
#define LORA_SPI_MISO 8
#define LORA_SPI_NSS 41
#define LORA_SPI_MOSI 9
#define LORA_RST 42
#define LORA_BUSY 40
#define LORA_DIO1 39
#define LORA_RF_SW1 38

/**
 * Wio-SX1262 Onboard LED
 * - LED On = 1
 * - LED Off = 0
 */
#define LORA_LED 48
/**
 * Button on Wio-SX1262 - wired to User LED on XIAO ESP32S3 Plus
 * - Default = 1 (pull-up to 3.3V)
 * - Push = 0 (LED on XIAO lights up, drops to 0V)
 */
#define LORA_BUTTON 21 

// Base frequency of radio for TX / RX
#define LORA_RFFREQ 868299000

// See 6.2.1 for modulation
#define LORA_BITRATE 9600
#define LORA_FREQDEV 4800

/*
 * Packet details - see FSK Packet Format (6.2.3) in the datasheet
 */
// Optimises the buffer and reception, as this example uses dynamic length
#define MAX_PACKET_SIZE 0x1B

// TX preamble in bits
#define PREAMBLE_TX_BITS 96

// RX preamble is predefined sizes - datasheet recommends 8bit or 16 bit
#define PREAMBLE_RX_BITS PREAMBLE_RX_16_BITS

// Two-byte syncword with some preamble padding (See 6.2.2.1 in Datasheet)
#define SYNCWORD 0x55, 0x55, 0x2D, 0xD4 

// Addresses
#define NODE_ADDRESS 0x8B 
#define BROADCAST_ADDRESS 0x0B

// Sample bolier data: temp@19ºC (0xA6) thermostat change, 18.5ºC (0xA5) local temperature recording
#define PACKET_DATA 0x08, 0x7A, 0x00, 0x06, 0x16, 0xFC, 0x44, 0x04, \
                    0x03, 0x10, 0x22, 0xA6, 0xA5, 0x02, 0x06, 0x00, \
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, \
                    0x00, 0x00, 0x30 

// CRC Calculation - CRC-16 UMTS
#define CRC_INIT 0x00, 0x00
#define CRC_POLY 0x80, 0x05

// Queue definition
QueueHandle_t queueMsg;
volatile int queueLength = 0;

// Simple CRC checker for the internal payload, not the packet
uint8_t crc8(uint8_t *buf, uint8_t size) {
  uint8_t crc = 0;

  for (int x = 0; x < size; x++)
      crc -= buf[x]; // Keep subtracting and rolling over

  return crc;
}

// This is the interrupt routine for DIO1
static void IRAM_ATTR rx_handler(void* arg)
{
    spi_device_handle_t spi = (spi_device_handle_t) arg;
    
    // We have an interrupt, lets store it
    uint16_t irqstatus = getIRQStatus(spi, LORA_BUSY);
    
    // Clear the status ASAP to free up DIO1
    clearIRQStatus(spi, LORA_BUSY);

    // Check IRQ to see if RxDone (0x02) and not anything else (i.e. CRC Error (0x40))
    if ((irqstatus == IRQ_RXDONE)) {
        // Get the data location (offset) and size
        uint16_t rxbuf = getRxBufferStatus(spi, LORA_BUSY);
        uint8_t length = (uint8_t)((rxbuf >> 8) & 0xFF);
        uint8_t offset = (uint8_t)(rxbuf & 0xFF);

        // If the data is the right length
        if(length == MAX_PACKET_SIZE) {
            // Make space for the data
            uint8_t *data = malloc(length);
            // Read it off the buffer into the space
            readBuffer(spi, LORA_BUSY, offset, length, data);
            // Pop the data onto the queue
            xQueueSendFromISR(queueMsg, data, NULL);
            // Increment the queue length
            queueLength++;
            // Free up the space used by the data
            free(data);
        }
    }
}

void app_main(void)
{
    esp_err_t err;

    // Set up the data points
    uint8_t syncword[] = { SYNCWORD };
    uint8_t filter[] = { NODE_ADDRESS, BROADCAST_ADDRESS };
    uint8_t pkt[] = { PACKET_DATA };
    uint8_t crcdata[] = { CRC_INIT, CRC_POLY };
    
    // Set the power boot for receving
    uint8_t rxgain[] = { RX_GAIN_POWER_BOOST };

    // Pause to stop rapid crashing
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    /************************************
     * SPI SETUP
     ************************************/
    spi_device_handle_t spi;

    // Main SPI bus setup
    spi_bus_config_t spi_bus_cfg = {
        .miso_io_num = LORA_SPI_MISO,
        .mosi_io_num = LORA_SPI_MOSI,
        .sclk_io_num = LORA_SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64
    };

    // Configure the SX1262 SPI device
    spi_device_interface_config_t sx1262_cfg = {
        .mode = 0,
        .clock_speed_hz = SPI_MASTER_FREQ_11M,  // Can't use 16MHz - causes SPI issues regardless of DMA setting
        .spics_io_num = LORA_SPI_NSS,           // NSS / CS line
        .queue_size = 1,                        // Mandatory to have queue size
        .command_bits = 8,                      // All commands are 8 bits in length
        .address_bits = 0,                      // Only used for read / write operations on registers and buffers
        .dummy_bits = 0,                        // Only used on write
        .flags = SPI_DEVICE_HALFDUPLEX          // The radio needs half-duplex for dummy bits and NOPs
    };

    // Start up the SPI bus
    err =  spi_bus_initialize(SPI_HOST_ID, &spi_bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(err);

    // Attach the SX1262 to the SPI bus
    err =  spi_bus_add_device(SPI_HOST_ID, &sx1262_cfg, &spi);
    ESP_ERROR_CHECK(err);

    /************************************
     * GPIO SETUP
     ************************************/
    // Interrupt Service Routine (ISR) setup for DIO1
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LORA_DIO1), // DIO1 is an interrupt
        .intr_type = GPIO_INTR_POSEDGE, // Rising Edge
        .mode = GPIO_MODE_INPUT, // DIO1 is an input
        .pull_down_en = 0, // No pull down
        .pull_up_en = 0 // No pull up
    };
    err = gpio_config(&io_conf);
    ESP_ERROR_CHECK(err);

    // Set up interrupt service
    err = gpio_install_isr_service(0);
    ESP_ERROR_CHECK(err);

    // Attach the interrupt service routine, rx_handler(), to DIO1
    err = gpio_isr_handler_add(LORA_DIO1, rx_handler, (void*)spi);
    ESP_ERROR_CHECK(err);

    // LED pin
    err = gpio_reset_pin(LORA_LED);
    ESP_ERROR_CHECK(err);
    err = gpio_set_direction(LORA_LED, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(err);

    // BUSY pin
    err = gpio_reset_pin(LORA_BUSY);
    ESP_ERROR_CHECK(err);
    err = gpio_set_direction(LORA_BUSY, GPIO_MODE_INPUT);
    ESP_ERROR_CHECK(err);

    // RESET pin
    err = gpio_reset_pin(LORA_RST);
    ESP_ERROR_CHECK(err);
    err = gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    ESP_ERROR_CHECK(err);

    // BUTTON pin
    err = gpio_reset_pin(LORA_BUTTON);
    ESP_ERROR_CHECK(err);
    err = gpio_set_direction(LORA_BUTTON, GPIO_MODE_INPUT);
    ESP_ERROR_CHECK(err);

    /************************************
     * RADIO RESET
     ************************************/
    // Power-on - RST needs to be pulled-up!
    err = gpio_set_level(LORA_RST, 1);
    ESP_ERROR_CHECK(err);
    
    // Wait for 100us
    usleep(100);
    
    // Pull down reset
    err =  gpio_set_level(LORA_RST, 0);
    ESP_ERROR_CHECK(err);

    // Wait for 100us (See 8.1)
    usleep(100);
    
    // Release reset
    err =  gpio_set_level(LORA_RST, 1);
    ESP_ERROR_CHECK(err);

    // Check the BUSY status now - it does take some time to be ready
    while(gpio_get_level(LORA_BUSY) != 0 ) {
        usleep(1);
    }

    /************************************
     * RADIO CONFIGURATION
     * (Section 14.2 & 14.3 in datasheet)
     ************************************/
    // 0x80: SetStandby - Go to STDBY_RC for configuration
    setStandby(spi, LORA_BUSY, STDBY_RC);
    getStatus(spi, LORA_BUSY, true);

    // 0x07: ClearDeviceErrors, used to wipe clean issues caused by TCXO not being ready on cold start
    clearDeviceErrors(spi, LORA_BUSY);

    // 0x02: Clear any IRQ flags from setup
    clearIRQStatus(spi, LORA_BUSY);

    // 0x8A: SetPacketType is the mode 0=GFSK, 1=LoRa. Has to be the FIRST configuration command.
    setPacketType(spi, LORA_BUSY, PACKET_TYPE_GFSK);

    // 0x86: SetRfFrequency, used to set the frequency
    setRfFrequency(spi, LORA_BUSY, LORA_RFFREQ);

    // 0x95: (TX) SetPaConfig configure the power amplifier (See datasheet)
    setPaConfig(spi, LORA_BUSY, PA_SX1262_22DBM);
    
    // 0x8E: (TX) SetTxParams to set the power at 22dB at 40us ramp up
    setTxParams(spi, LORA_BUSY, 22, SET_RAMP_40U);

    // 0x8F: SetBufferBaseAddress for pointer locations in FIFO for Tx and Rx
    setBufferBaseAddress(spi, LORA_BUSY, 0x00, 0x00);

    // 0x0E: (TX) WriteBuffer at 0x00 with pkt[] contents
    writeBuffer(spi, LORA_BUSY, 0x00, sizeof(pkt), pkt);

    // 0x8B: SetModulationParams to set the bitrate, bandwidth, shaping and frequency deviation.
    // Note: Must be executed some time after SetPacketType() but at some time before SetPacketParams()
    setModulationParams(spi, LORA_BUSY, LORA_BITRATE, PULSESHAPE_BT_1_0, RX_BW_11700, LORA_FREQDEV);

    // 0x8C: SetPacketParams, set all the options for the packets incoming / outgoing. 
    // Note: Must be executed at some point after SetPacketParams(), not before
    setPacketParams(spi, LORA_BUSY, PREAMBLE_TX_BITS, PREAMBLE_RX_BITS, sizeof(syncword) * 8, ADDRESS_FILTER_NODE_BROADCAST, LENGTH_VARIABLE, MAX_PACKET_SIZE, CRC_2_BYTE, WHITENING_OFF);

    // 0x0D: WriteRegister for CRC, SyncWord, Node/Broadcast Address and RX gain
    writeRegister(spi, LORA_BUSY, 0x06BC, sizeof(crcdata), crcdata);
    writeRegister(spi, LORA_BUSY, 0x06C0, sizeof(syncword), syncword);
    writeRegister(spi, LORA_BUSY, 0x06CD, sizeof(filter), filter);
    writeRegister(spi, LORA_BUSY, 0x08AC, sizeof(rxgain), rxgain);

    // 0x1D: (TX) ReadRegister: Look at Over-Current Protection (OCP) to see if it has changed for SX1262
    uint8_t ocpvalue;
    readRegister(spi, LORA_BUSY, 0x08E7, sizeof(ocpvalue), &ocpvalue);
    printf("Register: [0x08E7: OCP Configuration]: 0x%x (%0.f mA)\n\n", ocpvalue, ocpvalue * 2.5);

    // 0x08: SetDioIrqParams for Timeouts, CRC Errors and RXDone/TXDone for IRQ, with just CRC Error | RXDONE for DIO1, but not DIO2 or DIO3
    setDioIrqParams(spi, LORA_BUSY, IRQ_TIMEOUT | IRQ_CRCERR | IRQ_RXDONE | IRQ_TXDONE, IRQ_CRCERR | IRQ_RXDONE, 0, 0);
    
    /************************************
     * Seeed Studio Wio-SX1262 settings
     ************************************/
    // 0x9D: SetDIO2AsRfSwitchCtrl as true means DIO2 controls the RF switch for TX, RX, etc
    setDio2AsRfSwitchCtrl(spi, LORA_BUSY, true);

    // 0x97: SetDIO3AsTCXOCtrl controls the TCXO. Set to @ 3.0V (0x06), 5ms (320 ticks)
    setDio3AsTCXOCtrl(spi, LORA_BUSY, TCXOVOLTAGE_3_0V, 320);

    /************************************
     * Tx Mode
     ************************************/
    // Wio SX1262 LED on to indicate start of TX 
    err = gpio_set_level(LORA_LED, 1);
    ESP_ERROR_CHECK(err);

    // 0x02: Clear any IRQ flags from setup
    clearIRQStatus(spi, LORA_BUSY);

    // 0x83: SetTx to send one packet and stop (0x000000)
    setTx(spi, LORA_BUSY, 0x0FFFFF);
    getStatus(spi, LORA_BUSY, true);

    // Wait until Tx mode is exited
    // - Doesn't work with GetIRQStatus() which returns erroneous timeouts
    // - Can't use bits as it is a bit pattern on first octet
    while ((getStatus(spi, LORA_BUSY, false) & 0x70) == STATUS_TX) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    getStatus(spi, LORA_BUSY, true);

    // 0x12: Get the IRQ status to see if either Timeout (0x0200) or TXDone (0x0001) 
    uint16_t irqstatus = getIRQStatus(spi, LORA_BUSY);
    if (irqstatus & IRQ_TXDONE) {
        printf("Tx Done\n\n");
    } else {
        printf("Tx Timeout\n\n");
    }
    
    // Wio SX1262 LED off to indicate end of TX
    err = gpio_set_level(LORA_LED, 0);
    ESP_ERROR_CHECK(err);

    /************************************
     * Rx Mode
     ************************************/
    // 0x02: Clear any IRQ flags from Tx procedure before
    clearIRQStatus(spi, LORA_BUSY);

    // Create the message queue
    queueMsg = xQueueCreate(10, MAX_PACKET_SIZE);

    // 0x82: SetRx in continuous mode (0xFFFFFF)
    setRx(spi, LORA_BUSY, 0xFFFFFF);    
    getStatus(spi, LORA_BUSY, true);

    // Receive forever from ISR
    while(1) {
        // Wait, so the watchdog doesn't kick in
        vTaskDelay(10 / portTICK_PERIOD_MS);

        // If we have something on the queue
        while (queueLength > 0) {
            // Assign storage for the data
            uint8_t data[MAX_PACKET_SIZE];

            // Collect data off the queue
            xQueueReceive(queueMsg, data, portMAX_DELAY);

            // Decrement the counter
            queueLength--;

            // Do something, such as print the payload and calculate the internal CRC
            for (int x=0; x < MAX_PACKET_SIZE; x++) {
                printf("%02x ", data[x]);
            }            
            printf("- CRC Check: %02x\n", crc8(data+6, MAX_PACKET_SIZE - 7));
        }
    }
}
