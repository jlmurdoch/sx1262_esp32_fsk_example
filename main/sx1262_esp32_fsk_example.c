/*
 * SX1262 ESP32 FSK Example
 *
 * This is for a Seeed Studio XIAO ESP32-S3 Plus with a Wio-SX1262 module.
 * 
 * This is just a receive-only example for the time being.
 * 
 * Generated with the Espressif IoT Development Framework:
 *   cd esp-idf
 *   . ./export.sh
 * 
 * Then:
 *   idf.py create-project sx1262_esp32_fsk_example
 *   cd sx1262_esp32_fsk_example
 *   idf.py set-target esp32s3
 *   idf.py menuconfig
 *      Serial flasher config
 *          Flash size (8MB)
 * 
 * Edit CMakeLists.txt: REQUIRES sx1262
 */

#include "sx1262_esp32_fsk_example.h"

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
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    spi_device_handle_t spi = (spi_device_handle_t) arg;
    
    // We have an interrupt, lets store it
    uint16_t irqstatus = getIRQStatus(spi, LORA_BUSY);
    
    // Clear the status ASAP to free up DIO1
    clearIRQStatus(spi, LORA_BUSY);

    // Check to see RxDone (0x02) and not anything else
    if((irqstatus & 0xFF) == 0x02) {
        // Get the data location (offset) and size
        uint16_t rxbuf = getRxBufferStatus(spi, LORA_BUSY);
        uint8_t offset = rxbuf & 0xFF;
        uint8_t length = rxbuf >> 8;

        // If the data is the right length
        if(length == PACKET_SIZE) {
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
    esp_err_t ret;

    // Pause to stop rapid crashing
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    /*
     * SPI Setup
     */
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
        .clock_speed_hz = SPI_MASTER_FREQ_11M,  // Can't use 16MHz - causes SPI issues
        .spics_io_num = LORA_SPI_NSS,   // NSS / CS line
        .queue_size = 1,                // Mandatory to have queue size
        .command_bits = 8,              // All commands are 8 bits in length
        .address_bits = 0,              // Only used for read / write operations on registers and buffers
        .dummy_bits = 0,                // Only used on write
        .flags = SPI_DEVICE_HALFDUPLEX  // The radio needs half-duplex for dummy bits and NOPs
    };

    // Start up the SPI bus
    ret = spi_bus_initialize(SPI_HOST_ID, &spi_bus_cfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    // Attach the SX1262 to the SPI bus
    ret = spi_bus_add_device(SPI_HOST_ID, &sx1262_cfg, &spi);
    ESP_ERROR_CHECK(ret);

    /*
     * GPIO Setup
     */
    // Interrupt Service Routine (ISR) setup for DIO1
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LORA_DIO1), // DIO1 is an interrupt
        .intr_type = GPIO_INTR_POSEDGE, // Rising Edge
        .mode = GPIO_MODE_INPUT, // DIO1 is an input
        .pull_down_en = 0, // No pull down
        .pull_up_en = 0 // No pull up
    };
    gpio_config(&io_conf);

    // LORA BUSY indicator is an input that we read for SPI unavailability
    gpio_reset_pin(LORA_BUSY);
    ret = gpio_set_direction(LORA_BUSY, GPIO_MODE_INPUT); 

    while( gpio_get_level(LORA_BUSY) != 0 ) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    /*
     * Power on process
     */

    // Power-on - RST needs to be pulled-up!
    gpio_reset_pin(LORA_RST);
    ret = gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
    ret = gpio_set_level(LORA_RST, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    
    // Now do a reset
    ret = gpio_set_level(LORA_RST, 0);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    
    // Power back on
    ret = gpio_set_level(LORA_RST, 1);
    vTaskDelay(1 / portTICK_PERIOD_MS);

    // Set up interrupt on DIO1
    gpio_install_isr_service(0);
    gpio_isr_handler_add(LORA_DIO1, gpio_isr_handler, (void*)spi);

    /*
     * Start Circuit Configuration for Basic Rx Operation 
     * (Section 14.3 in SX1261/2 V2.2 Datasheet)
     */

    // 0x80: SetStandby - Go to STDBY_RC for configuration
    setStandby(spi, LORA_BUSY, STDBY_RC);
    getStatus(spi, LORA_BUSY, true);

    // 0x1D: ReadRegister: Simple verification of SPI working - look at the first byte of default syncword
    uint8_t testinput;
    readRegister(spi, LORA_BUSY, 0x06BC, sizeof(testinput), &testinput);
    printf("Test Register Read: 0x%x (0x1d expected)\n", testinput);
    
    // 0x8A: SetPacketType is the mode 0=GFSK, 1=LoRa
    setPacketType(spi, LORA_BUSY, PACKET_TYPE_GFSK);

    // 0x07: ClearDeviceErrors, used to wipe clean issues caused by TCXO not being ready on cold start
    clearDeviceErrors(spi, LORA_BUSY);

    // 0x86: SetRfFrequency, used to set the frequency
    setRfFrequency(spi, LORA_BUSY, 868299000);

    // 0x8F: SetBufferBaseAddress for locations in FIFO for Rx and Tx
    setBufferBaseAddress(spi, LORA_BUSY, 0x00, 0x00);

    // 0x8B: SetModulationParams to set the bitrate, bandwidth, shaping and 
    setModulationParams(spi, LORA_BUSY, 9600, PULSESHAPE_BT_1_0, RX_BW_11700, 4800);

    // 0x8C: SetPacketParams, set all the options for the packets incoming
    setPacketParams(spi, LORA_BUSY, 60, PREAMBLE_SIZE_8, 16, ADDRESS_FILTER_NODE_BROADCAST, LENGTH_VARIABLE, 27, CRC_2_BYTE, WHITENING_OFF);

    // 0x0D: WriteRegister at SyncWord Register(0x06C0), to insert 16-bit syncword of 0x2DD4
    uint8_t syncword[2] = { 0x2D, 0xD4 };
    writeRegister(spi, LORA_BUSY, 0x06C0, sizeof(syncword), syncword);

    // 0x0D: WriteRegister of node and broadcast addresses
    uint8_t filter[2] = { 0x0B, 0x8B };
    writeRegister(spi, LORA_BUSY, 0x06CD, sizeof(filter), filter);

    // 0x0D: WriteRegister at CRC Registers 0x6BC of 0x0000 initial, 0x8005 CRC polynomial (CRC-16/UTMS)
    uint8_t crcdata[4] = { 0x00, 0x00, 0x80, 0x05 };
    writeRegister(spi, LORA_BUSY, 0x06BC, sizeof(crcdata), crcdata);

    // 0x0D: WriteRegister at SyncWord Register(0x06C0), to insert 16-bit syncword of 0x2DD4
    uint8_t rxgain[1] = { 0x96 };
    writeRegister(spi, LORA_BUSY, 0x08AC, sizeof(rxgain), rxgain);

    // 0x08: SetDioIrqParams for both CRCErr and RxDone (0x0202) for IRQ and DIO1, not DIO2 or DIO3
    setDioIrqParams(spi, LORA_BUSY, IRQ_CRCERR | IRQ_RXDONE, IRQ_CRCERR | IRQ_RXDONE, 0, 0);

    /*
     * Seeed Studio XIAO Wio-SX1262 specifics
     * - Use DIO2 as RF Switch Control
     * - Use DIO3 as TCXO Control at 3.0V
     */
    // 0x9D: SetDIO2AsRfSwitchCtrl as true means DIO2 controls the RF switch for TX, RX, etc
    setDio2AsRfSwitchCtrl(spi, LORA_BUSY, true);

    // 0x97: SetDIO3AsTCXOCtrl controls the TCXO. Set to @ 3.0V (0x06), 4.7ms (0x012C)
    setDio3AsTCXOCtrl(spi, LORA_BUSY, TCXOVOLTAGE_3_0V, 0x012C);

    /*
     * RX Mode
     */

    // 0x82: SetRx in continuous mode (0xFFFFFF)
    setRx(spi, LORA_BUSY, 0xFFFFFF);    
    getStatus(spi, LORA_BUSY, true);

    // Create the queue
    queueMsg = xQueueCreate(10, PACKET_SIZE);

    while(1) {
        // Wait, so the watchdog doesn't kick in
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ret = gpio_set_level(GPIO_NUM_21, 1);

        // If we have something on the queue
        if(queueLength > 0) {
            ret = gpio_set_level(GPIO_NUM_21, 0);

            // Assign storate for the data
            uint8_t data[PACKET_SIZE];

            // Collect data off the queue
            xQueueReceive(queueMsg, data, portMAX_DELAY);

            // Decrement the counter
            queueLength--;

            // Do something, such as print the payload and calculate the internal CRC
            for (int x=0; x < PACKET_SIZE; x++) {
                printf("%02x ", data[x]);
            }
            printf("- CRC Check: %02x\n", crc8(data+6, PACKET_SIZE - 7));
        }
    }
}
