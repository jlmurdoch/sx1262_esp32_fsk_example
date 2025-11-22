# SX1262 ESP32 FSK Example
This is a short example of how to use the [Semtech SX1262 LoRa Connect](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262):tm::
- (G)FSK packet mode. No LoRa yet.
- Little obfuscation or library use so the workings can be understood for difficult use-cases / debugging. 
- This example transmits a packet and then goes into continuous receiving.
- It performs CRC-16 UMTS generation and checking, which is not available in other chipsets and allows rapid discarding of junk packets.

### Hardware notes:
- Written for a [Seeed Studio XIAO ESP32-S3 Plus and Wio-SX1262 radio](https://wiki.seeedstudio.com/wio_sx1262_with_xiao_esp32s3_kit/#introduction).
- This chipset has configurable CRC, allowing a [wide range of CRC schemes](https://reveng.sourceforge.io/crc-catalogue/all.htm).
- The SX1261/2 is packet-focused, so it is impractical for signal discovery without an SDR to assist.
- Configurations needed for the Wio-SX1262 are:
    - [Use the B2B connector for GPIO pins](https://github.com/Lora-net/one_channel_hub/blob/7c0be623fefb1a16afd9372d10c978bddbf6f29e/components/smtc_ral/bsp/sx126x/seeed_xiao_sx1262.c#L359) otherwise [use this for external pins](https://github.com/Lora-net/one_channel_hub/blob/7c0be623fefb1a16afd9372d10c978bddbf6f29e/components/smtc_ral/bsp/sx126x/seeed_xiao_sx1262.c#L372).
    - Use DIO2 as RF switch control.
    - [Use DIO3 for TCXO Control at 3.0V](https://github.com/Lora-net/one_channel_hub/blob/7c0be623fefb1a16afd9372d10c978bddbf6f29e/components/smtc_ral/bsp/sx126x/seeed_xiao_sx1262.c#L351).

### Development notes
- Written for the [Espressif IoT Development Framework](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html) (ESP-IDF) and Visual Studio Code, as the Arduino IDE has fallen out of favour with the author. :disappointed: Some Arduino challenges were:
  - [No official Linux ARM64 support for the Arduino IDE](https://github.com/arduino/arduino-ide/issues/107)
  - [No official VS Code extension](https://github.com/microsoft/vscode-arduino)
  - Frequently having to reassert code when libraries change.
- No 3rd-party libraries used here other than those supplied with ESP IDF.
- Code is written to reflect the keywords / functions in the datasheet for easy reference.
- Use of lightweight SPI/command wrapper functions to simplify the use of variable address and dummy bits.
- VS Code comments for the keywords / functions. 
- The main flow is that of the Basic Tx and Rx Application guide in the datasheet:
  - Configure both modes, consolidating the setup into one configuration sequence.
  - Enter transmit mode and send one packet.
  - Enter receive mode forever (continuous mode).
- SPI throws errors beyond 11MHz, regardless of DMA -  the software SPI may need some tuning.
- SPI runs in half-duplex and variable dummy bits for NOPs before/during reading.
- SPI could be run in full-duplex, but no operations need it.
- SPI uses variable addresses for commands (0), buffer offsets (8), and register addresses (16).
- For TX completion, it's better to check GetStatus() and look for a return to STDBY_RC & TXDone. IRQ for TXDone seems to be populated after this has been done.
- For RX in continuous mode, GetPacketStatus() doesn't reset. Best option is to poll IRQ and clear immediately.