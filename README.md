# SX1262 ESP32 FSK Example
This is a short example of how to use the [Semtech SX1262 LoRa Connect](https://www.semtech.com/products/wireless-rf/lora-connect/sx1262):tm: in (G)FSK packet mode with little obfuscation when working with the radio. At the moment this is only a packet-receiving example.

### Development notes
- Written for the [Espressif IoT Development Framework](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html) (ESP-IDF) and Visual Studio Code, as the Arduino IDE has fallen out of favour with the author. :disappointed: Some Arduino challenges were:
  - [No official Linux ARM64 support](https://github.com/arduino/arduino-ide/issues/107)
  - [No official VS Code extension](https://github.com/microsoft/vscode-arduino)
  - Frequently having to correct things when libraries change.
- No 3rd-party libraries used here other than those supplied with ESP IDF.
- Code is written to reflect the keywords / functions in the datasheet for easy reference.
- The main flow is that of the Basic Rx Application guide in the datasheet, with it remaining in Rx mode forever (continuous mode).
- SPI runs in half-duplex and variable dummy bits for NOPs before/during reading.
- SPI uses variable addresses for commands (0), buffer offsets (8), and register addresses (16), 
- Use of SPI/command wrapper functions to simplify the use of variable address and dummy bits.

### Hardware notes:
- Written for a [Seeed Studio XIAO ESP32-S3 Plus and Wio-SX1262 radio](https://wiki.seeedstudio.com/wio_sx1262_with_xiao_esp32s3_kit/#introduction).
- This chipset has configurable CRC, which is not present in some other LoRa chipsets. This permits a fast CRC check. 
- The SX1261/2 is entirely packet-focused, so it is impractical for signal discovery without an SDR to assist.
- The SPI throws errors beyond 11MHz the software SPI may need some tuning.
- Software customisations needed for this hardware are:
    - Use DIO2 as RF switch control
    - Use DIO3 for TCXO Control at 3.0V

