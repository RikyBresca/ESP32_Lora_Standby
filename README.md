# ESP32 LoRa Standby

This project demonstrates how to use an ESP32 with a LoRa E220 module to send data via LoRa only when the device is woken up by an external pin. After sending the data, the ESP32 automatically returns to deep sleep to save power.

## Features

- Wakeup from deep sleep via external pin (`AUX_WAKEUP_PIN`)
- Reading an analog value (ADC)
- Sending the read value via LoRa E220 module
- Automatic return to deep sleep

## Hardware

- ESP32 Dev Board
- LoRa E220 module (E220-900T22S or similar)
- GPIO connections:
  - `AUX_WAKEUP_PIN`: GPIO 3 (input for wakeup)
  - `ADC_PIN`: GPIO 4 (analog reading)
  - `E220_TX`: GPIO 20
  - `E220_RX`: GPIO 21
  - `E220_AUX`: GPIO 10 (AUX for the library)
  - `E220_M0`: GPIO 8
  - `E220_M1`: GPIO 9
    NOTE: set right RX and TX for serial

## Wiring Table

| ESP32 GPIO | E220 Pin     |
| ---------- | ------------ |
| 20         | RX           |
| 21         | TX           |
| 10         | AUX          |
| 9          | M0           |
| 8          | M1           |
| 4          | Wakeup Input |
| 3          | ADC Input    |

## Dependencies

- [Arduino core for ESP32](https://github.com/espressif/arduino-esp32)
- [LoRa_E220 library](https://github.com/xreef/LoRa_E220_Series_Library)

## Installation

1. Clone the repository:
   ```sh
   git clone https://github.com/your-username/ESP32_Lora_Standby.git
   ```
2. Install the dependencies using PlatformIO or Arduino IDE.
3. Connect the hardware as shown above.

## Usage

1. Upload the sketch to the ESP32.
2. On first boot, the device enters deep sleep.
3. When the `AUX_WAKEUP_PIN` is pulled low, the ESP32 wakes up, reads the ADC value, sends it via LoRa, and returns to deep sleep.

## Pre-commit

The repository includes a `.pre-commit-config.yaml` for automatic formatting of C++ code (`clang-format`) and Markdown files (`prettier`).

## DEBUG

To debug `ESP32-C3` it is needed to install driver and ESP TOOL. [Download here](https://docs.espressif.com/projects/esp-idf/en/stable/esp32s3/get-started/windows-setup.html#get-started-windows-tools-installer)

## License

MIT
