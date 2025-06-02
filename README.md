# ESP32 LoRa Standby

This project demonstrates how to use an ESP32 with a LoRa E220 module to send data via LoRa only when the device is woken up by an external pin. After sending the data, the ESP32 automatically returns to deep sleep to save power.

## Features

- Wakeup from deep sleep via external pin (`AUX_WAKEUP_PIN`)
- Reading an analog value (ADC)
- Sending the read value via LoRa E220 module
- Automatic return to deep sleep

## Logic Overview (`main.cpp`)

The main application logic is implemented in `main.cpp` and is structured as follows:

- **Device Role Selection:**  
  The code can be compiled either as a sender or receiver by defining `SENDER_DEVICE` or `RECEIVE_DEVICE`. Only one role should be active at a time.

- **Initialization:**  
  On startup, the ESP32 initializes serial communication, configures the LoRa E220 module, and sets up the required GPIO pins.

- **Sender Mode:**

  - The sender can operate in a normal mode or a test mode (enabled by defining `TEST_MODE`).
  - In test mode, a serial menu allows the user to initiate a test sequence. The sender sends a "TEST" message to each slave device and waits for an "OK" response, reporting the results for each device.
  - In normal operation, the sender wakes up (e.g., from deep sleep via `AUX_WAKEUP_PIN`), reads an ADC value, sends it via LoRa, and returns to deep sleep.

- **Receiver Mode:**

  - The receiver listens for incoming messages from the sender.
  - If a "TEST" message is received, it replies with "OK".
  - For other messages, it prints the received data and sender information to the serial monitor.

- **Deep Sleep Handling:**

  - The ESP32 can be configured to enter deep sleep and wake up on an external pin (`AUX_WAKEUP_PIN`).
  - After completing its task (sending or receiving), the device can return to deep sleep to save power.

- **LoRa E220 Configuration:**

  - The code sets up the LoRa E220 module with the appropriate addresses, channel, and communication parameters.
  - Device addresses and names are managed using enums and arrays for easy expansion.

- **Testing and Debugging:**
  - The code includes serial output for debugging and status reporting.
  - A test mode is available for verifying communication with all slave devices.

## Hardware

- ESP32 Dev Board
- LoRa E220 module (E220-900T22S or similar)
- GPIO connections:
  - `AUX_WAKEUP_PIN`: GPIO 0 (input for wakeup)
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
| 0          | Wakeup Input |
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
