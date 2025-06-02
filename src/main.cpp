/**
 * @file main.cpp
 * @brief ESP32 LoRa Standby Example
 *
 * This example demonstrates how to use the ESP32 with a LoRa E220 module,
 * enter deep sleep, and wake up using an external pin (AUX_WAKEUP_PIN).
 * Upon wakeup, it reads an ADC value, sends it via LoRa, and returns to sleep.
 */

// Frequency band definition
//  Uncomment the following line to use the 433 MHz frequency band
// #define FREQUENCY_868
// #define LoRa_E220_DEBUG

#include <Arduino.h>
#include <sdkconfig.h>

#include "LoRa_E220.h"
#include "esp_sleep.h"

// Define the device type
// #define SENDER_DEVICE  // Uncomment this line for sender device
#define TEST_MODE  // Uncomment this line to enable test mode for sender device

#define RECEIVE_DEVICE  // Uncomment this line for receiver device

// Ensure only one device type is defined
#if defined(SENDER_DEVICE) && defined(RECEIVE_DEVICE)
#error "Only one of SENDER_DEVICE or RECEIVE_DEVICE should be defined."
#endif
#if !defined(SENDER_DEVICE) && !defined(RECEIVE_DEVICE)
#error "Either SENDER_DEVICE or RECEIVE_DEVICE must be defined."
#endif

// Write message to notify mode type
#if defined(SENDER_DEVICE)
#pragma message("Compiling for SENDER_DEVICE")
#define PRINT_MODE "SENDER_DEVICE"
#endif

#if defined(RECEIVE_DEVICE)
#pragma message("Compiling for RECEIVE_DEVICE")
#define PRINT_MODE "RECEIVE_DEVICE"
#endif

#undef ENABLE_RSSI

/**
 * @def ADC_PIN
 * @brief GPIO pin used for ADC voltage reading.
 */
#define ADC_PIN 4

/**
 * @def E220_TX
 * @brief GPIO pin connected to E220 TX.
 */
#define E220_TX 20

/**
 * @def E220_RX
 * @brief GPIO pin connected to E220 RX.
 */
#define E220_RX 21

/**
 * @def E220_AUX
 * @brief GPIO pin connected to E220 AUX (for library, not for wakeup).
 */
#define E220_AUX 0  // AUX per la libreria, NON quello di wakeup

/**
 * @def AUX_WAKEUP_PIN
 * @brief GPIO pin used to wake up ESP32 from deep sleep.
 */
#define AUX_WAKEUP_PIN E220_AUX
/**
 * @def E220_M0
 * @brief GPIO pin connected to E220 M0.
 */
#define E220_M0 8

/**
 * @def E220_M1
 * @brief GPIO pin connected to E220 M1.
 */
#define E220_M1 9

/**
 * @def E220_ADDH
 * @brief Address High byte for E220 module.
 */
#define E220_ADDH 0x00

/**
 * @def E220_ADDL
 * @brief Address Low byte for E220 module.
 */
#define E220_ADDL 0x01

/**
/**
 * @def E220_CH
 * @brief Communication channel for E220 module.
 */
#define E220_CH 18

#define PIN_LED 5  // Pin for LED or other output, used in receive mode

/**
 * @def MESSAGE_TO_CHECK
 * @brief Message to check for in the received data.
 */
#define MESSAGE_TO_CHECK "START 1"

#define TEST_TIME_OUT (3000)

/**
 * @brief LoRa E220 module instance.
 */
LoRa_E220 e220ttl(&Serial0, E220_AUX, E220_M0, E220_M1);

/**
 * @brief Prints the reason for the last wakeup from deep sleep.
 */
void print_wakeup_reason();

/**
 * @brief Prints the parameters of the E220 module configuration.
 * @param configuration The configuration structure to print.
 */
void printParameters(struct Configuration configuration);

#if defined(RECEIVE_DEVICE)
/**
 * @brief Main loop function for receiving mode.
 */
void loop_receive_fnc_lora_e220();

/**
 * @brief Setup function for receiving mode.
 */
void setup_receive_fnc_lora_e220();

/**
 * @brief Setup function for deep sleep in receive mode.
 */
void setup_receive_sleep();

/**
 * @brief Loop function for deep sleep in receive mode.
 */
void loop_receive_sleep();

#elif defined(SENDER_DEVICE)
/**
 * @brief Main loop function for sender mode.
 */
void loop_sender_fnc_lora_e220();

/**
 * @brief Setup function for sender mode.
 */
void setup_sender_fnc_lora_e220();

#if defined(TEST_MODE)
/**
 * @brief Handles the testing process for all slave devices connected via Lora E220.
 */
void test_sender_fnc_lora_e220();
#endif
#endif

struct Message
{
  uint8_t ADDH;  // Address High byte
  uint8_t ADDL;  // Address Low byte
  uint8_t CHAN;  // Communication channel
  char message[10];
};

typedef enum
{
  DEVICE_MASTER_ADDL = 0x00,  // Address Low byte for Master Device
  //------------------------------------------------------------------
  DEVICE_1_ADDL = 0x01,  // Address Low byte for Device 1
  DEVICE_2_ADDL = 0x02,  // Address Low byte for Device 2
  DEVICE_3_ADDL = 0x03,  // Address Low byte for Device 3
  //------------------------------------------------------------------
  DEVICE_MAX_ADDL,  // Maximum Address Low byte for devices
} DeviceAddress;

#if defined(SENDER_DEVICE) && E220_ADDL != DEVICE_MASTER_ADDL && E220_ADDL < DEVICE_1_ADDL
#pragma error \
    "The SENDER_DEVICE E220_ADDL must be set to DEVICE_MASTER_ADDL or a value greater than DEVICE_1_ADDL. Please change the E220_ADDL value."
#endif

#if defined(RECEIVE_DEVICE) && E220_ADDL > DEVICE_MAX_ADDL && E220_ADDL < DEVICE_1_ADDL
#pragma error \
    "The RECEIVE_DEVICE E220_ADDL must be set to a value between DEVICE_1_ADDL and DEVICE_MAX_ADDL. Please change the E220_ADDL value."
#endif

String device_name[DEVICE_MAX_ADDL] = {
    "Master Device", "Device Pippo", "Device Pluto", "Device Paperino",
    // Add more device names as needed
};

/**
 * @brief Arduino setup function. Initializes serial, configures pins, handles wakeup, and enters deep sleep.
 */
void setup()
{
  Serial.begin(115200);

  // communication
  while (!Serial)
  {
    ;
  }

  Serial.println(F("LoRa E220 MODE - " PRINT_MODE));
#if defined(RECEIVE_DEVICE)
  setup_receive_fnc_lora_e220();
  setup_receive_sleep();  // Setup for deep sleep mode
#elif defined(SENDER_DEVICE)
  setup_sender_fnc_lora_e220();
#endif
}

/**
 * @brief Arduino loop function. Main application loop.
 */
void loop()
{
#if defined(RECEIVE_DEVICE)
  loop_receive_fnc_lora_e220();
  loop_receive_sleep();  // Enter deep sleep after processing
#elif defined(SENDER_DEVICE)
  loop_sender_fnc_lora_e220();
#endif
}

/**
 * @brief Prints the parameters of the E220 module configuration.
 * @param configuration The configuration structure to print.
 */
void printParameters(struct Configuration configuration)
{
  Serial.println("----------------------------------------");

  Serial.print(F("HEAD : "));
  Serial.print(configuration.COMMAND, HEX);
  Serial.print(" ");
  Serial.print(configuration.STARTING_ADDRESS, HEX);
  Serial.print(" ");
  Serial.println(configuration.LENGHT, HEX);
  Serial.println(F(" "));
  Serial.print(F("AddH : "));
  Serial.println(configuration.ADDH, HEX);
  Serial.print(F("AddL : "));
  Serial.println(configuration.ADDL, HEX);
  Serial.println(F(" "));
  Serial.print(F("Chan : "));
  Serial.print(configuration.CHAN, DEC);
  Serial.print(" -> ");
  Serial.println(configuration.getChannelDescription());
  Serial.println(F(" "));
  Serial.print(F("SpeedParityBit     : "));
  Serial.print(configuration.SPED.uartParity, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTParityDescription());
  Serial.print(F("SpeedUARTDatte     : "));
  Serial.print(configuration.SPED.uartBaudRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getUARTBaudRateDescription());
  Serial.print(F("SpeedAirDataRate   : "));
  Serial.print(configuration.SPED.airDataRate, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.SPED.getAirDataRateDescription());
  Serial.println(F(" "));
  Serial.print(F("OptionSubPacketSett: "));
  Serial.print(configuration.OPTION.subPacketSetting, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getSubPacketSetting());
  Serial.print(F("OptionTranPower    : "));
  Serial.print(configuration.OPTION.transmissionPower, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getTransmissionPowerDescription());
  Serial.print(F("OptionRSSIAmbientNo: "));
  Serial.print(configuration.OPTION.RSSIAmbientNoise, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.OPTION.getRSSIAmbientNoiseEnable());
  Serial.println(F(" "));
  Serial.print(F("TransModeWORPeriod : "));
  Serial.print(configuration.TRANSMISSION_MODE.WORPeriod, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  Serial.print(F("TransModeEnableLBT : "));
  Serial.print(configuration.TRANSMISSION_MODE.enableLBT, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  Serial.print(F("TransModeEnableRSSI: "));
  Serial.print(configuration.TRANSMISSION_MODE.enableRSSI, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  Serial.print(F("TransModeFixedTrans: "));
  Serial.print(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);
  Serial.print(" -> ");
  Serial.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());

  Serial.println("----------------------------------------");
}

// This function is required to avoid the ESP32-IDF
extern "C" void app_main()
{
  initArduino();
  setup();  // Call the setup function
  while (true)
  {
    loop();  // Call the loop function
  }
}

#if defined(RECEIVE_DEVICE)

/**
 * @brief Setup function for deep sleep in receive mode.
 */
void setup_receive_sleep()
{
  pinMode(AUX_WAKEUP_PIN, INPUT_PULLDOWN);  // Imposta il pin come input

  // Set the wakeup pin
  esp_deep_sleep_enable_gpio_wakeup(1 << AUX_WAKEUP_PIN, ESP_GPIO_WAKEUP_GPIO_HIGH);
}

/**
 * @brief Loop function for deep sleep in receive mode.
 */
void loop_receive_sleep()
{
  // Enter deep sleep mode
  Serial.println(F("Entering deep sleep..."));
  delay(1000);
  esp_deep_sleep_start();
}

/**
 * @brief Setup function for LoRa E220 in receive mode.
 */
void setup_receive_fnc_lora_e220()
{
  // Startup all pins and UART
  e220ttl.begin();

  ResponseStructContainer c;
  c = e220ttl.getConfiguration();
  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration*)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);

  printParameters(configuration);
  configuration.ADDL = E220_ADDL;  // First part of address
  configuration.ADDH = 0x00;       // Second part

  configuration.CHAN = E220_CH;  // Communication channel

  configuration.SPED.uartBaudRate = UART_BPS_9600;        // Serial baud rate
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;  // Air baud rate
  configuration.SPED.uartParity = MODE_00_8N1;            // Parity bit

  configuration.OPTION.subPacketSetting = SPS_200_00;                   // Packet size
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;  // Need to send special command
  configuration.OPTION.transmissionPower = POWER_22;                    // Device power

  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;                 // Enable RSSI info
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;  // Enable repeater mode
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;                   // Check interfere
  ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);

  c = e220ttl.getConfiguration();

  // It's important get configuration pointer before all other operation
  configuration = *(Configuration*)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);

  printParameters(configuration);
  c.close();

  Serial.println("Ready to receive data on LoRa E220 module.");
}

/**
 * @brief Main loop function for LoRa E220 in receive mode.
 */
void loop_receive_fnc_lora_e220()
{
  // Se è arrivato un messaggio
  uint32_t loop = 0;
  do
  {
    if (e220ttl.available() > 1)
    {
      Serial.println("Message received!");

      ResponseStructContainer rc = e220ttl.receiveMessage(sizeof(Message));
      if (rc.status.code != 1)
      {
        Serial.println(rc.status.getResponseDescription());
      }
      else
      {
        Message receivedMessage = *(Message*)rc.data;
        Serial.println(rc.status.getResponseDescription());
        Serial.print("Data: ");
        Serial.println(receivedMessage.message);
        Serial.print("From ADDH: ");
        Serial.print(receivedMessage.ADDH, HEX);
        Serial.print(" ADDL: ");
        Serial.println(receivedMessage.ADDL, HEX);
        Serial.print("Channel: ");
        Serial.println(receivedMessage.CHAN, HEX);

        // Se il messaggio è "TEST" rispondi "OK"
        /**
         * @brief Handles incoming "TEST" messages.
         *
         * If a message with content "TEST" is received, this block sends back an "OK" response
         * to the sender using the same addressing and channel information.
         */
        if (strcmp(receivedMessage.message, "TEST") == 0)
        {
          Message sendMessage = {E220_ADDH, E220_ADDL, E220_CH, "OK"};
          e220ttl.sendFixedMessage(receivedMessage.ADDH, receivedMessage.ADDL, receivedMessage.CHAN, &sendMessage,
                                   sizeof(Message));
          Serial.println("OK response sent!");
          break;  // Exit the loop after sending the response
        }
        else
        {
          /**
           * @brief Handles unrecognized messages.
           *
           * If the received message does not match the expected "TEST" string,
           * this block prints a notification that the message is not recognized.
           */
          Serial.println("Unrecognized message.");
        }
      }
    }
    delay(100);
    loop++;
    Serial.println("Waiting for messages...");
    Serial.print("Loop count: ");
    Serial.println(loop);
  } while (loop < 100);
}

#elif defined(SENDER_DEVICE)
/**
 * @brief Setup function for LoRa E220 in sender mode.
 */
void setup_sender_fnc_lora_e220()
{
  // Startup all pins and UART
  e220ttl.begin();

  ResponseStructContainer c;
  c = e220ttl.getConfiguration();
  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration*)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);

  printParameters(configuration);
  configuration.ADDL = E220_ADDL;  // First part of address
  configuration.ADDH = 0x00;       // Second part

  configuration.CHAN = E220_CH;  // Communication channel

  configuration.SPED.uartBaudRate = UART_BPS_9600;        // Serial baud rate
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;  // Air baud rate
  configuration.SPED.uartParity = MODE_00_8N1;            // Parity bit

  configuration.OPTION.subPacketSetting = SPS_200_00;                   // Packet size
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;  // Need to send special command
  configuration.OPTION.transmissionPower = POWER_22;                    // Device power

  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;                 // Enable RSSI info
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;  // Enable repeater mode
  configuration.TRANSMISSION_MODE.enableLBT = LBT_DISABLED;                   // Check interfere
  ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);

  c = e220ttl.getConfiguration();
  // It's important get configuration pointer before all other operation
  configuration = *(Configuration*)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);

  printParameters(configuration);
  c.close();

  Serial.println("Ready to send data on LoRa E220 module.");
}

/**
 * @brief Main loop function for LoRa E220 in sender mode.
 */
void loop_sender_fnc_lora_e220()
{

#if defined(TEST_MODE)
  test_sender_fnc_lora_e220();
#endif
  // Delay to avoid wd reset
  //  This is important to avoid watchdog reset
  delay(1);
}

/**
 * @brief Handles the testing process for all slave devices connected via Lora E220.
 *
 * This function provides a simple serial menu to initiate a test sequence or return to normal operation.
 * When the test is started, it sends a test message ("TEST") to each slave device in sequence and waits
 * for a response ("OK") from each. The response status for each device is tracked and, upon completion,
 * a summary of the results is printed to the serial monitor, indicating which devices responded and which did not.
 *
 * The function uses static variables to maintain state across calls, allowing it to be called repeatedly
 * (e.g., from the main loop) without losing progress.
 *
 * User interaction:
 * - Sends a menu prompt via Serial.
 * - Waits for user input ('1' to start the test).
 * - Displays test results for each device after completion.
 *
 * Communication:
 * - Sends a fixed message to each device.
 * - Waits up to TEST_TIME_OUT milliseconds for a response.
 * - Marks devices as "OK" if a valid response is received, otherwise as "NO RESPONSE".
 *
 * @note This function is designed to be called repeatedly, such as from the main loop.
 * @note Relies on global variables/constants: DEVICE_1_ADDL, DEVICE_MAX_ADDL, device_name, E220_ADDH, E220_ADDL,
 * E220_CH, TEST_TIME_OUT, e220ttl.
 */
void test_sender_fnc_lora_e220()
{
  static bool testInProgress = false;
  static int currentDevice = DEVICE_1_ADDL;
  static unsigned long testStartTime = 0;
  static bool deviceResponded[DEVICE_MAX_ADDL] = {false};
  static bool testCompleted = false;
  static bool menuShown = false;

  // Mostra il menu solo una volta
  if (!menuShown)
  {
    Serial.println("Send 1 for test and send 2 for normal operation");
    menuShown = true;
  }

  // Menù semplice via seriale
  if (!testInProgress && Serial.available())
  {
    char cmd = Serial.read();
    if (cmd == '1')
    {
      Serial.println("Avvio test di tutti i device slave...");
      testInProgress = true;
      currentDevice = DEVICE_1_ADDL;
      memset(deviceResponded, 0, sizeof(deviceResponded));
      testCompleted = false;
    }
  }

  if (testInProgress && !testCompleted)
  {
    if (currentDevice < DEVICE_MAX_ADDL)
    {
      // Invia messaggio di test al device corrente
      Message message = {E220_ADDH, E220_ADDL, E220_CH, "TEST"};
      ResponseStatus rs = e220ttl.sendFixedMessage(E220_ADDH, currentDevice, E220_CH, &message, sizeof(Message));
      Serial.print("Test al device ADDL: ");
      Serial.print(currentDevice, HEX);
      Serial.print(" - ");
      Serial.println(rs.getResponseDescription());
      testStartTime = millis();

      // Attendi risposta per max 3 secondi
      bool rispostaRicevuta = false;
      while (millis() - testStartTime < TEST_TIME_OUT)
      {
        if (e220ttl.available() > 1)
        {
          ResponseStructContainer rc = e220ttl.receiveMessage(sizeof(Message));
          if (rc.status.code == 1)
          {
            Message receivedMessage = *(Message*)rc.data;
            if (strcmp(receivedMessage.message, "OK") == 0 && receivedMessage.ADDL == currentDevice)
            {
              rispostaRicevuta = true;
              deviceResponded[currentDevice] = true;
              Serial.print("Response received from device OK");
              Serial.println(currentDevice, HEX);
              break;
            }
          }
        }
        delay(10);
      }
      /// @brief If no response is received from the current device, print a message.
      if (!rispostaRicevuta)
      {
        Serial.print("No response from device ");
        Serial.println(currentDevice, HEX);
      }
      currentDevice++;
      delay(200);  // Small pause between tests
    }
    else
    {
      /**
       * @brief Test completed: print the results for all devices.
       * Iterates through all slave devices and prints whether each device responded ("OK")
       * or did not respond ("NO RESPONSE").
       */
      Serial.println("Device test results:");
      for (int i = DEVICE_1_ADDL; i < DEVICE_MAX_ADDL; i++)
      {
        Serial.print("Device ");
        Serial.print(i, HEX);
        Serial.print("-> ");
        Serial.print(device_name[i].c_str());
        Serial.print(": ");
        Serial.println(deviceResponded[i] ? "OK" : "NO RESPONSE");
      }
      testCompleted = true;
      testInProgress = false;
      menuShown = false;  // Resetta il flag per mostrare il menu di nuovo
    }
  }
}
#endif