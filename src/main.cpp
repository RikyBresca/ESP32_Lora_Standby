/**
 * @file main.cpp
 * @brief ESP32 LoRa Standby Example
 *
 * This example demonstrates how to use the ESP32 with a LoRa E220 module,
 * enter deep sleep, and wake up using an external pin (AUX_WAKEUP_PIN).
 * Upon wakeup, it reads an ADC value, sends it via LoRa, and returns to sleep.
 */

#include <Arduino.h>
#include <sdkconfig.h>

#include "LoRa_E220.h"
#include "esp_sleep.h"

/* * @def LoRa_E220_DEBUG
 * @brief Uncomment to enable debug messages.
 * Comment to disable debug messages.
 */

/* * @def LORA_RECEIVE
 * @brief Uncomment to enable LoRa receive mode.
 * Comment to enable LoRa send mode.
 */
#define LORA_RECEIVE

/**
 * @def AUX_WAKEUP_PIN
 * @brief GPIO pin used to wake up ESP32 from deep sleep.
 */
#define AUX_WAKEUP_PIN 3

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
#define E220_AUX 10  // AUX per la libreria, NON quello di wakeup

/**
 * @def E220_M0
 * @brief GPIO pin connected to E220 M0.
 */
#define E220_M0 9

/**
 * @def E220_M1
 * @brief GPIO pin connected to E220 M1.
 */
#define E220_M1 8

/**
 * @def E220_ADDH
 * @brief Address High byte for E220 module.
 */
#define E220_ADDH 0x00

/**
 * @def E220_ADDL
 * @brief Address Low byte for E220 module.
 */
#define E220_ADDL 0x03

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
 *
 * @param configuration The configuration structure to print.
 */
void printParameters(struct Configuration configuration);

/**
 * @brief Function to receive data from the E220 module.
 *
 * @param e220 Reference to the LoRa_E220 instance.
 */
void receive_fnc_lora_e220(LoRa_E220 &e220);

/**
 * @brief Function to send data via the E220 module.
 *
 * @param e220 Reference to the LoRa_E220 instance.
 */
void send_fnc_lora_e220(LoRa_E220 &e220);

/**
 * @brief Arduino setup function. Initializes serial, configures pins, handles wakeup, and enters
 * deep sleep.
 */
void setup()
{
  Serial.begin(115200);

  // communication
  while (!Serial)
  {
    ;
  }

#ifdef LORA_RECEIVE                // If LORA_RECEIVE is defined, we will receive data
  receive_fnc_lora_e220(e220ttl);  ///< Call the function to receive data from E220
#else                              // If LORA_RECEIVE is not defined, we will send data
  send_fnc_lora_e220(e220ttl);  ///< Call the function to send data via E220
#endif
}

/**
 * @brief Arduino loop function. Not used in this example.
 */
void loop()
{
#ifdef LORA_RECEIVE
  // In receive mode, we don't need to do anything in the loop.
  // The receive function handles everything.
  if (e220ttl.available() > 1)
  {
    Serial.println("E220 message received!");
    Serial.println("Reading message from E220...");
    ResponseContainer rc = e220ttl.receiveMessage();  ///< Receive message from E220

    // Check if the message is received successfully
    if (rc.status.code != 1)
    {
      Serial.println(rc.status.getResponseDescription());
    }
    else
    {
      Serial.print("Received message: ");
      Serial.println(rc.status.getResponseDescription());
      Serial.println(rc.data);

      /**
       * @brief If the received data is "START 1", set pin 12 HIGH for 500ms.
       */
      if (String(rc.data) == MESSAGE_TO_CHECK)
      {
        Serial.println("Command START 1 received: setting pin 12 HIGH");
        digitalWrite(PIN_LED, HIGH);  ///< Set pin 12 HIGH
        delay(500);                   ///< Keep pin 12 HIGH for 500ms
        digitalWrite(PIN_LED, LOW);   ///< Set pin 12 LOW
      }
    }
  }
  else
  {
    Serial.println("No message received.");
  }
  delay(100);  ///< Delay to avoid flooding the serial output
#else
  // In send mode, we can send data or perform other tasks.
  // This example does not implement sending in the loop.
  Serial.println("Send data via E220...");
  String msg = MESSAGE_TO_CHECK;  ///< Prepare voltage message
  ResponseStatus rs = e220ttl.sendFixedMessage(0, 3, 0x01, msg.c_str(),
                                               msg.length());  ///< Send voltage message via E220
  Serial.print("Send status: ");
  Serial.println(rs.getResponseDescription());
  delay(1000);  ///< Delay to avoid flooding the serial output
#endif
}

void printParameters(struct Configuration configuration)
{
  DEBUG_PRINTLN("----------------------------------------");

  DEBUG_PRINT(F("HEAD : "));
  DEBUG_PRINT(configuration.COMMAND, HEX);
  DEBUG_PRINT(" ");
  DEBUG_PRINT(configuration.STARTING_ADDRESS, HEX);
  DEBUG_PRINT(" ");
  DEBUG_PRINTLN(configuration.LENGHT, HEX);
  DEBUG_PRINTLN(F(" "));
  DEBUG_PRINT(F("AddH : "));
  DEBUG_PRINTLN(configuration.ADDH, HEX);
  DEBUG_PRINT(F("AddL : "));
  DEBUG_PRINTLN(configuration.ADDL, HEX);
  DEBUG_PRINTLN(F(" "));
  DEBUG_PRINT(F("Chan : "));
  DEBUG_PRINT(configuration.CHAN, DEC);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.getChannelDescription());
  DEBUG_PRINTLN(F(" "));
  DEBUG_PRINT(F("SpeedParityBit     : "));
  DEBUG_PRINT(configuration.SPED.uartParity, BIN);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.SPED.getUARTParityDescription());
  DEBUG_PRINT(F("SpeedUARTDatte     : "));
  DEBUG_PRINT(configuration.SPED.uartBaudRate, BIN);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.SPED.getUARTBaudRateDescription());
  DEBUG_PRINT(F("SpeedAirDataRate   : "));
  DEBUG_PRINT(configuration.SPED.airDataRate, BIN);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.SPED.getAirDataRateDescription());
  DEBUG_PRINTLN(F(" "));
  DEBUG_PRINT(F("OptionSubPacketSett: "));
  DEBUG_PRINT(configuration.OPTION.subPacketSetting, BIN);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.OPTION.getSubPacketSetting());
  DEBUG_PRINT(F("OptionTranPower    : "));
  DEBUG_PRINT(configuration.OPTION.transmissionPower, BIN);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.OPTION.getTransmissionPowerDescription());
  DEBUG_PRINT(F("OptionRSSIAmbientNo: "));
  DEBUG_PRINT(configuration.OPTION.RSSIAmbientNoise, BIN);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.OPTION.getRSSIAmbientNoiseEnable());
  DEBUG_PRINTLN(F(" "));
  DEBUG_PRINT(F("TransModeWORPeriod : "));
  DEBUG_PRINT(configuration.TRANSMISSION_MODE.WORPeriod, BIN);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getWORPeriodByParamsDescription());
  DEBUG_PRINT(F("TransModeEnableLBT : "));
  DEBUG_PRINT(configuration.TRANSMISSION_MODE.enableLBT, BIN);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  DEBUG_PRINT(F("TransModeEnableRSSI: "));
  DEBUG_PRINT(configuration.TRANSMISSION_MODE.enableRSSI, BIN);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getRSSIEnableByteDescription());
  DEBUG_PRINT(F("TransModeFixedTrans: "));
  DEBUG_PRINT(configuration.TRANSMISSION_MODE.fixedTransmission, BIN);
  DEBUG_PRINT(" -> ");
  DEBUG_PRINTLN(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());

  DEBUG_PRINTLN("----------------------------------------");
}

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by AUX pin");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
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

// Receive functions
void receive_fnc_lora_e220(LoRa_E220 &e220)
{
  // Pin configuration
  pinMode(AUX_WAKEUP_PIN, INPUT_PULLUP);  ///< Configure AUX_WAKEUP_PIN as input with pull-up
  pinMode(PIN_LED, OUTPUT);               ///< Set pin 5 as OUTPUT for command action
  digitalWrite(PIN_LED, LOW);             ///< Ensure pin 5 is LOW at startup

#ifdef ESP32_SLEEP_WAKEUP_EXT0
  print_wakeup_reason();
  Serial.println("Configuring AUX wakeup pin...");
  esp_deep_sleep_enable_gpio_wakeup(
      (1ULL << AUX_WAKEUP_PIN),
      ESP_GPIO_WAKEUP_GPIO_LOW);  ///< Enable external wakeup on AUX_WAKEUP_PIN
#endif
  // Initialize E220 module
  Serial.println("Initializing E220 module...");
  if (e220ttl.begin())
  {  ///< Initialize the E220 LoRa module
    delay(1000);
    Serial.println("E220 module initialized.");
  }
  else
  {
    Serial.println("Failed to initialize E220 module.");
    return;
  }

  // E220 configuration
  Serial.println("Configuring E220 module...");
  ResponseStructContainer c;
  c = e220ttl.getConfiguration();

  // It's important get configuration pointer before all other operation
  Configuration configuration = *(Configuration *)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);

  printParameters(configuration);

  configuration.ADDL = E220_ADDL;  // First part of address
  configuration.ADDH = E220_ADDH;  // Second part

  configuration.CHAN = E220_CH;  // Communication channel

  // Set configuration changed and set to not hold the configuration
  ResponseStatus rs = e220ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial.println(rs.getResponseDescription());
  Serial.println(rs.code);

  c = e220ttl.getConfiguration();
  // It's important get configuration pointer before all other operation
  configuration = *(Configuration *)c.data;
  Serial.println(c.status.getResponseDescription());
  Serial.println(c.status.code);
  Serial.println("Configuration: ");
  printParameters(configuration);
  Serial.println("E220 module configured successfully.");
  delay(1000);
  c.close();

  // Check the wakeup cause
#ifdef ESP32_SLEEP_WAKEUP_EXT0
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
#else
  if (true)  // Always true for this example, as we are using AUX_WAKEUP_PIN
#endif
  {
    // Read message from E220
    if (e220ttl.available() > 1)
    {
      Serial.println("E220 message received!");
      Serial.println("Reading message from E220...");
      ResponseContainer rc = e220ttl.receiveMessage();  ///< Receive message from E220

      // Check if the message is received successfully
      if (rc.status.code != 1)
      {
        Serial.println(rc.status.getResponseDescription());
      }
      else
      {
        Serial.print("Received message: ");
        Serial.println(rc.status.getResponseDescription());
        Serial.println(rc.data);

        /**
         * @brief If the received data is "START 1", set pin 12 HIGH for 500ms.
         */
        if (String(rc.data) == MESSAGE_TO_CHECK)
        {
          Serial.println("Command START 1 received: setting pin 12 HIGH");
          digitalWrite(PIN_LED, HIGH);  ///< Set pin 12 HIGH
          delay(500);                   ///< Keep pin 12 HIGH for 500ms
          digitalWrite(PIN_LED, LOW);   ///< Set pin 12 LOW
        }
      }
    }

    // In this case send anyway the voltage value
    //  Send ADC voltage value via E220
    int adcValue = analogRead(ADC_PIN);         ///< Read ADC value from ADC_PIN
    float voltage = adcValue * (3.3 / 4095.0);  ///< Convert ADC value to voltage
    Serial.print("ADC Voltage: ");
    Serial.println(voltage);

    String msg = String("V:") + String(voltage, 2);  ///< Prepare voltage message
    ResponseStatus rs = e220ttl.sendFixedMessage(0, 3, 0x01, msg.c_str(),
                                                 msg.length());  ///< Send voltage message via E220
    Serial.print("Send status: ");
    Serial.println(rs.getResponseDescription());
    delay(100);
  }

  delay(100);
#ifdef ESP32_SLEEP_WAKEUP_EXT0
  Serial.println("Entering deep sleep mode...");
  esp_deep_sleep_start();  ///< Enter deep sleep mode
#endif
}

// Send functions
void send_fnc_lora_e220(LoRa_E220 &e220)
{
  // Initialize E220 module
  Serial.println("Initializing E220 module...");
  if (e220ttl.begin())
  {  ///< Initialize the E220 LoRa module
    delay(1000);
    Serial.println("E220 module initialized.");
  }
  else
  {
    Serial.println("Failed to initialize E220 module.");
    return;
  }
}