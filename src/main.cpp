/**
 * @file main.cpp
 * @brief ESP32 LoRa Standby Example
 *
 * This example demonstrates how to use the ESP32 with a LoRa E220 module,
 * enter deep sleep, and wake up using an external pin (AUX_WAKEUP_PIN).
 * Upon wakeup, it reads an ADC value, sends it via LoRa, and returns to sleep.
 */

#include <Arduino.h>

#include "LoRa_E220.h"

/**
 * @def AUX_WAKEUP_PIN
 * @brief GPIO pin used to wake up ESP32 from deep sleep.
 */
#define AUX_WAKEUP_PIN 33

/**
 * @def ADC_PIN
 * @brief GPIO pin used for ADC voltage reading.
 */
#define ADC_PIN 34

/**
 * @def E220_TX
 * @brief GPIO pin connected to E220 TX.
 */
#define E220_TX 17

/**
 * @def E220_RX
 * @brief GPIO pin connected to E220 RX.
 */
#define E220_RX 16

/**
 * @def E220_AUX
 * @brief GPIO pin connected to E220 AUX (for library, not for wakeup).
 */
#define E220_AUX 4  // AUX per la libreria, NON quello di wakeup

/**
 * @def E220_M0
 * @brief GPIO pin connected to E220 M0.
 */
#define E220_M0 18

/**
 * @def E220_M1
 * @brief GPIO pin connected to E220 M1.
 */
#define E220_M1 19

#define MESSAGE_TO_CHECK "START 1"

/**
 * @brief LoRa E220 module instance.
 */
LoRa_E220 e220ttl(&Serial2, E220_AUX, E220_M0, E220_M1);

/**
 * @brief Prints the reason for the last wakeup from deep sleep.
 */
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

/**
 * @brief Arduino setup function. Initializes serial, configures pins, handles wakeup, and enters
 * deep sleep.
 */
void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  }
  pinMode(AUX_WAKEUP_PIN, INPUT_PULLUP);  ///< Configure AUX_WAKEUP_PIN as input with pull-up
  print_wakeup_reason();
  Serial.println("Configuring AUX wakeup pin...");
  esp_sleep_enable_ext0_wakeup((gpio_num_t)AUX_WAKEUP_PIN,
                               0);  ///< Enable external wakeup on AUX_WAKEUP_PIN

  // Initialize E220 module
  Serial.println("Initializing E220 module...");
  e220ttl.begin();  ///< Initialize the E220 LoRa module
  delay(1000);
  Serial.println("E220 module initialized.");

  pinMode(12, OUTPUT);    ///< Set pin 12 as OUTPUT for command action
  digitalWrite(12, LOW);  ///< Ensure pin 12 is LOW at startup

  // Check the wakeup cause
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0)
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
          digitalWrite(12, HIGH);  ///< Set pin 12 HIGH
          delay(500);              ///< Keep pin 12 HIGH for 500ms
          digitalWrite(12, LOW);   ///< Set pin 12 LOW
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

  Serial.println("Entering deep sleep mode...");
  delay(100);
  esp_deep_sleep_start();  ///< Enter deep sleep mode
}

/**
 * @brief Arduino loop function. Not used in this example.
 */
void loop()
{
  // Vuoto
}