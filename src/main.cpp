#include <Arduino.h>

//-- PIN CONFIGURATION --//
#define WAKEUP_PIN 33                 // Pin to wake up the ESP32
#define WAKEUP_PIN_MODE INPUT_PULLUP  // Pin mode for the wakeup pin

void print_wakeup_reason()
{
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0:
      Serial.println("Wakeup caused by external signal using RTC_IO");
      break;
    case ESP_SLEEP_WAKEUP_EXT1:
      Serial.println("Wakeup caused by external signal using RTC_CNTL");
      break;
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("Wakeup caused by timer");
      break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      break;
    case ESP_SLEEP_WAKEUP_ULP:
      Serial.println("Wakeup caused by ULP program");
      break;
    default:
      Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason);
      break;
  }
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ;  // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(WAKEUP_PIN, INPUT_PULLUP);
  print_wakeup_reason();  // Print the reason for waking up
  Serial.println("Configuring wakeup pin...");
  esp_sleep_enable_ext0_wakeup((gpio_num_t)WAKEUP_PIN,
                               0);  // Enable wakeup on the specified pin (active low)
  // esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0); // Disable the wakeup source to prevent
  // immediate wakeup
  Serial.println("Starting...");
}

void loop()
{
  // Test Sleep mode
  Serial.println("Sleeping for 5 seconds...");
  delay(5000);
  Serial.println("Entering deep sleep mode...");
  esp_deep_sleep_start();  // Enter deep sleep mode
}
