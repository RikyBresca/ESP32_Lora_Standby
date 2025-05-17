#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);
void setup()
{
  Serial.begin(115200);
  while (!Serial)
  {
    ;  // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println("Hello, world!");
}

void loop()
{
  // put your main code here, to run repeatedly:
}
