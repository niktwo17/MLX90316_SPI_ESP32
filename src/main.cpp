#include <Arduino.h>
#include <MLX.h>

MLX encoder;

void setup() 
{
  encoder.begin();
  Serial.begin(115200);
}

void loop() 
{
  // -10 value is bit error, -20 means no value received from sensor
  float readRadians = encoder.getAngle();
  Serial.println(readRadians);
  delay(1000);
}