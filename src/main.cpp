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
  float readRadians = encoder.getAngle();
  Serial.println(readRadians);
  delay(1000);
}