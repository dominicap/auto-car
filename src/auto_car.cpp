#include <Arduino.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>

#define BAUD_RATE 9600

void setup()
{
    Serial.begin(BAUD_RATE);
}

void loop()
{
    if (Serial.available()) {
        Serial.println("Hello, World!");
        delay(2000);
    }
}