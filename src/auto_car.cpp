#include <Arduino.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>

#define BAUD_RATE 9600

Adafruit_BNO055 bno055 = Adafruit_BNO055(55);
Adafruit_VL53L0X vl53L0X = Adafruit_VL53L0X();

void setup()
{
    Serial.begin(BAUD_RATE);

    if (!vl53L0X.begin()) {
        Serial.println("Error: Lidar sensor is not setup properly.");
    }

    if (!bno055.begin()) {
        Serial.println("Error: IMU sensor is not setup properly.");
    }
}

void loop()
{
    // Loop
}
