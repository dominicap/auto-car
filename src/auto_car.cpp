#include <Arduino.h>

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_VL53L0X.h>

#define BAUD_RATE 9600

Adafruit_BNO055 imuSensor = Adafruit_BNO055(55);
Adafruit_VL53L0X lidarSensor = Adafruit_VL53L0X();

void setup()
{
    Serial.begin(BAUD_RATE);

    if (!lidarSensor.begin()) {
        Serial.println("Error: Lidar sensor is not setup properly.");
    }

    if (!imuSensor.begin()) {
        Serial.println("Error: IMU sensor is not setup properly.");
    }
}

void loop()
{
    // Loop
}
