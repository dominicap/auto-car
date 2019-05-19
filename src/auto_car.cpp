#include <Arduino.h>

#include <Adafruit_VL53L0X.h>
#include <Adafruit_BNO055.h>

#include <Servo.h>

#define PRINT_TO_SERIAL true

#define ENABLE_IMU true
#define ENABLE_LIDAR true
#define ENABLE_RC true

#define ENABLE_US1 true
#define ENABLE_US2 true
#define ENABLE_US3 true
#define ENABLE_US4 true

#define LED_STATUS_PIN_1 40
#define LED_STATUS_PIN_2 48

#define VIN_VOLTAGE_READ_PIN A3

#define ULTRASONIC_TRIGGER_PIN_1 23
#define ULTRASONIC_ECHO_PIN_1 25
#define ULTRASONIC_TRIGGER_PIN_2 5
#define ULTRASONIC_ECHO_PIN_2 4
#define ULTRASONIC_TRIGGER_PIN_3 7
#define ULTRASONIC_ECHO_PIN_3 6
#define ULTRASONIC_TRIGGER_PIN_4 9
#define ULTRASONIC_ECHO_PIN_4 8

#define ARDUINO_STEERING_PIN 44
#define ARDUINO_THRUST_PIN 46

#define STEERING_SERVO_MIN 919
#define STEERING_SERVO_MAX 2088

#define THRUST_SERVO_MIN 1292
#define THRUST_SERVO_MAX 1688

#if ENABLE_LIDAR
    Adafruit_VL53L0X opticalSensor = Adafruit_VL53L0X();
    VL53L0X_RangingMeasurementData_t opticalDistanceObject;

    unsigned int opticalDistance;
#endif

#if ENABLE_IMU
    Adafruit_BNO055 imuSensor = Adafruit_BNO055(55);

    imu::Vector<3> eulerAngles;
    imu::Vector<3> linearAcceleration;
#endif

Servo steeringServo;
Servo thrustServo;

unsigned int measureVINVoltage()
{
    float R1 = 100.3;
    float R2 = 10.03;

    int sampleNumber = 5;

    float scaleFactor = (R1 + R2) / R2;
    unsigned int measuredVoltage = 0;

    int sampleIndex;

    analogReference(INTERNAL2V56);

    for (sampleIndex = 0; sampleIndex < sampleNumber; sampleIndex++) {
        measuredVoltage += analogRead(VIN_VOLTAGE_READ_PIN);
        delay(10);

    }

    measuredVoltage = 2.5 * scaleFactor * (measuredVoltage / (float) sampleNumber);
    analogReference(DEFAULT);

    return measuredVoltage;
}

unsigned int measureUltrasonicDistance(int triggerPin, int echoPin)
{
    int sampleNumber = 5;
    int sampleIndex;

    unsigned long int echoDuration = 0;

    for (sampleIndex = 0; sampleIndex < sampleNumber; sampleIndex++) {
        digitalWrite(triggerPin, LOW);
        delayMicroseconds(2);
        digitalWrite(triggerPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(triggerPin, LOW);
        echoDuration += pulseIn(echoPin, HIGH);
        delay(10);
    }
    unsigned int echoDistance = ((float) echoDuration / (float) sampleNumber) / 5.9;

    return echoDistance;
}

#if ENABLE_LIDAR
    int measureOpticalDistance()
    {
        opticalSensor.rangingTest(&opticalDistanceObject, false);

        int tofDistance = -1;
        if (opticalDistanceObject.RangeStatus != 4) {
            tofDistance = opticalDistanceObject.RangeMilliMeter;
        }

        return tofDistance;
    }
#endif

void ledErrorStatus()
{
    while (true) {
        digitalWrite(LED_STATUS_PIN_1, HIGH);
        digitalWrite(LED_STATUS_PIN_2, LOW);
        delay(1000);
        digitalWrite(LED_STATUS_PIN_1, LOW);
        digitalWrite(LED_STATUS_PIN_2, HIGH);
        delay(1000);
    }
}

void setup()
{
    if (PRINT_TO_SERIAL) {
        Serial.begin(9600);
    }

    pinMode(LED_STATUS_PIN_1, OUTPUT);
    pinMode(LED_STATUS_PIN_2, OUTPUT);

    #if ENABLE_US1
        pinMode(ULTRASONIC_TRIGGER_PIN_1, OUTPUT);
        pinMode(ULTRASONIC_ECHO_PIN_1, INPUT);
    #endif

    #if ENABLE_US2
        pinMode(ULTRASONIC_TRIGGER_PIN_2, OUTPUT);
        pinMode(ULTRASONIC_ECHO_PIN_2, INPUT);
    #endif

    #if ENABLE_US3
        pinMode(ULTRASONIC_TRIGGER_PIN_3, OUTPUT);
        pinMode(ULTRASONIC_ECHO_PIN_3, INPUT);
    #endif

    #if ENABLE_US4
        pinMode(ULTRASONIC_TRIGGER_PIN_4, OUTPUT);
        pinMode(ULTRASONIC_ECHO_PIN_4, INPUT);
    #endif

    #if ENABLE_LIDAR
        if (!opticalSensor.begin()) {
            if (PRINT_TO_SERIAL) {
                Serial.println(F("Cannot successfully connect to VL53L0X."));
                while (1);
            }
        }
    #endif

    #if ENABLE_IMU
        Serial.println("About to load IMU");
        if (!imuSensor.begin()) {
            if (PRINT_TO_SERIAL) {
                Serial.println(F("Cannot successfully connect to BNO055."));
                while (1);
            }
        }
        else {
            Serial.println("Can Connect to BNO055");
        }
        imuSensor.setExtCrystalUse(true);
    #endif

    #if ENABLE_RC
        steeringServo.attach(ARDUINO_STEERING_PIN, STEERING_SERVO_MIN, STEERING_SERVO_MAX);
        thrustServo.attach(ARDUINO_THRUST_PIN, THRUST_SERVO_MIN, THRUST_SERVO_MAX);
    #endif

    thrustServo.write(90);
    delay(5000);
}

void loop()
{
    opticalDistance = measureOpticalDistance();
    eulerAngles = imuSensor.getVector(Adafruit_BNO055::VECTOR_EULER);

    unsigned int ultrasonicDistance = measureUltrasonicDistance(ULTRASONIC_TRIGGER_PIN_1, ULTRASONIC_ECHO_PIN_1);

    int throttleCommand;

    unsigned int ultrasonicDistanceAngleRight = measureUltrasonicDistance(ULTRASONIC_TRIGGER_PIN_2,
            ULTRASONIC_ECHO_PIN_2);
    unsigned int ultrasonicDistanceAngleLeft = measureUltrasonicDistance(ULTRASONIC_TRIGGER_PIN_3,
            ULTRASONIC_ECHO_PIN_3);

    if ((opticalDistance >= 0 && opticalDistance <= 1600) ||
            (ultrasonicDistanceAngleRight <= 850) ||
            (ultrasonicDistanceAngleLeft <= 850) ||
            (ultrasonicDistance <= 850)) {

        Serial.println("Stopping Motors");
        throttleCommand = 90;
        thrustServo.write(throttleCommand);

        throttleCommand = 85;
        thrustServo.write(throttleCommand);
        delay(275);

        throttleCommand = 90;
        thrustServo.write(throttleCommand);

        delay(500);

        ultrasonicDistanceAngleRight = measureUltrasonicDistance(ULTRASONIC_TRIGGER_PIN_2, ULTRASONIC_ECHO_PIN_2);
        ultrasonicDistanceAngleLeft = measureUltrasonicDistance(ULTRASONIC_TRIGGER_PIN_3, ULTRASONIC_ECHO_PIN_3);

        Serial.print("RIGHT: ");
        Serial.println(ultrasonicDistanceAngleRight);
        Serial.print("LEFT: ");
        Serial.println(ultrasonicDistanceAngleLeft);

        if (ultrasonicDistanceAngleRight > ultrasonicDistanceAngleLeft) {
            Serial.println("Turning Right");
            digitalWrite(LED_STATUS_PIN_1, HIGH);
            steeringServo.write(175);
            opticalDistance = measureOpticalDistance();
            throttleCommand = 95;
            thrustServo.write(throttleCommand);
            delay(750);
            steeringServo.write(90);
            opticalDistance = measureOpticalDistance();
            throttleCommand = 95;
            thrustServo.write(throttleCommand);
            digitalWrite(LED_STATUS_PIN_1, LOW);
            delay(100);
        }
        else if (ultrasonicDistanceAngleLeft > ultrasonicDistanceAngleRight) {
            Serial.println("Turning Left");
            digitalWrite(LED_STATUS_PIN_2, HIGH);
            steeringServo.write(5);
            opticalDistance = measureOpticalDistance();
            throttleCommand = 95;
            thrustServo.write(throttleCommand);
            delay(750);
            steeringServo.write(90);
            opticalDistance = measureOpticalDistance();
            throttleCommand = 95;
            thrustServo.write(throttleCommand);
            digitalWrite(LED_STATUS_PIN_2, LOW);
            delay(100);
        }
        else {
            Serial.println("Turning Left");
            digitalWrite(LED_STATUS_PIN_2, HIGH);
            steeringServo.write(5);
            throttleCommand = 95;
            thrustServo.write(throttleCommand);
            delay(750);
            steeringServo.write(90);
            opticalDistance = measureOpticalDistance();
            throttleCommand = 95;
            thrustServo.write(throttleCommand);
            digitalWrite(LED_STATUS_PIN_2, LOW);
            delay(100);
        }
    }
    else {
        Serial.println("Moving forward");
        if (eulerAngles.x() > 350 && eulerAngles.x() <= 360) {
            steeringServo.write(90);
            Serial.println("Going Straight");
        }
        else if (eulerAngles.x() > 0 && eulerAngles.x() <= 10) {
            steeringServo.write(90);
            Serial.println("Going Straight");
        }
        else if (eulerAngles.x() > 0 && eulerAngles.x() <= 180) {
            Serial.println("Straightening Left");
            ultrasonicDistanceAngleLeft = measureUltrasonicDistance(ULTRASONIC_TRIGGER_PIN_3,
                    ULTRASONIC_ECHO_PIN_3);
            if (ultrasonicDistanceAngleLeft > 350) {
                steeringServo.write(15);
            }
        }
        else if (eulerAngles.x() > 180 && eulerAngles.x() <= 360) {
            Serial.println("Straightening Right");
            ultrasonicDistanceAngleRight = measureUltrasonicDistance(ULTRASONIC_TRIGGER_PIN_2,
                    ULTRASONIC_ECHO_PIN_2);
            if (ultrasonicDistanceAngleRight > 350) {
                steeringServo.write(175);
            }
        }
        throttleCommand = 95;
        thrustServo.write(throttleCommand);
    }
}