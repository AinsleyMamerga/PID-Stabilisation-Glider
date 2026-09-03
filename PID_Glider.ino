/*
 * Glider attitude controller
 *
 * MPU6050 DMP is used to measure pitch and roll.
 * Two PID controllers command:
 *
 *   Roll  -> left and right ailerons
 *   Pitch -> left and right elevators
 *
 * The derivative term uses the gyroscope measurement rather
 * than differentiating the angle, which gives a cleaner signal.
 */

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>

MPU6050 mpu;

const int INTERRUPT_PIN = 2;

volatile bool mpuInterrupt = false;
bool dmpReady = false;

uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 gyro;
VectorFloat gravity;
float ypr[3];

Servo leftAileron;
Servo rightAileron;

Servo leftElevator;
Servo rightElevator;

const int LEFT_AILERON_PIN = 9;
const int RIGHT_AILERON_PIN = 10;

const int LEFT_ELEVATOR_PIN = 11;
const int RIGHT_ELEVATOR_PIN = 12;

const int AILERON_NEUTRAL = 135;
const int ELEVATOR_NEUTRAL = 135;

const int AILERON_MIN = 105;
const int AILERON_MAX = 165;

const int ELEVATOR_MIN = 105;
const int ELEVATOR_MAX = 165;

const float ROLL_TARGET = 0.0;
const float PITCH_TARGET = 0.0;

float rollKp = 2.0;
float rollKi = 0.0;
float rollKd = 0.5;

float pitchKp = 2.0;
float pitchKi = 0.0;
float pitchKd = 0.5;

const float ROLL_INTEGRAL_LIMIT = 20.0;
const float PITCH_INTEGRAL_LIMIT = 20.0;

float rollIntegral = 0.0;
float pitchIntegral = 0.0;

unsigned long previousTime;

void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup()
{
    Serial.begin(115200);

    leftAileron.attach(LEFT_AILERON_PIN);
    rightAileron.attach(RIGHT_AILERON_PIN);

    leftElevator.attach(LEFT_ELEVATOR_PIN);
    rightElevator.attach(RIGHT_ELEVATOR_PIN);

    setServos(
        AILERON_NEUTRAL,
        AILERON_NEUTRAL,
        ELEVATOR_NEUTRAL,
        ELEVATOR_NEUTRAL
    );

    delay(500);

    Wire.begin();
    Wire.setClock(400000);

    Serial.println("Initialising MPU6050...");

    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    if (!mpu.testConnection())
    {
        Serial.println("MPU6050 connection failed.");

        while (true)
        {
        }
    }

    Serial.println("MPU6050 connected.");

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    if (devStatus != 0)
    {
        Serial.print("DMP initialisation failed: ");
        Serial.println(devStatus);

        while (true)
        {
        }
    }

    Serial.println("Calibrating accelerometer...");
    mpu.CalibrateAccel(6);

    Serial.println("Calibrating gyroscope...");
    mpu.CalibrateGyro(6);

    Serial.println("Calibration complete.");

    mpu.setDMPEnabled(true);

    attachInterrupt(
        digitalPinToInterrupt(INTERRUPT_PIN),
        dmpDataReady,
        RISING
    );

    mpu.getIntStatus();

    packetSize = mpu.dmpGetFIFOPacketSize();

    dmpReady = true;

    previousTime = micros();

    Serial.println("DMP ready.");
    Serial.println("Glider controller active.");
}

float calculatePID(
    float error,
    float gyroRate,
    float dt,
    float kp,
    float ki,
    float kd,
    float &integral
)
{
    integral += error * dt;

    // Prevent integral wind-up
    integral = constrain(integral, -20.0, 20.0);

    /*
     * The gyro measures the rate of change of the angle.
     * The negative sign makes it act as damping.
     */
    float derivative = -gyroRate;

    return (kp * error)
         + (ki * integral)
         + (kd * derivative);
}

void setServos(
    int leftAileronAngle,
    int rightAileronAngle,
    int leftElevatorAngle,
    int rightElevatorAngle
)
{
    leftAileron.write(
        constrain(
            leftAileronAngle,
            AILERON_MIN,
            AILERON_MAX
        )
    );

    rightAileron.write(
        constrain(
            rightAileronAngle,
            AILERON_MIN,
            AILERON_MAX
        )
    );

    leftElevator.write(
        constrain(
            leftElevatorAngle,
            ELEVATOR_MIN,
            ELEVATOR_MAX
        )
    );

    rightElevator.write(
        constrain(
            rightElevatorAngle,
            ELEVATOR_MIN,
            ELEVATOR_MAX
        )
    );
}

void loop()
{
    if (!dmpReady)
        return;

    if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
        return;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float pitch = ypr[1] * 180.0 / M_PI;
    float roll = ypr[2] * 180.0 / M_PI;

    /*
     * MPU6050 default gyro sensitivity is 131 LSB/(deg/s)
     * for the ±250 deg/s range.
     */
    mpu.dmpGetGyro(&gyro, fifoBuffer);

    float rollRate = gyro.x / 131.0;
    float pitchRate = gyro.y / 131.0;

    unsigned long currentTime = micros();

    float dt =
        (currentTime - previousTime) / 1000000.0;

    previousTime = currentTime;

    if (dt <= 0.0 || dt > 0.1)
    {
        rollIntegral = 0.0;
        pitchIntegral = 0.0;

        setServos(
            AILERON_NEUTRAL,
            AILERON_NEUTRAL,
            ELEVATOR_NEUTRAL,
            ELEVATOR_NEUTRAL
        );

        return;
    }

    float rollError = ROLL_TARGET - roll;
    float pitchError = PITCH_TARGET - pitch;

    float rollOutput = calculatePID(
        rollError,
        rollRate,
        dt,
        rollKp,
        rollKi,
        rollKd,
        rollIntegral
    );

    float pitchOutput = calculatePID(
        pitchError,
        pitchRate,
        dt,
        pitchKp,
        pitchKi,
        pitchKd,
        pitchIntegral
    );

    rollOutput = constrain(
        rollOutput,
        -30.0,
        30.0
    );

    pitchOutput = constrain(
        pitchOutput,
        -30.0,
        30.0
    );

    /*
     * Ailerons move in opposite directions.
     */
    int leftAileronAngle =
        AILERON_NEUTRAL + rollOutput;

    int rightAileronAngle =
        AILERON_NEUTRAL - rollOutput;

    /*
     * Both elevator surfaces move together.
     */
    int leftElevatorAngle =
        ELEVATOR_NEUTRAL + pitchOutput;

    int rightElevatorAngle =
        ELEVATOR_NEUTRAL + pitchOutput;

    setServos(
        leftAileronAngle,
        rightAileronAngle,
        leftElevatorAngle,
        rightElevatorAngle
    );

    Serial.print("Pitch: ");
    Serial.print(pitch, 2);

    Serial.print(" | Roll: ");
    Serial.print(roll, 2);

    Serial.print(" | Pitch PID: ");
    Serial.print(pitchOutput, 2);

    Serial.print(" | Roll PID: ");
    Serial.print(rollOutput, 2);

    Serial.print(" | Ailerons: ");
    Serial.print(leftAileronAngle);
    Serial.print("/");
    Serial.print(rightAileronAngle);

    Serial.print(" | Elevators: ");
    Serial.print(leftElevatorAngle);
    Serial.print("/");
    Serial.println(rightElevatorAngle);
}