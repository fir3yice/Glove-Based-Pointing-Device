#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <Mouse.h>

MPU6050 mpu;

#define INTERRUPT_PIN 2
#define TOUCH_LEFT 7   // TTP223B for Left Click
#define TOUCH_RIGHT 8  // TTP223B for Right Click
#define TOUCH_MIDDLE 9 // TTP223B for Middle Click
#define LED_PIN 13     // On-board LED (optional)

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(115200);
    Mouse.begin();

    pinMode(TOUCH_LEFT, INPUT);
    pinMode(TOUCH_RIGHT, INPUT);
    pinMode(TOUCH_MIDDLE, INPUT);
    pinMode(LED_PIN, OUTPUT);

    mpu.initialize();
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
    }
}

float sensitivity = 0.5;
float threshold = 8.0;

void moveMouse(float x, float y) {
    if (abs(x) < threshold) x = 0;
    else x -= (x > 0) ? threshold : -threshold;
    
    if (abs(y) < threshold / 2) y = 0;
    else y -= (y > 0) ? threshold / 2 : -threshold / 2;
    
    float converted = 0.75 * sensitivity + 0.25;
    Mouse.move(x * converted, y * converted);
}

int clickDelay = 200;  // Adjustable delay for debouncing clicks

void checkMouseClicks() {
    if (digitalRead(TOUCH_LEFT) == HIGH) {
        digitalWrite(LED_PIN, HIGH);
        Mouse.click(MOUSE_LEFT);
        delay(clickDelay);
    }
    if (digitalRead(TOUCH_RIGHT) == HIGH) {
        digitalWrite(LED_PIN, HIGH);
        Mouse.click(MOUSE_RIGHT);
        delay(clickDelay);
    }
    if (digitalRead(TOUCH_MIDDLE) == HIGH) {
        digitalWrite(LED_PIN, HIGH);
        Mouse.click(MOUSE_MIDDLE);
        delay(clickDelay);
    }
    if (digitalRead(TOUCH_LEFT) == LOW && digitalRead(TOUCH_RIGHT) == LOW && digitalRead(TOUCH_MIDDLE) == LOW) {
        digitalWrite(LED_PIN, LOW);
    }
}

void loop() {
    if (!dmpReady) return;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        if (abs(ypr[1] * 180 / M_PI) < 3 * threshold) {
            moveMouse(ypr[0] * 180 / M_PI, ypr[2] * 180 / M_PI);
        }
        checkMouseClicks();
    }
}
