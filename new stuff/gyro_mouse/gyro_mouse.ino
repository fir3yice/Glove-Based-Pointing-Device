#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <Mouse.h>
#include "Adafruit_HMC5883_U.h"

MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

#define INTERRUPT_PIN 2
#define TOUCH_LEFT 7   // TTP223B for Left Click
#define TOUCH_RIGHT 8  // TTP223B for Right Click
#define TOUCH_MIDDLE 9 // TTP223B for Middle Click
#define LED_PIN 13    
#define IR_EMITTER_PIN 10

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3] = {0, 0, 0};
float initial_values[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}

int mouseX = 720, mouseY = 450;
int screenWidth = 1440, screenHeight = 900;

void moveMouseToOrigin() {
    // Move mouse to upper-left corner (0,0)
    for (int i = 0; i < (screenWidth*2 / 127) + 1; i++) {
        Mouse.move(-127, 0, 0);
    }
    for (int i = 0; i < (screenHeight*2 / 127) + 1; i++) {
        Mouse.move(0, -127, 0);
    }
}

void setup() {
    Wire.begin();
    Wire.setClock(400000);
    Serial.begin(115200);
    delay(1000);
    Serial1.begin(9600);
    Mouse.begin();
    //Mouse.move(960 - mouseX, 540 - mouseY);
    //mouseX = 960;
    //mouseY = 540;
    moveMouseToOrigin();

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

    if (!mag.begin()) {
        Serial.println("HMC5883L not detected!");
    } else {
        Serial.println("HMC5883L detected!");
    }

    //do calibrations
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    initial_values[0] = ypr[0] * 180 / M_PI;
    initial_values[1] = ypr[1] * 180 / M_PI;
    initial_values[2] = ypr[2] * 180 / M_PI;
    sensors_event_t event;
    mag.getEvent(&event);
    initial_values[3] = event.magnetic.x;
    initial_values[4] = event.magnetic.y;
    initial_values[5] = event.magnetic.z;
    float heading = atan2(event.magnetic.y, event.magnetic.x);
    float declinationAngle = -0.0468;
    heading += declinationAngle;
    if (heading < 0) heading += 2 * PI;
    if (heading > 2 * PI) heading -= 2 * PI;
    float heading_deg = heading * 180 / M_PI;
    initial_values[6] = heading_deg;
    Serial.print("Initial values: ");
    for (int i = 0; i < 6; i++) {
        Serial.print(initial_values[i]);
        Serial.print(" ");
    }

    pinMode(IR_EMITTER_PIN, OUTPUT);
    digitalWrite(IR_EMITTER_PIN, HIGH);  // Always ON (emitting) (might change to make it dependent on the gyroscope angle or something?)
    Serial.print("Emitter on");
}

float sensitivity = 0.5;
float threshold = 8.0;

// void moveMouse(float x, float y) {
//     if (abs(x) < threshold) x = 0;
//     else x -= (x > 0) ? threshold : -threshold;
    
//     if (abs(y) < threshold / 2) y = 0;
//     else y -= (y > 0) ? threshold / 2 : -threshold / 2;
    
//     float converted = 0.75 * sensitivity + 0.25;
//     Mouse.move(x * converted, y * converted);
// }


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

sensors_event_t event;
float declinationAngle = -0.0468;
float heading = 0;
float mag_xyz[3] = {0, 0, 0};
void get_mag(){
    mag.getEvent(&event);
    mag_xyz[0] = event.magnetic.x;
    mag_xyz[1] = event.magnetic.y;
    mag_xyz[2] = event.magnetic.z;
    heading = atan2(event.magnetic.y, event.magnetic.x);
    heading += declinationAngle;
    if (heading < 0) heading += 2 * PI;
    if (heading > 2 * PI) heading -= 2 * PI;
    heading = heading * 180 / M_PI; // degrees

    float x_drift = event.magnetic.x - initial_values[3];
    float y_drift = event.magnetic.y - initial_values[4];
    float z_drift = event.magnetic.z - initial_values[5];
    float heading_drift = heading - initial_values[6];
}

void moveMouse(float x, float y) {
    // Get magnetometer drift values
    float drift_x = mag_xyz[0] - initial_values[3];
    float drift_y = mag_xyz[1] - initial_values[4];

    // Apply drift correction
    x -= drift_x * 0.1; // Scaling factor to tune correction
    y -= drift_y * 0.1;

    if (abs(x) < threshold) x = 0;
    else x -= (x > 0) ? threshold : -threshold;
    
    if (abs(y) < threshold / 2) y = 0;
    else y -= (y > 0) ? threshold / 2 : -threshold / 2;
    
    float converted = 0.75 * sensitivity + 0.25;
    Mouse.move(x * converted, y * converted);
}

//WORKS
void snap_mouse(int id) {
    // Define target positions based on IR receiver ID
    int x = screenWidth / 2, y = screenHeight / 2;  // Default to center

    // Base positions for IR receivers
    // switch (id) {
    //     case 26: x = 0; y = screenHeight; break;  // Left Edge
    //     case 33: x = screenWidth*2; y = screenHeight; break; // Right Edge
    //     case 27: x = screenWidth; y = 0; break;    // Top Center
    //     case 25: x = screenWidth; y = screenHeight*2; break; // Bottom Center
    // }

    switch(id){
        case 26: x = 0; y = screenHeight; break;  //bottom left
        case 33: x = 0; y = 0; break; // top left
        case 27: x = screenWidth; y = 0; break;    // top right
        case 25: x = screenWidth; y = screenHeight; break; // bottom right
    }

    // Apply magnetometer drift correction
    float heading_drift = heading - initial_values[6];
    int offsetX = (heading_drift * 10);  // Scale this based on testing
    int offsetY = 0;  // Optional, depends on the magnetometer axes

    x += offsetX;
    y += offsetY;

    // Ensure the values stay within the screen bounds
    x = constrain(x, 0, screenWidth);
    y = constrain(y, 0, screenHeight);

    // Move mouse to upper-left first (forces predictable behavior)
    moveMouseToOrigin();

    // Calculate movement in small steps (since Mouse.move() max is 127)
    int moveXtimes = x / 127;
    int moveYtimes = y / 127;
    signed char lastMoveX = x - (moveXtimes * 127);
    signed char lastMoveY = y - (moveYtimes * 127);

    // Serial.print("Moving to ");Serial.print(x);Serial.print(" ");Serial.println(y);
    // Serial.print("Move x "); Serial.print(moveXtimes); 
    // Serial.print("Move y "); Serial.println(moveYtimes); 
    // Serial.print("Move x last "); Serial.print(lastMoveX); 
    // Serial.print("Move y last "); Serial.println(lastMoveY); 

    for (int i = 0; i < moveXtimes; i++) {
        Mouse.move(127, 0, 0);
        delay(3);
    }
    Mouse.move(lastMoveX, 0, 0);

    for (int i = 0; i < moveYtimes; i++) {
        Mouse.move(0, 127, 0);
        delay(3);
    }
    Mouse.move(0, lastMoveY, 0);
}


int which_IR = -1;
unsigned long lastIRTime = 0;
bool IRActive = false;

void loop() {
    if (!dmpReady) return;
    get_mag();
    // Check for IR receiver activation
    if (Serial1.available()) {
        String input = Serial1.readStringUntil('\n');
        which_IR = input.toInt();
        
        if (which_IR >= 0) {
            snap_mouse(which_IR);
            IRActive = true;
            lastIRTime = millis();
        }
    }

    // Block MPU movement if IR was triggered recently
    if (IRActive && millis() - lastIRTime < 1000) {
        return;  // Skip MPU-based movement
    } else {
        IRActive = false;  // Resume normal movement after 1 sec
    }

    // Normal MPU-based movement
    get_mag();
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


