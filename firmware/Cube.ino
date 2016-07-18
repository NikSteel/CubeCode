// Once you import this library into an app on the web based IDE, modify the code to look something like the following.
// This code is a heavily modified version of the MPU6050_raw.ino example found elsewhere in this repo.
// This code has been tested against the MPU-9150 breakout board from Sparkfun.

// This #include statement was automatically added by the Spark IDE.
#include "MPU6050/MPU6050.h"

int ledPin = D7;

// MPU variables:
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;


bool ledState = false;
void toggleLed() {
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
}

void setup() {
    pinMode(ledPin, OUTPUT);
    Wire.begin();
    accelgyro.initialize();
    Particle.publish("MPU6050 connection", (accelgyro.testConnection() ? "successful" : "failed"), 60, PRIVATE);
    Serial.begin(9600);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);
    
    toggleLed();
    
}
