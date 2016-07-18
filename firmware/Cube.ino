#include "simple-OSC/simple-OSC.h"
#include "neopixel/neopixel.h"
#include "MPU6050/MPU6050.h"

int ledPin = D7;

// MPU variables:
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;


//OSC variables
UDP udp;
IPAddress outIp(192, 168, 0, 108);//your computer IP
unsigned int outPort = 9000; //computer incoming port


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
    
    udp.begin(8888);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //SEND
    OSCMessage outMessage("/BattleCat");
    outMessage.addFloat(ax);
    outMessage.addFloat(ay);
    outMessage.addFloat(az);
    outMessage.addFloat(gx);
    outMessage.addFloat(gy);
    outMessage.addFloat(gz);
    outMessage.send(udp,outIp,outPort);
    
    toggleLed();
    
}