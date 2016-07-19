#include "simple-OSC/simple-OSC.h"
#include "application.h"
#include "neopixel/neopixel.h"
#include "MPU6050/MPU6050.h"

SYSTEM_MODE(AUTOMATIC);

// Neopixel
#define PIXEL_PIN D2
#define PIXEL_COUNT 40
#define PIXEL_TYPE WS2812B
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);
void rainbow(uint8_t wait);
uint32_t Wheel(byte WheelPos);

// MPU
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
void sendSixAxis(uint8_t wait);
int ledPin = D7;
void toggleLed();

//OSC
UDP udp;
IPAddress outIp(192, 168, 0, 107);//your computer IP
unsigned int outPort = 9000; //computer incoming port

void setup() {
    //Neopixe;
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    
    //MPU6050
    pinMode(ledPin, OUTPUT);
    Wire.begin();
    accelgyro.initialize();
    Particle.publish("MPU6050 connection", (accelgyro.testConnection() ? "successful" : "failed"), 60, PRIVATE);
    
    //osc
    udp.begin(8888);
}

void loop() {
    sendSixAxis(20);
    rainbow(20);
}


void sendSixAxis(uint8_t wait){
    static unsigned long previousTime = 0;
    unsigned long currentTime = millis();
    
    if ((currentTime - previousTime) > wait){
      previousTime = currentTime;
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
}

void toggleLed() {
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(ledPin, ledState);
}

void rainbow(uint8_t wait) {
  static uint16_t j = 0;
  static unsigned long previousTime = 0;
  
  unsigned long currentTime = millis();
  uint16_t i;
  
  if ((currentTime - previousTime) > wait){
      previousTime = currentTime;
      j = (j >= 255) ? 0 : j + 1;
      
      for(i=0; i<strip.numPixels(); ++i) {
        strip.setPixelColor(i, Wheel((i+j) & 255));
      }
      
      strip.show();      
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
