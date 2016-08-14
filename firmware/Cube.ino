#include "CAP1188.h"
#include "neopixel/neopixel.h"
#include "simple-OSC/simple-OSC.h"
#include "MPU6050_MOTION_APPS.h"

//General
#define LED_PIN D7
//SYSTEM_MODE(SEMI_AUTOMATIC);

//OSC
UDP udp;
IPAddress outIp(255, 255, 255, 255);
char macString[18];
byte mac[6];
unsigned int outPort;
unsigned int inPort;

//Neopixel
#define PIXEL_PIN D2
#define PIXEL_COUNT 40
#define PIXEL_TYPE WS2812B
Adafruit_NeoPixel strip = Adafruit_NeoPixel(PIXEL_COUNT, PIXEL_PIN, PIXEL_TYPE);

//MPU6050
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//CAP1188
Adafruit_CAP1188 cap = Adafruit_CAP1188();
bool touched[8];

void setup() {
    //General
    pinMode(LED_PIN,OUTPUT);
    Serial.begin(9600);
    //WiFi.connect();
    
    //OSC
    getPorts();
    udp.begin(inPort);    

    //Neopixel
    strip.begin();
    strip.show(); // Initialize all pixels to 'off'
    
    //MPU6050
    delay(1000);
	Wire.begin();
    setupAccelGyro();
    
    //CAP1188
    cap.begin();
    cap.setSensitivity(7); //sensitivity values from 1 to 7
}

void loop() {
    processAccelGyro(20);
    processTouched(20);
    sendOscData(50);
    rainbow(20);
}

void getPorts(){
    WiFi.macAddress(mac);
    sniprintf(macString, 18, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    uint32_t macId = mac[2] << 24 | mac[3] << 16 | mac[4] << 8 | mac[5];
    
    switch (macId){
    	case 0x84C5C1D5: inPort = 9101; outPort = 9001; break;
    	case 0x84C694F7: inPort = 9102; outPort = 9002; break;
    	case 0x84C6A699: inPort = 9103; outPort = 9003; break;
    	case 0x84C69500: inPort = 9104; outPort = 9004; break;
    	case 0x84C69557: inPort = 9105; outPort = 9005; break;
    	case 0x84C693D2: inPort = 9106; outPort = 9006; break;
    	case 0x84C69542: inPort = 9107; outPort = 9007; break;
    	case 0x84C69428: inPort = 9108; outPort = 9008; break;
    	case 0x84C691AA: inPort = 9109; outPort = 9009; break;
    	case 0x84C69B8E: inPort = 9110; outPort = 9010; break;
    	case 0x84C69445: inPort = 9111; outPort = 9011; break;
    	case 0x84C69889: inPort = 9112; outPort = 9012; break;
    	case 0x84C69448: inPort = 9113; outPort = 9013; break;
    	case 0x84C5CA32: inPort = 9114; outPort = 9014; break;
    	default        : inPort = 9100; outPort = 9000; break;
    }
}

void setupAccelGyro(){
    mpu.reset();
    mpu.initialize();
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    
    uint8_t devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // supply your own gyro offsets here, scaled for min sensitivity
        mpu.setXGyroOffset(220);
        mpu.setYGyroOffset(76);
        mpu.setZGyroOffset(-85);
        mpu.setZAccelOffset(1788);
        
        mpu.setDMPEnabled(true);
        packetSize = mpu.dmpGetFIFOPacketSize();
        dmpReady = true;
    
    } else {
        Serial.println((devStatus == 1) ? "DMP Initial memory load failed." : "DMP Configuration updates failed.");
    }
}


void sendOscData(uint8_t wait){
    static unsigned long previousTime = 0;
    static bool ledState = false;
    unsigned long currentTime = millis();
    
    if ((currentTime - previousTime) > wait){
      previousTime = currentTime;

      //SEND
      OSCMessage outMessage("/SensorData");
      outMessage.addFloat(aaReal.x);
      outMessage.addFloat(aaReal.y);
      outMessage.addFloat(aaReal.z);
      outMessage.addFloat(ypr[0]);
      outMessage.addFloat(ypr[1]);
      outMessage.addFloat(ypr[2]);
      for (uint8_t i=0; i<8; i++) {
        outMessage.addInt(touched[i]);
      }
      outMessage.send(udp,outIp,outPort);
      
      toggleLed();
    }
}


void sendSerialData(uint8_t wait){
    static unsigned long previousTime = 0;
    unsigned long currentTime = millis();
    
    if ((currentTime - previousTime) > wait){
        Serial.print(aaReal.x);
        Serial.print("\t");
        Serial.print(aaReal.y);
        Serial.print("\t");
        Serial.print(aaReal.z);
        Serial.print("\t");
        Serial.print(ypr[0]);
        Serial.print("\t");
        Serial.print(ypr[1]);
        Serial.print("\t");
        Serial.println(ypr[2]);
    }
}

void processAccelGyro(uint8_t wait){
    static uint16_t fifoCount = 0;
    static unsigned long previousTime = 0;
    unsigned long currentTime = millis();
    
    if ((currentTime - previousTime) > wait){
        previousTime = currentTime;
        fifoCount = mpu.getFIFOCount();
        if ((fifoCount >= packetSize) && dmpReady){
            mpu.getFIFOBytes(fifoBuffer, packetSize);
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            radianToDegrees(euler);
            radianToDegrees(ypr);   
        }
    }
}

void radianToDegrees(float rad[3]){
    rad[0] = rad[0] * 180/M_PI;
    rad[1] = rad[1] * 180/M_PI;
    rad[2] = rad[2] * 180/M_PI;
}

void toggleLed() {
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState);
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

void processTouched(uint8_t wait){
    static unsigned long previousTime = 0;
    unsigned long currentTime = millis();
    
    if ((currentTime - previousTime) > wait){
        previousTime = currentTime;
        uint8_t raw = cap.touched();
        for (uint8_t i=0; i<8; i++) {
            touched[i] = (raw & (1 << i)) ? true : false;
        }
    }
}