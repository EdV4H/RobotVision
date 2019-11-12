#include <Servo.h>
#define LED_PIN 13

Servo wheelL, wheelR, cameraV, cameraH;
const int motorPower=2;   // port for motor power control
const int cameraPower=10; // port for camera servo power control
int centerPosL = 1492;    // central position (in usec) of left motor
int centerPosR = 1492;    // central position (in usec) of right motor
int centerPosV = 85;      // central position (in degrees) of vertial servo
int centerPosH = 85;      // central position (in degrees) of horizontal servo
int k = 0;

void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(80);
  wheelL.attach(3);  // attaches the servo on pin 3 to the servo object
  wheelR.attach(5);
  cameraV.attach(9);
  cameraH.attach(6);
  pinMode(motorPower, OUTPUT);
  pinMode(cameraPower, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  delay(100);  // make the next four commands work
  digitalWrite(motorPower,LOW);  // LOW: power on
  digitalWrite(cameraPower,LOW); // LOW: power on
  wheelL.writeMicroseconds(centerPosL);
  wheelR.writeMicroseconds(centerPosR);
}

void loop()
{
  int c;
  int i;
  byte buf[10];
  
  if (Serial.available() > 0) {
    Serial.readBytesUntil(0, (char *)buf, 5);
 //   if (buf[0] != 255) break;
    c = buf[1];
   
    switch (c) {
      case 11:  // power on motor servos
        digitalWrite(motorPower,LOW);
        break;
      case 12:  // power off motor servos
        digitalWrite(motorPower,HIGH);
        break;
      case 13:  // set central pulse widths of motor servos
        centerPosL = 1500 - buf[2] + 128;
        centerPosR = 1500 + buf[3] - 128;
        break;
      case 14:  // set motor speed
        wheelL.writeMicroseconds(centerPosL - buf[2] + 128);
        wheelR.writeMicroseconds(centerPosR + buf[3] - 128);
        break;
      case 15:  // set left motor speed
        wheelL.writeMicroseconds(centerPosL - buf[2] + 128);
        break;
      case 16:  // set right motor speed
        wheelR.writeMicroseconds(centerPosR + buf[2] - 128);
        break;
      case 17:   // stop motors 
        wheelL.writeMicroseconds(centerPosL);
        wheelR.writeMicroseconds(centerPosR);
        break;
      case 21:  // power on camera servos
        digitalWrite(cameraPower,LOW);
        break;
      case 22:  // power off camera servos
        digitalWrite(cameraPower,HIGH);
        break;
      case 23:  // set center camera angle
        centerPosV = buf[2];
        centerPosH = buf[3];
        break;
      case 24:  // move vertical camera servo
        c = buf[2];
        if (c < 30) c = 30;
        else if (c > 105) c = 105;
        cameraV.write(centerPosV + c - 90);
        break;
      case 25:  // move horizontal camera servo 
        c = buf[2];
        if (c < 30) c = 30;
        else if (c > 150) c = 150;
        cameraH.write(centerPosH + c - 90);
        break;
      case 99:  // blink on-board LED
        for (i=0; i<buf[2]; i++) {
          digitalWrite(LED_PIN, HIGH);
          delay(100);
          digitalWrite(LED_PIN, LOW);
          delay(233);
        }
        break;
      default:
        break;    
    }
  }
}

