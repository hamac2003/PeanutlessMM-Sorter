/*
  Author: Harrison McIntyre
  Last Updated: 11.04.2020

  References / Credits:
  Below are links to some of the example code and/or libraries that I integrated into my project.

  [Raspberry Pi / Desktop - Arduino Serial Communication](https://roboticsbackend.com/raspberry-pi-arduino-serial-communication/)

  I also made use of some of the example code that comes with the various arduino libraries included in this script.
 */


#include <Adafruit_MotorShield.h>
#include <Adafruit_NeoPixel.h>
#include <Servo.h>
#include <Wire.h>

Servo sweeper;
Servo scale;

#define LED_PIN    3
#define LED_COUNT 13

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 


// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(200, 2);

// LED String
Adafruit_NeoPixel leds(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

// Servo delay in ms (the larger the delay the slower the servo moves)
int servoSpeed = 10;

// Define some other variables
boolean Step = false;
int retracted = 170;
int swept = 90;
boolean Tilt_Left = false;
boolean Tilt_Right = false;
int left = 60;
int right = 120;
int center = 95;

void setup() {
  Serial.begin(9600);  // set up Serial library at 9600 bps

  // Initialize servos
  scale.attach(10);
  scale.write(center);
  
  sweeper.attach(9);
  sweeper.write(retracted);
  
  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  myMotor->setSpeed(10);  // 10 rpm

  // Initialize LEDs
  leds.begin();
  leds.show();
  leds.setBrightness(50);
}


void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data.indexOf("I") >= 0) {
      Step = true;
    }
    if (data.indexOf("S") >= 0) {
      sweeper.attach(9);
      turnServo(sweeper,swept,servoSpeed);
      delay(1000);
      turnServo(sweeper,retracted,servoSpeed);
      delay(1000);
      sweeper.detach();
    }
    if (data.indexOf("P") >= 0) {
      scale.attach(10);
      turnServo(scale,right,servoSpeed);
      delay(1000);
      turnServo(scale,center,servoSpeed);
      delay(1000);
      scale.detach();
    }
    if (data.indexOf("N") >= 0) {
      scale.attach(10);
      turnServo(scale,left,servoSpeed);
      delay(1000);
      turnServo(scale,center,servoSpeed);
      delay(1000);
      scale.detach();
    }
    if (data.indexOf("C") >= 0) {
      scale.attach(10);
      turnServo(scale,right,servoSpeed);
      delay(1000);
    }
    if (data.indexOf("C_D") >= 0) {
      scale.attach(10);
      turnServo(scale, center, servoSpeed);
    }
    if (data.indexOf("L_W") >= 0) {
      setLightColor(leds.Color(255,255,255));
    }else if (data.indexOf("L_B") >= 0) {
      setLightColor(leds.Color(0,0,0));
    }
  }
  
  // Step Types: SINGLE, DOUBLE, INTERLEAVE, MICROSTEP
  if (Step) {
    myMotor->step(1, FORWARD, MICROSTEP);
    Step = false;
  }

}

void turnServo(Servo servo, int angle, int delayVal) {
  if (angle > servo.read()) {
    for (int i = servo.read(); i < angle; i++) {
      if (i > angle) {
        i = angle;
      }
      //Serial.println(i);
      servo.write(i);
      delay(delayVal);
    }
  }else if (angle < servo.read()) {
    for (double i = servo.read(); i > angle; i--) {
      if (i < angle) {
        i = angle;
      }
      servo.write(i);
      delay(delayVal);
    }
  }
}

void setLightColor(uint32_t color) {
  //leds.clear();
  for(int i=0; i<leds.numPixels(); i++) { // For each pixel in strip...
    leds.setPixelColor(i, color);         //  Set pixel's color (in RAM)
  }
  leds.show();                          //  Update strip to match
}
