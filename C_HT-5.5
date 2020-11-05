//TTB Aerospace: Alto: Aurora ------ VERSION 5.5
///////////////////////////////////////---------------- VERSION FOR HT-5 - 11/4 -----------------------///////////////////////////////////////////////
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "MegunoLink.h"
#include "Filter.h"

// Set servo start value
const int initialServoValue1 = 60 ;
const int initialServoValue2 = 120 ;
const float scaledServoGain = .65 ;
Servo servo1;
Servo servo2;

//pinouts for Aurora R1
const int servo_x = 5;
const int servo_y = 6;
const int ledPinG = 7;
const int ledPinR = 8;
const int ledPinB = 9;
const int buzzer = 10;

//PID Values
double desired, input, output;
double kp = .1;
double ki = .05;
double kd = .2;
PID pid(&input, &output, &desired, kp, ki, kd, DIRECT);

//gyro
Adafruit_BNO055 bno(55);

//filter
ExponentialFilter<long> ADCFilter(10, 0);
float filteredZ = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  // Start serial connection
  Serial.begin(9600);

  // Trap application into endless loop, if gyro sensor fails
  if (!bno.begin()) {
    Serial.println("Error initializing gyro sensor.\nPlease check connection!");
    while (1);
  }

  delay(200);
  bno.setExtCrystalUse(true);

  // Define servo data pins (PWM enabled)
  servo1.attach(5);
  servo2.attach(6);
  servo1.write(initialServoValue1);
  servo2.write(initialServoValue2);

  // Initialize PID
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  desired = 0;

  //buzzer stuff
  pinMode(buzzer, OUTPUT); // Set buzzer
  tone(buzzer, 200, 200);
  delay(300);
  tone(buzzer, 800, 200);
  delay(300);
  tone(buzzer, 500, 200);
  digitalWrite(ledPinG, HIGH);
  delay(300);
  digitalWrite(ledPinG, LOW);
  delay(500);
  digitalWrite(ledPinG, HIGH);
  delay(300);
  digitalWrite(ledPinG, LOW);
  delay(500);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {
  //Set LED pin High
  digitalWrite(ledPinB, HIGH);

  //get BNO055 data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  //Run PID loop
  double degreeZ = euler.z();
  input = abs(degreeZ);
  pid.Compute();

  // Apply correction value for Z axis
  double newZ;
  if (degreeZ < 0) {
    newZ = degreeZ + output;
  } else {
    newZ = degreeZ - output;
  }

  //Apply filter
  double RawValue = newZ;
  ADCFilter.Filter(RawValue);
  filteredZ = ADCFilter.Current();

  //print different values to the monitor
  Serial.print(RawValue);
  Serial.print(",");
  Serial.print(degreeZ);
  Serial.print(",");
  Serial.println(filteredZ);

  //Write to servo motor
  servo1.write((filteredZ * -(scaledServoGain) ) + (  initialServoValue1));; // Add initialServoValue to make sure Servo is between 0 and 180

}
