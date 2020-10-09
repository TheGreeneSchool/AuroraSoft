//TTB Aerospace: Alto: Aurora
///////////////////////////////////////---------------- VERSION FOR HT-5 - 10/8 -----------------------///////////////////////////////////////////////
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "MegunoLink.h"
#include "Filter.h"

// Set servo start value 90 degrees
const int initialServoValue1 = 60 ;
const int initialServoValue2 = 120 ;
Servo servo1;
Servo servo2;


//pinouts for DARWIN Z
const int ledPinG = 7;
const int ledPinR = 8;
const int ledPinB = 9;

const int buzzer = 10;
const int servo_x = 5;
const int servo_y = 6;
float filteredZ = 0;


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

  // Set desired PID value to 0 degrees
  desired = 0;


  // Define servo data pins (PWM enabled)
  servo1.attach(5);
  servo2.attach(6);


  // Set both servos to 90 degrees
  servo1.write(initialServoValue1);
  servo2.write(initialServoValue2);


  // Initialize PID
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);


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
  digitalWrite(ledPinB, HIGH);

  
  //get data
  imu::Vector<3> li_ac = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  sensors_event_t event;
  bno.getEvent(&event);

  // Get current degrees for Z axis
  double degreeZ = event.orientation.z;
  input = abs(degreeZ);
  pid.Compute();

  // Apply correction value for Z axis
  double newZ;
  if (degreeZ < 0) {
    newZ = degreeZ + output;
  } else {
    newZ = degreeZ - output;
  }

  int RawValue = newZ;
  ADCFilter.Filter(RawValue);
  filteredZ = ADCFilter.Current();
  Serial.print(RawValue);
  Serial.print(",");
  Serial.print(event.orientation.z);
  Serial.print(",");
  Serial.println(filteredZ);
  

  servo1.write((filteredZ * -.65 ) + (  initialServoValue1));; // Add initialServoValue to make sure Servo is between 0 and 180




}
