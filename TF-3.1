//TTB Aerospace: Alto: AuroraR1

#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <Arduino.h>
#include <Servo.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//sd card
File myFile;

// Set servo start value
const int initialServoValue1 = 105 ;
const int initialServoValue2 = 85 ;

// Declare servos
Servo servo1;
Servo servo2;

////////////////////////
int threshold = 12;

//pinouts for Aurora R1

const int ledPinG = 7;
const int ledPinR = 8;
const int ledPinB = 9;

const int buzzer = 10;
const int servo_x = 5;
const int servo_y = 6;
const int pyro1 = 11;

//PID Values


double desired, input, output;
double kp = 0.075;
double ki = 0.05;
double kd = 0.1;
//////////////////////////////
double desired2, input2, output2;
double kp2 = 0.075;
double ki2 = 0.05;
double kd2 = 0.1;
/////////////////////////////
PID pid(&input, &output, &desired, kp, ki, kd, DIRECT);
PID pid2(&input2, &output2, &desired2, kp2, ki2, kd2, DIRECT);


Adafruit_BNO055 bno(55);


void setup() {
  // Start serial connection
  Serial.begin(9600);


  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("test.csv", FILE_WRITE);
  // if the file opened okay, write to it:
  if (myFile) {


    myFile.println("time,z,y,updown, servo_z, servo_y");

  }

  // Trap application into endless loop, if gyro sensor fails
  if (!bno.begin()) {
    Serial.println("Error initializing gyro sensor.\nPlease check connection!");
    while (1);
  }

  delay(200);
  bno.setExtCrystalUse(true);

  // Set desired PID value to 0 degrees
  desired = 0;
  desired2 = 0;


  // Define servo data pins (PWM enabled)
  servo1.attach(5);
  servo2.attach(6);

  // Set both servos to 90 degrees
  servo1.write(initialServoValue1);
  servo2.write(initialServoValue2);

  // Initialize PID
  pid.SetMode(AUTOMATIC);
  pid2.SetMode(AUTOMATIC);

  pinMode(buzzer, OUTPUT); // Set buzzer -
  pinMode(pyro1, OUTPUT);

  tone(buzzer, 200, 200);
  delay(300);
  tone(buzzer, 800, 200);
  delay(300);
  tone(buzzer, 500, 200);


  digitalWrite(ledPinG, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(300);                       // wait for a second
  digitalWrite(ledPinG, LOW);    // turn the LED off by making the voltage LOW
  delay(500);
  digitalWrite(ledPinG, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(300);                       // wait for a second
  digitalWrite(ledPinG, LOW);    // turn the LED off by making the voltage LOW
  delay(500);
}

int state = 0 ;
int times = 0;

void loop() {
  // Read gyro sensor data
  if (state == 0) {
    imu::Vector<3> li_ac = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    digitalWrite(ledPinR, HIGH);   // turn the LED on (HIGH is the voltage level)
    if (li_ac.z() > threshold ) {
      Serial.print(li_ac.z());

      state++ ;
    }

  }



  if (state == 1) {
    imu::Vector<3> li_ac = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    digitalWrite(ledPinR, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(ledPinB, HIGH);   // turn the LED on (HIGH is the voltage level)


    // Get current degrees for Z axis
    int degreeZ = euler.z();
    input = abs(degreeZ);               // Calculate to absolute degrees
    pid.Compute();                      // Get correction value for Z axis

    // Apply correction value for Z axis
    int newZ;
    if (degreeZ < 0) {
      newZ = degreeZ + output;          // If gyro was moved counter clock-wise, add correction value
    } else {
      newZ = degreeZ - output;          // If gyro was moved clock-wise, subtract correction value
    }

    // Limit servo to min 0, or max 180 degrees
    if (newZ < -90) {
      newZ = -90;
    } else if (newZ > 90) {
      newZ = 90;
    }
    servo1.write((newZ * -1 ) + (  initialServoValue1));; // Add initialServoValue to make sure Servo is between 0 and 180

    // Communication functions to PID Frontend

    /////////////////////////////////////////////////////////////////////////////////////////////////second axis




    // Get current degrees for Z axis
    int degreeY = euler.y();
    input2 = abs(degreeY);               // Calculate to absolute degrees
    pid2.Compute();                      // Get correction value for Z axis

    // Apply correction value for Z axis
    int newY;
    if (degreeY < 0) {
      newY = degreeY + output2;          // If gyro was moved counter clock-wise, add correction value
    } else {
      newY = degreeY - output2;          // If gyro was moved clock-wise, subtract correction value
    }

    // Limit servo to min 0, or max 180 degrees
    if (newY < -90) {
      newY = -90;
    } else if (newY > 90) {
      newY = 90;
    }
    servo2.write((newY * -1 ) + (  initialServoValue2));; // Add initialServoValue to make sure Servo is between 0 and 180

    // Communication functions to PID Frontend
    // for tuning PID values and reading out current PID calculations



    times++;
    Serial.print((newY * -1 ) + (  initialServoValue2));
    Serial.print(",");
    Serial.println((newZ * -1 ) + (  initialServoValue1));

    //datalogging
    myFile.print(times);
    myFile.print(",");
    myFile.print(degreeY);
    myFile.print(",");
    myFile.print(degreeZ);
    myFile.print(",");
    myFile.print(li_ac.z());
    myFile.print(",");
    myFile.print(newY);
    myFile.print(",");
    myFile.println(newZ);


    if (li_ac.z() < -.5 ) {

      state++;
    }

  }


  if (state == 2) {
    digitalWrite(ledPinB, LOW);   // turn the LED on (HIGH is the voltage level)
    digitalWrite(ledPinG, HIGH);
    myFile.close();
    Serial.print("done!!");
    digitalWrite(pyro1, HIGH);
    delay(2000);
    digitalWrite(pyro1, LOW);
    state++ ;
  }

  if (state == 3) {
    Serial.print("program terminated");

  }

}
