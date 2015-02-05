#include <Servo.h>
#include <math.h>
#include <AFMotor.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> // Libraries for the servo shield

// Define SERVOMIN and MAX values for each servo; out of 4096
#define SERVOMIN_BASE 150
#define SERVOMAX_BASE 610
#define SERVOMIN_SHOULDER 550
#define SERVOMAX_SHOULDER 160
#define SERVOMIN_ELBOW 190
#define SERVOMAX_ELBOW 600
#define SERVOMIN_WRIST 140
#define SERVOMAX_WRIST 600
#define SERVOMIN_TWIST 700
#define SERVOMAX_TWIST 170
#define SERVOMIN_GRIP 700
#define SERVOMAX_GRIP 450

// Initiate a Servo Shield object; uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Define constants and variables for lengths and angles || Inches for lengths and degrees for angles

double x = 3.0; // Initial xyz position
double y = 0;
double z = 5.0;
double radius = sqrt(pow(x, 2) + pow(y, 2));
double length1 = 2.5; // length from base to shoulder
double length2 = 5.75; // length from shoulder to elbow
double length3 = 7.375; // length from elbow to wrist
double length4 = 3.375; // length from wrist to tip of gripper
double length5 = sqrt(pow(radius, 2) + pow((z - length1), 2)); // hypotenuse length

// Define angles
double base_angle = degrees(atan2(x, y));
double angle6 = degrees(acos((pow(length2, 2) + pow(length5, 2) - pow(length3, 2))/(2*length2*length5)));
double angle7 = degrees(atan2((z - length1), radius));
double shoulder_angle = (angle6 + angle7); // This is angle 2 in diagram
double elbow_angle = degrees(acos((pow(length2, 2) + pow(length3, 2) - pow(length5, 2))/(2*length2*length3))); // This is angle 3 in diagram
double orientation_angle = 0; // This is angle 5 in the diagram
double twist_angle = 0;
double grip_angle = 0;
double angle8 = degrees(acos((pow(length3, 2) + pow(length5, 2) - pow(length2, 2))/(2*length3*length5)));
double angle9 = (90 - angle7);
double angle10 = (90 - orientation_angle);
double wrist_angle = (angle8 + angle9 + angle10); // This is angle 4 in diagram

// Set up digital pins for switches & buttons
int button1_pin = 2;
int button2_pin = 3;
int button3_pin = 4;
int button4_pin = 5;
int button5_pin = 6;
int slide_pin = 7;

void setup()
{
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60); // Maximum PWM frequency
  
  //Save I2C bitrate
  uint8_t twbrbackup = TWBR;
  // must be changed after calling Wire.begin() (inside pwm.begin())
  TWBR = 12; // Taken from pwmtest
  
  pinMode(button1_pin, INPUT);
  pinMode(button2_pin, INPUT);
  pinMode(button3_pin, INPUT);
  pinMode(button4_pin, INPUT);
  pinMode(button5_pin, INPUT);
  pinMode(slide_pin, INPUT);
}

void loop()
{ 
  while (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    int x = Serial.parseFloat();
    // do it again:
    int y = Serial.parseFloat();
    // do it again:
    int z = Serial.parseFloat();
    // do it again:
    int orientation_angle = Serial.parseFloat();
  
    // look for the newline. That's the end of your sentence:
    if (Serial.read() == '\n') { 
      
      // Perform inverse kinematics calculations 
      // Same code from setup without needing to redefine constants or values modified in modes
      
      double radius = sqrt(pow(x, 2) + pow(y, 2));
      double length5 = sqrt(pow(radius, 2) + pow((z - length1), 2)); // hypotenuse length
      
      // Define angles
      double base_angle = degrees(atan2(x, y));
      double angle6 = degrees(acos((pow(length2, 2) + pow(length5, 2) - pow(length3, 2))/(2*length2*length5)));
      double angle7 = degrees(atan2((z - length1), radius));
      double shoulder_angle = (angle6 + angle7); // This is angle 2 in diagram
      double elbow_angle = degrees(acos((pow(length2, 2) + pow(length3, 2) - pow(length5, 2))/(2*length3*length2))); // This is angle 3 in diagram
      double angle8 = degrees(acos((pow(length3, 2) + pow(length5, 2) - pow(length2, 2))/(2*length3*length5)));
      double angle9 = (90 - angle7);
      double angle10 = (90 - orientation_angle);
      double wrist_angle = (angle8 + angle9 + angle10); // This is angle 4 in diagram
      
      // Define pulselengths for the servos
      double pulselength_base = map(base_angle, 0, 180, SERVOMIN_BASE, SERVOMAX_BASE);
      double pulselength_shoulder = map(shoulder_angle, 0, 180, SERVOMIN_SHOULDER, SERVOMAX_SHOULDER);
      double pulselength_elbow = map(elbow_angle - 90, 0, 180, SERVOMIN_SHOULDER, SERVOMAX_SHOULDER);
      double pulselength_wrist = map(wrist_angle - 90, 0, 180, SERVOMIN_WRIST, SERVOMAX_WRIST);
      double pulselength_twist = map(twist_angle, 0, 180, SERVOMIN_TWIST, SERVOMAX_TWIST);
      double pulselength_grip = map(grip_angle, 0, 180, SERVOMIN_GRIP, SERVOMAX_GRIP);
      
      // Print out angles for the different servos
      Serial.print("Base: ");
      Serial.print(base_angle);
      Serial.print(" Shoulder: ");
      Serial.print(shoulder_angle);
      Serial.print(" Elbow: ");
      Serial.println(elbow_angle);
      Serial.print("Wrist: ");
      Serial.print(wrist_angle);
      Serial.print(" Twist: ");
      Serial.print(twist_angle);
      Serial.print(" Grip: ");
      Serial.println(grip_angle);
      
      // Move servos to wanted pulselength angles
      pwm.setPWM(0, 0, pulselength_base);
      pwm.setPWM(1, 0, pulselength_shoulder);
      pwm.setPWM(2, 0, pulselength_elbow);
      pwm.setPWM(3, 0, pulselength_wrist);
      pwm.setPWM(4, 0, pulselength_twist);
      pwm.setPWM(5, 0, pulselength_grip);
  }
  }
}

