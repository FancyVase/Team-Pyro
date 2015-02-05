#include <Servo.h>
#include <math.h>
#include <AFMotor.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h> // Libraries for the servo shield

// Define SERVOMIN and MAX values for each servo (in Pulse Width Modulation signals; out of 4096)
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

// Define EMG pins and their values
const int rightpin = A1;
int rightval = 0;
const int leftpin = A0;
int leftval = 0;

// Define constants and variables for lengths and angles; inches for lengths and degrees for angles

double x = 5.0; // Starting xyz position
double y = 0;
double z = 10.0;
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

// Initialize EMG sensor values
boolean right_relaxed = false;
boolean right_tensed = false;
boolean left_relaxed = false;
boolean left_tensed = false;

// Set up digital pins for switches & buttons
int slide_pin = 13;
int button1_pin = 12;
int button2_pin = 11;
int button3_pin = 10;
int button4_pin = 9;
int button5_pin = 8;

//// Set up modes; Will initially be set to mode 4 (x-control)
boolean slide = true; // slide = switches between control of modes 1-5 and 6-9
boolean mode1 = false; // mode1 = grip control
boolean mode2 = false; // mode2 = twist control
boolean mode3 = false; // mode3 = orientation control
boolean mode4 = false; // mode4 = x control
boolean mode5 = false; // mode5 = y control
boolean mode6 = true; // mode6 = z control
boolean mode7 = false; // mode7 = x & y control
boolean mode8 = false; // mode8 = x & orientation control
boolean mode9 = false; // mode9 = y & orientation control

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
  pinMode(slide_pin, INPUT);
}

void loop()
{ 
  // Read the EMG values; values between 0 and 1023
  double rightval = analogRead(rightpin);
  double leftval = analogRead(leftpin);
  
  // Code for digital-input through EMGs
  if (rightval < 500) { // Right muscle is relaxed
    right_relaxed = true;
    right_tensed = false;
  }
  else if (rightval >= 500) { // Right muscle is tensed
    right_tensed = true;
    right_relaxed = false;
  }
  
  if (leftval < 500) { // Left muscle is relaxed
    left_relaxed = true;
    left_tensed = false;
  }
  else if (leftval >= 500) { // Left muscle is tensed
    left_tensed = true;
    left_relaxed = false;
  }
  
  Serial.print("Right EMG = ");
  Serial.println(rightval);
  Serial.print("Left EMG = ");
  Serial.println(leftval); 
  
  int slide_val = digitalRead(slide_pin);
  if (slide_val == 1) {
    int button1_val = digitalRead(button1_pin);
    if (button1_val == 1) {
      mode1 = true; mode2 = false; mode3 = false; mode4 = false; mode5 = false; mode6 = false; mode7 = false; mode8 = false; mode9 = false;
    }
    int button2_val = digitalRead(button2_pin);
    if (button2_val == 1) {
      mode1 = false; mode2 = true; mode3 = false; mode4 = false; mode5 = false; mode6 = false; mode7 = false; mode8 = false; mode9 = false;
    }
    int button3_val = digitalRead(button3_pin);
    if (button3_val == 1) {
      mode1 = false; mode2 = false; mode3 = true; mode4 = false; mode5 = false; mode6 = false; mode7 = false; mode8 = false; mode9 = false;
    }
    int button4_val = digitalRead(button4_pin);
    if (button4_val == 1) {
      mode1 = false; mode2 = false; mode3 = false; mode4 = true; mode5 = false; mode6 = false; mode7 = false; mode8 = false; mode9 = false;
    }
    int button5_val = digitalRead(button5_pin);
    if (button5_val == 1) {
      mode1 = false; mode2 = false; mode3 = false; mode4 = false; mode5 = true; mode6 = false; mode7 = false; mode8 = false; mode9 = false;
    }
  } else {
    int button1_val = digitalRead(button1_pin);
    if (button1_val == 1) {
      mode1 = false; mode2 = false; mode3 = false; mode4 = false; mode5 = false; mode6 = true; mode7 = false; mode8 = false; mode9 = false;
    }
    int button2_val = digitalRead(button2_pin);
    if (button2_val == 1) {
      mode1 = false; mode2 = false; mode3 = false; mode4 = false; mode5 = false; mode6 = false; mode7 = true; mode8 = false; mode9 = false;
    }
    int button3_val = digitalRead(button3_pin);
    if (button3_val == 1) {
      mode1 = false; mode2 = false; mode3 = false; mode4 = false; mode5 = false; mode6 = false; mode7 = false; mode8 = true; mode9 = false;
    }
    int button4_val = digitalRead(button4_pin);
    if (button4_val == 1) {
      mode1 = false; mode2 = false; mode3 = false; mode4 = false; mode5 = false; mode6 = false; mode7 = false; mode8 = false; mode9 = true;
    }
  }
  
  // Mode 1: Right EMG increases the grip (angle), left EMG decreases the grip (angle)
  if (mode1) {
    if (right_tensed) {
      grip_angle += 3; // grip angle increases by 5 degrees
      if (grip_angle > 180) {
        grip_angle = 180;
      }
    }
    else if (left_tensed) {
      grip_angle -= 3; // grip angle decreases by 5 degrees
      if (grip_angle < 0) {
       grip_angle = 0; 
      }
    }
  }
  
  // Mode 2: Right EMG increases the twist angle, left EMG decreases the twist angle
  if (mode2) {
    if (right_tensed) {
      twist_angle += 3; // twist angle increases by 5 degrees
      if (twist_angle > 180) {
        twist_angle = 180;
      }
    }
    else if (left_tensed) {
      twist_angle -= 3; // twist angle decreases by 5 degrees
      if (twist_angle < 0) {
        twist_angle = 0;
      }
    }
  }  
  
  // Mode 3: Right EMG increases the orientation angle, left EMG decreases the orientation angle
  if (mode3) {
    if (right_tensed) {
      orientation_angle += 3; // orientation angle increases by 5 degrees
      if (orientation_angle > 180) {
        orientation_angle = 180;
      }
    }
    else if (left_tensed) {
      orientation_angle -= 3; // orientation angle decreases by 5 degrees 
      if (orientation_angle < 0) {
        orientation_angle = 0;
      }
    }
  }
  
  // Mode 4: Right EMG increases x, left EMG decreases x
  if (mode4) {
    if (right_tensed) {
      x += 0.07; // endpoint moves 0.1 inches per tick in the positive x direction
      if (x > 13.5) {
        x = 13.5;
      }
    }
    else if (left_tensed) {
      x -= 0.07; // endpoint moves 0.1 inches per tick in the negative x direction
      if (x < 0) {
        x = 0;
      }
    }
  }
  
  // Mode 5: Right EMG increases y, left EMG decreases y
  if (mode5) {
    if (right_tensed) {
      y += 0.07; // endpoint moves 0.1 inches per tick in the positive y direction
      if (y > 6.75) {
        y = 6.75;
      }  
    }
    else if (left_tensed) {
      y -= 0.07; // endpoint moves 0.1 inches per tick in the negative y direction
      if (y < -6.75) {
        y = -6.75;
      }
    }
  }
  
  // Mode 6: Right EMG increases z, left EMG decreases z
  if (mode6) {
    if (right_tensed) {
      z += 0.07; // endpoint moves 0.1 inches per tick in the positive z direction
      if (z > 10) {
        z = 10;
      }
    }
    else if (left_tensed) {
      z -= 0.07; // endpoint moves 0.1 inches per tick in the negative z direction
      if (z < 0) {
        z = 0;
      }
    }
  }
  
  // Mode 7: Right EMG increases/decreases x, left EMG increases/decreases y
  if (mode7) {
    x = (rightval/102.3); // limits x motion to 0 -> 10 inches
    if (leftval >= 511.5) {
      y = (leftval/102.3);
    }
    else if (leftval < 511.5) {
      y = (-leftval/102.3);
    }
    if (x > 13.5) {
      x = 13.5; 
    }
    else if (x < 0) {
      x = 0;
    }
/    if (y > 6.75) {
      y = 6.75;
    }
    else if (y < 0) {
      y = 0;
    }
  }
  
  // Mode 8: Right EMG increases/decreases x, left EMG increases/decreases orientation angle
  if (mode8) {
    x = (rightval/102.3);
    if (x > 13.5) {
      x = 13.5; 
    }
    else if (x < 0) {
      x = 0;
    }
    orientation_angle = (leftval/5.6833);
    if (orientation_angle > 180) {
      orientation_angle = 180;
    }
    else if (orientation_angle < 0) {
      orientation_angle = 0;
    }
  }
  
  // Mode 9: Right EMG increases/decreases y, left EMG increases/decreases orientation angle
  if (mode9) {
    if (rightval >= 511.5) {
      y = (rightval/102.3);
    }
    else if (rightval < 511.5) {
      y = (-rightval/102.3);
    }
    if (y > 6.75) {
      y = 6.75;
    }
    else if (y < -6.75) {
      y = -6.75;
    }
    
    orientation_angle = (leftval/5.6833);
    if (orientation_angle > 180) {
      orientation_angle = 180;
    }
    else if (orientation_angle < 0) {
      orientation_angle = 0;
    }
  }
  
  
  // Perform inverse kinematics calculations 
  // Same code from setup without needing to redefine constants or values modified in modes
  
  double radius = sqrt(pow(x, 2) + pow(y, 2));
  double length5 = sqrt(pow(radius, 2) + pow((z - length1), 2)); // hypotenuse length
  
  // Define angles
  double base_angle = degrees(atan2(x, y));
  double angle6 = degrees(acos((pow(length3, 2) - pow(length2, 2) - pow(length5, 2))/(-2*length2*length5)));
  double angle7 = degrees(atan2((z - length1), radius));
  double shoulder_angle = (angle6 + angle7); // This is angle 2 in diagram
  double elbow_angle = degrees(acos((pow(length5, 2) - pow(length3, 2) - pow(length2, 2))/(-2*length3*length2))); // This is angle 3 in diagram
  double angle8 = degrees(acos((pow(length2, 2) - pow(length3, 2) - pow(length5, 2))/(-2*length2*length5)));
  double angle9 = (90 - angle7);
  double angle10 = (90 - orientation_angle);
  double wrist_angle = (angle8 + angle9 + angle10); // This is angle 4 in diagram
  
  // Define pulselengths for the servos || Uses arduino map function
  double pulselength_base = map(base_angle, 0, 180, SERVOMIN_BASE, SERVOMAX_BASE);
  double pulselength_shoulder = map(shoulder_angle, 0, 180, SERVOMIN_SHOULDER, SERVOMAX_SHOULDER);
  double pulselength_elbow = map(elbow_angle - 90, 0, 180, SERVOMIN_SHOULDER, SERVOMAX_SHOULDER);
  double pulselength_wrist = map(wrist_angle - 90, 0, 180, SERVOMIN_WRIST, SERVOMAX_WRIST);
  double pulselength_twist = map(twist_angle, 0, 180, SERVOMIN_TWIST, SERVOMAX_TWIST);
  double pulselength_grip = map(grip_angle, 0, 180, SERVOMIN_GRIP, SERVOMAX_GRIP);
  
  // Move servos to wanted pulselength angles
  pwm.setPWM(0, 0, pulselength_base);
  pwm.setPWM(1, 0, pulselength_shoulder);
  pwm.setPWM(2, 0, pulselength_elbow);
  pwm.setPWM(3, 0, pulselength_wrist);
  pwm.setPWM(4, 0, pulselength_twist);
  pwm.setPWM(5, 0, pulselength_grip);
  
}

