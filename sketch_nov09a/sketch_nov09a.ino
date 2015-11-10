#include <Servo.h> // include the library for the servo functions
#include <stdio.h>
#include <stdlib.h>
/*
  Servo
 */
 
// Pin Assignment for MyQuad Shield
// pin 0 - Arduino RX
// pin 1 - Arucino TX
// Pins 10,11,12  are free for other uses, 
// Pin 13 is connected to Arduino LED and resistor.
// Pin A0 is the Battery Sense pin
// Pin A1 is the ZBee Reset
// Pin A2 and A3 are free analog pins
// Pin A4 is the I2C SDA - Serial Data
// Pin A5 is the I2C SCL - Serial Clock

// Names for MyQuad Shield pins

// LED Pins
int redLed = 8;
int greenLed = 7;

// Motor control pins
// if wires are connected to the motors as per the pinout 
int backMTR = 9;    // pin to control back motor  (9 or 6)
int rightMTR = 6;   // pin to control right (6 or 3)
int leftMTR = 5;    // pin to control left (5 or 9)
int frontMTR = 3;   // pin to control front (3 or 5)

/*                                                                                                                                                                                                                                                                                                                                             
// if wires are connected the opposite direction
int backMTR = 6;    // pin to control back motor  (9 or 6)
int rightMTR = 3;   // pin to control right (6 or 3)
int leftMTR = 9;    // pin to control left (5 or 9)
int frontMTR = 5;   // pin to control front (3 or 5)
*/

// Some other pin connections
int i2cEOC = 4;
int MPUint = 2;

// declaring variables of type Servo for the program

Servo frontServo;
Servo backServo;
Servo leftServo;
Servo rightServo;

// Some program variables
int servoOff = 0; // Variable to turn off the Servos
int first; // Loop control variable, 
           //to control only one pass thru the loop


// the setup routine runs once when you press reset:
void setup() {                
  // initialize the the red and green LED and turn them on;

  Serial.begin(9600); // Initializes the USB connection, 
                      // for debug and communication between
                      // the host system (laptop) and the Arduino
                      
 // set the LED pin modes and an initial value for each                     
  pinMode(greenLed, OUTPUT);                        
  pinMode(redLed, OUTPUT);
  digitalWrite(greenLed, HIGH);
  digitalWrite(redLed, HIGH);
  
  // Attach each motor to a Servo variable and
  // turn the motor OFF!
  frontServo.attach(frontMTR);
  frontServo.write(servoOff);
  
  backServo.attach(backMTR);
  backServo.write(servoOff);
  
  leftServo.attach(leftMTR);
  leftServo.write(servoOff);
  
  rightServo.attach(rightMTR);
  rightServo.write(servoOff);
   
  first = 1; // initialize the loop variable
            // so we can check that this is the first
            // time through the infinite loop
   
}

// Some variables for serial communication
//  Expected input format: s{Speed}
//                         d{"w", "s"}
char in_type = '\0';
char in_dump_buf[10];

// the loop routine runs over and over again forever:
void loop() {
  
  // setup some control variables and initialize them
  // Some of the variables are used as "constant values"
  // in this program and should not be changed.
  
  int speedServo;  // current speed of servo
  int minSpeed = 30; 
  int maxSpeed = 90;  // absolute max speed: 180
  int servoDelay = 100;
  int lowSpeed = 30;
  int ledDuration = 2000;
  int duration = 4000;
  
  int input_byte = 0;
  first = 0;
  boolean manual_state = false;
  int speed_dump = 0;
  if(Serial.available() > 0){
    input_byte = Serial.read();
    first = input_byte == 'k';
    manual_state = input_byte == 'M';
  }
  
  if (first) {
    // check that each LED works
 
 ////////////////////////////////////////////////////////
    digitalWrite(greenLed, HIGH);   // turn the LED on  
    Serial.println("Check that the Green LED is ON.");
    delay(ledDuration);               // wait for an interval
    digitalWrite(greenLed, LOW);    // turn the LED off by making the voltage LOW
    Serial.println("Check that the Green LED is OFF.");
    delay(ledDuration);               // wait for an interval
    
  /////////////////////////////////////////////////////////
    digitalWrite(redLed, HIGH);   // turn the LED on 
    Serial.println("Check that the Red LED is ON.");
    delay(ledDuration);               // wait for an interval
    digitalWrite(redLed, LOW);    // turn the LED off by making the voltage LOW
    Serial.println("Check that the Red LED is OFF.");
    delay(ledDuration);               // wait for an interval
   
   // Slowly ramp up the speed of all motors
    ///////////////////////////////////////////////
    Serial.println("Ramping up motor speed!!!!!!!!!!!!!!!!!!");
    for ( speedServo = minSpeed; speedServo < maxSpeed; speedServo++) {
      Serial.print("Motor speed setting: ");
      Serial.println(speedServo);
      frontServo.write(speedServo);
      backServo.write(speedServo);
      leftServo.write(speedServo);
      rightServo.write(speedServo);
      delay(servoDelay);
    }
    Serial.println("Slowing down motors!!!!!!!!!!!!!!!!!!!!!!");
    for ( ; speedServo >= minSpeed; speedServo--) {
      Serial.print("Motor speed setting: ");
      Serial.println(speedServo);
      frontServo.write(speedServo);
      backServo.write(speedServo);
      leftServo.write(speedServo);
      rightServo.write(speedServo);
      delay(servoDelay);
    }
    Serial.println("Motors stopped!");
    
    first = 0; // We've gone thru the loop once and will not repeat
  }
  if(manual_state){
    speedServo = 0;
    if(Serial.available() > 0){
      scanf("%c%s", &in_type, in_dump_buf);
      if(in_type == 'd'){
        if(in_dump_buf[0] == 'w' && speedServo < maxSpeed)
          ++speedServo;
        else if(in_dump_buf[0] == 's' && speedServo > minSpeed)
          --speedServo;
      } else if(in_type == 's'){
        speed_dump = atoi(in_dump_buf);
        if(speed_dump < maxSpeed && speed_dump >= 0)
          speedServo = speed_dump;
      }
    }
  }
}

