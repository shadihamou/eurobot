#include <Wire.h>          //vector library added contaning predefined functions 
#include "PUBPRIV.h"       //includes the header file where i list the public and private functions 
#include "definingVars.h"  //includes the header file 
#include <Servo.h>

float LastErrorF = 0, SumErrorF = 0;   //declaring variables --->data-type, variable name<---

Servo myServo;
/*
 * Function used to initialize all starting parameters of the robot
 */
void Eurobot::Initialize()
{
  //BEGIN SERIAL TRANSMISSION @ SPECIFIED BAUD RATE
  Serial.begin(9600);
  //BEGIN WIRE TRANSMISSION (i2C)
  Wire.begin();
  //servos ATTACH
  myServo.attach(ServoMotor2);
  myServo.attach(ServoMotor);
  myServo.attach(LatchServo);
  myServo.write(servoPos);
  //ultrasonic sensor declaration of pin nature (OUT PULSE, IN PULSE)
  pinMode(EchoPinFront, INPUT);
  pinMode(TriggerPinFront, OUTPUT);
  pinMode(EchoPinBack, INPUT);
  pinMode(TriggerPinBack, OUTPUT);
  //used for ball launching mechanism that was DISCONTINUED
  /*pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  digitalWrite(motorPin1, LOW); //sets the direction, 0 and 1 (want the motors spinning in opposite directions to provide forward force to the balls)
  digitalWrite(motorPin2, HIGH);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, 0); //when the robot initialises i do not want the motors for the launcher to start spinning, know its potential ---> its ZERO */
  //pullstart mechanism
  pinMode(pullstart, INPUT);
  //green/orange side switch to make life easier
  pinMode(OG, INPUT);
}

/*
 * Function used to read ultrasonic sensor used for the obstacle avoidance spec
 * 
 */
long Eurobot::ReadUltrasonic(int TriggerPin, int EchoPin)
{
    long duration, inches, cm;

    // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(TriggerPin, LOW);  //low pulse emitted
    delayMicroseconds(2);
    digitalWrite(TriggerPin, HIGH); //high pulse emitted
    delayMicroseconds(5);
    digitalWrite(TriggerPin, LOW);

    duration = pulseIn(EchoPin, HIGH);

    // convert the time into a distance
    cm = (duration/2) / 29.1;
    inches = (duration/2) / 74;
    return cm;

    /* Optional printout to the Serial monitor if i need to test it */
    //Serial.print(inches);
    //Serial.print("in, ");
    //Serial.print(cm);
    //Serial.print("cm");
    //Serial.println();
}

/*
 * Function used to read both encoders and store their value into the global variables E1 and E2
 * code is taken from the Odometry Briefing document and adapted to work for both encoders
 */
void Eurobot::ReadEncoders()
{
  E1 = 0;
  E2 = 0;
  Wire.beginTransmission(Adress);                      // Send byte to get a reading from encoder 1
  Wire.write(Encoder1);
  Wire.endTransmission();

  Wire.requestFrom(Adress, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                             // Wait for 4 bytes to arrive
  E1 = Wire.read();                                 // First byte for encoder 1, HH.
  E1 <<= 8;
  E1 += Wire.read();                                     // Second byte for encoder 1, HL
  E1 <<= 8;
  E1 += Wire.read();                                     // Third byte for encoder 1, LH
  E1 <<= 8;
  E1  +=Wire.read();                                     // Fourth byte for encoder 1, LL

  Wire.beginTransmission(Adress);                      // Send byte to get a reading from encoder 2
  Wire.write(Encoder2);
  Wire.endTransmission();

  Wire.requestFrom(Adress, 4);                         // Request 4 bytes from MD25
  while(Wire.available() < 4);                              // Wait for 4 bytes to arrive
  E2 = Wire.read();                                 // First byte for encoder 2, HH.
  E2 <<= 8;
  E2 += Wire.read();                                     // Second byte for encoder 2, HL
  E2 <<= 8;
  E2 += Wire.read();                                     // Third byte for encoder 2, LH
  E2 <<= 8;
  E2  +=Wire.read();                                     // Fourth byte for encoder 2, LL
//  Serial.print(E1);
//  Serial.print("  ");
//  Serial.println(E2);

}
/*
 * Function that takes as an input a distance in mm and makes the robot drive in a straight line that specified distance
 * Uses a PID controller to compute the required speed which is send to the motors through the Drive() function
 * Implements an acceleration as the function is called in order to prevent sudden acceleration to top speed
 * Uses a counter to check wheter or not it has reached its required distance and it hasnt overshot (hall-effect sensors already included in EMG-30 motors)
 */
void Eurobot::Forward(float Distance){       // function for robot to drive forward in a straight line for a specified distance
  Serial.print("Forward");
  Wire.beginTransmission(Adress);          //transmission to adress of MD25 (0x58)
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();

  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  //Wire.write(0x30);
  Wire.endTransmission();
  Distance = int(360 * Distance / (CircumferenceWheel1/2 + CircumferenceWheel2/2) + 0.5);


  float LastError = 0, SumError = 0, Error;
  float Speed;
  float CurrentDistance;
  int n=0;
  Goal = 0;
  SumErrorF = 0;
  while(!Goal)                           // while it hasnt reached the target distance, it repeats this loop
  {

    ReadEncoders();                      // read the values of the encoders and stores them in E1 and E2
    CurrentDistance = (E1+E2) / 2;       // computes the distance travelled as the average of the 2 encoders
    Error = Distance - CurrentDistance;  // computes the error in order to use the PID controller
    if(CurrentDistance < 180 && Distance > 401) Speed = 25 + CurrentDistance*0.6;

    else if(Error > THRESHOLD) Speed = 80;   // if the error is quite big, it travells at maximum speed
    else
    {
      Speed = Kp*Error + Kd*(Error - LastError) + Ki * SumError;    // standard PID controller, though Ki might not be needed as the Integral of the distance does not have any significance
      LastError = Error;                                            // in order to compute the derivative term
      SumError = SumError + Error;                                  // in order to compute the integral term
    }

    Drive(Speed);
    if((Error < 5) && (Error >-5)) n++;         // every time the error is 0, the counter goes up by 1
    if((Error > 20)&&(Error < -20)) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;       // random value, as soon as the counter reaches 10, it breaks out of the loop (it reaches the goal)
    Serial.print("   ");
    Serial.print(n);
    Serial.print("   ");
  }

  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();      // set speed of motor 2 (right) to 0, stops both motors
}

/*
 * Modified forward function capable of stopping the robot in between in case it faces an obstacle in between.
 * 
 */
void Eurobot::Forward_with_obstacle_detection(float Distance){       // function that should make the robot drive forward in a straight line for a specified distance
  Serial.print("Forward with obstacle detection");
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();

  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  //Wire.write(0x30);
  Wire.endTransmission();
  Distance = int(360 * Distance / (CircumferenceWheel1/2 + CircumferenceWheel2/2) + 0.5);


  float LastError = 0, SumError = 0, Error;
  float Speed;
  float CurrentDistance;
  int n=0;
  Goal = 0;
  SumErrorF = 0;
  while(!Goal)                           // while it hasnt reached the target distance, it repeats this loop
  {
    // This condition occurs when there is an obstacle in the path
    bool stopped = false;
    while(ReadUltrasonic(TriggerPinFront, EchoPinFront) < OBSTACLE_THRESHOLD) {  //while the reading of ultrasonic is smaller than predefined threshold
        if(!stopped) {                              //if not stopped i.e. true, stops both motors
            Wire.beginTransmission(Adress);
            Wire.write(Speed1);
            Wire.write(0);
            Wire.endTransmission();       // set speed of motor 1 (left) to 0

            Wire.beginTransmission(Adress);
            Wire.write(Speed2);
            Wire.write(0);
            Wire.endTransmission();      // set speed of motor 2 (right) to 0, stops both motors

            stopped = true;     // This is to make sure that the robot does not keep on transmitting stop transmission to the motors
                                // since it is a waste of resources. This flag makes sure motors are stopped just once.
        }
        // else: do nothing else until the obstacle moves away
    }

    ReadEncoders();                      // read the values of the encoders and stores them in E1 and E2
    CurrentDistance = (E1+E2) / 2;       // computes the distance travelled as the average of the 2 encoders
    Error = Distance - CurrentDistance;  // computes the error in order to use the PID controller
    if(CurrentDistance < 180 && Distance > 401) Speed = 25 + CurrentDistance*0.6;

    else if(Error > THRESHOLD) Speed = 80;   // is the error is quite big, it travells at maximum speed
    else
    {
      Speed = Kp*Error + Kd*(Error - LastError) + Ki * SumError;    // standard PID controller, though Ki might not be needed as the Integral of the distance does not have any significance
      LastError = Error;                                            // in order to compute the derivative term
      SumError = SumError + Error;                                  // in order to compute the integral term
    }

    Drive(Speed);
    if((Error < 5) && (Error >-5)) n++;         // every time the error is 0, the counter goes up by 1
    if((Error > 20)&&(Error < -20)) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;       // random value, as soon as the counter reaches 10, it breaks out of the loop (it reaches the goal)
    Serial.print("   ");
    Serial.print(n);
    Serial.print("   ");
  }

  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();      // set speed of motor 2 (right) to 0, stops both motors
}
void Eurobot::Backward_with_obstacle_detection(float Distance){       // function that should make the robot drive forward in a straight line for a specified distance
  Serial.print("backward with obstacle detection");
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();

  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  //Wire.write(0x30);
  Wire.endTransmission();
  Distance = int(360 * Distance / (CircumferenceWheel1/2 + CircumferenceWheel2/2) + 0.5);


  float LastError = 0, SumError = 0, Error;
  float Speed;
  float CurrentDistance;
  int n=0;
  Goal = 0;
  SumErrorF = 0;
  while(!Goal)                           // while it hasnt reached the target distance, it repeats this loop
  {
    // This condition occurs when there is an obstacle in the path
    bool stopped = false;
    while(ReadUltrasonic(TriggerPinBack, EchoPinBack) < OBSTACLE_THRESHOLD) {  //while the reading of ultrasonic is smaller than predefined threshold
        if(!stopped) {                              //if not stopped i.e. true, stops both motors
            Wire.beginTransmission(Adress);
            Wire.write(Speed1);
            Wire.write(0);
            Wire.endTransmission();       // set speed of motor 1 (left) to 0

            Wire.beginTransmission(Adress);
            Wire.write(Speed2);
            Wire.write(0);
            Wire.endTransmission();      // set speed of motor 2 (right) to 0, stops both motors

            stopped = true;     // This is to make sure that the robot does not keep on transmitting stop transmission to the motors
                                // since it is a waste of resources. This flag makes sure motors are stopped just once.
        }
        // else: do nothing else until the obstacle moves away
    }

    ReadEncoders();                      // read the values of the encoders and stores them in E1 and E2
    CurrentDistance = (E1+E2) / 2;       // computes the distance travelled as the average of the 2 encoders
    Error = -Distance - CurrentDistance;  // computes the error in order to use the PID controller
    if(CurrentDistance < 180 && Distance > 401) Speed = 25 + CurrentDistance*0.6;

    else if(Error > THRESHOLD) Speed = 80;   // is the error is quite big, it travells at maximum speed
    else
    {
      Speed = Kp*Error + Kd*(Error - LastError) + Ki * SumError;    // standard PID controller, though Ki might not be needed as the Integral of the distance does not have any significance
      LastError = Error;                                            // in order to compute the derivative term
      SumError = SumError + Error;                                  // in order to compute the integral term
    }

    Drive(Speed);
    if((Error < 5) && (Error >-5)) n++;         // every time the error is 0, the counter goes up by 1
    if((Error > 20)&&(Error < -20)) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;       // random value, as soon as the counter reaches 10, it breaks out of the loop (it reaches the goal)
    Serial.print("   ");
    Serial.print(n);
    Serial.print("   ");
  }

  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();      // set speed of motor 2 (right) to 0, stops both motors
}
/*
 * Private function only called by the Forward() function
 * Implements a PID controller to ensure the robot drives straight at all times
 */
void Eurobot::Drive(float Speed){
  float MotorCorrection, S1, S2;
  float Error;
  if(Speed > 100) Speed = 100;
  if(Speed < -100) Speed = -100;      // caps the speed
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(1);            // sets the MD25 to mode 1
  Wire.endTransmission();


  Error = E2 * CircumferenceWheel2/1000 - E1 * CircumferenceWheel1/1000; // uses the circumfernce of the wheels as a parameter because the wheels are slighty different in size, size calculated experimentally
  Serial.print("   ");
  Serial.print(Error);
  Serial.print("   ");
  MotorCorrection = DKp * Error + DKd * (Error - LastErrorF) + DKi * SumErrorF;    // standard PID controller, needs the Ki term, calculates a motor correction factor
//  MotorCorrection = 0;
  LastErrorF = Error;              // in order to compute the derivative term
  SumErrorF = SumErrorF + Error;   // in order to compute the integral term

  S1 = Speed + MotorCorrection;
  S2 = Speed - MotorCorrection;  // adds the motor correction factor to the left motor, while it substracts it from the right one

  if(S1 > 127) S1 = 127;
  if(S2 > 127) S2 = 127;
  if(S1 < -128) S1 = -128;     // capping the values for the motor speeds, as a value bigger than 127 or smaller than -128 might screw up the controller
  if(S2 < -128) S2 = -128;
//  Serial.print("   ");
//  Serial.print(Speed);
//  Serial.print("   ");
//  Serial.print("   ");
//  Serial.print(S1);
//  Serial.print("   ");
//  Serial.print("   ");
//  Serial.print(S2);
//  Serial.print("   ");        // debbuging purpose

//  Serial.print("PID");
  if(nor%2 == 0){                     // implements a randomizer to power one motor before the other, by powering only one motor before the other, over time the error becomes significant
    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(S2+0.5));
    Wire.endTransmission();       // set speed of motor 1 (left)

    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(S1+0.5));
    Wire.endTransmission();      // set speed of motor 2 (right)
    //delay(2);
    nor++;
  }
  else{
    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(S1+0.5));
    Wire.endTransmission();       // set speed of motor 1 (left)

    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(S2+0.5));
    Wire.endTransmission();      // set speed of motor 2 (right)
    //delay(2);
    nor++;
  }
}

/*
 * This function is used to rotate the robot around its axis by a given number of degrees
 * Implements a PID  controller
 */
void Eurobot::SpinLeft(float Degrees){
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();

  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  Wire.endTransmission();
  float Error, LastError;
  float Speed;
  int n=0;
  Degrees = 3.1428 *WidthRobot * Degrees / ( CircumferenceWheel2);   // converts degrees to encoder ticks for the right wheel, because it's the one spinning forward
  Goal = 0;
  while(!Goal){
    ReadEncoders();
    Error = Degrees - E2;
    Speed = TKp * Error + TKd*(Error - LastError)+0.5;
    //Speed = 80;
    LastError = Error;
    if(Speed > 100) Speed = 100;
    if(Speed < -100) Speed = -100;
    Serial.print(Speed);
    Serial.print("   ");
    Serial.print(Error);
    Serial.println("   ");
    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(Speed+0.5));
    Wire.endTransmission();

    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(-Speed-0.5));
    Wire.endTransmission();
    if((Error < 5) && (Error >-5)) n++;         // every time the error is 0, the counter goes up by 1
    if((Error > 20)||(Error < -20)) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;       // random value, as soon as the counter reaches 10, it breaks out of the loop (it reaches the goal)
    Serial.println(Goal);
  }
  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();
}
/*
 * Very similar to SpinLeft(), only this function reverses the spin direction
 */
void Eurobot::SpinRight(float Degrees){
  Wire.beginTransmission(Adress);
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();

  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  Wire.endTransmission();
  float Error, LastError;
  float Speed;
  int n=0;
  Degrees = 3.1428 *WidthRobot * Degrees / (CircumferenceWheel1);
  Goal = 0;
  while(!Goal){
    ReadEncoders();
    Error = Degrees - E1;
    //Serial.println(Error);
    Speed = TKp * Error + TKd*(Error - LastError)+0.5;
    //Speed = 80;
    LastError = Error;
    if(Speed > 100) Speed = 100;
    if(Speed < -100) Speed = -100;
    Serial.print(Speed);
    Serial.print("   ");
    Serial.print(Error);
    Serial.print("   ");
    Serial.println(n);
    Wire.beginTransmission(Adress);
    Wire.write(Speed2);
    Wire.write(int(-Speed-0.5));
    Wire.endTransmission();

    Wire.beginTransmission(Adress);
    Wire.write(Speed1);
    Wire.write(int(Speed+0.5));
    Wire.endTransmission();
    if((Error < 5) && (Error >-5)) n++;         // every time the error is 0, the counter goes up by 1
    if((Error > 20)&&(Error < -20)) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;       // random value, as soon as the counter reaches 10, it breaks out of the loop (it reaches the goal)
  }
  //Serial.print("FINISH");
   Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();
}
/*
 * Used to operate the servo that hits the bee on the orange side
 */
void Eurobot::HITBEE_ORANGE(int ServoAngle) {
  myServo.attach(ServoMotor);
  myServo.write(ServoAngle);
  Serial.print(ServoAngle);
  delay(200);
}
/*
 *Used to operate motors used in launching mechanism-------- DISCONTINUED ---------
 */
void Eurobot::ActivateMotors(float A){
  analogWrite(ENA,A*255); //255 is the maximum duty cycle because it is 8 bit (motor shield for launcher), A is a float (number between 0 and 1) in order to regulate motor speed
  delay(40000);// how long do i want the launching mechanism to work for
  analogWrite(ENA, 0); //stops the motors in the launching mechanism otherwise they'd keep spinning
}
/*
 * initially used as a function for the pullstart, it is a recursive function however, it was ditched
 */
void Eurobot::Pull(){   
  if (digitalRead(pullstart) == HIGH){
  Serial.println("Waiting");
  delay(200);
  Pull();
}
  delay(1000);
}
/*
 * Used to hit bee on green side
 */
void Eurobot::HITBEE_GREEN(int ServoAngle){
  myServo.attach(ServoMotor2);
  myServo.write(ServoAngle);
  delay(200);
  }
/*
 * Used to operate LatchServo that opens latch for balls to drop (extra points) ask Dr.Prior to check if sabotage
 */

void Eurobot::open_sesame(int ServoAngle){
  myServo.attach(LatchServo);
  myServo.write(ServoAngle);
  delay(200);
  }
  
/*
 * Used to calibrate circumference of wheels experimentally to input the correct values into defines, commented out line that converts input of function into encoder ticks, now the input is purely encoder ticks
 */
void Eurobot::CIRCUM_CHECK(float Distance){       
  Serial.print("Forward");
  Wire.beginTransmission(Adress);          //transmission to adress of MD25 (0x58)
  Wire.write(Mode);
  Wire.write(0x01);            // sets the MD25 to mode 1
  Wire.endTransmission();

  Wire.beginTransmission(Adress);
  Wire.write(Command);
  Wire.write(0x20);            // sends byte to restore the encoders to 0
  //Wire.write(0x30);
  Wire.endTransmission();
  //Distance = int(360 * Distance / (CircumferenceWheel1/2 + CircumferenceWheel2/2) + 0.5);


  float LastError = 0, SumError = 0, Error;
  float Speed;
  float CurrentDistance;
  int n=0;
  Goal = 0;
  SumErrorF = 0;
  while(!Goal)                           // while it hasnt reached the target distance, it repeats this loop
  {

    ReadEncoders();                      // read the values of the encoders and stores them in E1 and E2
    CurrentDistance = (E1+E2) / 2;       // computes the distance travelled as the average of the 2 encoders
    Error = Distance - CurrentDistance;  // computes the error in order to use the PID controller
    if(CurrentDistance < 180 && Distance > 401) Speed = 25 + CurrentDistance*0.6;

    else if(Error > THRESHOLD) Speed = 80;   // if the error is quite big, it travells at maximum speed
    else
    {
      Speed = Kp*Error + Kd*(Error - LastError) + Ki * SumError;    // standard PID controller, though Ki might not be needed as the Integral of the distance does not have any significance
      LastError = Error;                                            // in order to compute the derivative term
      SumError = SumError + Error;                                  // in order to compute the integral term
    }

    Drive(Speed);
    if((Error < 5) && (Error >-5)) n++;         // every time the error is 0, the counter goes up by 1
    if((Error > 20)&&(Error < -20)) n=0;         // random value, if the value of the error increases (overshoots), the counter resets
    if(n == 5) Goal = 1;       // random value, as soon as the counter reaches 10, it breaks out of the loop (it reaches the goal)
    Serial.print("   ");
    Serial.print(n);
    Serial.print("   ");
  }

  Wire.beginTransmission(Adress);
  Wire.write(Speed1);
  Wire.write(0);
  Wire.endTransmission();       // set speed of motor 1 (left) to 0

  Wire.beginTransmission(Adress);
  Wire.write(Speed2);
  Wire.write(0);
  Wire.endTransmission();      // set speed of motor 2 (right) to 0, stops both motors
}
