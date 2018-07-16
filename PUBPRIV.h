#ifndef PUBPRIV_H_INCLUDED //checks if library is imported, if it doesnt exist u define it, prevents double importation of library 
#define PUBPRIV_H_INCLUDED //if not defined, library is imported/defined
#include "Arduino.h"  //inlcludes the arduino library that is standard
#include "definingVars.h"  //includes the header file where i define all the constants i use in the main code for the functions so that if i want to make any changes i simply access the header file
class Eurobot
{
  public:  //call in loop or setup (outside the class) in the high level code
    // Driving Functions
    void Forward(float distance);
    void Forward_with_obstacle_detection(float Distance); 
    void Backward_with_obstacle_detection(float Distance);
    void Initialize();
    void SpinLeft(float Degrees);
    void SpinRight(float Degrees);
    // Encoder Reading
    void ReadEncoders();
    //servo that HITS BEE ORANGE SIDE
    void HITBEE_ORANGE(int ServoAngle);
    // Read ultrasonic sensor for obstacle avoidance
    long ReadUltrasonic(int, int);
    // Operate Launching Mechanism
    void ActivateMotors(float A);
    // start pulling bitchhes
    void Pull();
    //servo that HITS BEE GREEN SIDE
    void HITBEE_GREEN(int ServoAngle);
    //function used to check circumference of wheels experimentally (input encoder ticks)
    void CIRCUM_CHECK(float Distance);
    //function used to open latch for balls
    void open_sesame(int ServoAngle);


  private: //these are private functions that can only be called by object within the same class: object is our Eurobot robot!

    bool Goal;
    int nor=0;
    //long duration, inches, cm;
    long E1, E2;    //global variables storing encoder values (NOT SHADOWED)
    float SumError;
    void Drive(float Speed);

}
;
#endif

