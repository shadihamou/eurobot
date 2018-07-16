#ifndef DEFININGVARS_H_INCLUDED  //if not defined, define the header file (next line)
#define DEFININGVARS_H_INCLUDED

// pins on arduino uno board
#define LatchServo         2 
#define pullstart          3
#define OG                 4
#define EchoPinBack        5
#define TriggerPinBack     6
#define ServoMotor2        7
#define GREEN              8   //LED
#define RED                9   //LED
#define TriggerPinFront    11 
#define ServoMotor         12 
#define EchoPinFront       13 // blue-->GND, green-->echo, trigger--> yellow, orange-->vcc.

//DC L293D Motor Driver defining motor pin constants 
//----------------MOTOR A (LAUNCHER) ------------------------------------------------------------ DISCONTINUED
#define motorPin1	         9   // Pin 14 of L293     change pins depending on arduino connections      
#define motorPin2          10  // Pin 10 of L293
#define ENA                3   // Pin 3 PWM output from arduino to l298 ENA port   
//-----------------------------------------------------------------------------------------------
// Memory Adresses
#define Speed1     0x00   // register to acces the speed of motor 1 (left)
#define Speed2     0x01   // register to acces the speed of motor 2 (right)

#define Mode       0xF   // register to acces the control mode (0 1 2 3)
#define Encoder1   0x02   // register to acces the encoder of motor 1
#define Encoder2   0x06   // register to acces the encoder of motor 2
#define Command    0x10   // register to acces the command register of the MD25
#define Adress     0x58   // adress of the MD25

#define DSR        0x30   // byte to send to disable automatic speed regulation to command
#define ESR        0x31   // byte to send to enable automatic speed regulation to command

#define THRESHOLD   700   // random value, threshold below which the PID controller for the distance function driving activates
#define OBSTACLE_THRESHOLD  21  // Can be set in cm or inches depending on use. This is the minimum distance when the robot stops
                               // in case of an obstacle
// physical dimensions
#define CircumferenceWheel1 321.92   // mm   size of left wheel
#define CircumferenceWheel2 322.92   // mm   size of right wheel
#define WidthRobot          260   // mm was 287

// PID Forward Driving costants (used in the Drive function, to make the robot drive straight)
#define DKp           0.6   // best value until now (deals with big error)
#define DKd           0.2   // best value until now was 0.2!!!! (deals with fluctuations)
#define DKi           0   // zero because the integral part doesnt work well with the PID Controller (deals with steady state error)

// PID Turn Radius Degrees
#define TKp           0.6     // yet to be determined
#define TKd           0.2     // yet to be determined

// PID Turn Straight Degrees
#define DeKp          0     // yet to be determined
#define DeKd          0     // yet to be determined
#define DeKi          0     // yet to be determined

// PID Distance(used to compute the accelerations when reaching the desired distance)
#define Kp            0.47  // EXCELLENT value
#define Kd            0.8   // EXCELLENT value
#define Ki            0   // zero: dont need the integral of the distance

//Servo Initial Angle
#define servoPos      10

#endif

