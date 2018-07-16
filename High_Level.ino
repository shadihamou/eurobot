/*
 * 
 */

#include "definingVars.h"
#include "PUBPRIV.h"
#include <Wire.h>

Eurobot R; //shortens the the process of calling class before function(functions that are defined are stored in PUBPRIV header file, saves time, the class Robot is set to letter R


void setup() {
  R.Initialize();
  /*--------------------------------ORANGE SIDE----------------------------*/
  if(digitalRead(OG) == HIGH){
  digitalWrite(RED, HIGH);
  while(!digitalRead(pullstart));
  R.Forward_with_obstacle_detection(995);
  delay(500);
  R.SpinRight(80);
  delay(500);
  R.Forward(135); //hit switch
  delay(500);
  R.Backward_with_obstacle_detection(300);
  delay(500);
  R.SpinRight(160);
  delay(500);
  R.Forward_with_obstacle_detection(650);
  delay(500);
  R.SpinLeft(53);
  delay(500);
  //R.Forward_with_obstacle_detection(600);
  //delay(500);
  R.Forward(1075); //400 diagonal
  delay(500);
  R.SpinRight(55);
  delay(1000);
  R.HITBEE_ORANGE(180);
  delay(1000);
  R.Forward(275); //towards wall of bee
  delay(500);
  R.HITBEE_ORANGE(5);
  delay(500);
  R.Forward(-360);
  delay(500);
  R.SpinRight(80);
  delay(500);
  R.Forward(585);
  delay(500);
  R.SpinLeft(80);
  delay(500);
  R.Forward(-1275);  //650
  delay(500);
  R.Forward(500);
  delay(500);
  R.SpinRight(80);
  delay(500);
  R.Forward(-600);
  delay(500);
  }
  /*--------------------------------GREEN SIDE-----------------------------*/
  else{
  digitalWrite(GREEN, HIGH);
  while(!digitalRead(pullstart));
  R.Forward_with_obstacle_detection(995); //1015
  delay(500);
  R.SpinLeft(80);
  delay(500);
  R.Forward(130);//hits switch
  delay(500);
  R.Backward_with_obstacle_detection(300);
  delay(500);
  R.SpinRight(165);
  delay(500);
  R.Forward_with_obstacle_detection(650);
  delay(500);
  R.SpinRight(50);
  delay(500);
  R.Forward(1070); //400 diagonal
  delay(500);
  R.SpinLeft(50);
  delay(1000);
  R.HITBEE_GREEN(10);
  delay(1000);
  R.Forward(280); //towards wall of bee
  delay(1000);
  R.HITBEE_GREEN(180);
  delay(500);
  R.Backward_with_obstacle_detection(350);
  delay(500);
  R.SpinLeft(80);
  delay(500);
  R.Forward(585);
  delay(500);
  R.SpinRight(80);
  delay(500);
  R.Forward(-1275);  //650
  delay(500);
  R.Forward(500);
  delay(500);
  R.SpinLeft(80);
  delay(500);
  R.Forward(-600);
  delay(500);
  }
}
  
  

void loop() {
  

}

