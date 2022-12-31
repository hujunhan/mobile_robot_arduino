//Motor Init
#include "robot.h"
#include "Arduino.h"
int speed[8]={0,50,100,150,200,150,100,50};
int motor_speed_index=0;
long motor_current,motor_previous;
Robot robot=Robot();
void setup() {
  // put your setup code here, to run once:
  //https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency
  TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  robot.init();
  
  motor_previous=millis();



}

void loop() {
  // put your main code here, to run repeatedly:
  motor_current=millis();
  if((motor_current-motor_previous)>2000){
    motor_speed_index+=1;
    motor_previous=motor_current;
  }

  robot.rotate(1,speed[motor_speed_index%8]);

  
}