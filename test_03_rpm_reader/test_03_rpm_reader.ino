#include "robot.h"
//Motor Init
int speed[8]={0,50,100,150,200,150,100,50};
int motor_speed_index=0;
long motor_current,motor_previous;

//Encoder Init
  volatile long encoder_value=0;
  long previous_time=0;
  long current_time=0;
  int interval=1000;
void setup() {
  // put your setup code here, to run once:
  Robot robot=Robot();
  
  motor_previous=millis();

  // Encoder setup
  Serial.begin(115200);
  // Encoder input
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(3), updateEncoder, RISING);

  // 5v;
  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);



  //Init time
  previous_time=millis();

}

void loop() {
  // put your main code here, to run repeatedly:
  motor_current=millis();
  if((motor_current-motor_previous)>2000){
    motor_speed_index+=1;
    motor_previous=motor_current;
  }
  current_time=millis();
  if((current_time-previous_time)>interval){
    previous_time=current_time;
    Serial.print("RPM: ");
    Serial.print(encoder_value*60/187.95);
    Serial.print('\n');
    encoder_value=0;
  }
  analogWrite(5,speed[motor_speed_index%8]);

  
}
void updateEncoder()
{
  encoder_value=encoder_value+1;
}