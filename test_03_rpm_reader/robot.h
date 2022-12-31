#include <Arduino.h>
static short right_dir=2;
static short right_pwm=3;
static short left_dir=4;
static short left_pwm=5;
class Robot
{
private:
    // rihgt=motor 1
    // left=motor 2
    //right dir pin2
    //right pwm pin3
    //left dir pin4
    //left pwm pin5
    

public:
     Robot(/* args */);
    ~ Robot();

    /**
     * @brief 
     * 
     * @param dir 1 for forward, 0 for backward
     * @param speed range in 0-255
     * @return int 0 for success, 1 for error
     */
    int move(int dir,int speed){
        digitalWrite(left_dir,dir);
        digitalWrite(right_dir,dir);
        analogWrite(left_pwm,speed);
        analogWrite(right_pwm,speed);

    }
};

 Robot:: Robot(/* args */)
{
    pinMode(left_dir,OUTPUT);
    pinMode(left_pwm,OUTPUT);
    pinMode(right_dir,OUTPUT);
    pinMode(right_pwm,OUTPUT);
}

 Robot::~ Robot()
{
}

