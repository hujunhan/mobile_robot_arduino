// Motor Init
#include "robot.h"
#include "Arduino.h"
int speed[8] = {0, 50, 100, 150, 200, 150, 100, 50};
double test_speed = 45;
double pid_output = 0;
int motor_speed_index = 0;
double rpm_goal = 33;
unsigned long motor_current, motor_previous;
unsigned long encoder_timer_pre, encoder_timer_cur;

Robot robot = Robot();
int i = 0;

unsigned long start_time;
int move_time = 1000;
int rotate_time = 1000;
void setup()
{
    // put your setup code here, to run once:
    // https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency
    TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
    robot.init();
    Serial.begin(115200);
    motor_previous = millis();
    encoder_timer_pre = millis();
    start_time = millis();
}

void loop()
{
    // put your main code here, to run repeatedly:
    
    motor_current = millis();
    if ((motor_current - motor_previous) > 100)
    {
        i++;
        motor_previous = motor_current;
        robot.left_rpm_encoder = left_encoder_value * 600 / 753.2;
        robot.right_rpm_encoder = right_encoder_value * 600 / 753.2;
        left_encoder_value = 0;
        right_encoder_value = 0;
        if (i<20)
        {
            robot.move(0, 0.2);
        }
        else if ( i<30)
        {
            robot.rotate(1, 0.2);
        }
        else if (i<50)
        {
            robot.move(0, 0.2);
        }
        else{
            robot.stop();
        }
        // motor_speed_index += 1;
        // if (i < 10)
        // {
        //     robot.rotate(1, 0.2);
        // }
        // else if (i < 20)
        // {
        //     robot.move(1, 0.2);
        // }
        // else if (i < 30)
        // {
        //     robot.move(0, 0.2);
        // }
        // else if (i < 40)
        // {
        //     robot.rotate(0, 0.2);
        // }
        // else{
        //     i=0;
        // }

        if (test_speed == 255)
        {
            test_speed = 0;
        }

        Serial.print(robot.left_rpm_encoder);
        Serial.print(",");
        Serial.print(robot.left_rpm_goal);
        Serial.print(",");
        Serial.println(robot.left_output);
    }

    // encoder_timer_cur = millis();
    // if ((encoder_timer_cur - encoder_timer_pre) > 1000)
    // {
    //     encoder_timer_pre = encoder_timer_cur;
    //     rpm = right_encoder_value * 60 / 187.95;
    //     right_encoder_value = 0;
    //     Serial.print(encoder_timer_cur);
    //     Serial.print(",");
    //     Serial.print(rpm);
    //     Serial.print(",");
    //     Serial.println(test_speed);
    // }
}