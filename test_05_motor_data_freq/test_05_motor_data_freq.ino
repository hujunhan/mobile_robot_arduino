// Motor Init
#include "robot.h"
#include "Arduino.h"
int speed[8] = {0, 50, 100, 150, 200, 150, 100, 50};
int test_speed = 45;
int motor_speed_index = 0;
unsigned long motor_current, motor_previous;
unsigned long encoder_timer_pre, encoder_timer_cur;
float rpm;
Robot robot = Robot();
void setup()
{
    // put your setup code here, to run once:
    // https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency
    TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
    robot.init();
    Serial.begin(115200);
    motor_previous = millis();
    encoder_timer_pre = millis();
}

void loop()
{
    // put your main code here, to run repeatedly:
    motor_current = millis();
    if ((motor_current - motor_previous) > 50)
    {
        motor_previous = motor_current;
        // motor_speed_index += 1;

        robot.rotate(1, test_speed);

        if (test_speed == 255)
        {
            test_speed = 0;
        }
        
        // Serial.println(test_speed);
        // test_speed++;
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