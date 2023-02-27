// Motor Init
#include "robot.h"
#include "Arduino.h"
#include <IRremote.h>

int RECV_PIN = 12; // the pin where you connect the output pin of IR sensor
IRrecv irrecv(RECV_PIN);
decode_results results;

int speed[8] = {0, 50, 100, 150, 200, 150, 100, 50};
double test_speed = 45;
double pid_output = 0;
int motor_speed_index = 0;
double rpm_goal = 33;
unsigned long motor_current, motor_previous;
unsigned long encoder_timer_pre, encoder_timer_cur;

Robot robot = Robot();
int i = 0;

int dir = 0;
void setup()
{
    // put your setup code here, to run once:
    // https://arduinoinfo.mywikis.net/wiki/Arduino-PWM-Frequency
    TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz
    robot.init();
    Serial.begin(115200);
    motor_previous = millis();
    encoder_timer_pre = millis();
    irrecv.enableIRIn();
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
        // dir=1;
        switch (dir)
        {
        case 0:
            // robot.move(1,0);
            break;
        case 1:
            robot.move(1, 0.2);
            break;
        case 2:
            robot.rotate(1, 0.2);
            break;
        case 3:
            robot.move(0, 0.2);
            break;
        case 4:
            robot.rotate(0, 0.2);
            break;
        case 5:
            robot.stop();
            break;
        }

        if (test_speed == 255)
        {
            test_speed = 0;
        }

        // Serial.print(robot.left_rpm_encoder);
        // Serial.print(",");
        // Serial.print(robot.left_rpm_goal);
        // Serial.print(",");
        // Serial.println(robot.left_output);
        // Serial.println(robot.left_pid.lastTime);
        if (irrecv.decode(&results)) // Returns 0 if no data ready, 1 if data ready.
        {
            Serial.println(results.value);
            switch (results.value)
            {
            case 0xFF18E7:
                Serial.println("up");
                dir = 1;
                break;
            case 0xFF5AA5:
                Serial.println("right");
                dir = 2;
                break;
            case 0xFF4AB5:
                Serial.println("down");
                dir = 3;
                break;
            case 0xFF10EF:
                Serial.println("left");
                dir = 4;
                break;
            case 0xFF38C7:
                Serial.println("ok");
                dir = 5;
                break;
            }
            // int value = results.value; ;// Results of decoding are stored in result.value
            irrecv.resume(); // Restart the ISR state machine and Receive the next value
        }
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
