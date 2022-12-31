#include <Arduino.h>
static short right_dir = 7;
static short right_pwm = 9;
static short left_dir = 8;
static short left_pwm = 10;
class Robot
{
private:
    // rihgt=motor 1
    // left=motor 2
    // right dir pin2
    // right pwm pin3
    // left dir pin4
    // left pwm pin5

public:
    int init()
    {
        pinMode(left_dir, OUTPUT);
        pinMode(left_pwm, OUTPUT);
        pinMode(right_dir, OUTPUT);
        pinMode(right_pwm, OUTPUT);
    }

    /**
     * @brief
     *
     * @param dir direction,1 for forward, 0 for backward
     * @param speed mvoe speed, range in 0-255
     * @return int 0 for success, 1 for error
     */
    int move(int dir, int speed)
    {
        if (dir == 1)
        {
            digitalWrite(left_dir, LOW);
            digitalWrite(right_dir, HIGH);
        }
        else if (dir == 0)
        {
            digitalWrite(left_dir, HIGH);
            digitalWrite(right_dir, LOW);
        }
        else
        {
            return 1;
        }
        analogWrite(left_pwm, speed);
        analogWrite(right_pwm, speed);
        return 0;
    }

    /**
     * @brief
     *
     * @param dir direction,1 for right, 0 for left
     * @param speed mvoe speed, range in 0-255
     * @return int 0 for success, 1 for error
     */
    int rotate(int dir, int speed)
    {
        if (dir == 1)
        {
            digitalWrite(left_dir, LOW);
            digitalWrite(right_dir, LOW);
        }
        else if (dir == 0)
        {
            digitalWrite(left_dir, HIGH);
            digitalWrite(right_dir, HIGH);
        }
        else
        {
            return 1;
        }
        analogWrite(left_pwm, speed);
        analogWrite(right_pwm, speed);
        return 0;
    }
};
