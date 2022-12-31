#include <Arduino.h>
static short right_dir = 7;
static short right_pwm = 9;
static short right_encoder = 2;

static short left_dir = 8;
static short left_pwm = 10;
static short left_encoder = 3;
static short left_power = 13;

void updateRightEncoder(void);
void updateLeftEncoder(void);

volatile unsigned long right_encoder_value = 0;
volatile unsigned long left_encoder_value = 0;

double right_pre = 0;
double right_cur = 0;

double robot_rpm = 0;
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
        // Init motor related pin
        pinMode(left_dir, OUTPUT);           // direction for left
        pinMode(left_pwm, OUTPUT);           // speed for left
        pinMode(left_encoder, INPUT_PULLUP); // left encoder reader
        pinMode(left_power, OUTPUT);         // power for left
        digitalWrite(left_power, HIGH);
        attachInterrupt(digitalPinToInterrupt(right_encoder), updateRightEncoder, RISING);
        pinMode(right_dir, OUTPUT);           // direction for right
        pinMode(right_pwm, OUTPUT);           // speed for right
        pinMode(right_encoder, INPUT_PULLUP); // right encoder reader
        attachInterrupt(digitalPinToInterrupt(left_encoder), updateLeftEncoder, RISING);
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

void updateRightEncoder(void)
{
    right_pre = right_cur;
    right_cur = millis();
    robot_rpm = 60 * 1000 / (right_cur - right_pre) / 187.95;
    Serial.print(right_cur-right_pre);
    Serial.print(",");
    Serial.println(robot_rpm);
}

void updateLeftEncoder(void)
{
    left_encoder_value++;
}