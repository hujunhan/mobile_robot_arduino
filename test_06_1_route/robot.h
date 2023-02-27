#include <Arduino.h>
#include <PID_v1.h>

/**
 * @brief Pin setup for right motor
 *
 */
static short right_dir = 7;
static short right_pwm = 9;
uint8_t right_encoder_1 = A2;
uint8_t right_encoder_2 = A3;

/**
 * @brief Pin setup for left motor
 *
 */
static short left_dir = 8;
static short left_pwm = 10;
static short left_encoder_1 = 3;
static short left_encoder_2 = 2;
static short left_power = 13;

void updateRightEncoder(void);
void updateLeftEncoder(void);
int RPMToPWM(float RPM);
float speedToRPM(float speed);
void pciSetup(byte pin);

volatile unsigned long right_encoder_value = 0;
volatile unsigned long left_encoder_value = 0;

class Robot
{
public:
    // rihgt=motor 1
    // left=motor 2
    // right dir pin2
    // right pwm pin3
    // left dir pin4
    // left pwm pin5
    bool MOTOR_START = false;
    unsigned long timer_pre;
    unsigned long timer_cur;
    double rpm_goal;

    // for left motor
    double left_rpm_encoder = 0;
    double left_rpm_goal = 0;
    int left_output = 0;
    double left_pid_output = 0;
    PID left_pid = PID(&left_rpm_encoder, &left_pid_output, &left_rpm_goal, 0.86, 0, 0, 0);

    // for right motor
    double right_rpm_encoder = 0;
    double right_rpm_goal = 0;
    int right_output = 0;
    double right_pid_output = 0;
    PID right_pid = PID(&right_rpm_encoder, &right_pid_output, &right_rpm_goal, 0.86, 0, 0, 0);

public:
    int init()
    {
        // Init motor related pin
        pinMode(left_dir, OUTPUT);      // direction for left
        pinMode(left_pwm, OUTPUT);      // speed for left
        pinMode(left_encoder_1, INPUT); // left encoder reader
        pinMode(left_encoder_2, INPUT); // left encoder reader
        pinMode(left_power, OUTPUT);    // power for left
        digitalWrite(left_power, HIGH);
        attachInterrupt(digitalPinToInterrupt(left_encoder_1), updateLeftEncoder, CHANGE);
        attachInterrupt(digitalPinToInterrupt(left_encoder_2), updateLeftEncoder, CHANGE);
        left_pid.SetMode(AUTOMATIC);

        pinMode(right_dir, OUTPUT); // direction for right
        pinMode(right_pwm, OUTPUT); // speed for right
        pinMode(right_encoder_1, INPUT);
        pinMode(right_encoder_2, INPUT);
        pciSetup(right_encoder_1);
        pciSetup(right_encoder_2);
        right_pid.SetMode(AUTOMATIC);
    }
    int stop(void)
    {
        MOTOR_START = false;
        left_pid.SetMode(0);
        right_pid.SetMode(0);
        digitalWrite(left_dir, LOW);
        digitalWrite(right_dir, LOW);
        analogWrite(left_pwm, 0);
        analogWrite(right_pwm, 0);
    }
    /**
     * @brief
     *
     * @param dir direction,1 for forward, 0 for backward
     * @param speed mvoe speed, range in 0-255
     * @return int 0 for success, 1 for error
     */
    int move(int dir, float speed)
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
        speedControl(speed);
        return 0;
    }
    
    /**
     * @brief
     *
     * @param dir direction,1 for right, 0 for left
     * @param speed mvoe speed, range in 0-255
     * @return int 0 for success, 1 for error
     */
    int rotate(int dir, float speed)
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
        speedControl(speed);
        return 0;

    }
    /**
     * @brief using PID to control speed in certain speed m/s
     * 1. Transform from speed to RPM
     * 2. Compute PID output PWM
     * 3. Assign PWM to pin
     *
     * @param speed
     * @return int
     */
    int speedControl(float speed)
    {
        rpm_goal = speedToRPM(speed);
        left_rpm_goal = rpm_goal;
        right_rpm_goal = rpm_goal;
        if (!MOTOR_START)
        {
            MOTOR_START = true;
            left_output = RPMToPWM(left_rpm_goal);
            right_output = RPMToPWM(right_rpm_goal);
            analogWrite(left_pwm, left_output);
            analogWrite(right_pwm, right_output);

            return 0;
        }

        left_pid.Compute();
        right_pid.Compute();
        left_output += left_pid_output;
        right_output += right_pid_output;
        analogWrite(left_pwm, left_output);
        analogWrite(right_pwm, right_output);
    }
};

void updateRightEncoder(void)
{
    right_encoder_value++;
}

void updateLeftEncoder(void)
{
    left_encoder_value++;
}

float speedToRPM(float speed)
{
    // return 60*speed/(2*PI*0.06);
    return 159.15 * speed;
}

float RPMToSpeed(float RPM)
{
    return RPM / 159.15;
}
int RPMToPWM(float RPM)
{
    return (RPM + 8.1) / 0.88;
}
void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin)); // enable pin
    PCIFR |= bit(digitalPinToPCICRbit(pin));                   // clear any outstanding interrupt
    PCICR |= bit(digitalPinToPCICRbit(pin));                   // enable interrupt for the group
}
ISR(PCINT1_vect) // handle pin change interrupt for A0 to A5 here
{
    right_encoder_value++;
}