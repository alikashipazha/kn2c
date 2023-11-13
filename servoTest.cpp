#include <wiringPi.h>
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
#include <unistd.h>
#include </home/kn2c/vision/HG-PWM-master/pwm.hpp>
// #include </home/kn2c/vision/HG-PWM-master/pwm.cpp>

using namespace std;

#define HG_PWM 0
#define SOFT_PWM 0 

//MISSION
#define MISSION_DO_3 1

//USLEEP
#define USLEEP_servo 7000000

//SERVO
#define SERVO_frequency 50
#define SERVO_pin 18
#define SERVO_rotationRate 30
#define SERVO_PWM_clock_divisor 192
#define SERVO_PWM_range 2000 //NOTE for SERVO_frequency 50?

void setServoPosition(int pin, int position){
    int dutyCycle = (position*200/180)+SERVO_frequency; //+50
    pwmWrite(pin, dutyCycle);
}

int main(){
    if(MISSION_DO_3){
        if(wiringPiSetupGpio() == -1){
            cerr << "Failed to initialize WiringPi" << endl;
            return -3;
        }
    } else return 0;
    if(HG_PWM){
        // pulse(100, 5000, 0.5);
        return 0;
    } else if(SOFT_PWM) {
        return 0;
    } else {
        pinMode(SERVO_pin, PWM_OUTPUT);
        pwmSetMode(PWM_MODE_MS);
        pwmSetClock(SERVO_PWM_clock_divisor);
        pwmSetRange(SERVO_PWM_range);
        for(int position = 0; position <= 180; position += SERVO_rotationRate){
            cout << "iter " << position/10 << endl;
            setServoPosition(SERVO_pin, position);
            usleep(USLEEP_servo);
        }
    }
}