#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include <softPwm.h>

using namespace std;

//MISSION
#define MISSION_DO_3 1

//USLEEP
#define USLEEP_servo 1000

//SERVO
// #define SERVO_frequency 50
#define SERVO_pin 1 // = GPIO18 = pin 12
// #define SERVO_rotationRate 30
// #define SERVO_PWM_clock_divisor 192
#define SERVO_PWM_range 50 //NOTE for SERVO_frequency 50?
#define SERVO_POS_0 10
#define SERVO_POS_90 15
#define SERVO_POS_180 20
int main() {
    if(MISSION_DO_3){
        if(wiringPiSetupGpio() == -1){
            cerr << "Failed to initialize WiringPi" << endl;
            return -3;
        }
    } else return 0;
    wiringPiSetup();
    softPwmCreate(SERVO_pin, 0, SERVO_PWM_range);
    while(1){
        cout << "90" << endl;
        softPwmWrite(SERVO_pin, SERVO_POS_90);
        //usleep(7000000);
        delay(USLEEP_servo);
        // cout << "0" << endl;
        // softPwmWrite(SERVO_pin, SERVO_POS_0);
        // //usleep(7000000);
        // delay(USLEEP_servo);
        
        // delay(USLEEP_servo);
        // // usleep(USLEEP_servo);
        // cout << "0" << endl;
        // softPwmWrite(SERVO_pin, SERVO_POS_0);
        // delay(USLEEP_servo);
        // // usleep(USLEEP_servo);
        // cout << "180" << endl;
        // softPwmWrite(SERVO_pin, SERVO_POS_180);
        // delay(USLEEP_servo);
        // usleep(USLEEP_servo);
    }
}