#ifndef CONSTANTS_H
    #define CONSTANTS_H

    #define LEFT_MOTOR_IN1 6     //Left motor IN1
    #define LEFT_MOTOR_IN2 5     //Left motor IN2
    #define LEFT_MOTOR_EN 4      //Left motor EN
    #define LEFT_MOTOR_ENC_A 18  //Left motor encoder A
    #define LEFT_MOTOR_ENC_B 19  //Left motor encoder B
    #define LEFT_MOTOR_KP 0.002  //Left motor proportional gain
    #define LEFT_MOTOR_KI 0.012  //Left motor integral gain

    #define RIGHT_MOTOR_IN1 8    //Right motor IN1
    #define RIGHT_MOTOR_IN2 7    //Right motor IN2
    #define RIGHT_MOTOR_EN 9     //Right motor EN
    #define RIGHT_MOTOR_ENC_A 2  //Right motor encoder A
    #define RIGHT_MOTOR_ENC_B 3  //Right motor encoder B
    #define RIGHT_MOTOR_KP 0.002 //Right motor proportional gain
    #define RIGHT_MOTOR_KI 0.012 //Right motor integral gain

    #define VMAX 0.5             //Maximum speed of the car
    #define AMAX 0.2             //Maximum acceleration of the car
    #define CAR_KP 0.4           //Car proportional gain (for delta error on trapezoidal profile)

    #define CONTAINERS_NUM 12    //Number of containers
#endif