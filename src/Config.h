#pragma once

//------------------SENSOR CONFIG------------------

// #define USE_VL53L0X //Uncomment this to use the VL53L0X sensor

#ifdef USE_VL53L0X
#define READ_INTERVAL 200
#else
#define ECHO_PIN 3
#define TRIG_PIN 5
#define READ_INTERVAL 60
#endif

#define SERVO_PIN 6
#define LEFT_ANGLE 180
#define RIGHT_ANGLE 0
#define FORWARD_ANGLE 90

//------------------MOTOR CONTROLLER CONFIG------------------
//If the rover is moving in the wrong directions, try swapping some of these values
#define MOTOR_LEFT_ENGAGE 14 //a0
#define MOTOR_LEFT_ONE_A 4
#define MOTOR_LEFT_TWO_A 7 

#define MOTOR_RIGHT_ENGAGE 2
#define MOTOR_RIGHT_ONE_B 16 // a1
#define MOTOR_RIGHT_TWO_B 15 // a2

//------------------CALIBRATION CONFIG------------------
#define CALIBRATION_SENSOR_MEASUREMENTS 25
#define CALIBRATION_MOVE_TIME_MS 2000.0f //ms

//can ignore
#define DISTANCE_FROM_LEFT_RIGHT_WHEEL_CENTER 113.09f //mm
#define TREAD_WIDTH 12.0f//8.0f //mm

// //tank
// #define MOTOR_WHEEL_DIAMETER 25.0f//33.0f//32.0f//68.0f////<-with rubber band //30.5f //mm
// #define WHEEL_TO_WHEEL_WIDTH 125.0f//100.0f 113.09f//114.0f //mm

//dont ignore
#define MOTOR_WHEEL_DIAMETER 69.6f//33.0f//69.0f//33.0f//32.0f//68.0f////<-with rubber band //30.5f //mm
#define WHEEL_TO_WHEEL_WIDTH 102.4f//120.5f//100.0f//100.0f 113.09f//114.0f //mm


constexpr float MOTOR_WHEEL_CIRCUMFERENCE = MOTOR_WHEEL_DIAMETER * PI;

enum Direction{
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT
};

    
enum Units{
    MM,
    CM,
    M
};




