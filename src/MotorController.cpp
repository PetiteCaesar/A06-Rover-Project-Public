#include <Arduino.h>    
#include "MotorController.h"

//Motor Controller Class Definition - Written By: Lachlan


MotorController::MotorController(){
    
}

void MotorController::Begin(){
    //Left motor
    pinMode(MOTOR_LEFT_ENGAGE, OUTPUT);
    pinMode(MOTOR_LEFT_ONE_A, OUTPUT);
    pinMode(MOTOR_LEFT_TWO_A, OUTPUT);
    digitalWrite(MOTOR_LEFT_ENGAGE, HIGH);

    //Right motor
    pinMode(MOTOR_RIGHT_ENGAGE, OUTPUT);
    pinMode(MOTOR_RIGHT_ONE_B, OUTPUT);
    pinMode(MOTOR_RIGHT_TWO_B, OUTPUT);
    digitalWrite(MOTOR_RIGHT_ENGAGE, HIGH);
}



bool MotorController::MoveFor(Direction direction, unsigned long timeMs){
    if(m_interrupt != nullptr && m_useInterrupt && m_interrupt()){
        StopMotors();
        return false;
    }
    BeginMovement(direction);
    unsigned long startTime = millis();
    while(millis() - startTime < timeMs){
        if(m_interrupt != nullptr && m_useInterrupt && m_interrupt()){
            StopMotors();
            return false;
            break;
        } else{
            BeginMovement(direction);
        }
    }
    StopMotors();
    return true;
}

void MotorController::MoveUntil(Direction direction, bool (*conditionFunction)(), unsigned long timeoutMs){
    if(conditionFunction()){
        return;
    }
    BeginMovement(direction);
    unsigned long startTime = millis();
    while(true){
        if(conditionFunction()){
            break;
        } else if(millis() - startTime > timeoutMs){
            break;
        }
    }
    StopMotors();
}

/*
    XXXXX
         
    X   X
    X   X
    X 0 X
*/

void MotorController::BeginMovement(Direction direction){
    switch(direction){
        case Direction::FORWARD:
            digitalWrite(MOTOR_LEFT_ONE_A, HIGH);
            digitalWrite(MOTOR_LEFT_TWO_A, LOW);
            digitalWrite(MOTOR_RIGHT_ONE_B, HIGH);
            digitalWrite(MOTOR_RIGHT_TWO_B, LOW);
            break;
        case Direction::BACKWARD:
            digitalWrite(MOTOR_LEFT_ONE_A, LOW);
            digitalWrite(MOTOR_LEFT_TWO_A, HIGH);
            digitalWrite(MOTOR_RIGHT_ONE_B, LOW);
            digitalWrite(MOTOR_RIGHT_TWO_B, HIGH);
            break;
        case Direction::LEFT:
            digitalWrite(MOTOR_LEFT_ONE_A, LOW);
            digitalWrite(MOTOR_LEFT_TWO_A, HIGH);
            digitalWrite(MOTOR_RIGHT_ONE_B, HIGH);
            digitalWrite(MOTOR_RIGHT_TWO_B, LOW);
            break;
        case Direction::RIGHT:
            digitalWrite(MOTOR_LEFT_ONE_A, HIGH);
            digitalWrite(MOTOR_LEFT_TWO_A, LOW);
            digitalWrite(MOTOR_RIGHT_ONE_B, LOW);
            digitalWrite(MOTOR_RIGHT_TWO_B, HIGH);
            break;
    }
}

void MotorController::StopMotors(){
    digitalWrite(MOTOR_LEFT_ONE_A, LOW);
    digitalWrite(MOTOR_LEFT_TWO_A, LOW);
    digitalWrite(MOTOR_RIGHT_ONE_B, LOW);
    digitalWrite(MOTOR_RIGHT_TWO_B, LOW);
}


void MotorController::Calibrate(Sensor& sensor){
    StopMotors();
    bool useInterruptOld = m_useInterrupt;
    UseInterrupt(false);
    float startDistanceMM = 0.0f;
    for(int i = 0; i < CALIBRATION_SENSOR_MEASUREMENTS;){
        if(sensor.HasNewData()){
            auto d = sensor.GetDistance(Units::MM);
            Serial.println(d);
            startDistanceMM += d;
            i++;
        } else {
            delay(10);
        }
    }
    startDistanceMM /= (float)CALIBRATION_SENSOR_MEASUREMENTS;
    Serial.println(startDistanceMM);

    
    MoveFor(Direction::FORWARD, (uint32_t)CALIBRATION_MOVE_TIME_MS);  
    
    float endDistanceMM = 0.0f;
    for(int i = 0; i < CALIBRATION_SENSOR_MEASUREMENTS;){
        if(sensor.HasNewData()){
            auto d = sensor.GetDistance(Units::MM);
            endDistanceMM += d;
            Serial.println(d);
            i++;
        } else{
            delay(10);

        }
    }
    endDistanceMM /= (float)CALIBRATION_SENSOR_MEASUREMENTS;
    Serial.println(endDistanceMM);

    float distanceDeltaMM = startDistanceMM - endDistanceMM;

    float totalRotations = distanceDeltaMM / MOTOR_WHEEL_CIRCUMFERENCE;
    m_timePerRotation = CALIBRATION_MOVE_TIME_MS / totalRotations;
    Serial.println(m_timePerRotation);
    UseInterrupt(useInterruptOld);
    
}

bool MotorController::Turn(Direction direction, unsigned short degrees){
    if(direction == Direction::FORWARD || direction == Direction::BACKWARD){
        static_assert(true, "Turn function called with FORWARD or BACKWARD. Use MoveFor() instead.");
        return false;
    }
    // +TREAD_WIDTH
    float distanceToTravelPerWheel = (WHEEL_TO_WHEEL_WIDTH*PI)/(360.0f/(float)degrees);


    return MoveDistance(direction, distanceToTravelPerWheel, Units::MM);


    // float totalRotations = distanceToTravelPerWheel / MOTOR_WHEEL_CIRCUMFERENCE;
    // Serial.println(totalRotations);
    // unsigned long timeToTurn = m_timePerRotation * totalRotations;
    // Serial.println(timeToTurn);
    // return MoveFor(direction, timeToTurn);
}

bool MotorController::MoveDistance(Direction direction, float distance, Units units){
    //TODO Convert all uinits to mm
    float totalRotations = distance / MOTOR_WHEEL_CIRCUMFERENCE;
    unsigned long timeToMove = m_timePerRotation * totalRotations;
    return MoveFor(direction, timeToMove);
}