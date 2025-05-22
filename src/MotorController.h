#pragma once
#include "Config.h"
#include "Sensor.h"

//Motor Controller Class Declaration - Written By: Lachlan

class MotorController{
public:

    MotorController();
    //Call this in the setup function
    void Begin();

    void RegisterInterrupt(bool (*conditionFunction)()){
        m_interrupt = conditionFunction;
        m_useInterrupt = true;
    }

    //Call this function at the end of the setup function, after the Begin() function
    void Calibrate(Sensor& sensor);

    //Turn on the spot. 
    //Is blocking, meaning it will not return until the rover has completed a turn
    //If forward or backward is given, function will return immediately
    //If degrees is given (0-360), the rover will turn that many degrees in the given direction
    bool Turn(Direction direction, unsigned short degrees = 90);

    //Moves in given direction until the given time has passed, or interrupt is triggered (if enabled)
    //If LEFT or RIGHT is given, the rover will turn on the spot in that direction
    bool MoveFor(Direction direction, unsigned long timeMs);

    //Moves the rover in the given direction until the condition function returns true
    //Blocks until the condition is true
    //The interrupt function is disabled while this function is running
    void MoveUntil(Direction direction, bool (*conditionFunction)(), unsigned long timeoutMs = 10000);

    bool MoveDistance(Direction direction, float distance, Units units = Units::MM);

    void BeginMovement(Direction direction);

    inline void UseInterrupt(bool useInterrupt){
        m_useInterrupt = useInterrupt;
    }

    inline void SetTimePerRotation(float timePerRotation){
        m_timePerRotation = timePerRotation;
    }

    float GetTimePerRotation() const{
        return m_timePerRotation;
    }

    // inline void SetMotorSpeed(unsigned char speed){
    //     m_motorSpeed = speed;
    // }

    //Stops all movement.
    void StopMotors();

private:
    float m_timePerRotation = 0.0f;
    bool (*m_interrupt)() = nullptr;
    bool m_useInterrupt = false;
    // unsigned char m_motorSpeed = 255;
};