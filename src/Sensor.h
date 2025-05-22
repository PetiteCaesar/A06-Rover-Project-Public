#pragma once
#include "Config.h"
#include <Servo.h>

#ifdef USE_VL53L0X
#include <Wire.h>
#include <VL53L0X.h>
#endif

//Sensor Class Declaration - Written By: Lachlan



class Sensor{
public:

    #ifdef USE_VL53L0X
    VL53L0X lidar;
    #endif

    Sensor();

    //Call this in the setup function
    void Begin();

    //Returns true if there is new data to be read
    inline bool HasNewData() const {
        if(millis() - m_lastReadTime > m_readInterval){
            return true;
        } 
        return false;
    }

    //Default units are in mm
    //If there is no new data to be read, it will simply return the most recent data.
    //Otherwise it will get new sensor data
    float GetDistance(Units unit = Units::MM);

    //Can't look backwards.
    void Look(Direction dir) { 
        switch(dir){
            case Direction::FORWARD:
                SetAngle(FORWARD_ANGLE);
                break;
            case Direction::RIGHT:
                SetAngle(RIGHT_ANGLE);
                break;
            case Direction::LEFT:
                SetAngle(LEFT_ANGLE);
                break;
            default:
            break;
        }
    }

    //angle 0 - 180 degrees
    void SetAngle(unsigned short angle){
        m_angle = angle;
        m_servo.write(angle);
    }
    
    inline unsigned short GetAngle() const { return m_angle; }

    inline void SetReadInterval(unsigned int newInterval) {m_readInterval = newInterval;}

    unsigned int GetReadInterval() const {return m_readInterval;}

    void SetLongRange();
    void SetFast();
    void SetAccurate();
    void Continuous();
private:
    float m_distanceMM = 0.0f;
    unsigned long m_lastReadTime = 0;
    unsigned short m_angle = 90;
    unsigned int m_readInterval = READ_INTERVAL;
    bool m_continuous = false;
    //mm
    void SensorRead();
    Servo m_servo;
};