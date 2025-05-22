#include <Arduino.h>
#include "Sensor.h"

//Sensor Class Definition - Written By: Lachlan



#ifdef USE_VL53L0X
    //from the example the library gives
    void Sensor::SetLongRange(){
        m_readInterval = 200;//honestly not sure how long the long range
        //measurements take, but it doesnt really matter as we wont
        //need to use it
        
        lidar.setSignalRateLimit(0.1);
        // increase laser pulse periods (defaults are 14 and 10 PCLKs)
        lidar.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        lidar.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

        m_continuous = false;
    }

    //from the example the library gives
    void Sensor::SetFast(){
        m_readInterval = 60;
        // Set timing budget to 60ms
        lidar.setMeasurementTimingBudget(60000);
        m_continuous = false;
    }

    //from the example the library gives
    void Sensor::SetAccurate(){
        m_readInterval = 200;
        // increase timing budget to 200 ms
        lidar.setMeasurementTimingBudget(200000);
        m_continuous = false;
    }

    void Sensor::Continuous(){
        m_readInterval = 0;
        lidar.startContinuous();
        m_continuous = true;

    }

#else
    void Sensor::SetLongRange(){}
    void Sensor::SetFast(){}
    void Sensor::SetAccurate(){}
    void Sensor::Continuous(){}
#endif








Sensor::Sensor(){}

void Sensor::Begin(){
    #ifndef USE_VL53L0X
    pinMode(ECHO_PIN, INPUT);
	pinMode(TRIG_PIN, OUTPUT);
    #else
    Wire.begin();
    lidar.setTimeout(1000);
    if (!lidar.init()) {
      Serial.println("Failed to detect and initialize lidar!");
      while (1) {}
    }
    
    SetAccurate();
    
    #endif

    m_servo.attach(SERVO_PIN);
}



float Sensor::GetDistance(Units unit){
    SensorRead();
    switch(unit){
        case Units::MM:
            return m_distanceMM;
        case Units::CM:
            return m_distanceMM/10.0f;
        case Units::M:
            return m_distanceMM/1000.0f;
        default:
            return m_distanceMM;
    }
}

constexpr float temperatureC = 25.0f;
constexpr float speedOfSound = 331.3f + 0.606f * temperatureC; // m/s

void Sensor::SensorRead(){
    
    if(millis() - m_lastReadTime > m_readInterval){
        #ifndef USE_VL53L0X
        digitalWrite(TRIG_PIN, LOW);
        delayMicroseconds(2);
        digitalWrite(TRIG_PIN, HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIG_PIN, LOW);
        unsigned long duration = pulseIn(ECHO_PIN, HIGH);
        //m_distanceMM = (5*duration)/29.0f; // no temp compensation
        m_distanceMM = (duration * speedOfSound * 1000.0f) / (2.0f * 1e6); // mm

        #else
        //TODO - add the VL53L0X sensor code
        if(m_continuous)
            m_distanceMM = lidar.readRangeContinuousMillimeters();
        else
            m_distanceMM = lidar.readRangeSingleMillimeters();
        if (lidar.timeoutOccurred()) { Serial.println("Lidar TIMEOUT"); }
        #endif

        m_lastReadTime = millis();
    }
}

