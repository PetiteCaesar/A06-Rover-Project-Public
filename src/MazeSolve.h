#pragma once

#include <Arduino.h>
#include "Sensor.h"
#include "MotorController.h"
// #include <EEPROM.h>


class MazeSolve {
    public:
    void Start();

    void Loop();
};