#include "Test1.h"
#include "CalibrationSave.h"


Sensor sensor1;
MotorController motors1;

EEPROMSave eSave1(motors1, sensor1);

void Test1::Start(){
    
  	sensor1.Begin();
    motors1.Begin();

    // eSave1.Init(&motors1, &sensor1);

    // sensor1.SetAccurate();

    sensor1.Look(Direction::FORWARD);
    delay(500);
    sensor1.Look(Direction::RIGHT);
    delay(500);
    sensor1.Look(Direction::FORWARD);
    delay(2000);

    // motors1.Calibrate(sensor1);
    //eSave1.calibrateAndSave();
    eSave1.loadCalibration();

    Serial.begin(115200);
    Serial.println(motors1.GetTimePerRotation());

    sensor1.Look(Direction::LEFT);
    delay(1000);
    sensor1.Look(Direction::FORWARD);
    delay(2000);

    motors1.MoveFor(Direction::FORWARD, 3000);
    delay(3000);
    motors1.MoveFor(Direction::BACKWARD, 3000);
    delay(3000);

    sensor1.Look(Direction::RIGHT);
    delay(500);
    sensor1.Look(Direction::FORWARD);
    delay(500);
}

void Test1::Loop(){
    for(int i = 0; i<4; i++){
        motors1.MoveDistance(Direction::FORWARD, 250);
        delay(800);
        motors1.Turn(Direction::LEFT);
    }
    delay(1000);
    for(int i = 0; i<4; i++){
        motors1.MoveDistance(Direction::FORWARD, 250);
        delay(800);
        motors1.Turn(Direction::RIGHT);
    }
}