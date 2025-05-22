#include "MazeSolve2.h"
#include "CalibrationSave.h"

Sensor sensor2;
MotorController motors2;

EEPROMSave eSave3(motors2, sensor2);


#define WALL_DISTANCE 30
#define MAZE_GRID_SIZE 185.0f

#define WHEN_CENTER 85.0f
#define sensor2_TO_ROVER_CENTER 74.0f

float getCorrectionAngle2(float d){
	float diff = WHEN_CENTER - d;
	float angleRads = atan(diff/sensor2_TO_ROVER_CENTER);
	return degrees(angleRads)/4.0f;//4.0f 
}

void MazeSolve2::Start(){
    // Serial.begin(115200);
	// Serial.println("Starting up...");

  	sensor2.Begin();
    motors2.Begin();
  
    
    sensor2.SetAccurate();


    sensor2.Look(Direction::FORWARD);
    delay(500);
    sensor2.Look(Direction::RIGHT);
    delay(500);
    sensor2.Look(Direction::FORWARD);
    delay(2000);

    // motors2.Calibrate(sensor2);
    // eepromClear();
    sensor2.SetAccurate();
    // sensor2.SetFast();
    //eSave3.calibrateAndSave();
    // sensor2.SetFast();
    sensor2.SetLongRange();
    eSave3.loadCalibration();

    sensor2.Look(Direction::LEFT);
    delay(1000);
    sensor2.Look(Direction::FORWARD);
    delay(2000);
  
}


void MazeSolve2::Loop(){
   
    sensor2.Look(Direction::FORWARD);
    auto forwardDist = sensor2.GetDistance();

    float distToTravel = forwardDist - WALL_DISTANCE;
    motors2.MoveDistance(Direction::FORWARD,distToTravel-40);

    motors2.StopMotors();
    delay(1000);
    motors2.Turn(Direction::RIGHT);
    motors2.Turn(Direction::RIGHT);

}