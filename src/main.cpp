// #include <Arduino.h>

// #include "Sensor.h"
// #include "MotorController.h"
// #include <EEPROM.h>

// Sensor sensor;
// MotorController motors;

// void eepromClear(){
// 	//Serial.println("Clearing EEPROM...");
// 	for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
// 		EEPROM.write(i, 0);
// 	}
// }

// struct CalibrationData{
// 	float timePerRotation;
// };

// void calibrateAndSave(){
// 	//Serial.println("Calibrating motors...");
// 	motors.Calibrate(sensor);
// 	CalibrationData data {motors.GetTimePerRotation()};
// 	EEPROM.put(0, data);
// 	//Serial.println("Calibration data saved to EEPROM");
// }

// void loadCalibration(){
// 	//Serial.println("Loading calibration data from EEPROM...");
// 	CalibrationData data;
// 	EEPROM.get(0, data);
// 	motors.SetTimePerRotation(data.timePerRotation);

// }

// #define WALL_DISTANCE 30
// #define MAZE_GRID_SIZE 185.0f
// void OnDetectWall(){
// 	motors.StopMotors();
// 	motors.UseInterrupt(false);
// 	sensor.Look(Direction::LEFT);
// 	delay(READ_INTERVAL*2);
// 	float wallLeft = sensor.GetDistance(Units::MM);
// 	//Serial.println("Left: " + String(wallLeft));
// 	if(wallLeft < WALL_DISTANCE*2.5 || (wallLeft > WALL_DISTANCE && wallLeft < MAZE_GRID_SIZE)){//no wall
// 		//Serial.println("TURNING LEFT");
// 		sensor.Look(Direction::RIGHT);
// 		delay(READ_INTERVAL*15);
// 		float wallRight = sensor.GetDistance(Units::MM);
// 		//Serial.println("RIGHT: " + String(wallLeft));
// 		if(wallRight < WALL_DISTANCE*2.5 || (wallRight > WALL_DISTANCE && wallRight < MAZE_GRID_SIZE)){
// 			sensor.Look(Direction::FORWARD);
// 			motors.Turn(Direction::RIGHT);
// 			motors.Turn(Direction::RIGHT);

// 		} else{
// 			motors.Turn(Direction::RIGHT);
// 		}
		
// 	} else{
// 		motors.Turn(Direction::LEFT);




// 		sensor.Look(Direction::RIGHT);
// 		delay(READ_INTERVAL*15);
// 		float wallRight = sensor.GetDistance(Units::MM);
// 		//Serial.println("RIGHT: " + String(wallLeft));
// 		if(wallRight < WALL_DISTANCE*2.5 || (wallRight > WALL_DISTANCE && wallRight < MAZE_GRID_SIZE)){//no wall
// 			//Serial.println("TURNING LEFT");
// 			motors.Turn(Direction::RIGHT);
// 		} else{
// 			//Serial.println("TURNING AROUND");
// 			//turn around
// 			motors.Turn(Direction::LEFT);
// 			motors.Turn(Direction::LEFT);
// 		}
// 	}
// 	sensor.Look(Direction::FORWARD);
// 	motors.UseInterrupt(true);
// }


// void setup(){

// 	//Serial.begin(115200);
// 	//Serial.println("Starting up...");

//   	sensor.Begin();
// 	motors.Begin();

// 	sensor.SetAccurate();

// 	//register an interrupt for when the sensor reads a distance of less than 4cm
// 	motors.RegisterInterrupt([](){
// 		auto d = sensor.GetDistance(Units::MM);
// 		//Serial.println(d);
// 		// if(d < WALL_DISTANCE){
// 		// 	OnDetectWall();
// 		// }

// 		if(d < WALL_DISTANCE){

// 			motors.StopMotors();
// 			motors.UseInterrupt(false);
// 			sensor.Look(Direction::LEFT);
// 			delay(READ_INTERVAL*2);
// 			float wallLeft = sensor.GetDistance(Units::MM);

// 			if(wallLeft < WALL_DISTANCE*2.5 || (wallLeft > WALL_DISTANCE && wallLeft < MAZE_GRID_SIZE)){

// 				sensor.Look(Direction::RIGHT);
// 				delay(READ_INTERVAL*2);
// 				float wallRight = sensor.GetDistance(Units::MM);
// 				if(wallRight < WALL_DISTANCE*2.5 || (wallRight > WALL_DISTANCE && wallRight < MAZE_GRID_SIZE)){
// 					sensor.Look(Direction::FORWARD);
// 					motors.Turn(Direction::RIGHT);
// 					motors.Turn(Direction::RIGHT);
// 				} else{
// 					motors.Turn(Direction::RIGHT);
// 				}
// 			} else{
// 				motors.Turn(Direction::LEFT);
// 			}

// 			sensor.Look(Direction::FORWARD);
// 			motors.UseInterrupt(true);
// 		}

// 		return false;
// 	});



// 	sensor.Look(Direction::FORWARD);
// 	delay(500);
// 	sensor.Look(Direction::RIGHT);
// 	delay(500);
// 	sensor.Look(Direction::FORWARD);
// 	delay(2000);

// 	// motors.Calibrate(sensor);
// 	// eepromClear();
// 	sensor.SetAccurate();
// 	// sensor.SetFast();
// 	//calibrateAndSave();
// 	sensor.SetFast();
// 	loadCalibration();

// 	//2574.22
// 	// auto tr = motors.GetTimePerRotation();
// 	// Serial.println(tr);
// 	// motors.SetTimePerRotation(tr - 100);

// 	//Serial.print("Calibration data: ");
// 	//Serial.println(motors.GetTimePerRotation());

// 	//Serial.println("Calibration complete");
// 	sensor.Look(Direction::LEFT);
// 	delay(1000);
// 	sensor.Look(Direction::FORWARD);
// 	delay(2000);



// 	//motors.MoveDistance(Direction::FORWARD, 200);

// 	// delay(2000);
// }




// //Works!!
// void testRunKnownMaze(){
// 	//Mainly for testing turning and moving set distance accuracy
// 	constexpr int CENTER_CENTER = 189;
// 	constexpr int EDGE_CENTER = 235;

// 	motors.MoveDistance(Direction::FORWARD, EDGE_CENTER);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.Turn(Direction::LEFT);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.Turn(Direction::RIGHT);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.Turn(Direction::LEFT);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.Turn(Direction::LEFT);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.Turn(Direction::RIGHT);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.Turn(Direction::RIGHT);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
// 	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);

// 	motors.StopMotors();
// 	delay(4000);
// }



// bool twoNumbersWithin(float n1, float n2, float percent){
// 	float _percent = percent / 100.0f;
// 	float division = n1/n2;
// 	float within = abs(1 - division);
// 	return within < _percent;
// }


// float p = 0;
// float dir = 0.01;
// #define WHEN_CENTER 85.0f
// #define SENSOR_TO_ROVER_CENTER 74.0f

// float getCorrectionAngle(float d){
// 	float diff = WHEN_CENTER - d;
// 	float angleRads = atan(diff/SENSOR_TO_ROVER_CENTER);

// 	return degrees(angleRads)/4.0f;//4.0f 
// }

// void loop(){
// 	// motors.Turn(Direction::LEFT);
// 	// delay(500);
// 	// motors.MoveDistance(Direction::FORWARD, 1000);
// 	// delay(500);
// 	// motors.Turn(Direction::RIGHT);
// 	// motors.Turn(Direction::RIGHT);
// 	//testRunKnownMaze();
// 	//motors.MoveDistance(Direction::FORWARD, 1000);
// 	motors.MoveDistance(Direction::FORWARD, 185);//90 is arbitrary
// 	motors.StopMotors();
// 	sensor.Look(Direction::LEFT);
// 	delay(200);
// 	float dl = sensor.GetDistance();//mm

// 	delay(100);

// 	sensor.Look(Direction::RIGHT);
// 	delay(200);
// 	float dr = sensor.GetDistance();

// 	// delay(100);

// 	// float diffL = WHEN_CENTER - dl;
// 	// float diffR = WHEN_CENTER - dr;

// 	// Serial.print("Diff L: ");
// 	// Serial.println(diffL);
// 	// Serial.print("Diff R: ");
// 	// Serial.println(diffR);

	
// 	// // bool within = twoNumbersWithin(abs(diffL), abs(diffR), 20);

// 	// // if(within){
// 	// 	if(diffL > 0 && diffR < 0){
// 	// 		//regular case, need to turn right

// 	// 		float angle = getCorrectionAngle(abs(diffL));
// 	// 		Serial.print("Turning right at angle: ");
// 	// 		Serial.println(angle);
// 	// 		motors.MoveDistance(Direction::BACKWARD, sqrt(sq(WHEN_CENTER - abs(diffL)) + sq(SENSOR_TO_ROVER_CENTER)));
// 	// 	motors.Turn(Direction::RIGHT, angle);
	
// 	// 	} else if(diffL < 0 && diffR > 0){
// 	// 		//regular case, need to turn left
// 	// 		float angle = getCorrectionAngle(abs(diffR));
// 	// 		Serial.print("Turning Left at angle: ");
// 	// 		Serial.println(angle);
// 	// 		motors.MoveDistance(Direction::BACKWARD, sqrt(sq(WHEN_CENTER - abs(diffR)) + sq(SENSOR_TO_ROVER_CENTER)));
// 	// 	motors.Turn(Direction::LEFT, angle);
// 	// 	}
// 	// }

	

// 	Serial.print("Distance left: ");
// 	Serial.println(dl);
// 	Serial.print("Distance Right: ");
// 	Serial.print(dr);

// 	if(dl > WHEN_CENTER){

// 		if(dr < WHEN_CENTER){//check there is no wall
// 			//deviating right

// 			float angle = getCorrectionAngle(dr);
// 			Serial.print("Turning left angle: ");
// 			Serial.println(angle);
// 			motors.MoveDistance(Direction::BACKWARD, SENSOR_TO_ROVER_CENTER - sqrt(sq(WHEN_CENTER - dr) + sq(SENSOR_TO_ROVER_CENTER)));
// 			motors.Turn(Direction::LEFT, angle);
			
// 		}
// 	} else{
// 		//deviating left
// 		float angle = getCorrectionAngle(dl);
// 		Serial.print("Turning right angle: ");
// 		Serial.println(angle);
// 		motors.MoveDistance(Direction::BACKWARD, SENSOR_TO_ROVER_CENTER-sqrt(sq(WHEN_CENTER - dl) + sq(SENSOR_TO_ROVER_CENTER)));
// 		motors.Turn(Direction::RIGHT, angle);
// 	}
// 	sensor.Look(Direction::FORWARD);
// 	delay(100);


// 	// if(sensor.HasNewData())
// 	// 	//Serial.println(sensor.GetDistance());
// }



// #define RunTest1
// #define AngleWall
#define RunMazeSolve

#ifdef RunTest1

#include "Test1.h"

Test1 test1;
void setup(){
	test1.Start();
}

void loop(){
	test1.Loop();
}

#endif

#ifdef AngleWall

#include "MazeSolve2.h"

MazeSolve2 angleWall;
void setup(){
	angleWall.Start();
}

void loop(){
	angleWall.Loop();
}

#endif

#ifdef RunMazeSolve

#include "MazeSolve.h"

MazeSolve mazeSolve;
void setup(){
	mazeSolve.Start();
}

void loop(){
	mazeSolve.Loop();
}


#endif
