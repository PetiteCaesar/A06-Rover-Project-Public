#include "MazeSolve.h"
#include "CalibrationSave.h"

Sensor sensor;
MotorController motors;

EEPROMSave eSave(motors, sensor);


// #define WALL_DISTANCE 30
#define WALL_DISTANCE 32
#define MAZE_GRID_SIZE 185.0f
void OnDetectWall(){
	motors.StopMotors();
	motors.UseInterrupt(false);
	sensor.Look(Direction::LEFT);
	delay(READ_INTERVAL*2);
	float wallLeft = sensor.GetDistance(Units::MM);
	//Serial.println("Left: " + String(wallLeft));
	if(wallLeft < WALL_DISTANCE*2.5 || (wallLeft > WALL_DISTANCE && wallLeft < MAZE_GRID_SIZE)){//no wall
		//Serial.println("TURNING LEFT");
		sensor.Look(Direction::RIGHT);
		delay(READ_INTERVAL*15);
		float wallRight = sensor.GetDistance(Units::MM);
		//Serial.println("RIGHT: " + String(wallLeft));
		if(wallRight < WALL_DISTANCE*2.5 || (wallRight > WALL_DISTANCE && wallRight < MAZE_GRID_SIZE)){
			sensor.Look(Direction::FORWARD);
			motors.Turn(Direction::RIGHT);
			motors.Turn(Direction::RIGHT);

		} else{
			motors.Turn(Direction::RIGHT);
		}
		
	} else{
		motors.Turn(Direction::LEFT);




		sensor.Look(Direction::RIGHT);
		delay(READ_INTERVAL*15);
		float wallRight = sensor.GetDistance(Units::MM);
		//Serial.println("RIGHT: " + String(wallLeft));
		if(wallRight < WALL_DISTANCE*2.5 || (wallRight > WALL_DISTANCE && wallRight < MAZE_GRID_SIZE)){//no wall
			//Serial.println("TURNING LEFT");
			motors.Turn(Direction::RIGHT);
		} else{
			//Serial.println("TURNING AROUND");
			//turn around
			motors.Turn(Direction::LEFT);
			motors.Turn(Direction::LEFT);
		}
	}
	sensor.Look(Direction::FORWARD);
	motors.UseInterrupt(true);
}


//Works!!
void testRunKnownMaze(){
	//Mainly for testing turning and moving set distance accuracy
	constexpr int CENTER_CENTER = 189;
	constexpr int EDGE_CENTER = 235;

	motors.MoveDistance(Direction::FORWARD, EDGE_CENTER);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.Turn(Direction::LEFT);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.Turn(Direction::RIGHT);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.Turn(Direction::LEFT);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.Turn(Direction::LEFT);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.Turn(Direction::RIGHT);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.Turn(Direction::RIGHT);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);
	motors.MoveDistance(Direction::FORWARD, CENTER_CENTER);

	motors.StopMotors();
	delay(4000);
}



bool twoNumbersWithin(float n1, float n2, float percent){
	float _percent = percent / 100.0f;
	float division = n1/n2;
	float within = abs(1 - division);
	return within < _percent;
}


float p = 0;
float dir = 0.01;
#define WHEN_CENTER 85.0f
#define SENSOR_TO_ROVER_CENTER 74.0f

float getCorrectionAngle(float d, float whenCenter){
	float diff = whenCenter - d;
	float angleRads = atan(diff/SENSOR_TO_ROVER_CENTER);

	return degrees(angleRads)/4.0f; 
}

unsigned long lastTime = 0;

void MazeSolve::Start(){
    // Serial.begin(115200);
	// Serial.println("Starting up...");

  	sensor.Begin();
    motors.Begin();
  
    // // motors.Calibrate(sensor);
    // // eepromClear();
    sensor.SetAccurate();
    // // sensor.SetFast();
    //eSave.calibrateAndSave();
    // // sensor.SetFast();
	sensor.Continuous();
    eSave.loadCalibration();

    sensor.Look(Direction::LEFT);
    delay(1000);
    sensor.Look(Direction::FORWARD);
    delay(2000);


	lastTime = millis();

	motors.UseInterrupt(false);

}



void whenAtWallLidar(float d){
	if(d <= WALL_DISTANCE){
		// Serial.println("At WALL");
		motors.StopMotors();
		sensor.Look(Direction::LEFT);
		delay(500);
		// float wallLeft = sensor.GetDistance(Units::MM);
		float wallLeft = sensor.lidar.readRangeContinuousMillimeters();
		for(int i = 0; i < 5; i++){
			wallLeft += sensor.lidar.readRangeContinuousMillimeters();
			delay(33);
		}
		wallLeft /= 6.0f;

		// motors.UseInterrupt(false);
		if(wallLeft < WALL_DISTANCE*2.5 || (wallLeft > WALL_DISTANCE && wallLeft < MAZE_GRID_SIZE)){
			// Serial.println("Looking right!");
			sensor.Look(Direction::RIGHT);
			delay(800);
			// float wallRight = sensor.GetDistance(Units::MM);
			float wallRight = sensor.lidar.readRangeContinuousMillimeters();

			for(int i = 0; i < 5; i++){
				wallRight += sensor.lidar.readRangeContinuousMillimeters();
				delay(33);
			}
			wallRight /= 6.0f;
			
			Serial.println(wallRight);
			if(wallRight < WALL_DISTANCE*2.5 || (wallRight > WALL_DISTANCE && wallRight < MAZE_GRID_SIZE)){
				sensor.Look(Direction::FORWARD);
				motors.Turn(Direction::RIGHT);
				motors.Turn(Direction::RIGHT);
			} else{
				// Serial.println("Turning right");
				motors.Turn(Direction::RIGHT);
			}
		} else{
			motors.Turn(Direction::LEFT);
		}

		sensor.Look(Direction::FORWARD);
		// motors.UseInterrupt(true);
	}
}


void deviation(){
	motors.StopMotors();
	sensor.Look(Direction::LEFT);
	delay(600);
	float distLeft = sensor.lidar.readRangeContinuousMillimeters();
	delay(100);

	sensor.Look(Direction::RIGHT);
	delay(600);
	float distRight = sensor.lidar.readRangeContinuousMillimeters();
	delay(100);

	float smaller = min(distRight, distLeft);
	bool turnLeft = distRight < distLeft;

	if(smaller < WHEN_CENTER){
		auto angle = getCorrectionAngle(smaller, WHEN_CENTER);
		if(turnLeft){
			motors.Turn(Direction::LEFT, angle);
		} else{
			motors.Turn(Direction::RIGHT, angle);
		}
	}
	sensor.Look(Direction::FORWARD);

	delay(500);
	lastTime = millis();
}

bool first = true;
void MazeSolve::Loop(){
	//testRunKnownMaze();
	// motors.MoveDistance(Direction::FORWARD, 100000);




	//FOR LIDAR CODE
	auto d = sensor.lidar.readRangeContinuousMillimeters();
	while (d > WALL_DISTANCE){

		motors.BeginMovement(Direction::FORWARD);
		for(int i = 0; i < 5; i++){
			d = sensor.lidar.readRangeContinuousMillimeters();
			if(d <= WALL_DISTANCE){
				motors.StopMotors();
				break;
			}
		}


		//every 2500 ms
		if((millis() - lastTime) > 2000 && d > WALL_DISTANCE*4){
			deviation();
			delay(1000);
			// sensor.lidar.startContinuous();
			// d = sensor.lidar.readRangeContinuousMillimeters();
			lastTime = millis();
		}

	}
	motors.StopMotors();
	delay(200);
	whenAtWallLidar(0);
	// deviation();




	// motors.MoveDistance(Direction::FORWARD, 185);
	//motors.MoveDistance(Direction::FORWARD, MAZE_GRID_SIZE/2.0f);

	// motors.StopMotors();
	// sensor.SetAccurate();
	// sensor.Look(Direction::LEFT);
	// delay(500);
	// float dl = sensor.GetDistance();//mm

	// delay(200);

	// sensor.Look(Direction::RIGHT);
	// delay(500);
	// float dr = sensor.GetDistance();


	// Serial.print("Distance left: ");
	// Serial.println(dl);
	// Serial.print("Distance Right: ");
	// Serial.print(dr);
	// const float whenCenter = min(WHEN_CENTER*3, dl+dr)/2.0f;
	// // const float whenCenter = WHEN_CENTER;
	// if(dl > whenCenter){

	// 	if(dr < whenCenter){//check there is no wall
	// 		//deviating right

	// 		float angle = getCorrectionAngle(dr, whenCenter);
	// 		Serial.print("Turning left angle: ");
	// 		Serial.println(angle);
	// 		// motors.MoveDistance(Direction::BACKWARD, sqrt(sq(WHEN_CENTER - dr) + sq(SENSOR_TO_ROVER_CENTER)) - SENSOR_TO_ROVER_CENTER);
	// 		motors.Turn(Direction::LEFT, angle);
			
	// 	}
	// } else{
	// 	//deviating left
	// 	float angle = getCorrectionAngle(dl, whenCenter);
	// 	Serial.print("Turning right angle: ");
	// 	Serial.println(angle);
	// 	// motors.MoveDistance(Direction::BACKWARD, sqrt(sq(WHEN_CENTER - dl) + sq(SENSOR_TO_ROVER_CENTER)) - SENSOR_TO_ROVER_CENTER);
	// 	motors.Turn(Direction::RIGHT, angle);
	// }
	// sensor.Continuous();
	// sensor.Look(Direction::FORWARD);
	// delay(200);

	// sensor.SetAccurate();
	// sensor.Look(Direction::FORWARD);
	// delay(500);
	// float dist = sensor.GetDistance();
	// sensor.Continuous();
	// delay(500);
	// sensor.Look(Direction::LEFT);
	// delay(500);

	// if(motors.MoveDistance(Direction::FORWARD, max(0,dist-WALL_DISTANCE))){
	// 	//reached end and couldnt turn
	// 	// motors.Turn(Direction::LEFT);
	// 	// motors.Turn(Direction::LEFT);
	// 	Serial.println("Couldnt turn left");
	// 	whenAtWall(0);
	// }


	// if(sensor.HasNewData())
	// 	//Serial.println(sensor.GetDistance());
}