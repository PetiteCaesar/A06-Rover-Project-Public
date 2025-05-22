#pragma once

#include <Arduino.h>
#include "Sensor.h"
#include "MotorController.h"
#include <EEPROM.h>


struct CalibrationData{
	float timePerRotation;
};

class EEPROMSave{
	public:
	MotorController& mc;
	Sensor& sc;
	EEPROMSave(MotorController& m, Sensor& s):mc(m), sc(s) {}

	// void Init(MotorController& m, Sensor& s) {mc = m; sc = s;}

	
	void eepromClear(){
		//Serial.println("Clearing EEPROM...");
		for (uint16_t i = 0 ; i < EEPROM.length() ; i++) {
			EEPROM.write(i, 0);
		}
	}

	void calibrateAndSave(){
		//Serial.println("Calibrating motors...");
		mc.Calibrate(sc);
		CalibrationData data {mc.GetTimePerRotation()};
		EEPROM.put(0, data);
		//Serial.println("Calibration data saved to EEPROM");
	}

	void loadCalibration(){
		//Serial.println("Loading calibration data from EEPROM...");
		CalibrationData data;
		EEPROM.get(0, data);
		mc.SetTimePerRotation(data.timePerRotation);
	}

};
