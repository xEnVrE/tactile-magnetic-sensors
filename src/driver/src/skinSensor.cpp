#include <iostream>
#include <chrono>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <fstream>
#include <sstream>

#include "skinSensor.h"

skinSensor::skinSensor(std::string channel){
    std::cout << "SS: Opening CAN port" << std::endl;

    device.openCAN(channel);

    sensors = new std::vector<taxel>;
    sensors->resize(numberOfSensors);

    for (unsigned i = 0; i < numberOfSensors; i++){
        sensors->at(i).setID(SensorIds[i]);
        std::cout << "Creating Sensor with ID " << SensorIds[i] << std::endl;
    }

    triggerOnMTB();
}




skinSensor::~skinSensor(){
    triggerOffMTB();
    delete sensors;
}

void skinSensor::triggerOnMTB(){
    unsigned short len_;
    unsigned char data_[8];

    std::cout << "Activating MTB Id " << std::hex << MTB_ID << std::endl;

    data_[0] = 0x07;
    data_[1] = 0x00;
    len_ = 2;

    if(device.writeCAN(MTB_ID, len_, data_) == 0)
    {
        std::cout << "MTB Actived!!!!!!" << std::endl;
    }
    else
    {
        std::cout << "Could not Activate-----------" << std::endl;
    }

    //Wait until MTB inits
    std::this_thread::sleep_for(std::chrono::microseconds(2000));
}

void skinSensor::triggerOffMTB(){
    unsigned short len_;
    unsigned char data_[8];

    data_[0] = 0x07;
    data_[1] = 0x01;
    len_ = 2;
    device.writeCAN(MTB_ID, len_, data_);

    std::this_thread::sleep_for(std::chrono::microseconds(1000));
}

int skinSensor::calibrate(){
    unsigned id_;
    unsigned short len_;
    unsigned char data_[8];

    std::stringstream sstr;

    //std::cout << "FTS: Calibrating fingertip sensors" << std::endl;

    // std::cout << "Triggering on MTB ID 0x" << std::hex << ID_Base + ID_16 << std::endl;
    // Trigger on 16 sensor MTB

	triggerOnMTB();

	//Fill Data during 5 seg
	// std::cout << "Filling data ... " << std::endl;
	std::chrono::system_clock::time_point startTime = std::chrono::system_clock::now();
	std::chrono::system_clock::time_point endTime = startTime + std::chrono::seconds(calibrationTimeSeconds);

	while (endTime > std::chrono::system_clock::now())
	{
		if (device.readCAN(&id_, &len_, data_) == 0)
		{
			//# Logics from Python original code: Put all MSB & LSB in a buffer
			sensors->at(id_).saveXData(data_[1] << 8 | data_[2]);
			sensors->at(id_).saveYData(data_[3] << 8 | data_[4]);
			sensors->at(id_).saveZData(data_[5] << 8 | data_[6]);
		}
	}
	// std::cout << std::endl;

	// std::cout << "Triggering off MTB ID 0x" << std::hex << ID_Base + ID_16 << std::endl;
	triggerOffMTB();


    for(size_t i = 0; i < numberOfSensors; i++){
        sensors->at(i).calibrate();
        sensors->at(i).printToSSTRCalibration(&sstr);
    }

    //std::cout << "FTS: Calibration finished" << std::endl;

    // std::ofstream fileC;
    // fileC.open("calibration.txt");
    // fileC << sstr.str();
    // fileC.close();

    return 0;
}

bool skinSensor::allSensorsRead(){
    bool state = true;

    // Check for all sensors
    for (size_t i=0; i<numberOfSensors; i++){
        // If sensors[i] has data to process
        if(sensors->at(i).sizeData() < numReadings){
            state = false;
        }
    }

    return state;
}

// Update the X Y Z values of the fingertip sensors
void skinSensor::updateSensors(){
    unsigned id_;
    unsigned short len_;
    unsigned char data_[8];
    taxel CurrentSensor;

    // Clear the buffer data
    for(size_t i=0;i<numberOfSensors;i++){
        sensors->at(i).clearData();
    }

	// Wait for at least numReadings measures of each sensor on MTB1
	while (!allSensorsRead())
	{
        //std::cout << "Reading CAN... " << std::endl;
		if (device.readCAN(&id_, &len_, data_) == 0)
		{
            for(size_t i=0;i<numberOfSensors;i++){
                if(sensors->at(i).getID() == id_)
                {
			        sensors->at(i).saveXData(data_[1] << 8 | data_[2]);
			        sensors->at(i).saveYData(data_[3] << 8 | data_[4]);
			        sensors->at(i).saveZData(data_[5] << 8 | data_[6]);
                    break;
                }
            }
		}
	}

    for(size_t i = 0; i < numberOfSensors; i++){
        sensors->at(i).update();
        sensors->at(i).printToSSTRXYZValues(&sstrData);
    }

    sstrData << std::endl;
}

// int skinSensor::getData(std::vector<double> *data){
    // std::vector<double> taxelData;
    // taxelData.resize(3);

    // for(size_t i = 0; i < numSensors; i++){
        // sensors->at(i).getData(&taxelData);
        // data->insert(data->end(),taxelData.begin(),taxelData.end());
    // }

    // return 0;
// }

void skinSensor::printBaselineValues(){
    for(size_t i = 0;i < numberOfSensors; i++){
        std::cout << "Sensor id = " << i << ", Values: ";
        sensors->at(i).printBaselineValues();
        std::cout << std::endl;
    }
}

void skinSensor::printValues(){
	for(size_t id = 0;id < numberOfSensors; id++){
		sensors->at(id).printValues();
		std::cout << std::endl;
	}
}

void skinSensor::saveData(){
    std::ofstream file;
    file.open("skinsensorData.txt");
    file << sstrData.str();
    file.close();
}
