#include <iostream>
#include <chrono>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <fstream>
#include <sstream>

#include "skinSensor.h"

skinSensor::skinSensor
(
    std::string channel,
    std::vector<unsigned int>& sensor_ids,
    std::size_t average_window_size
) :
    SensorIds(sensor_ids),
    numberOfSensors(sensor_ids.size()),
    averageWindowSize(average_window_size)
{
    std::cout << "SS: Opening CAN port" << std::endl;

    device.openCAN(channel);

    sensors = new std::vector<taxel>;
    sensors->resize(numberOfSensors);

    for (unsigned i = 0; i < numberOfSensors; i++){
        sensors->at(i).setID(SensorIds[i]);
        std::cout << "Creating Sensor with ID " << SensorIds[i] << std::endl;
    }
}


skinSensor::~skinSensor()
{
    triggerOffMTB();
    delete sensors;
}


bool skinSensor::triggerOnMTB()
{
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
        return false;
    }

    //Wait until MTB inits
    std::this_thread::sleep_for(std::chrono::microseconds(2000));

    return true;
}


void skinSensor::triggerOffMTB()
{
    unsigned short len_;
    unsigned char data_[8];

    data_[0] = 0x07;
    data_[1] = 0x01;
    len_ = 2;
    device.writeCAN(MTB_ID, len_, data_);

    // std::this_thread::sleep_for(std::chrono::microseconds(1000));
}


bool skinSensor::init()
{
    return triggerOnMTB();
}

int skinSensor::calibrate()
{
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


bool skinSensor::allSensorsRead()
{
    bool state = true;

    // Check for all sensors
    for (size_t i=0; i<numberOfSensors; i++){
        // If sensors[i] has data to process
        if(sensors->at(i).sizeData() < averageWindowSize){
            state = false;
        }
    }

    return state;
}


// Update the X Y Z values of the fingertip sensors
void skinSensor::updateSensors()
{
    unsigned id_;
    unsigned short len_;
    unsigned char data_[8];
    taxel CurrentSensor;

    // Clear the buffer data
    for(size_t i=0;i<numberOfSensors;i++){
        sensors->at(i).clearData();
    }

    // Wait for at least averageWindowSize measures of each sensor on MTB1
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

    for(size_t i = 0; i < numberOfSensors; i++)
        sensors->at(i).update();

    sstrData << std::endl;
}


std::vector<int> skinSensor::getData()
{
    std::vector<int> all_sensors;

    for (std::size_t i = 0; i < numberOfSensors; i++)
    {
        std::vector<int> triplet(3);
        sensors->at(i).getData(&triplet);

        all_sensors.push_back(triplet[0]);
        all_sensors.push_back(triplet[1]);
        all_sensors.push_back(triplet[2]);
    }

    return all_sensors;
}
