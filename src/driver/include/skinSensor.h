#ifndef FINGERTIPSENSOR_H
#define FINGERTIPSENSOR_H

#include <string>
#include <vector>
#include <thread>
#include <mutex>

//#include "canfd.h"
#include "socketcan.h"
#include "taxel.h"

class skinSensor
{

public:
    skinSensor(std::string channel);
    ~skinSensor();

    int calibrate();
    void updateSensors();

    void printBaselineValues();
    void printValues();

    void saveData();
    // int getData(std::vector<double> *data);
    //inline unsigned getNumSensors(){return numSensors;}

protected:
    const unsigned calibrationTimeSeconds = 2;

    const unsigned MTB_ID = 0x201;

    //Ids of the incoming sensor messages
    // const unsigned SensorIds[8] = [0x750, 0x741, 0x761, 0x744, 0x754, 0x764, 0x774, 0x775]
    // const unsigned numberOfSensors = 8;
    //TESTING 1 SENSOR
    const unsigned SensorIds[1] = {1876}; //1872 = 0x750, 1876 = 0x754
    const unsigned numberOfSensors = 1;

    // Number of readings to obtain an average measure from the sensor
    const unsigned numReadings = 2;

    // CAN driver used for communications
    //canfd device;
    socketCAN device;

    std::stringstream sstrData;

    // Vector with information of each sensor
    std::vector<taxel> *sensors;

    bool stop = true;

    void triggerOnMTB();
    void triggerOffMTB();

    bool allSensorsRead();
};

#endif // FINGERTIPSENSOR_H
