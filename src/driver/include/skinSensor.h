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
    skinSensor(std::string channel, std::vector<unsigned int>& sensor_ids, std::size_t average_window_size);
    ~skinSensor();

    int calibrate();

    void updateSensors();

    std::vector<int> getData();

protected:
    const unsigned calibrationTimeSeconds = 2;

    const unsigned MTB_ID = 0x201;

    std::vector<unsigned int> SensorIds;
    const unsigned numberOfSensors;

    // Number of readings to obtain an average measure from the sensor
    const std::size_t averageWindowSize;

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
