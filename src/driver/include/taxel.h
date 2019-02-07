#ifndef TAXEL_H
#define TAXEL_H

#include <thread>
#include <string>
#include <vector>
#include <sstream>

class taxel
{
public:

    taxel();
    ~taxel();

    void clearData();
    void saveXData(double data);
    void saveYData(double data);
    void saveZData(double data);

    inline double getXData(){ return X; }
    inline double getYData(){ return Y; }
    inline double getZData(){ return Z; }

    int getData(std::vector<double> *XYZ);

    void setID(unsigned int id);
    unsigned int getID();

    int calibrate();
    int update();

    inline double getForce(){return M;}

    void printBaselineValues();
    void printValues();
    void printToSSTRCalibration(std::stringstream *sstr);
    void printToSSTRXYZValues(std::stringstream *sstr);

    inline unsigned long sizeData(){return X_Data.size();}

protected:

    unsigned int sensor_id;

    // raw data
    std::vector<double> X_Data;
    std::vector<double> Y_Data;
    std::vector<double> Z_Data;

    // Baseline calibration data
    double X_Base;
    double Y_Base;
    double Z_Base;

    // Sensor data
    double X;
    double Y;
    double Z;

    double M;

    //ToDo: current MTB version does not give any information about sensor status
    bool stop = false;

};

#endif // TAXEL_H
