#include "taxel.h"

#include <iostream>
#include <cmath>
#include <numeric>
#include <algorithm>

taxel::taxel(){
    clearData();

    X_Base = 0;
    Y_Base = 0;
    Z_Base = 0;
}

taxel::~taxel(){

}

void taxel::setID(unsigned int id){
    sensor_id = id;
}

unsigned int taxel::getID(){
    return sensor_id;
}

// Compute the {X Y Z}_Base values, as the average of the readings stored in {X Y Z}_BaselineData.
int taxel::calibrate(){

    double sum_X = std::accumulate(X_Data.begin(), X_Data.end(), 0.0);
    double sum_Y = std::accumulate(Y_Data.begin(), Y_Data.end(), 0.0);
    double sum_Z = std::accumulate(Z_Data.begin(), Z_Data.end(), 0.0);

    // Update the Base values
    X_Base = std::round(sum_X / X_Data.size());
    Y_Base = std::round(sum_Y / Y_Data.size());
    Z_Base = std::round(sum_Z / Z_Data.size());

    return 0;
}

void taxel::printToSSTRCalibration(std::stringstream *sstr){
    *sstr << X_Base << " " << Y_Base << " " << Z_Base << " ";
}

void taxel::printToSSTRXYZValues(std::stringstream *sstr){
    *sstr << X << " " << Y << " " << Z << " ";
}

// Compute the {X Y Z}_Base values, as the average of the readings stored in {X Y Z}_BaselineData.
int taxel::update(){

    double sum_X = std::accumulate(X_Data.begin(), X_Data.end(), 0.0);
    double sum_Y = std::accumulate(Y_Data.begin(), Y_Data.end(), 0.0);
    double sum_Z = std::accumulate(Z_Data.begin(), Z_Data.end(), 0.0);

    // Update the Base values
    X = std::round(sum_X / X_Data.size()) - X_Base;
    Y = std::round(sum_Y / Y_Data.size()) - Y_Base;
    Z = std::round(sum_Z / Z_Data.size()) - Z_Base;

    M = sqrt(X*X + Y*Y + Z*Z);

    return 0;
}

void taxel::clearData(){
    X_Data.clear();
    Y_Data.clear();
    Z_Data.clear();
}

int taxel::getData(std::vector<int> *XYZ){
    XYZ->at(0) = X;
    XYZ->at(1) = Y;
    XYZ->at(2) = Z;


    return 0;
}

void taxel::printBaselineValues(){
    std::cout << "X_Base = " << X_Base << " ";
    std::cout << "Y_Base = " << Y_Base << " ";
    std::cout << "Z_Base = " << Z_Base << " ";
}

void taxel::printValues(){
    std::cout << "ID = " << std::dec << sensor_id << " ";
    std::cout << "X = " << X << " ";
    std::cout << "Y = " << Y << " ";
    std::cout << "Z = " << Z << " ";
}

void taxel::saveXData(double data){
    X_Data.push_back(data);
}

void taxel::saveYData(double data){
    Y_Data.push_back(data);
}

void taxel::saveZData(double data){
    Z_Data.push_back(data);
}
