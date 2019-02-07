#include <string>
#include <iostream>
#include <fstream>

#include "skinSensor.h"
#include "socketcan.h"

int main()
{

    try {

        std::cout << "******************** STARTING ********************" << std::endl;
        std::string channel = "can0";

        skinSensor *skin = new skinSensor(channel);

        for(size_t i=0;i<100;i++){

            std::cout << "******************** " << i << " ********************" << std::endl;

            skin->updateSensors();

            skin->printValues();
        }

        //skin->saveData();

        delete skin;
    } catch (const std::exception& e) {
    std::cout << "EXCEPTION #### " << e.what() << " #####" << std::endl;
    }

    return 0;
}
