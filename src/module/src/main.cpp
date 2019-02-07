#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::os;

int main(int argc, char** argv)
{
    const std::string log_ID = "[Main]";

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("tactile-magnetic-sensor-module");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);
}
