#include <Logger.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include <cstdlib>
#include <string>

using namespace yarp::os;


int main(int argc, char** argv)
{
    const std::string log_ID = "[Main]";
    yInfo() << log_ID << "Configuring and starting module...";

    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError() << log_ID << "Yarp is not available.";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("tactile-magnetic-sensor-logger");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    /* Get period. */
    const double period = rf.check("period", Value(0.05)).asDouble();

    /* Get perfix. */
    const std::string prefix = rf.check("prefix", Value(".")).asString();

    /* Run module. */
    Logger logger("tactile-magnetic-sensor-logger", period, prefix);

    logger.runModule(rf);

    return EXIT_SUCCESS;
}
