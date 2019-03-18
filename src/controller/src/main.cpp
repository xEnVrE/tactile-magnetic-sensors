#include <Controller.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::os;


int main(int argc, char** argv)
{
    const std::string log_ID = "[Main]";

    // Check for YARP network
    Network yarp_network;
    if (!yarp_network.checkNetwork())
    {
        yError() << log_ID << "Cannot find YARP server. Is YARP running?";

        return EXIT_FAILURE;
    }

    // Initialize the resource finder,
    // this is the object we are getting the configuration parameters from
    ResourceFinder rf;
    rf.setVerbose();
    rf.setDefaultContext("tactile-magnetic-sensor-controller");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    // Instantiate the controller
    Controller ctl;

    // Configure and start the module
    ctl.runModule(rf);

    return EXIT_SUCCESS;
}
