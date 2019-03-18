#include <Controller.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::os;
using namespace yarp::sig;


Controller::~Controller()
{ }


/**
 * RFModule interface.
 */
bool Controller::configure(yarp::os::ResourceFinder& rf)
{
    // Get the port prefix
    const std::string port_prefix = rf.check("port_prefix", Value("tactile-magnetic-sensor-controller")).asString();

    // Get the period
    period_ = rf.check("period", Value(1.0)).asDouble();

    // Open tactile data input port
    if (!tactile_sensors_data_in_.open("/" + port_prefix + "/data:i"))
    {
        yError() << log_ID_ << "Cannot open the tactile sensors data input port.";

        return false;
    }

    // Open RPC input port for commands
    if (!port_rpc_command_.open("/" + port_prefix + "/cmd:i"))
    {
        yError() << log_ID_ << "Cannot open the RPC command input port.";

        return false;
    }

    // Attach the underlying RPC server to the RPC input command port
    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        yError() << log_ID_ << "Cannot attach the RPC command input port.";

        return false;
    }

    // Reset flag
    is_tactile_reading_available_ = false;

    yInfo() << log_ID_ << "RPC command port opened and attached. Ready to receive commands!";

    return true;
}


bool Controller::updateModule()
{
    // Note: the module continue to run at approximately 1.0 / period_ unless
    // 'false' is returned

    // Get feedback from tactile sensors
    bool blocking_read = false;
    VectorOf<int>* data = tactile_sensors_data_in_.read(false);

    // Check if data is valid
    if (data != nullptr)
    {
        // If available, update data
        tactile_readings_ = *data;

        // Remember we received data at least one time
        is_tactile_reading_available_ = true;
    }

    // Do control if feedback is available
    if (is_tactile_reading_available_)
    {
        // TODO: control fingers
    }

    return true;
}


double Controller::getPeriod()
{
    return period_;
}


bool Controller::close()
{
    return true;
}


/**
 * IDL interface.
 */
bool Controller::quit()
{
    // Note: stopModule is a method provided by class RFModule.
    stopModule();

    return true;
}
