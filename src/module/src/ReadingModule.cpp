#include <ReadingModule.h>

#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::os;
using namespace yarp::sig;


ReadingModule::~ReadingModule()
{ }


bool ReadingModule::configure(yarp::os::ResourceFinder& rf)
{
    // Get the port prefix
    const std::string port_prefix = rf.check("port_prefix", Value("tactile-magnetic-sensor")).asString();

    // Get the period
    period_ = rf.check("period", Value(1.0)).asDouble();

    // Get the name of the can device
    const std::string can_name = rf.check("can_name", Value("can0")).asString();

    // Get the size of the window used to average on each sensor measurements
    const std::size_t average_window_size = rf.check("average_window_size", Value(1)).asInt();

    // Get the list of sensor ids
    bool valid_ids;
    std::vector<unsigned int> list_ids;
    std::tie(valid_ids, list_ids) = loadListUnsigned(rf, "sensors_ids");
    if ((!valid_ids) || list_ids.size() == 0)
    {
        yError() << log_ID_ << "Cannot retrieve the sensors ids.";

        return false;
    }

    // Log
    yInfo() << log_ID_ << "Loaded configuration:";
    yInfo() << log_ID_ << "- port_prefix:" << port_prefix;
    yInfo() << log_ID_ << "- period:" << period_;
    yInfo() << log_ID_ << "- sensors_ids:";
    for (unsigned int& id : list_ids)
        yInfo() << log_ID_ << id;

    // Initialize the driver
    skin_sensor_drv_ = std::unique_ptr<skinSensor>(new skinSensor(can_name, list_ids, average_window_size));

    // Open data output port
    if (!port_data_out_.open("/" + port_prefix + "/data:o"))
    {
        yError() << log_ID_ << "Cannot open the data output port.";

        return false;
    }

    if (!skin_sensor_drv_->init())
    {
        yError() << log_ID_ << "Cannot initialize the MTB.";

        return false;
    }

    // Open RPC input port for commands
    if (!port_rpc_command_.open("/" + port_prefix + "/cmd:i"))
    {
        yError() << log_ID_ << "Cannot open the RPC command input port.";

        return false;
    }

    if (!(this->yarp().attachAsServer(port_rpc_command_)))
    {
        yError() << log_ID_ << "Cannot attach the RPC command input port.";

        return false;
    }

    // Reset flags
    is_ongoing_calibration_ = false;

    yInfo() << log_ID_ << "RPC command port opened and attached. Ready to recieve commands!";
}


bool ReadingModule::updateModule()
{
    // Check if calibration is ongoing
    bool is_calibration;

    mutex_.lock();

    is_calibration = is_ongoing_calibration_;

    mutex_.unlock();

    if (is_calibration)
    {
        // do nothing
        return true;
    }

    // If not ongoing calibration, it is safe to fetch data
    try
    {
        // Update sensors
        skin_sensor_drv_->updateSensors();

        std::vector<int> sensors_data;
        sensors_data = skin_sensor_drv_->getData();

        // Fill the output yarp vector
        VectorOf<int>& data_out = port_data_out_.prepare();

        data_out = VectorOf<int>(sensors_data.size(), sensors_data.data());

        // Send the data
        port_data_out_.write();

    }
    catch (const std::exception& e)
    {
        yError() << "Exception" << e.what();
        return false;
    }

    return true;
}


double ReadingModule::getPeriod()
{
    return period_;
}


bool ReadingModule::close()
{
    return true;
}


bool ReadingModule::calibrate()
{
    // // Notify that calibration is ongoing
    // mutex_.lock();

    // is_ongoing_calibration_ = true;

    // mutex_.unlock();

    // // Do calibration
    // skin_sensor_drv_->calibrate();

    // // Notify calibration is over
    // mutex_.lock();

    // is_ongoing_calibration_ = false;

    // mutex_.unlock();

    return true;
}


bool ReadingModule::quit()
{
    stopModule();

    return true;
}


std::pair<bool, std::vector<unsigned int>> ReadingModule::loadListUnsigned(ResourceFinder &rf, const std::string& key)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        ok = false;

    if (!ok)
        return std::make_pair(false, std::vector<unsigned int>());

    std::vector<unsigned int> list;
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return std::make_pair(false, std::vector<unsigned int>());

        if (!item_v.isInt())
            return std::make_pair(false, std::vector<unsigned int>());

        // Is it ok to use 'asInt' for unsigned int?
        list.push_back(item_v.asInt());
    }

    return std::make_pair(true, list);
}
