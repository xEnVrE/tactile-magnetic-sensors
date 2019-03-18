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

    yInfo() << log_ID_ << "Period: " << period_;

    // Get the laterality
    laterality_ = rf.check("laterality", Value("right")).asString();

    // Get the robot name
    robot_ = rf.check("robot", Value("icub")).asString();

    // Use or not tactile feedback
    use_tactile_feedback_ = rf.check("use_tactile_feedback", Value("false")).asBool();
    yInfo() << log_ID_ << "Using tactile feedback:" << use_tactile_feedback_;

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

    // Reset flags
    is_tactile_reading_available_ = false;
    is_encoders_reading_available_ = false;

    // prepare properties for the Encoders
    yarp::os::Property prop;
    prop.put("device", "remote_controlboard");
    prop.put("remote", "/" + robot_ + "/" + laterality_ + "_arm");
    prop.put("local", "/" + port_prefix + "/" + laterality_ + "_arm");
    if (!drv_arm.open(prop))
    {
        yError() << log_ID_ << "Cannot open the remote control board";

        return false;
    }

    // try to retrieve the views
    if (!(drv_arm.view(ienc_arm_)) || ienc_arm_ == nullptr)
    {
        yError() << log_ID_ << "Cannot retrieve the encoders view";

        return false;
    }

    if (!(drv_arm.view(ipos_arm_)) || ipos_arm_ == 0)
    {
        yError() << log_ID_ << "Cannot open the position control view.";

        return false;
    }

    if (!(drv_arm.view(ivel_arm_)) || ivel_arm_ == 0)
    {
        yError() << log_ID_ <<  "Cannot open the velocity control view.";

        return false;
    }

    if (!(drv_arm.view(imod_arm_)) || imod_arm_ == 0)
    {
        yError() << log_ID_ << "Cannot open the control mode view.";

        return false;
    }

    if (!(drv_arm.view(ilimits_arm_)) || ilimits_arm_ == 0)
    {
        yError() << log_ID_ << "Cannot open the control limits view.";

        return false;
    }

    // resize the vector containing the encoders
    int n_encs;
    if (ienc_arm_->getAxes(&n_encs))
    {
        encoder_readings_.resize(n_encs);
    }

    // load list of active fingers
    active_fingers_ = loadListString(rf, "active_fingers");

    for (std::string name : active_fingers_)
    {
        // instantiate and configure fingers
        FingerController finger;
        if (!(finger.init(laterality_, name, imod_arm_, ilimits_arm_, ipos_arm_, ivel_arm_)))
        {
            yError() << log_ID_ << "Cannot initialize controller for finger"
                     << laterality_ << "-" << name;

            return false;
        }

        if (!(finger.configure(rf)))
        {
            yError() << log_ID_ << "Cannot configure controller for finger"
                     << laterality_ << "-" << name;

            return false;
        }

        // load opening velocities
        std::string key = laterality_ + "_" + name;
        ResourceFinder nested_rf = rf.findNestedResourceFinder(key.c_str());
        fingers_opening_vels_[name] = nested_rf.check("opening_vel", Value(10.0)).asDouble();

        yInfo() << "Opening vels:";
        yInfo() << fingers_opening_vels_.at(name);

        // load closing velocities
        fingers_closing_vels_[name] = loadVectorDouble(nested_rf, "closing_vels", finger.getNumberJoints());

        yInfo() << "Closing vels:";
        yInfo() << fingers_closing_vels_.at(name).toString();

        // load deltas for adaptive grasping
        fingers_deltas_[name] = loadVectorDouble(nested_rf, "deltas", finger.getNumberJoints());
        fingers_deltas_vels_[name] = loadVectorDouble(nested_rf, "deltas_vels", finger.getNumberJoints());

        yInfo() << "Deltas:";
        yInfo() << fingers_deltas_.at(name).toString();

        yInfo() << "Deltas feedforward velocities:";
        yInfo() << fingers_deltas_vels_.at(name).toString();

        fingers_[name] = finger;
    }

    // reset control mode
    mode_ = ControlMode::Idle;

    yInfo() << log_ID_ << "RPC command port opened and attached. Ready to receive commands!";

    return true;
}


bool Controller::updateModule()
{
    // Note: the module continue to run at approximately 1.0 / period_ unless
    // 'false' is returned

    // Get feedback from encoders
    if(ienc_arm_->getEncoders(encoder_readings_.data()))
        is_encoders_reading_available_ = true;

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

    // If tactile feedback is not required override it
    is_tactile_reading_available_ |= !use_tactile_feedback_;

    ControlMode mode_local;
    mutex_.lock();
    mode_local = mode_;
    mutex_.unlock();

    // update encoders motors for each finger
    if (is_encoders_reading_available_)
    {
        for (auto& finger : fingers_)
            finger.second.updateEncoders(encoder_readings_);
    }

    switch (mode_local)
    {

    case ControlMode::Close:
    {
        // Do control if feedback is available
        if (is_tactile_reading_available_ && is_encoders_reading_available_)
        {
            // TODO: control fingers
            for (auto& finger : fingers_)
            {
                // if (condition_0)
                    finger.second.setJointsVelocities(fingers_closing_vels_[finger.first], true);
                // else if (condition_1)
                //     finger.second.switchToPositionControl();

            }
        }

        break;
    }

    case ControlMode::Hold:
    {
        for (auto& finger : fingers_)
            finger.second.switchToPositionControl();

        // go back to Idle
        mutex_.lock();
        mode_ = ControlMode::Idle;
        yInfo() << "Switching from ControlMode::Hold to ControlMode::Idle";
        mutex_.unlock();
        break;
    }

    case ControlMode::Idle:
    {
        // Do nothing
        break;
    }

    case ControlMode::Open:
    {
        for (auto& finger : fingers_)
            finger.second.goHome(fingers_opening_vels_[finger.first]);

        // go back to Idle
        mutex_.lock();
        mode_ = ControlMode::Idle;
        yInfo() << "Switching from ControlMode::Open to ControlMode::Idle";
        mutex_.unlock();
        break;
    }

    case ControlMode::Step:
    {
        if (is_encoders_reading_available_)
        {
            for (auto& finger : fingers_)
                finger.second.setJointsRelativePosition(fingers_deltas_.at(finger.first), fingers_deltas_vels_.at(finger.first));
        }

        // go back to Idle
        mutex_.lock();
        mode_ = ControlMode::Idle;
        yInfo() << "Switching from ControlMode::Open to ControlMode::Idle";
        mutex_.unlock();
        break;
    }

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
bool Controller::grasp()
{
    mutex_.lock();
    mode_ = ControlMode::Close;
    mutex_.unlock();

    return true;
}


bool Controller::hold()
{
    mutex_.lock();
    mode_ = ControlMode::Hold;
    mutex_.unlock();
}


bool Controller::open()
{
    mutex_.lock();
    mode_ = ControlMode::Open;
    mutex_.unlock();

    return true;
}


bool Controller::quit()
{
    // go to idle
    mutex_.lock();
    mode_ = ControlMode::Idle;
    mutex_.unlock();

    for (auto& finger : fingers_)
    {
        // Stop any finger movements
        finger.second.stop();

        // Close the finger controller
        finger.second.close();
    }

    // Note: stopModule is a method provided by class RFModule.
    stopModule();

    return true;
}


bool Controller::step()
{
    // go to idle
    mutex_.lock();
    mode_ = ControlMode::Step;
    mutex_.unlock();

    return true;
}


bool Controller::stop()
{
    // go to idle
    mutex_.lock();
    mode_ = ControlMode::Idle;
    mutex_.unlock();

    for (auto& finger : fingers_)
    {
        // Stop any finger movements
        finger.second.stop();
    }

    return true;
}


std::vector<std::string> Controller::loadListString(ResourceFinder& rf, const std::string key)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if (b == nullptr)
        ok = false;

    if (!ok)
    {
        yError() << "[Main]" << "Unable to load list of strings with key" << key;
        std::exit(EXIT_FAILURE);
    }

    std::vector<std::string> list;
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
            return std::vector<std::string>();

        if (!item_v.isString())
            return std::vector<std::string>();

        list.push_back(item_v.asString());
    }

    return list;
}


yarp::sig::Vector Controller::loadVectorDouble(yarp::os::ResourceFinder& rf, const std::string key, const std::size_t size)
{
    bool ok = true;

    if (rf.find(key).isNull())
        ok = false;

    Bottle* b = rf.find(key).asList();
    if ((b == nullptr) || (b->size() != size))
    {
        yError() << "[Main]" << "Unable to load vector" << key;
        std::exit(EXIT_FAILURE);
    }

    yarp::sig::Vector vector(size);
    for (std::size_t i = 0; i < b->size(); i++)
    {
        Value item_v = b->get(i);
        if (item_v.isNull())
        {
            yError() << "[Main]" << "Unable to load vector" << key;
            std::exit(EXIT_FAILURE);
        }

        if (!item_v.isDouble())
        {
            yError() << "[Main]" << "Unable to load vector" << key;
            std::exit(EXIT_FAILURE);
        }

        vector(i) = item_v.asDouble();
    }

    return vector;
}

