#include <Controller.h>

#include <yarp/os/Bottle.h>
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

    yInfo() << log_ID_ << "Laterality: " << laterality_;

    // Get the robot name
    robot_ = rf.check("robot", Value("icub")).asString();

    yInfo() << log_ID_ << "Robot name:" << robot_;

    // Use or not tactile feedback
    use_tactile_feedback_ = rf.check("use_tactile_feedback", Value("false")).asBool();
    yInfo() << log_ID_ << "Using tactile feedback:" << use_tactile_feedback_;

    // Get the timeouts
    grasp_timeout_ = rf.check("timeout_grasp", Value(-1.0)).asDouble();

    open_timeout_ = rf.check("timeout_open", Value(-1.0)).asDouble();

    yInfo() << log_ID_ << "Grasp timeout:" << grasp_timeout_;
    yInfo() << log_ID_ << "Open timeout:" << open_timeout_;

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

    // Reset counters
    int fg_0_moves_ = 0;
    int fg_1_moves_ = 0;

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

        // load thresholds
        fingers_thresholds_[name] = rf.check("threshold", Value(10.0)).asDouble();
        yInfo() << "Thresholds:";
        yInfo() << fingers_thresholds_[name];

        fingers_[name] = finger;
    }

    // reset fingers detection
    reset_contact_detection();

    // open rpc clients
    if (!tactile_sensor_reader_port_.open("/" + port_prefix + "/tactile_sensors_reader_rpc:o"))
    {
        yError() << log_ID_ << "Unable to open the RPC client port to the 3d tactile sensors reading module.";

        return false;
    }

    if (!tactile_sensor_logger_port_.open("/" + port_prefix + "/tactile_sensors_logger_rpc:o"))
    {
        yError() << log_ID_ << "Unable to open the RPC client port to the 3d tactile sensors logger module.";

        return false;
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
        if (grasp_timeout_ > 0)
        {
            if((yarp::os::Time::now() - last_time_) > grasp_timeout_)
            {
                // Go to ControlMode::OpenWLog
                mutex_.lock();

                mode_ = ControlMode::OpenWStopLog;

                mutex_.unlock();

                yInfo() << "Switching from ControlMode::Close to ControlMode::OpenWLog";

                break;
            }
        }

        // Do control if feedback is available
        if (is_tactile_reading_available_ && is_encoders_reading_available_)
        {
            std::unordered_map<std::string, yarp::sig::VectorOf<int>> readings;
            readings["thumb_0"] = tactile_readings_.subVector(0, 2);
            readings["thumb_1"] = tactile_readings_.subVector(3, 5);
            readings["index_0"] = tactile_readings_.subVector(6, 8);
            readings["index_1"] = tactile_readings_.subVector(9, 11);
            readings["middle_0"] = tactile_readings_.subVector(12, 14);
            readings["middle_1"] = tactile_readings_.subVector(15, 17);
            readings["ring_0"] = tactile_readings_.subVector(18, 20);
            readings["ring_1"] = tactile_readings_.subVector(21, 23);

            for (auto& finger : fingers_)
            {
                yarp::sig::VectorOf<int> sensor_0 = readings[finger.first + "_0"];
                yarp::sig::VectorOf<int> sensor_1 = readings[finger.first + "_1"];
                double norm_0 = std::sqrt(std::pow(sensor_0[0], 2) + std::pow(sensor_0[1], 2) + std::pow(sensor_0[2], 2));
                double norm_1 = std::sqrt(std::pow(sensor_1[0], 2) + std::pow(sensor_1[1], 2) + std::pow(sensor_1[2], 2));

                if ((!fingers_detection_.at(finger.first)) && (norm_0 < fingers_thresholds_[finger.first]) && (norm_1 < fingers_thresholds_[finger.first]))
                {
                    finger.second.setJointsVelocities(fingers_closing_vels_[finger.first], true);
                }
                else
                {
                    finger.second.switchToPositionControl();
                    fingers_detection_.at(finger.first) = true;
                }
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
        mutex_.unlock();

        yInfo() << "Switching from ControlMode::Hold to ControlMode::Idle";

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
        mutex_.unlock();

        yInfo() << "Switching from ControlMode::Open to ControlMode::Idle";

        break;
    }

    case ControlMode::OpenWStopLog:
    {
        for (auto& finger : fingers_)
            finger.second.goHome(fingers_opening_vels_[finger.first]);

        if (open_timeout_ > 0)
        {
            // go to WaitOpen
            mutex_.lock();

            mode_ = ControlMode::WaitOpen;
            last_time_ = yarp::os::Time::now();

            mutex_.unlock();

            yInfo() << "Switching from ControlMode::OpenWLog to ControlMode::WaitOpen";
        }
        else
        {
            // go back to Idle
            mutex_.lock();
            mode_ = ControlMode::Idle;
            mutex_.unlock();

            yInfo() << "Switching from ControlMode::OpenWLog to ControlMode::Idle";
        }

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
        yInfo() << "Switching from ControlMode::Step to ControlMode::Idle";
        mutex_.unlock();
        break;
    }

    case ControlMode::WaitOpen:
    {
        if((yarp::os::Time::now() - last_time_) > open_timeout_)
        {
            // stop the logger
            if(stop_logger())
                yInfo() << "Logger stopped succesfully!";
            else
                yError() << "Cannot stop logger!";

            // Go to ControlMode::Idle
            mutex_.lock();

            mode_ = ControlMode::Idle;

            mutex_.unlock();

            yInfo() << "Switching from ControlMode::WaitOpen to ControlMode::Idle";
        }

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

    tactile_sensor_reader_port_.close();
    tactile_sensor_logger_port_.close();

    return true;
}


/**
 * IDL interface.
 */

std::string Controller::get_thr()
{
    std::string reply;

    reply = "Thresholds are:";
    for (auto& threshold : fingers_thresholds_)
        reply += threshold.first + ": " + std::to_string(threshold.second) + ",";

    return reply;
}


std::string Controller::grasp()
{
    std::string reply;

    if(calibrate_tactile_sensors())
        yInfo() << "Sensors calibrated succesfully!";
    else
    {
        yError() << "Cannot calibrate sensors!";

        reply = "[FAIL] Cannot calibrate sensors.";

        return reply;
    }

    if(run_logger())
        yInfo() << "Logger run succesfully!";
    else
    {
        yError() << "Cannot run logger!";

        reply = "[FAIL] Cannot run logger.";

        return reply;
    }

    mutex_.lock();

    mode_ = ControlMode::Close;
    reset_contact_detection();
    last_time_ = yarp::os::Time::now();

    mutex_.unlock();

    reply = "[OK] Command issued.";

    return reply;
}


bool Controller::hold()
{
    mutex_.lock();
    mode_ = ControlMode::Hold;
    mutex_.unlock();

    return true;
}


bool Controller::open()
{
    mutex_.lock();
    mode_ = ControlMode::Open;
    mutex_.unlock();

    return true;
}


bool Controller::open_stop_log()
{
    mutex_.lock();
    mode_ = ControlMode::OpenWStopLog;
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


bool Controller::reset()
{
    reset_contact_detection();

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


bool Controller::thr_thumb(const double threshold)
{
    mutex_.lock();
    fingers_thresholds_.at("thumb") = threshold;
    mutex_.unlock();

    yInfo() << "New threshold is:" << fingers_thresholds_.at("thumb");

    return true;
}


bool Controller::thr_index(const double threshold)
{
    mutex_.lock();
    fingers_thresholds_.at("index") = threshold;
    mutex_.unlock();

    yInfo() << "New threshold is:" << fingers_thresholds_.at("index");

    return true;
}


bool Controller::thr_middle(const double threshold)
{
    mutex_.lock();
    fingers_thresholds_.at("middle") = threshold;
    mutex_.unlock();

    yInfo() << "New threshold is:" << fingers_thresholds_.at("middle");

    return true;
}


bool Controller::thr_ring(const double threshold)
{
    mutex_.lock();
    fingers_thresholds_.at("ring") = threshold;
    mutex_.unlock();

    yInfo() << "New threshold is:" << fingers_thresholds_.at("ring");

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


bool Controller::calibrate_tactile_sensors()
{
    Bottle cmd, reply;
    cmd.addString("calibrate");

    if(!tactile_sensor_reader_port_.write(cmd, reply))
        return false;

    if (reply.get(0).asString() != "ok")
        return false;

    return true;
}


bool Controller::run_logger()
{
    Bottle cmd, reply;
    cmd.addString("run");

    if(!tactile_sensor_logger_port_.write(cmd, reply))
        return false;

    if (reply.get(0).asString() != "ok")
        return false;

    return true;
}


bool Controller::stop_logger()
{
    Bottle cmd, reply;
    cmd.addString("stop");

    if(!tactile_sensor_logger_port_.write(cmd, reply))
        return false;

    if (reply.get(0).asString() != "ok")
        return false;

    return true;
}


void Controller::reset_contact_detection()
{
    for (auto& finger: fingers_detection_)
        fingers_detection_[finger.first] = false;
}
