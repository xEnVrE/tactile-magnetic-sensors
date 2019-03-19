#include <Logger.h>

#include <Eigen/Dense>

#include <yarp/cv/Cv.h>
#include <yarp/eigen/Eigen.h>
#include <yarp/math/Math.h>
#include <yarp/os/Network.h>

#include <iostream>

using namespace Eigen;
using namespace iCub::iKin;
using namespace iCub::ctrl;
using namespace yarp::eigen;
using namespace yarp::math;


Logger::Logger(const std::string port_prefix, const double period, const std::string prefix) :
    port_prefix_(port_prefix),
    period_(period),
    prefix_(prefix),
    run_(false),
    quit_(false),
    time_0_set_(false),
    counter_(0),
    fingers_encoders_("tactile-magnetic-sensor-logger", "right", port_prefix),
    icub_kin_finger_{ iCubFinger("right_thumb"), iCubFinger("right_index"), iCubFinger("right_middle"), iCubFinger("right_ring"), iCubFinger("right_little") },
    is_arm_state_available_(false),
    is_arm_enc_available_(false),
    is_arm_analogs_available_(false),
    is_tactile_comp_available_(false),
    camera_fps_(24.0),
    current_camera_period_(0.0)
{

    icub_kin_finger_[0].setAllConstraints(false);
    icub_kin_finger_[1].setAllConstraints(false);
    icub_kin_finger_[2].setAllConstraints(false);
    icub_kin_finger_[3].setAllConstraints(false);
    icub_kin_finger_[4].setAllConstraints(false);

    if(!port_image_in_.open("/" + port_prefix + "/cam/right:i"))
    {
        std::string err = "LOGGER::CTOR::ERROR\n\tError: cannot open right camera input port.";
        throw(std::runtime_error(err));
    }
}


Logger::~Logger()
{ }


bool Logger::run()
{
    enable_log(".", prefix_ + "_data_" + std::to_string(counter_));

    // connect ports
    bool ok_connect = true;
    ok_connect &= yarp::os::NetworkBase::connect("/icub/cartesianController/right_arm/state:o", "/tactile-magnetic-sensor-logger/arm_state:i", "tcp", false);
    ok_connect &= yarp::os::NetworkBase::connect("/icub/right_arm/state:o", "/tactile-magnetic-sensor-logger/arm_encoders:i", "tcp", false);
    ok_connect &= yarp::os::NetworkBase::connect("/icub/right_hand/analog:o", "/tactile-magnetic-sensor-logger/arm_analogs:i", "tcp", false);
    // ok_connect &= yarp::os::NetworkBase::connect("/icub/torso/state:o", "/tactile-magnetic-sensor-logger/torso:i", "tcp", false);
    // ok_connect &= yarp::os::NetworkBase::connect("/icub/head/state:o", "/tactile-magnetic-sensor-logger/head:i", "tcp", false);
    // ok_connect &= yarp::os::NetworkBase::connect("/icub/skin/right_hand", "/tactile-magnetic-sensor-logger/tactile_raw:i", "tcp", false);
    ok_connect &= yarp::os::NetworkBase::connect("/icub/skin/right_hand_comp", "/tactile-magnetic-sensor-logger/tactile_comp:i", "tcp", false);
    ok_connect &= yarp::os::NetworkBase::connect("/tactile-magnetic-sensor/data:o", "/tactile-magnetic-sensor-logger/tactile_3d:i", "tcp", false);

    // offline only
    yarp::os::NetworkBase::connect("/icub/right_hand/analog:o", "/tactile-magnetic-sensor-logger/right_hand/analog:i", "tcp", false);

    if (!ok_connect)
        return false;

    // create a video writer object
    video_writer_ = std::unique_ptr<cv::VideoWriter>(new cv::VideoWriter("./" + prefix_ + "_video_" + std::to_string(counter_) + ".mp4", CV_FOURCC('M','P','4','V'), camera_fps_, cv::Size(320, 240)));

    mutex_.lock();

    run_ = true;

    mutex_.unlock();

    std::cout << "*********************" << std::endl;
    std::cout << "*                   *" << std::endl;
    std::cout << "Session " << counter_ << " running." << std::endl;
    std::cout << "*                   *" << std::endl;
    std::cout << "*********************" << std::endl;

    return true;
}


bool Logger::stop()
{
    mutex_.lock();

    run_ = false;

    mutex_.unlock();

    // close files
    disable_log();

    // close video
    video_writer_->release();

    // reset flags
    is_arm_state_available_ = false;
    is_arm_enc_available_ = false;
    is_arm_analogs_available_ = false;
    is_tactile_comp_available_ = false;

    // reset timer
    time_0_set_ = false;

    // reset camera period
    current_camera_period_ = 0.0;

    // disconnect ports
    bool ok_disconnect = true;
    ok_disconnect &= yarp::os::NetworkBase::disconnect("/icub/cartesianController/right_arm/state:o", "/tactile-magnetic-sensor-logger/arm_state:i", false);
    ok_disconnect &= yarp::os::NetworkBase::disconnect("/icub/right_arm/state:o", "/tactile-magnetic-sensor-logger/arm_encoders:i", false);
    ok_disconnect &= yarp::os::NetworkBase::disconnect("/icub/right_hand/analog:o", "/tactile-magnetic-sensor-logger/arm_analogs:i", false);
    // ok_disconnect &= yarp::os::NetworkBase::disconnect("/icub/torso/state:o", "/tactile-magnetic-sensor-logger/torso:i", false);
    // ok_disconnect &= yarp::os::NetworkBase::disconnect("/icub/head/state:o", "/tactile-magnetic-sensor-logger/head:i", false);
    // ok_disconnect &= yarp::os::NetworkBase::disconnect("/icub/skin/right_hand", "/tactile-magnetic-sensor-logger/tactile_raw:i", false);
    ok_disconnect &= yarp::os::NetworkBase::disconnect("/icub/skin/right_hand_comp", "/tactile-magnetic-sensor-logger/tactile_comp:i", false);
    ok_disconnect &= yarp::os::NetworkBase::disconnect("/tactile-magnetic-sensor/data:o", "/tactile-magnetic-sensor-logger/tactile_3d:i", false);
    if (!ok_disconnect)
        return false;

    // flush port buffer
    while (port_arm_state_.getPendingReads() > 0)
        port_arm_state_.read(false);
    while (port_arm_enc_.getPendingReads() > 0)
        port_arm_enc_.read(false);
    while (port_analogs_.getPendingReads() > 0)
        port_analogs_.read(false);
    while (port_tactile_comp_.getPendingReads() > 0)
        port_tactile_comp_.read(false);
    while (port_tactile_3d_.getPendingReads() > 0)
        port_tactile_3d_.read(false);

    std::cout << "*********************" << std::endl;
    std::cout << "*                   *" << std::endl;
    std::cout << "Session " << counter_ << " stopped correctly." << std::endl;
    std::cout << "*                   *" << std::endl;
    std::cout << "*********************" << std::endl;

    // increase counter
    counter_++;

    return true;
}


bool Logger::quit()
{
    disable_log();

    mutex_.lock();

    quit_ = true;

    stopModule();

    mutex_.unlock();

    return true;
}


bool Logger::configure(yarp::os::ResourceFinder& rf)
{
    bool ports_ok = true;

    ports_ok &= port_arm_state_.open("/" + port_prefix_ + "/arm_state:i");

    ports_ok &= port_arm_enc_.open("/" + port_prefix_ + "/arm_encoders:i");

    ports_ok &= port_analogs_.open("/" + port_prefix_ + "/arm_analogs:i");

    // ports_ok &= port_torso_.open("/" + port_prefix_ + "/torso:i");

    // ports_ok &= port_head_.open("/" + port_prefix_ + "/head:i");

    // ports_ok &= port_tactile_raw_.open("/" + port_prefix_ + "/tactile_raw:i");

    ports_ok &= port_tactile_comp_.open("/" + port_prefix_ + "/tactile_comp:i");

    ports_ok &= port_tactile_3d_.open("/" + port_prefix_ + "/tactile_3d:i");

    ports_ok &= port_rpc_command_.open("/" + port_prefix_ + "/cmd:i");

    ports_ok &= this->yarp().attachAsServer(port_rpc_command_);

    return ports_ok;
}


double Logger::getPeriod()
{
    return period_;
}


bool Logger::setFingersJoints(const yarp::sig::Vector& q)
{
    // Get analog readings
    bool valid_analogs = false;
    yarp::sig::Vector fingers_analogs;
    std::tie(valid_analogs, fingers_analogs) = fingers_encoders_.getEncoders();

    bool valid_bounds = false;
    yarp::sig::Matrix fingers_bounds;
    if (valid_analogs)
    {
        // Check if analog bounds are available
        std::tie(valid_bounds, fingers_bounds) = fingers_encoders_.getAnalogBounds();
    }

    yarp::sig::Vector chainjoints;
    for (size_t i = 0; i < 5; ++i)
    {
        if (valid_analogs)
        {
            if (valid_bounds)
            {
                if (!(icub_kin_finger_[i].getChainJoints(q.subVector(0, 15), fingers_analogs, chainjoints, fingers_bounds)))
                    return false;
            }
            else
            {
                if (!(icub_kin_finger_[i].getChainJoints(q.subVector(0, 15), fingers_analogs, chainjoints)))
                    return false;
            }
        }
        else
        {
            if (!icub_kin_finger_[i].getChainJoints(q.subVector(0, 15), chainjoints))
                return false;
        }

        icub_kin_finger_[i].setAng(chainjoints * CTRL_DEG2RAD);
    }

    return true;
}


bool Logger::updateModule()
{
    bool run_local;

    bool quit_local;

    mutex_.lock();

    run_local = run_;
    quit_local = quit_;

    mutex_.unlock();

    if (!quit_local)
    {
        if (run_local)
        {
            // Block on the 3d tactile sensors since their are the fastest
            yarp::sig::VectorOf<int>* tactile_3d = port_tactile_3d_.read(true);

            // Use non-blocking calls here instead
            // since not loosing information from the 3d tactile sensors is more important
            yarp::os::Bottle* arm_state = port_arm_state_.read(false);
            if (arm_state != nullptr)
            {
                last_arm_state_ = *arm_state;
                is_arm_state_available_ = true;
            }
            yarp::os::Bottle* arm_enc = port_arm_enc_.read(false);
            if (arm_enc != nullptr)
            {
                last_arm_enc_ = *arm_enc;
                is_arm_enc_available_ = true;
            }
            yarp::os::Bottle* arm_analogs = port_analogs_.read(false);
            if (arm_analogs != nullptr)
            {
                last_arm_analogs_ = *arm_analogs;
                is_arm_analogs_available_ = true;
            }
            // yarp::os::Bottle* torso = port_torso_.read(true);
            // yarp::os::Bottle* head = port_head_.read(true);
            // yarp::sig::Vector* tactile_raw = port_tactile_raw_.read(true);
            // yarp::sig::Vector* tactile_comp = port_tactile_comp_.read(false);
            // if (tactile_comp != nullptr)
            // {
            //     last_tactile_comp_ = *tactile_comp;
            //     is_tactile_comp_available_ = true;
            // }

            // Synchronize so that at least one reading is available for each sensor
            if (!(is_arm_state_available_ &&
                  is_arm_enc_available_ &&
                  is_arm_analogs_available_))//  &&
                  // is_tactile_comp_available_)
            {
                // We need at least one reading to be available for each sensor
                return true;
            }

            // Once all the data is available start counting
            std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
            if (!time_0_set_)
            {
                time_0_ = current_time;

                last_time_ = 0.0;

                time_0_set_ = true;
            }

            yarp::sig::Vector arm_state_yarp(last_arm_state_.size());
            for (size_t i = 0; i < arm_state_yarp.size(); i++)
                arm_state_yarp(i) = last_arm_state_.get(i).asDouble();
            VectorXd arm_state_eigen = toEigen(arm_state_yarp);

            yarp::sig::Vector arm_enc_yarp(last_arm_enc_.size());
            for (size_t i = 0; i < arm_enc_yarp.size(); i++)
                arm_enc_yarp(i) = last_arm_enc_.get(i).asDouble();
            VectorXd arm_enc_eigen = toEigen(arm_enc_yarp);
            setFingersJoints(arm_enc_yarp);

            VectorXd arm_analogs_eigen(last_arm_analogs_.size());
            for (size_t i = 0; i < arm_analogs_eigen.size(); i++)
                arm_analogs_eigen(i) = last_arm_analogs_.get(i).asDouble();

            // VectorXd torso_eigen(torso->size());
            // for (size_t i = 0; i < torso_eigen.size(); i++)
            //     torso_eigen(i) = torso->get(i).asDouble();

            // VectorXd head_eigen(head->size());
            // for (size_t i = 0; i < head_eigen.size(); i++)
            //     head_eigen(i) = head->get(i).asDouble();

            // VectorXd tactile_raw_eigen = toEigen(*tactile_raw);

            // Evaluate poses of all finger tips
            yarp::sig::Vector ee_t(4);
            ee_t(0) = arm_state_yarp(0);
            ee_t(1) = arm_state_yarp(1);
            ee_t(2) = arm_state_yarp(2);
            ee_t(3) = 1.0;

            yarp::sig::Vector ee_o(4);
            ee_o(0) = arm_state_yarp(3);
            ee_o(1) = arm_state_yarp(4);
            ee_o(2) = arm_state_yarp(5);
            ee_o(3) = arm_state_yarp(6);

            yarp::sig::Matrix H_palm = axis2dcm(ee_o);
            H_palm.setCol(3, ee_t);
            std::vector<VectorXd> fingertips_poses(5);
            for (std::size_t i = 0; i < 5; i++)
            {
                // std::string finger_s;
                yarp::sig::Matrix H_tip = icub_kin_finger_[i].getH();
                yarp::sig::Vector tip_pose(7);

                tip_pose.setSubvector(0, (H_palm * (H_tip.getCol(3))).subVector(0, 2));
                tip_pose.setSubvector(3, dcm2axis(H_palm * H_tip));
                fingertips_poses.at(i) = toEigen(tip_pose);
            }

            // VectorXd tactile_comp_eigen = toEigen(last_tactile_comp_);

            VectorXd tactile_3d_eigen(tactile_3d->size());
            for (std::size_t i = 0; i < tactile_3d->size(); i++)
                tactile_3d_eigen(i) = (*tactile_3d)[i];

            // evaluate elapsed time from the beginning
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - time_0_).count() / 1000.0;

            double step = elapsed - last_time_;

            // Get camera image
            yarp::sig::ImageOf<yarp::sig::PixelRgb>* image_in;
            image_in = port_image_in_.read(false);

	    if (image_in != nullptr)
	    {
	        last_image_in_ = *image_in;
		is_last_image_avaiable_ = true;
	    }

            // Try to save frames at approximately camera_fps_ Hz
            // Assuming that step always less than 1.0 / camera_fps_
	    if ((current_camera_period_ > 1.0 / camera_fps_) && is_last_image_avaiable_)
            {
	        cv::Mat image = yarp::cv::toCvMat(last_image_in_).clone();

	        video_writer_->write(image);
	        current_camera_period_ = 0.0;
            }
            else
                current_camera_period_ += step;

            std::cout << "Running @ "
                      << 1.0 / step
                      << " Hz"
                      << std::endl;

            // store last value for next iteration
            last_time_ = elapsed;

            logger(elapsed,
                   arm_state_eigen.transpose(),
                   arm_enc_eigen.transpose(),
                   arm_analogs_eigen.transpose(),
                   fingertips_poses.at(0).transpose(),
                   fingertips_poses.at(1).transpose(),
                   fingertips_poses.at(2).transpose(),
                   fingertips_poses.at(3).transpose(),
                   fingertips_poses.at(4).transpose(),
                   // torso_eigen.transpose(),
                   // head_eigen.transpose(),
                   // tactile_raw_eigen.transpose(),
                   // tactile_comp_eigen.transpose(),
                   tactile_3d_eigen.transpose());
        }

        return true;
    }

    return false;
}


bool Logger::close()
{
    port_arm_state_.close();

    port_arm_enc_.close();

    port_analogs_.close();

    // port_torso_.close();

    // port_head_.close();

    // port_tactile_raw_.close();

    port_tactile_comp_.close();

    port_tactile_3d_.close();

    port_rpc_command_.close();

    port_image_in_.close();

    return true;
}
