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
    icub_kin_finger_{ iCubFinger("right_thumb"), iCubFinger("right_index"), iCubFinger("right_middle"), iCubFinger("right_ring"), iCubFinger("right_little") }
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
    video_writer_ = std::unique_ptr<cv::VideoWriter>(new cv::VideoWriter("./" + prefix_ + "_video_" + std::to_string(counter_) + ".mp4", CV_FOURCC('M','P','4','V'), 30, cv::Size(320, 240)));

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
    // disconnect ports
    bool ok_connect = true;
    ok_connect &= yarp::os::NetworkBase::disconnect("/icub/cartesianController/right_arm/state:o", "/tactile-magnetic-sensor-logger/arm_state:i", false);
    ok_connect &= yarp::os::NetworkBase::disconnect("/icub/right_arm/state:o", "/tactile-magnetic-sensor-logger/arm_encoders:i", false);
    ok_connect &= yarp::os::NetworkBase::disconnect("/icub/right_hand/analog:o", "/tactile-magnetic-sensor-logger/arm_analogs:i", false);
    // ok_connect &= yarp::os::NetworkBase::disconnect("/icub/torso/state:o", "/tactile-magnetic-sensor-logger/torso:i", false);
    // ok_connect &= yarp::os::NetworkBase::disconnect("/icub/head/state:o", "/tactile-magnetic-sensor-logger/head:i", false);
    // ok_connect &= yarp::os::NetworkBase::disconnect("/icub/skin/right_hand", "/tactile-magnetic-sensor-logger/tactile_raw:i", false);
    ok_connect &= yarp::os::NetworkBase::disconnect("/icub/skin/right_hand_comp", "/tactile-magnetic-sensor-logger/tactile_comp:i", false);
    ok_connect &= yarp::os::NetworkBase::disconnect("/tactile-magnetic-sensor/data:o", "/tactile-magnetic-sensor-logger/tactile_3d:i", false);
    if (!ok_connect)
        return false;

    disable_log();

    // close video
    video_writer_->release();

    mutex_.lock();

    run_ = false;

    mutex_.unlock();

    std::cout << "*********************" << std::endl;
    std::cout << "*                   *" << std::endl;
    std::cout << "Session " << counter_ << " stopped correctly." << std::endl;
    std::cout << "*                   *" << std::endl;
    std::cout << "*********************" << std::endl;

    // reset timer
    time_0_set_ = false;

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
            std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
            
            yarp::os::Bottle* arm_state = port_arm_state_.read(true);
            yarp::os::Bottle* arm_enc = port_arm_enc_.read(true);
            yarp::os::Bottle* arm_analogs = port_analogs_.read(true);
            // yarp::os::Bottle* torso = port_torso_.read(true);
            // yarp::os::Bottle* head = port_head_.read(true);
            // yarp::sig::Vector* tactile_raw = port_tactile_raw_.read(true);
            yarp::sig::Vector* tactile_comp = port_tactile_comp_.read(true);
            yarp::sig::VectorOf<int>* tactile_3d = port_tactile_3d_.read(true);

            // Get camera image
            yarp::sig::ImageOf<yarp::sig::PixelRgb>* image_in;
            image_in = port_image_in_.read(false);

            if (image_in != nullptr)
            {
                cv::Mat image = yarp::cv::toCvMat(*image_in).clone();

                video_writer_->write(image);
            }

            // when here, all the blocking calls were successful
            std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
            if (!time_0_set_)
            {
                time_0_ = current_time;
                time_0_set_ = true;
            }

            yarp::sig::Vector arm_state_yarp(arm_state->size());
            for (size_t i = 0; i < arm_state_yarp.size(); i++)
                arm_state_yarp(i) = arm_state->get(i).asDouble();
            VectorXd arm_state_eigen = toEigen(arm_state_yarp);

            yarp::sig::Vector arm_enc_yarp(arm_enc->size());
            for (size_t i = 0; i < arm_enc_yarp.size(); i++)
                arm_enc_yarp(i) = arm_enc->get(i).asDouble();
            VectorXd arm_enc_eigen = toEigen(arm_enc_yarp);
            setFingersJoints(arm_enc_yarp);

            VectorXd arm_analogs_eigen(arm_analogs->size());
            for (size_t i = 0; i < arm_analogs_eigen.size(); i++)
                arm_analogs_eigen(i) = arm_analogs->get(i).asDouble();

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

            VectorXd tactile_comp_eigen = toEigen(*tactile_comp);

            VectorXd tactile_3d_eigen(tactile_3d->size());
            for (std::size_t i = 0; i < tactile_3d->size(); i++)
                tactile_3d_eigen(i) = (*tactile_3d)[i];

            // evaluate elapsed time from the beginning
            double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - time_0_).count() / 1000.0;

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
                   tactile_comp_eigen.transpose(),
                   tactile_3d_eigen.transpose());

                std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

                std::cout << "Running @ "
                          << 1 / (std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0)
                          << " Hz"
                          << std::endl;
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
