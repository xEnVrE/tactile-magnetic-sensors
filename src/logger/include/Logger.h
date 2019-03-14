#ifndef TACTILEMAGNETICLOGGER_H
#define TACTILEMAGNETICLOGGER_H

#include <BayesFilters/Logger.h>

#include <Eigen/Dense>

#include <iCubFingersEncoders.h>

#include <iCub/iKin/iKinFwd.h>

#include <thrift/LoggerIDL.h>

#include <opencv2/opencv.hpp>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Image.h>
#include <yarp/sig/Vector.h>

#include <chrono>
#include <vector>
#include <memory>
#include <string>


class Logger : public yarp::os::RFModule,
               public LoggerIDL,
               public bfl::Logger
{
public:
    Logger(const std::string port_prefix, const double period, const std::string prefix);

    virtual ~Logger();

    bool run() override;

    bool stop() override;

    bool quit() override;

    bool configure(yarp::os::ResourceFinder& rf) override;

    double getPeriod() override;

    bool updateModule() override;

    bool close() override;

protected:
    std::vector<std::string> log_filenames(const std::string& prefix_path, const std::string& prefix_name) /* override */
    {
        return {prefix_path + "/" + prefix_name + "_time",
                prefix_path + "/" + prefix_name + "_right_arm_state",
                prefix_path + "/" + prefix_name + "_right_arm_encoders",
                prefix_path + "/" + prefix_name + "_right_arm_analogs",
                prefix_path + "/" + prefix_name + "_right_thumb_pose",
                prefix_path + "/" + prefix_name + "_right_index_pose",
                prefix_path + "/" + prefix_name + "_right_middle_pose",
                prefix_path + "/" + prefix_name + "_right_ring_pose",
                prefix_path + "/" + prefix_name + "_right_little_pose",
                /* prefix_path + "/" + prefix_name + "_torso", */
                /* prefix_path + "/" + prefix_name + "_head", */
                /* prefix_path + "/" + prefix_name + "_tactile_raw", */
                prefix_path + "/" + prefix_name + "_tactile_comp",
                prefix_path + "/" + prefix_name + "_tactile_3d"};
    }

    bool setFingersJoints(const yarp::sig::Vector& q);

    const std::string log_ID_ = "[LOGGER]";

    const std::string port_prefix_;

    double period_;

    const std::string prefix_;

    yarp::os::Mutex mutex_;

    bool run_;

    bool quit_;

    int counter_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_arm_state_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_arm_enc_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_analogs_;

    /* yarp::os::BufferedPort<yarp::os::Bottle> port_torso_; */

    /* yarp::os::BufferedPort<yarp::os::Bottle> port_head_; */

    /* yarp::os::BufferedPort<yarp::sig::Vector> port_tactile_raw_; */

    yarp::os::BufferedPort<yarp::sig::Vector> port_tactile_comp_;

    yarp::os::BufferedPort<yarp::sig::VectorOf<int>> port_tactile_3d_;

    yarp::os::Port port_rpc_command_;

    std::chrono::steady_clock::time_point time_0_;

    bool time_0_set_;

    iCubFingersEncoders fingers_encoders_;

    iCub::iKin::iCubFinger icub_kin_finger_[5];

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> port_image_in_;

    yarp::sig::ImageOf<yarp::sig::PixelFloat> last_image_in;

    std::unique_ptr<cv::VideoWriter> video_writer_;
};

#endif /* TACTILEMAGNETICLOGGER_H */
