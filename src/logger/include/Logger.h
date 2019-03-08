#ifndef TACTILEMAGNETICLOGGER_H
#define TACTILEMAGNETICLOGGER_H

#include <BayesFilters/Logger.h>

#include <Eigen/Dense>

#include <thrift/LoggerIDL.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <vector>
#include <string>


class Logger : public yarp::os::RFModule,
               public LoggerIDL,
               public bfl::Logger
{
public:
    Logger(const std::string port_prefix, const double period);

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
        return {prefix_path + "/" + prefix_name + "_right_arm_encoders",
                prefix_path + "/" + prefix_name + "_right_arm_analogs",
                prefix_path + "/" + prefix_name + "_torso",
                prefix_path + "/" + prefix_name + "_head",
                prefix_path + "/" + prefix_name + "_tactile_raw",
                prefix_path + "/" + prefix_name + "_tactile_comp",
                prefix_path + "/" + prefix_name + "_tactile_3d"};
    }

    const std::string log_ID_ = "[LOGGER]";

    const std::string port_prefix_;

    double period_;

    yarp::os::Mutex mutex_;

    bool run_;

    bool quit_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_arm_enc_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_analogs_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_torso_;

    yarp::os::BufferedPort<yarp::os::Bottle> port_head_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_tactile_raw_;

    yarp::os::BufferedPort<yarp::sig::Vector> port_tactile_comp_;

    yarp::os::BufferedPort<yarp::sig::VectorOf<int>> port_tactile_3d_;

    yarp::os::Port port_rpc_command_;
};

#endif /* TACTILEMAGNETICLOGGER_H */