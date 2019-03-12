#include <Logger.h>

#include <Eigen/Dense>

#include <yarp/eigen/Eigen.h>

#include <iostream>

using namespace Eigen;
using namespace yarp::eigen;

Logger::Logger(const std::string port_prefix, const double period) :
    port_prefix_(port_prefix),
    period_(period),
    run_(false),
    quit_(false)
{ }


Logger::~Logger()
{ }


bool Logger::run()
{
    enable_log(".", "data");

    mutex_.lock();

    run_ = true;

    mutex_.unlock();

    return true;
}


bool Logger::stop()
{
    disable_log();

    mutex_.lock();

    run_ = false;

    mutex_.unlock();

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

    ports_ok &= port_torso_.open("/" + port_prefix_ + "/torso:i");

    ports_ok &= port_head_.open("/" + port_prefix_ + "/head:i");

    ports_ok &= port_tactile_raw_.open("/" + port_prefix_ + "/tactile_raw:i");

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
            yarp::os::Bottle* arm_state = port_arm_state_.read(true);
            yarp::os::Bottle* arm_enc = port_arm_enc_.read(true);
            yarp::os::Bottle* arm_analogs = port_analogs_.read(true);
            yarp::os::Bottle* torso = port_torso_.read(true);
            yarp::os::Bottle* head = port_head_.read(true);
            yarp::sig::Vector* tactile_raw = port_tactile_raw_.read(true);
            yarp::sig::Vector* tactile_comp = port_tactile_comp_.read(true);
            yarp::sig::VectorOf<int>* tactile_3d = port_tactile_3d_.read(true);

            VectorXd arm_state_eigen(arm_state->size());
            for (size_t i = 0; i < arm_state_eigen.size(); i++)
                arm_state_eigen(i) = arm_state->get(i).asDouble();

            VectorXd arm_enc_eigen(arm_enc->size());
            for (size_t i = 0; i < arm_enc_eigen.size(); i++)
                arm_enc_eigen(i) = arm_enc->get(i).asDouble();

            VectorXd arm_analogs_eigen(arm_analogs->size());
            for (size_t i = 0; i < arm_analogs_eigen.size(); i++)
                arm_analogs_eigen(i) = arm_analogs->get(i).asDouble();

            VectorXd torso_eigen(torso->size());
            for (size_t i = 0; i < torso_eigen.size(); i++)
                torso_eigen(i) = torso->get(i).asDouble();

            VectorXd head_eigen(head->size());
            for (size_t i = 0; i < head_eigen.size(); i++)
                head_eigen(i) = head->get(i).asDouble();

            VectorXd tactile_raw_eigen = toEigen(*tactile_raw);

            VectorXd tactile_comp_eigen = toEigen(*tactile_comp);

            VectorXd tactile_3d_eigen(tactile_3d->size());
            for (std::size_t i = 0; i < tactile_3d->size(); i++)
                tactile_3d_eigen(i) = (*tactile_3d)[i];

            logger(arm_enc_eigen.transpose(),
                   arm_analogs_eigen.transpose(),
                   torso_eigen.transpose(),
                   head_eigen.transpose(),
                   tactile_raw_eigen.transpose(),
                   tactile_comp_eigen.transpose(),
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

    port_torso_.close();

    port_head_.close();

    port_tactile_raw_.close();

    port_tactile_comp_.close();

    port_tactile_3d_.close();

    port_rpc_command_.close();

    return true;
}
