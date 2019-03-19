#include <FingerController.h>

#include <yarp/math/Math.h>

#include <limits>

using namespace yarp::dev;
using namespace yarp::math;
using namespace yarp::os;


bool FingerController::init
(
    const std::string& laterality,
    const std::string& name,
    IControlMode* imod,
    IControlLimits* ilim,
    IPositionControl* ipos,
    IVelocityControl* ivel
)
{
    bool ok;

    // reset current control mode
    control_mode_ = -1;

    // store pointer to ControlMode instance
    imod_ = imod;

    // store pointer to ControlLimits instance
    ilim_ = ilim;

    // store pointer to PositionControl instance
    ipos_ = ipos;

    // store pointer to VelocityControl instance
    ivel_ = ivel;

    // store name of the finger
    name_ = name;

    // store name of the hand
    laterality_ = laterality;

    // set the controlled joints depending on the finger name
    if (name_ == "thumb")
    {
        ctl_joints_.push_back(8);
        ctl_joints_.push_back(9);
        ctl_joints_.push_back(10);
    }
    else if (name_ == "index")
    {
        ctl_joints_.push_back(11);
        ctl_joints_.push_back(12);
    }
    else if (name_ == "middle")
    {
        ctl_joints_.push_back(13);
        ctl_joints_.push_back(14);
    }
    else if (name_ == "ring")
    {
        ctl_joints_.push_back(15);
    }
    else
    {
        yError() << "FingerController::init"
                 << "Error: finger"
                 << name_
                 << "is not valid or not supported";
        return false;
    }

    // get the current control modes for the controlled DoFs
    initial_modes_.resize(ctl_joints_.size());
    for (size_t i = 0; i < ctl_joints_.size(); i++)
    {
        imod_->getControlMode(ctl_joints_[i], &(initial_modes_[i]));
    }

    // set the velocity control mode for the controlled DoFs
    if (!(setControlMode(VOCAB_CM_VELOCITY)))
    {
        yError() << "FingerController::init"
                 << "Error: unable to set the velocity control"
                 << "mode for the joints of the"
                 << laterality_ << "-" << name_
                 << "finger";

        return false;
    }

    // get min/max joints limits
    joints_min_limits_.resize(ctl_joints_.size());
    joints_max_limits_.resize(ctl_joints_.size());
    for (size_t i=0; i<ctl_joints_.size(); i++)
    {
        double min;
        double max;
        ilim->getLimits(ctl_joints_[i], &min, &max);
        joints_min_limits_[i] = min;
        joints_max_limits_[i] = max;
    }

    yInfo() << "Finger name: " << name_;
    yInfo() << "Min:";
    yInfo() << joints_min_limits_.toString();
    yInfo() << "Max:";
    yInfo() << joints_max_limits_.toString();

    return true;
}


bool FingerController::configure(yarp::os::ResourceFinder& rf)
{
    std::string key = laterality_ + "_" + name_;
    ResourceFinder nested_rf = rf.findNestedResourceFinder(key.c_str());

    // load desired joint limits
    joints_des_limits_ = loadVectorDouble(nested_rf, "limits", ctl_joints_.size());

    yInfo() << "Desired limits:";
    yInfo() << joints_des_limits_.toString();

    // load desired joint home positions
    joints_home_ = loadVectorDouble(nested_rf, "home", ctl_joints_.size());

    yInfo() << "Home positions:";
    yInfo() << joints_home_.toString();

    return true;
}


bool FingerController::setControlMode(const int& mode)
{
    bool ok;

    for (size_t i=0; i<ctl_joints_.size(); i++)
    {
        // if (!(
        imod_->setControlMode(ctl_joints_[i], mode);//) )
        // {
        //     yError() << "FingerController:setControlMode"
        //              << "Error: unable to set control mode for one of the joint of the finger"
        //              << laterality_ << "-" << name_;
        //     return false;
        // }
    }

    // if all joints were set store the current control mode
    control_mode_ = mode;

    return true;
}


bool FingerController::close()
{
    bool ok;

    // stop motion
    if (!(ivel_->stop(ctl_joints_.size(), ctl_joints_.getFirst())))
    {
        yError() << "FingerController::close"
                 << "WARNING: unable to stop joints motion for finger"
                 << laterality_ << "-" << name_;
        return false;
    }

    // // restore initial control mode
    // for (size_t i=0; i<ctl_joints_.size(); i++)
    // {
    //     if (!(imod_->setControlMode(ctl_joints_[i], initial_modes_[i])))
    //     {
    //         yError() << "FingerController:close"
    //                  << "Error: unable to restore the initial control modes for finger"
    //                  << laterality_ << "-" << name_;
    //         return false;
    //     }
    // }

    return true;
}


bool FingerController::goHome(const double& ref_vel)
{
    // switch to position control
    // if ((
    setControlMode(VOCAB_CM_POSITION);// ))
    // {
    //     yInfo() << "FingerController::goHome Error:"
    //             << "unable to set Position control mode for finger"
    //             << laterality_ << "-" << name_;

    //     return false;
    // }

    // set reference joints velocities
    // the same velocity is used for all the joints
    for (std::size_t i = 0; i < ctl_joints_.size(); i++)
    {
        ipos_->setRefAcceleration(ctl_joints_[i], std::numeric_limits<double>::max());
        // if (!(
	ipos_->setRefSpeed(ctl_joints_[i], ref_vel);// ))
	// {
	//     yInfo() << "FingerController::goHome Error:"
	// 	    << "unable to set joints reference speeds for finger"
	// 	    << laterality_ << "-" << name_ << "(joint #" << ctl_joints_[i] << ")";

	//     return false;
	// }
    }
    // if (!(ipos_->setRefSpeeds(ctl_joints_.size(), ctl_joints_.getFirst(), speeds.data())))
    // {
    //     yInfo() << "FingerController::goHome Error:"
    //             << "unable to set joints reference speeds for finger"
    //             << laterality_ << "-" << name_;

    //     return false;
    // }

    // restore initial position of finger joints
    if (!(ipos_->positionMove(ctl_joints_.size(), ctl_joints_.getFirst(), joints_home_.data())))
    {
        yInfo() << "FingerController::goHome Error:"
                << "unable to restore initial positions of joints of finger"
                << laterality_ << "-" << name_;

        // stop movements for safety
        stop();

        return false;
    }

    return true;
}


bool FingerController::isPositionMoveDone(bool& done)
{
    bool ok;

    ok = ipos_->checkMotionDone(ctl_joints_.size(), ctl_joints_.getFirst(), &done);

    if (!ok)
    {
        yInfo() << "FingerController::isPositionMoveDone Error:"
                << "unable to get status from IPositionControl::checkMotionDone for finger"
                << laterality_ << "-" << name_;
        return false;
    }

    return true;
}


void FingerController::enforceJointsLimits(yarp::sig::Vector& vels, yarp::sig::Vector& motors_encoders)
{
    // enforce joints limits
    for (size_t i=0; i<ctl_joints_.size(); i++)
    {
        int &joint_index = ctl_joints_[i];
        double &des_limit = joints_des_limits_[i];
        if (motors_encoders[joint_index] > des_limit)
            vels[i] = 0.0;
    }
}


void FingerController::enforceJointsMaxLimits(yarp::sig::Vector& vels, yarp::sig::Vector& motors_encoders)
{
    // enforce joints limits
    for (size_t i=0; i<ctl_joints_.size(); i++)
    {
        int &joint_index = ctl_joints_[i];
        double &max_limit = joints_max_limits_[i];
        if (motors_encoders[joint_index] > max_limit)
            vels[i] = 0.0;
    }
}


bool FingerController::setJointsVelocities(const yarp::sig::Vector& vels, const bool& enforce_joints_limits)
{
    bool ok;

    // local copy of velocities
    yarp::sig::Vector velocities = vels;

    // switch to velocity control
    if (control_mode_ != VOCAB_CM_VELOCITY)
    {
        if (!setControlMode(VOCAB_CM_VELOCITY))
        {
            yInfo() << "FingerController::setJointsVelocites Error:"
                    << "unable to set Velocity control mode for finger"
                    << laterality_ << "-" << name_;

            return false;
        }
    }

    // if (name_ == "thumb")
    //   {
    // 	yInfo() << motors_encoders_[ctl_joints_[1]];
    //   }

    // enforce joints desired limits limits if required
    if (enforce_joints_limits)
        enforceJointsLimits(velocities, motors_encoders_);

    // always enforce joints max limits
    enforceJointsMaxLimits(velocities, motors_encoders_);

    // issue velocity command
    return ivel_->velocityMove(ctl_joints_.size(), ctl_joints_.getFirst(), velocities.data());
}


bool FingerController::setJointsRelativePosition(const yarp::sig::Vector& deltas, const yarp::sig::Vector& vels)
{
    // switch to position control
    // if ((
    setControlMode(VOCAB_CM_POSITION);// ))
    // {
    //     yInfo() << "FingerController::setJointsRelativePosition Error:"
    //             << "unable to set Position control mode for finger"
    //             << laterality_ << "-" << name_;

    //     return false;
    // }

    // set reference joints velocities
    // the same velocity is used for all the joints
    for (std::size_t i = 0; i < ctl_joints_.size(); i++)
    {
        if (!(ipos_->setRefSpeed(ctl_joints_[i], vels[i])))
	{
	    yInfo() << "FingerController::goHome Error:"
		    << "unable to set joints reference speeds for finger"
		    << laterality_ << "-" << name_ << "(joint #" << ctl_joints_[i] << ")";

	    return false;
	}
    }
    // if (!(ipos_->setRefSpeeds(ctl_joints_.size(), ctl_joints_.getFirst(), vels.data())))
    // {
    //     yInfo() << "FingerController::setJointsRelativePosition Error:"
    //             << "unable to set joints reference speeds for finger"
    //             << laterality_ << "-" << name_;

    //     return false;
    // }

    // evaluate final configuration
    yarp::sig::Vector final_configuration(ctl_joints_.size());
    for (std::size_t i = 0; i < ctl_joints_.size(); i++)
        final_configuration(i) = motors_encoders_(ctl_joints_[i]) + deltas[i];

    if (!(ipos_->positionMove(ctl_joints_.size(), ctl_joints_.getFirst(), final_configuration.data())))
    {
        yInfo() << "FingerController::setJointsRelativePosition Error:"
                << "unable to restore initial positions of joints of finger"
                << laterality_ << "-" << name_;

        // stop movements for safety
        stop();

        return false;
    }

    return true;
}


bool FingerController::stop()
{
    return ivel_->stop(ctl_joints_.size(), ctl_joints_.getFirst());
}


bool FingerController::switchToPositionControl()
{
    if (!(setControlMode(VOCAB_CM_POSITION)))
    {
        yError() << "FingerController:switchToPositionControl"
                 << "Error: unable to set the position control"
                 << "mode for the joints of the"
                 << laterality_ << name_
                 << "finger";

        return false;
    }

    return true;
}


void FingerController::updateEncoders(const yarp::sig::Vector& encoders)
{
    // store motor encoders that are required to enforce joints limits
    motors_encoders_ = encoders;
}


int FingerController::getNumberJoints()
{
    return ctl_joints_.size();
}


yarp::sig::Vector FingerController::loadVectorDouble
(
    ResourceFinder& rf,
    const std::string key,
    const std::size_t size
)
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
