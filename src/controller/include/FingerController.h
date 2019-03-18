#ifndef FINGER_CONTROLLER_H
#define FINGER_CONTROLLER_H

#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/dev/api.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/math/SVD.h>
#include <yarp/os/ResourceFinder.h>

#include <string>

#include <cmath>


class FingerController
{
public:
    /*
     * Initialize the controller.
     *
     * @param hand_name the name of the hand
     * @param finger_name the name of the finger
     * @param imod pointer to a ControlMode instance
     * @param ivel pointer to a ControlLimits instance
     * @param ipos pointer to a PositionControl instance
     * @param ivel pointer to a VelocityControl instance
     * @return true/false on success/failure
     */
    bool init
    (
        const std::string& hand_name,
        const std::string& finger_name,
        yarp::dev::IControlMode* imod,
        yarp::dev::IControlLimits* ilim,
        yarp::dev::IPositionControl* ipos,
        yarp::dev::IVelocityControl* ivel
    );

    /*
     * Configure controller taking parameters from ResourceFinder.
     *
     */
    bool configure(yarp::os::ResourceFinder& rf);

    /*
     * Set a given control mode for all the joints of the finger.
     *
     * @return true/false on success/failure
     */
    bool setControlMode(const int& mode);

    /*
     * Close the controller.
     *
     * @return true/false on success/failure
     */
    bool close();

    /*
     * Set the home position of the finger joints.
     *
     * @param encoders a vector containing the encoders
     * readings of the whole arm
     */
    void setHomePosition(const yarp::sig::Vector &encoders);

    /*
     * Restore the initial position of the finger.
     *
     * @param ref_vel reference joints velocity used during movement
     * @return true/false on success/failure
     */
    bool goHome(const double& ref_vel);

    /*
     * Return the status of a movement
     * issued with the position control inteface.
     *
     * @param done the status of the movement
     * @return true/false on success/failure
     */
    bool isPositionMoveDone(bool& done);

    /*
     * Set to zero commanded velocity of joints that are above desired limit
     *
     * @param vels a yarp::sig::Vector vector containing the velocities
     */
    void enforceJointsLimits(yarp::sig::Vector& vels, yarp::sig::Vector& motors_encoders);

    /*
     * Set to zero commanded velocity of joints that are above max allowed value
     *
     * @param vels a yarp::sig::Vector vector containing the velocities
     */
    void enforceJointsMaxLimits(yarp::sig::Vector& vels, yarp::sig::Vector& motors_encoders);

    /*
     * Set the velocities of the controlled joints of the finger.
     *
     * To be used in "streaming" mode.
     *
     * @param vels a yarp::sig::Vector vector containing the desired joints velocity
     * @param enforce_joints_limits whether to enforce joints limits or not
     * @return true/false on success/failure
     */
    bool setJointsVelocities(const yarp::sig::Vector& vels, const bool& enforce_joints_limits);

    /*
     * Set the position of the controlled joints of the finger given an incremental position.
     *
     * To be used in "streaming" mode.
     */
    bool setJointsRelativePosition(const yarp::sig::Vector& deltas, const yarp::sig::Vector& vels);

    /*
     * Stop the finger.
     *
     * @return true/false on success/failure
     */
    bool stop();

    /*
     * Switch to position control.
     *
     * @return true/false on success/failure
     */
    bool switchToPositionControl();

    void updateEncoders(const yarp::sig::Vector& encoders);

    int getNumberJoints();

private:
    // name of the finger
    std::string name_;

    // number of joints
    int number_joints_;

    // name of the hand
    std::string laterality_;

    // joints values
    yarp::sig::Vector joints_;

    // list of joints to be controlled
    yarp::sig::VectorOf<int> ctl_joints_;

    // list of desired joints limits
    yarp::sig::VectorOf<double> joints_des_limits_;

    // list of min joints limits
    yarp::sig::VectorOf<double> joints_min_limits_;

    // list of max joints limits
    yarp::sig::VectorOf<double> joints_max_limits_;

    // list of control modes at startup
    yarp::sig::VectorOf<int> initial_modes_;

    // initial joints configuration
    yarp::sig::Vector joints_home_;

    // velocity control interface
    // common to all fingers
    yarp::dev::IVelocityControl* ivel_;

    // position control interface
    // common to all fingers
    yarp::dev::IPositionControl* ipos_;

    // control mode interface
    // common to all fingers
    yarp::dev::IControlMode* imod_;

    // control limits interface
    // common to all fingers
    yarp::dev::IControlLimits* ilim_;

    // current motor encoders values
    yarp::sig::Vector motors_encoders_;

    // current control mode
    int control_mode_;

    yarp::sig::Vector loadVectorDouble(yarp::os::ResourceFinder& rf, const std::string key, const std::size_t size);
};

#endif /* FINGERCONTROLLER_H */
