#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <FingerController.h>

#include <yarp/dev/IAnalogSensor.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Port.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <thrift/TactileMagneticSensorControllerIDL.h>

#include<unordered_map>

enum class ControlMode { Close, Hold, Idle, Open, OpenWStopLog, Step, WaitOpen};


class Controller : public yarp::os::RFModule,
                   public TactileMagneticSensorControllerIDL
{
public:
    virtual ~Controller();

    /**
     * RFModule interface.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    bool updateModule() override;

    double getPeriod() override;

    bool close() override;

    /**
     * IDL interface.
     */

    std::string get_thr();

    std::string grasp() override;

    bool hold() override;

    bool open() override;

    bool open_stop_log() override;

    bool quit() override;

    bool step() override;

    bool stop() override;

    bool thr_thumb(const double threshold);

    bool thr_index(const double threshold);

    bool thr_middle(const double threshold);

    bool thr_ring(const double threshold);

protected:
    std::vector<std::string> loadListString(yarp::os::ResourceFinder& rf, const std::string key);

    yarp::sig::Vector loadVectorDouble(yarp::os::ResourceFinder& rf, const std::string key, const std::size_t size);

    bool calibrate_tactile_sensors();

    bool run_logger();

    bool stop_logger();

    /**
     * Rpc clients
     */
    yarp::os::RpcClient tactile_sensor_reader_port_;

    yarp::os::RpcClient tactile_sensor_logger_port_;

    /**
     * Port to get the signal of the tactile sensors from.
     */
    yarp::os::BufferedPort<yarp::sig::VectorOf<int>> tactile_sensors_data_in_;

    /**
     * Local copy of the last reading from the sensors
     */
    yarp::sig::VectorOf<int> tactile_readings_;

    /**
     * Local copy of the last reading from the encoders
     */
    yarp::sig::Vector encoder_readings_;

    /**
     * Remember if tactile data was received at least one time
     */
    bool is_tactile_reading_available_;

    bool use_tactile_feedback_;

    /**
     * Remember if encoders data was received at least one time
     */
    bool is_encoders_reading_available_;

    /**
     * Port to get user commands from.
     */
    yarp::os::Port port_rpc_command_;

    /**
     * Mutex for RPC communication.
     */
    yarp::os::Mutex mutex_;

    /**
     * ID to be used for logging.
     */
    const std::string log_ID_ = "[TACTILE_MAGNETIC_SENSORS_CONTROLLER]";

    /**
     * Controller sampling time in (s).
     */
    double period_;

    /**
     * Current mode of operation.
     */
    ControlMode mode_;

    /**
     * Arm driver.
     */
    yarp::dev::PolyDriver drv_arm;

    /**
     * Arm views.
     */
    yarp::dev::IEncoders* ienc_arm_;
    yarp::dev::IAnalogSensor* ianalog_arm_;
    yarp::dev::IControlMode* imod_arm_;
    yarp::dev::IControlLimits* ilimits_arm_;

    /**
     * Interfaces for joint position and velocity control.
     */
    yarp::dev::IPositionControl* ipos_arm_;
    yarp::dev::IVelocityControl* ivel_arm_;

    /**
     * Active fingers.
     */
    std::vector<std::string> active_fingers_;

    /**
     * Finger controllers.
     */
    std::unordered_map<std::string, FingerController> fingers_;

    /**
     * Finger closing velocities.
     */
    std::unordered_map<std::string, double> fingers_opening_vels_;

    /**
     * Finger closing velocities.
     */
    std::unordered_map<std::string, yarp::sig::Vector> fingers_closing_vels_;

    /**
     * Finger deltas (for adaptive grasping).
     */
    std::unordered_map<std::string, yarp::sig::Vector> fingers_deltas_;

    /**
     * Finger deltas feedforward velocity(for adaptive grasping).
     */
    std::unordered_map<std::string, yarp::sig::Vector> fingers_deltas_vels_;

    /**
     * Finger thresholds(for adaptive grasping).
     */
    std::unordered_map<std::string, double> fingers_thresholds_;

    /**
     * Name of the arm.
     */
    std::string laterality_;

    /**
     * Name of the robot.
     */
    std::string robot_;

    /**
     * Last time required for timeouts
     */
    double last_time_;

    /**
     * Timeouts
     */
    double grasp_timeout_;

    double open_timeout_;
};

#endif /* CONTROLLER_H */
