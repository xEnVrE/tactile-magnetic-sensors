#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <thrift/TactileMagneticSensorControllerIDL.h>


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

    bool quit() override;

protected:
    /**
     * Port to get the signal of the tactile sensors from.
     */
    yarp::os::BufferedPort<yarp::sig::VectorOf<int>> tactile_sensors_data_in_;

    /**
     * Port to get user commands from.
     */
    yarp::os::Port port_rpc_command_;

    /**
     * ID to be used for logging.
     */
    const std::string log_ID_ = "[TACTILE_MAGNETIC_SENSORS_CONTROLLER]";

    /**
     * Controller sampling time in (s).
     */
    double period_;
};

#endif /* CONTROLLER_H */
