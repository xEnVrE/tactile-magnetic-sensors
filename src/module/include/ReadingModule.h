#ifndef READINGMODULE_H
#define READINGMODULE_H

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Mutex.h>
#include <yarp/os/Port.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <skinSensor.h>

#include <thrift/TactileMagneticSensoreModuleIDL.h>


class ReadingModule : public yarp::os::RFModule,
                      public TactileMagneticSensoreModuleIDL
{
public:
    virtual ~ReadingModule();

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

    bool calibrate() override;

    bool quit() override;

protected:
    std::pair<bool, std::vector<unsigned int>> loadListUnsigned(yarp::os::ResourceFinder& rf, const std::string& key);

    std::unique_ptr<skinSensor> skin_sensor_drv_;

    yarp::os::Port port_rpc_command_;

    yarp::os::BufferedPort<yarp::sig::VectorOf<int>> port_data_out_;

    const std::string log_ID_ = "[TACTILE_MAGNETIC_SENSORS_MODULE]";

    double period_;

    yarp::os::Mutex mutex_;

    bool is_ongoing_calibration_;
};

#endif /* READINGMODULE_H */
