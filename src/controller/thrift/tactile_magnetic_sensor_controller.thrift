/**
 * TactileMagneticSensorController.thrift
 *
 */

service TactileMagneticSensorControllerIDL
{
    bool grasp();

    bool hold();

    bool open();

    bool quit();

    bool step();

    bool stop();

    bool thr(1:i16 threshold_0, 2:i16 threshold_1);
}
