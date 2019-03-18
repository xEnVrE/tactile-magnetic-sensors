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
}
