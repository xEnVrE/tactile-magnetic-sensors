/**
 * TactileMagneticSensorController.thrift
 *
 */

service TactileMagneticSensorControllerIDL
{
    string get_thr();

    string grasp();

    bool hold();

    bool open();

    bool open_stop_log();

    bool quit();

    bool step();

    bool stop();

    bool thr_thumb(1:double threshold);

    bool thr_index(1:double threshold);

    bool thr_middle(1:double threshold);

    bool thr_ring(1:double threshold);
}
