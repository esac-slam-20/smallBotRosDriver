#include "driver_base.h"
#include "timerAndColor/color.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "smallBot_driver");
    ros::NodeHandle nh("~");

    std::string port_name;
    std::string cmd_name;

    double reductionRate;
    double L;
    double wheelR;

    int baud_rate, timeout_millseconds, qs;
    nh.param<std::string>("port", port_name, "/dev/ttyAMA0");
    nh.param<std::string>("cmd", cmd_name, "/cmd_vel");
    nh.param<int>("baud_rate", baud_rate, 115200);
    nh.param<int>("timeout_millseconds", timeout_millseconds, 2);
    nh.param<int>("receive_queue_size", qs, 100);
    nh.param<double>("reductionRate", reductionRate, 1.0 / 30.0);
    nh.param<double>("L", L, 0.105);
    nh.param<double>("wheelR", wheelR, 0.029); //2.9cm
    std::shared_ptr<smallBot::serial_protocol>
        sp_ptr = std::make_shared<smallBot::serial_protocol>(
            port_name, baud_rate, timeout_millseconds, qs);
    GREEN_INFO(true, "Control smallBot via " << port_name << "." << std::endl);

    smallBot::driver_base dr_base(sp_ptr, cmd_name, reductionRate, L, wheelR);
    ros::spin();
    return 0;
}