/**
 * @file driver_node.cpp
 * @author littledang (857551494@qq.com)
 * @brief 驱动的节点
 * @version 0.1
 * @date 2021-08-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "driver_base.h"
#include "driver_batt.h"
#include "driver_odom.h"
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

    double tpr;
    std::string odom_name;
    std::string odom_frame;
    std::string child_frame;

    std::string batt_raw_name;
    std::string batt_percent_name;
    std::string batt_table_path;

    bool sendTF;
    bool sendOdom;

    int baud_rate, timeout_millseconds, sqs;
    double odom_rate, batt_rate;
    nh.param<std::string>("port", port_name, "/dev/ttyAMA0");
    nh.param<std::string>("cmd", cmd_name, "/cmd_vel");
    nh.param<int>("baud_rate", baud_rate, 115200);
    nh.param<int>("timeout_millseconds", timeout_millseconds, 5);
    nh.param<int>("send_queue_size", sqs, 100);

    nh.param<double>("odom_rate", odom_rate, 30);

    nh.param<double>("reductionRate", reductionRate, 1.0 / 30.0);
    nh.param<double>("L", L, 0.105);
    nh.param<double>("wheelR", wheelR, 0.029); //2.9cm

    nh.param<double>("tpr", tpr, 334);
    nh.param<std::string>("odom_name", odom_name, "/odom");
    nh.param<std::string>("odom_frame", odom_frame, "/odom");
    nh.param<std::string>("child_frame", child_frame, "/base_footprint");

    nh.param<bool>("sendTF", sendTF, true);
    nh.param<bool>("sendOdom", sendOdom, true);

    nh.param<std::string>("batt_raw_name", batt_raw_name, "batt_raw");
    nh.param<std::string>("batt_percent_name", batt_percent_name, "batt_percent");
    nh.param<std::string>("batt_table_path", batt_table_path, "");
    nh.param<double>("batt_rate", batt_rate, 30);

    std::shared_ptr<smallBot::serial_protocol>
        sp_ptr = std::make_shared<smallBot::serial_protocol>(
            port_name, baud_rate, timeout_millseconds, sqs);
    GREEN_INFO(true, "Control smallBot via " << port_name << "." << std::endl);

    smallBot::driver_base dr_base(sp_ptr, cmd_name, reductionRate, L, wheelR);                 //会自动注册到回调函数里面，所以这样就好了
    smallBot::driver_odom dr_odom(sp_ptr, odom_rate, reductionRate, L, wheelR, tpr, odom_name, //会自动注册到sp_ptr里面，所以这样就好了
                                  odom_frame, child_frame, sendOdom, sendTF);
    smallBot::driver_batt dr_batt(sp_ptr, batt_table_path, batt_percent_name, batt_raw_name, batt_rate);
    ros::spin(); //等回调
    return 0;
}