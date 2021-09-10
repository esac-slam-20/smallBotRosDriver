/**
 * @file driver_batt.h
 * @author littledang (857551494@qq.com)
 * @brief 主要用于接收下位机batt数据，并解析到ros话题上。
 * @version 0.1
 * @date 2021-09-10
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include "ros/node_handle.h"
#include "serial_protocol.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int32.h"
#include "timerAndColor/timer.h"
#include <fstream>
#include <sstream>
#include <thread>
namespace smallBot
{
    class driver_batt
    {

    public:
        driver_batt(std::shared_ptr<serial_protocol> &sp_ptr,
                    const std::string &batt_table_path,
                    const std::string &batt_percent_name,
                    const std::string &batt_raw_name,
                    const float &rate);
        ~driver_batt();

    private:
        std::shared_ptr<serial_protocol> sp_ptr; //其实所有的协议都是通过这个来处理的
        ros::NodeHandle n;

        ros::Publisher batt_raw_pub; //发布者
        ros::Publisher batt_per_pub; //发布者

        const std::string batt_table_path;
        const std::string batt_percent_name;
        const std::string batt_raw_name;

        float rate;
        //百分比% 电压V
        std::vector<std::pair<float, float>> vTable;

        void callback(const serial_protocol::frame_data &f);
        void run(float rate);
        std::thread runHandle;

        void loadTable(const std::string &batt_table_path);
        float getPercent(const float &v);
    };
}