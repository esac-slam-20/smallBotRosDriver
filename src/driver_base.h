/**
 * @file driver_base.h
 * @author littledang (857551494@qq.com)
 * @brief 主要用于解析ros 的 cmd指令，并发送到下位机
 * @version 0.1
 * @date 2021-08-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include "ros/node_handle.h"
#include "serial_protocol.h"
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <memory>
namespace smallBot
{
    class driver_base
    {

    public:
        /**
         * @brief Construct a new driver base object
         * 
         * @param sp_ptr 串口指针，内部会调用它
         * @param cmdName cmd topic name
         * @param reductionRate 减速比
         * @param L 车子半径
         * @param wheelR 轮子半径
         */
        driver_base(std::shared_ptr<serial_protocol> &sp_ptr,
                    const std::string &cmdName,
                    const float &reductionRate,
                    const float &L,
                    const float &wheelR);

    private:
        std::shared_ptr<serial_protocol> sp_ptr; //其实所有的协议都是通过这个来处理的
        ros::NodeHandle n;
        ros::Subscriber cmd_sub;
        float reductionRate; //减速比 例如1:30 要写成 1/30
        float L;             //轮子中心到圆心的距离
        float wheelR;        //轮子半径 注意是半径

        Eigen::Matrix3f magicMatrix; //核心矩阵 是个常量矩阵

        /**
         * @brief 回调函数，接收到msg会自动调用它，它会向下位机发一帧数据
         * 
         * @param msg 
         */
        void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void solveCmd(const geometry_msgs::Twist::ConstPtr &msg,
                      int16_t &w1, int16_t &w2, int16_t &w3);
    };
}
