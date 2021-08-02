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
        driver_base(std::shared_ptr<serial_protocol> &sp_ptr,
                    const std::string &cmdName,
                    const float &reductionRate,
                    const float &L,
                    const float &wheelR);
        void cmdCallback(const geometry_msgs::Twist::ConstPtr &msg);
        void solveCmd(const geometry_msgs::Twist::ConstPtr &msg,
                      int16_t &w1, int16_t &w2, int16_t &w3);

    private:
        std::shared_ptr<serial_protocol> sp_ptr; //其实所有的协议都是通过这个来处理的
        ros::NodeHandle n;
        ros::Subscriber cmd_sub;
        float reductionRate; //减速比 例如1:30 要写成 1/30
        float L;             //轮子中心到圆心的距离
        float wheelR;        //轮子半径 注意是半径

        Eigen::Matrix3f magicMatrix; //核心矩阵 是个常量矩阵
    };
}
