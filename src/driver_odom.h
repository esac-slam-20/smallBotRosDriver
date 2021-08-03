#pragma once
#include "ros/node_handle.h"
#include "serial_protocol.h"
#include "timerAndColor/timer.h"
#include <Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
namespace smallBot
{
    class driver_odom
    {

    public:
        driver_odom(std::shared_ptr<serial_protocol> &sp_ptr,
                    const float &reductionRate,
                    const float &L,
                    const float &wheelR,
                    const float &tpr,
                    const std::string &odom_name,
                    const std::string &_odom_frame,
                    const std::string &_odom_child_frame,
                    const bool &_sendMsg,
                    const bool &_sendTF);
        void run(const serial_protocol::frame_data &f);

    private:
        void publishOdom();
        void broadcastTF();
        std::shared_ptr<serial_protocol> sp_ptr; //其实所有的协议都是通过这个来处理的
        ros::NodeHandle n;

        ros::Publisher odom_pub;        //发布者
        tf::TransformBroadcaster tf_br; //tf
        std::string odom_frame;
        std::string odom_child_frame;

        Eigen::AngleAxisf R;
        Eigen::Vector3f t;

        Eigen::Vector3f vt;
        float vw;

        int32_t lo1, lo2, lo3;
        bool hasInit;
        bool sendMsg;
        bool sendTF;

        float reductionRate; //减速比 例如1:30 要写成 1/30
        float L;             //轮子中心到圆心的距离
        float wheelR;        //轮子半径 注意是半径

        float tpr;                   //多少tick 一圈
        Eigen::Matrix3f magicMatrix; //核心矩阵 是个常量矩阵
        Eigen::BDCSVD<Eigen::Matrix3f> svdMatrix;

        uint64_t mc;                      //记录过了多久的
        myTimer::millTimer<>::type timer; //计时器
    };
}