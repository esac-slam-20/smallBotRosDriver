#include "driver_odom.h"

namespace smallBot
{
    driver_odom::driver_odom(std::shared_ptr<serial_protocol> &sp_ptr,
                             const int rate,
                             const float &reductionRate,
                             const float &L,
                             const float &wheelR,
                             const float &tpr,
                             const std::string &odom_name,
                             const std::string &_odom_frame,
                             const std::string &_odom_child_frame,
                             const bool &_sendMsg,
                             const bool &_sendTF) : sp_ptr(sp_ptr), odom_frame(_odom_frame),
                                                    odom_child_frame(_odom_child_frame),
                                                    sendMsg(_sendMsg), sendTF(_sendTF),
                                                    reductionRate(reductionRate), L(L),
                                                    wheelR(wheelR), hasInit(false), tpr(tpr),
                                                    timer("time between two odom frame ")
    {
        R = Eigen::AngleAxisf::Identity();
        t = Eigen::Vector3f::Zero();
        vt = Eigen::Vector3f::Zero();
        vw = 0;
        if (sendMsg)
            odom_pub = n.advertise<nav_msgs::Odometry>(odom_name, 10, true);

        magicMatrix << cosf(-90.0 / 180.0 * M_PI), sinf(-90.0 / 180.0 * M_PI), L, //1
            cosf(30.0 / 180.0 * M_PI), sinf(30.0 / 180.0 * M_PI), L,              //2
            cosf(150.0 / 180.0 * M_PI), sinf(150.0 / 180.0 * M_PI), L;            //3
        //auto _magicMatrix = magicMatrix.inverse();
        //magicMatrix = _magicMatrix;
        svdMatrix = magicMatrix.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        sp_ptr->setCallback(serial_protocol::CMD::set_odom, std::bind(&driver_odom::callback, this, std::placeholders::_1));
        runHandle = std::thread(std::bind(&driver_odom::run, this, rate));
    }
    driver_odom::~driver_odom()
    {
        runHandle.join();
    }
    void driver_odom::run(int rate)
    {
        ros::Rate r(rate);
        while (ros::ok())
        {
            auto f = serial_protocol::get_set_odom_frame();
            sp_ptr->set_oneFrame(f);
            r.sleep();
        }
    }

    void driver_odom::callback(const serial_protocol::frame_data &f)
    {
        DEBUG_YELLOW_INFO(false, "be called\n");
        if (f.ptr && sp_ptr->judge_frame_type(f) != serial_protocol::CMD::get_odom)
            return; //没有doom，退出
        DEBUG_YELLOW_INFO(false, "get odom\n");
        if (!hasInit) //没有初始化，初始化后退出
        {
            int32_t tmp;
            sp_ptr->get_odom(f, lo1, lo2, lo3, tmp);
            hasInit = true;
            timer.end("", true, false); //刷新时间
            return;
        }
        DEBUG_YELLOW_INFO(false, "has inited\n");

        int32_t o1, o2, o3, o4; //o4是垃圾值
        sp_ptr->get_odom(f, o1, o2, o3, o4);
        int64_t do1, do2, do3, do4;
        do1 = o1 - lo1;
        do2 = o2 - lo2;
        do3 = o3 - lo3;
        uint64_t tmc = timer.end("", false, false);
        if (tmc < 10) //两帧太近了，多积分一点再说
            return;
        mc = timer.end("", true, false);
        do
        {
            if (std::abs(do1) > INT32_MAX / 2.0 ||
                std::abs(do2) > INT32_MAX / 2.0 ||
                std::abs(do3) > INT32_MAX / 2.0)
                break;                                //如果两者的差值大于int32的一半，那说明越界了，直接放弃这一个值
            DEBUG_YELLOW_INFO(false, "begin core\n"); //核心功能开始了

            Eigen::Vector3f vel = svdMatrix.solve(Eigen::Vector3f(do1, do2, do3));

            //Eigen::Vector3f vel = magicMatrix * Eigen::Vector3f(do1, do2, do3);
            //假设tpr tick一圈
            vel = vel * M_PI * 2 * wheelR * reductionRate / tpr;

            vt = Eigen::Vector3f(vel(0), vel(1), 0) * 1000 / mc;
            vw = vel(2) * 1000 / mc;
            Eigen::AngleAxisf tmpR(R * Eigen::AngleAxisf(vel(2), Eigen::Vector3f(0, 0, 1)));
            Eigen::Vector3f tmpt = R * Eigen::Vector3f(vel(0), vel(1), 0) + t;
            R = tmpR;
            t = tmpt;

            if (sendMsg)
                publishOdom();
            if (sendTF)
                broadcastTF();
        } while (0);
        lo1 = o1;
        lo2 = o2;
        lo3 = o3;
    }

    void driver_odom::publishOdom()
    {
        nav_msgs::Odometry odomMsg;
        odomMsg.child_frame_id = odom_child_frame;
        odomMsg.header.frame_id = odom_frame;
        odomMsg.header.stamp = ros::Time::now();
        odomMsg.pose.pose.position.x = t(0);
        odomMsg.pose.pose.position.y = t(1);
        odomMsg.pose.pose.position.z = t(2);
        Eigen::Quaternionf R_q(R);
        odomMsg.pose.pose.orientation.x = R_q.x();
        odomMsg.pose.pose.orientation.y = R_q.y();
        odomMsg.pose.pose.orientation.z = R_q.z();
        odomMsg.pose.pose.orientation.w = R_q.w();

        odomMsg.twist.twist.angular.z = vw;
        odomMsg.twist.twist.angular.y = 0;
        odomMsg.twist.twist.angular.x = 0;

        odomMsg.twist.twist.linear.x = vt(0);
        odomMsg.twist.twist.linear.y = vt(1);
        odomMsg.twist.twist.linear.z = 0;

        //速度缺省
        odom_pub.publish(odomMsg);
    }
    void driver_odom::broadcastTF()
    {
        geometry_msgs::TransformStamped tfStamp;
        tfStamp.header.frame_id = odom_frame;
        tfStamp.header.stamp = ros::Time::now();
        tfStamp.child_frame_id = odom_child_frame;
        geometry_msgs::Transform tfFrame;
        Eigen::Quaternionf R_q(R);
        tfFrame.rotation.w = R_q.w();
        tfFrame.rotation.x = R_q.x();
        tfFrame.rotation.y = R_q.y();
        tfFrame.rotation.z = R_q.z();
        tfFrame.translation.x = t(0);
        tfFrame.translation.y = t(1);
        tfFrame.translation.z = t(2);
        tfStamp.transform = tfFrame;
        tf_br.sendTransform(tfStamp);
    }

}