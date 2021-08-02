#include "driver_base.h"
namespace smallBot
{

    void driver_base::solveCmd(const geometry_msgs::Twist::ConstPtr &msg,
                               int16_t &w1, int16_t &w2, int16_t &w3)
    {
        //core
        float vx = msg->linear.x;
        float vy = msg->linear.y;
        float w = msg->angular.z;
        Eigen::Vector3f result = magicMatrix * Eigen::Vector3f(vx, vy, w);
        result = result * 60.0 / (M_PI * 2 * wheelR * reductionRate);
        w1 = result(0);
        w2 = result(1);
        w3 = result(2);
        //改了改了，已经改了
    }

    driver_base::driver_base(std::shared_ptr<serial_protocol> &sp_ptr,
                             const std::string &cmdName,
                             const float &reductionRate,
                             const float &L,
                             const float &wheelR) : sp_ptr(sp_ptr), reductionRate(reductionRate), L(L), wheelR(wheelR)
    {
        magicMatrix << cosf(-90.0 / 180.0 * M_PI), sinf(-90.0 / 180.0 * M_PI), L; //1
        cosf(30.0 / 180.0 * M_PI), sinf(30.0 / 180.0 * M_PI), L,                  //2
            cosf(150.0 / 180.0 * M_PI), sinf(150.0 / 180.0 * M_PI), L,            //3
            cmd_sub = n.subscribe<geometry_msgs::Twist>(cmdName,
                                                        1, std::bind(&driver_base::cmdCallback, this, std::placeholders::_1));
    }
    void driver_base::cmdCallback(const geometry_msgs::Twist::ConstPtr &msg) //收到一次，发一次
    {
        int16_t w1, w2, w3;
        solveCmd(msg, w1, w2, w3);
        serial_protocol::frame_data f = sp_ptr->get_set_speed_frame(w1, w2, w3);
        sp_ptr->set_oneFrame(f);
    }

}