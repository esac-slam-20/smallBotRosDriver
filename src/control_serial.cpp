/**
 * @file control_serial.cpp
 * @author littledang (857551494@qq.com)
 * @brief 类似串口调试助手？实现协议的所有功能
 * @version 0.1
 * @date 2021-08-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "serial_protocol.h"
#include "timerAndColor/color.h"
using namespace smallBot;
void getOdom(const smallBot::serial_protocol::frame_data &f)
{
    YELLOW_INFO(true, "[" << f.timeStamp << "] ODOM:");
    for (int i = 0; i < f.len; i++)
        YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
    std::cout << std::dec << std::endl;
    int32_t o1, o2, o3, o4;
    serial_protocol::get_odom(f, o1, o2, o3, o4);
    YELLOW_INFO(false, "analysis odom:"
                           << o1 << " " << o2 << " " << o3 << " " << o4 << std::endl);
}
int main(int argc, char *argv[])
{
    if (argc != 3 && argc != 1)
    {
        RED_INFO(true, "arg error.usage: ./control_serial /port baud_rate\ndefault /dev/ttyAMA0 115200");
        exit(-1);
    }
    int key;
    std::string port = "/dev/ttyAMA0";
    int baud_rate = 115200;
    if (argc == 3)
    {
        port = argv[1];
        baud_rate = std::atoi(argv[2]);
    }
    serial_protocol sp(port, baud_rate, 5, 1);
    sp.setCallback(serial_protocol::CMD::set_odom, getOdom);
    sp.setCallback(serial_protocol::CMD::set_batt, [=](const smallBot::serial_protocol::frame_data &f) {
        YELLOW_INFO(true, "[" << f.timeStamp << "] BATT:");
        for (int i = 0; i < f.len; i++)
            YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
        std::cout << std::dec << std::endl;
        uint16_t batt;
        serial_protocol::get_batt(f, batt);
        YELLOW_INFO(false, "analysis batt:"
                               << batt << std::endl);
    });
    do
    {
        GREEN_INFO(true,
                   "=========================\n\
0.set odom\n\
1.set speed [sp1 sp2 sp3 sp4]\n\
2.set encode [tick]\n\
3.set pid [p i d]\n\
4.set save\n\
5.set ignore\n\
6.set batt\n\
-1.quit\n\
=========================\n");
        std::cin >> key;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (key == 0)
        {
            GREEN_INFO(false, "odom " << std::endl);
            auto f = sp.get_set_odom_frame();
            for (int i = 0; i < f.len; i++)
                YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
            std::cout << std::dec << std::endl;
            sp.set_oneFrame(f);
        }
        else if (key == 1)
        {
            int16_t sp1, sp2, sp3, sp4;
            std::cin >> sp1 >> sp2 >> sp3 >> sp4;
            GREEN_INFO(false, "set speed " << sp1 << " " << sp2 << " " << sp3 << " " << sp4 << std::endl);
            auto f = sp.get_set_speed_frame(sp1, sp2, sp3, sp4);
            for (int i = 0; i < f.len; i++)
                YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
            std::cout << std::dec << std::endl;
            sp.set_oneFrame(f);
        }
        else if (key == 2)
        {
            uint16_t tick;
            std::cin >> tick;
            GREEN_INFO(false, "set encode " << tick << std::endl);
            auto f = sp.get_send_encoder_tick_frame(tick);
            for (int i = 0; i < f.len; i++)
                YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
            std::cout << std::dec << std::endl;
            sp.set_oneFrame(f);
        }
        else if (key == 3)
        {
            float p, i, d;
            std::cin >> p >> i >> d;
            GREEN_INFO(false, "set pid " << p << " " << i << " " << d << std::endl);
            auto f = sp.get_send_pid_frame(p, i, d);
            for (int i = 0; i < f.len; i++)
                YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
            std::cout << std::dec << std::endl;
            sp.set_oneFrame(f);
        }
        else if (key == 4)
        {
            GREEN_INFO(false, "set save" << std::endl);
            auto f = sp.get_send_save_frame();
            for (int i = 0; i < f.len; i++)
                YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
            std::cout << std::dec << std::endl;
            sp.set_oneFrame(f);
        }
        else if (key == 5)
        {
            GREEN_INFO(false, "set ignore" << std::endl);
            auto f = sp.get_send_ignore_frame();
            for (int i = 0; i < f.len; i++)
                YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
            std::cout << std::dec << std::endl;
            sp.set_oneFrame(f);
        }
        else if (key == 6)
        {
            GREEN_INFO(false, "batt " << std::endl);
            auto f = sp.get_set_batt_frame();
            for (int i = 0; i < f.len; i++)
                YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
            std::cout << std::dec << std::endl;
            sp.set_oneFrame(f);
        }
    } while (key != -1);
    GREEN_INFO(true, "BYE BYE\n");
    return 0;
}