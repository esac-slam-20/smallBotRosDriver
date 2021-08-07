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
std::atomic_bool stop_r;
void stop_receive()
{
    stop_r = false;
    int i;
    std::cin >> i;
    stop_r = true;
}
int main(int argc, char *argv[])
{
    stop_r = false;
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
    serial_protocol sp(port, baud_rate, 1000, 100, 1);

    do
    {
        GREEN_INFO(true,
                   "=========================\n\
0.wait receive.when you receive,input any integer to stop [0 all 1 filter odom]\n\
1.set speed [sp1 sp2 sp3 sp4]\n\
2.set encode [tick]\n\
3.set pid [p i d]\n\
4.set save\n\
5.set ignore\n\
6.quit\n\
=========================\n");
        std::cin >> key;
        if (key == 0)
        {
            int mode;
            std::cin >> mode;
            GREEN_INFO(false, "receive:\n");
            stop_r = false;
            std::thread stop_handle = std::thread(stop_receive);
            stop_handle.detach();
            while (!stop_r)
            {

                auto f = sp.get_oneFrame();
                if (!f.ptr)
                    continue;
                switch (sp.judge_frame_type(f))
                {
                case serial_protocol::CMD::get_ack:
                    YELLOW_INFO(true, f.frame_id << ". [" << f.timeStamp << "] ACK:");
                    for (int i = 0; i < f.len; i++)
                        YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
                    std::cout << std::dec << std::endl;

                    break;
                case serial_protocol::CMD::get_nack:
                    YELLOW_INFO(true, f.frame_id << ". [" << f.timeStamp << "] NACK:");
                    for (int i = 0; i < f.len; i++)
                        YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
                    std::cout << std::dec << std::endl;

                    break;
                case serial_protocol::CMD::get_odom:
                    if (!mode)
                    {
                        YELLOW_INFO(true, f.frame_id << ". [" << f.timeStamp << "] ODOM:");
                        for (int i = 0; i < f.len; i++)
                            YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
                        std::cout << std::dec << std::endl;
                    }
                    break;
                default:
                    RED_INFO(true, f.frame_id << ". [" << f.timeStamp << "] UNKNOW:");
                    for (int i = 0; i < f.len; i++)
                        RED_INFO(true, std::hex << int(f.ptr[i]) << " ");
                    std::cout << std::dec << std::endl;

                    break;
                }
                if (!mode)
                    if (sp.judge_frame_type(f) == serial_protocol::CMD::get_odom)
                    {
                        int32_t o1, o2, o3, o4;
                        sp.get_odom(f, o1, o2, o3, o4);
                        YELLOW_INFO(false, "analysis odom:"
                                               << o1 << " " << o2 << " " << o3 << " " << o4 << std::endl);
                    }
            }
            if (stop_handle.joinable())
                stop_handle.join();
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

    } while (key != 6);
    GREEN_INFO(true, "BYE BYE\n");
    return 0;
}