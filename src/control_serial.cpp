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
    if (argc != 3)
    {
        RED_INFO(true, "arg error.usage: ./control_serial /port baud_rate\n");
        exit(-1);
    }
    int key;
    serial_protocol sp(argv[1], std::atoi(argv[2]), 2, 1000);

    do
    {
        GREEN_INFO(true,
                   "=========================\n\
0.wait receive.when you receive,input any integer to stop\n\
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
                    YELLOW_INFO(true, f.frame_id << ".ACK:");
                    break;
                case serial_protocol::CMD::get_nack:
                    YELLOW_INFO(true, f.frame_id << ".NACK:");
                    break;
                case serial_protocol::CMD::get_odom:
                    YELLOW_INFO(true, f.frame_id << ".ODOM:");
                    break;
                default:
                    RED_INFO(true, f.frame_id << ".UNKNOW:");
                    break;
                }
                for (int i = 0; i < f.len; i++)
                    YELLOW_INFO(false, std::hex << int(f.ptr[i]) << " ");
                std::cout << std::dec << std::endl;
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