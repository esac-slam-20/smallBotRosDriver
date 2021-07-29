#include "serial_protocol.h"
#include "timerAndColor/color.h"
using namespace smallBot;
int main(int argc, char *argv[])
{
    if (argc != 3)
    {
        RED_INFO(true, "arg error.usage: ./control_serial /port baud_rate\n");
        exit(-1);
    }
    int key;
    serial_protocol sp(argv[1], std::atoi(argv[2]));

    do
    {
        GREEN_INFO(false,
                   "\
0.wait receive\n\
1.set speed [sp1 sp2 sp3 sp4]\n\
2.set encode [tick]\n\
3.set pid [p i d]\n\
4.set save\n\
5.set ignroe\n\
6.quit\n");
        std::cin >> key;
        if (key == 0)
        {
            GREEN_INFO(false, "receive:");
            uint8_t buffer[1024];
            size_t len;
            sp.read_oneFrame(buffer, len);
            for (int i = 0; i < len; i++)
                YELLOW_INFO(false, std::hex << int(buffer[i]) << " ");
            std::cout << std::endl;
            if (sp.recive_type(buffer) == serial_protocol::CMD::get_odom)
            {
                GREEN_INFO(true, "get odom data");
                int32_t o1, o2, o3, o4;
                sp.analysis_encodes(buffer, o1, o2, o3, o4);
                GREEN_INFO(true, o1 << " " << o2 << " " << o3 << " " << o4 << std::endl);
            }
        }
        else if (key == 1)
        {
            uint16_t sp1, sp2, sp3, sp4;
            std::cin >> sp1 >> sp2 >> sp3 >> sp4;
            sp.send_speed(sp1, sp2, sp3, sp4);
            GREEN_INFO(false, "set speed " << sp1 << " " << sp2 << " " << sp3 << " " << sp4 << std::endl);
        }
        else if (key == 2)
        {
            uint16_t tick;
            std::cin >> tick;
            sp.send_encoder_tick(tick);
            GREEN_INFO(false, "set encode " << tick << std::endl);
        }
        else if (key == 3)
        {
            float p, i, d;
            std::cin >> p >> i >> d;
            sp.send_pid(p, i, d);
            GREEN_INFO(false, "set pid " << p << " " << i << " " << d << std::endl);
        }
        else if (key == 4)
        {
            sp.send_save();
            GREEN_INFO(false, "set save" << std::endl);
        }
        else if (key == 5)
        {
            sp.send_ignore();
            GREEN_INFO(false, "set ignore" << std::endl);
        }

    } while (key != 6);
    GREEN_INFO(true, "BYE BYE\n");
    return 0;
}