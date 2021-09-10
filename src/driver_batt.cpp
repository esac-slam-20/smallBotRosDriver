#include "driver_batt.h"
namespace smallBot
{
    driver_batt::driver_batt(std::shared_ptr<serial_protocol> &sp_ptr,
                             const std::string &batt_table_path,
                             const std::string &batt_percent_name,
                             const std::string &batt_raw_name,
                             const float &rate) : sp_ptr(sp_ptr),
                                                  batt_table_path(batt_table_path),
                                                  batt_raw_name(batt_raw_name),
                                                  batt_percent_name(batt_percent_name),
                                                  rate(rate)
    {
        std::ifstream ifs(batt_table_path, std::ios::in);
        if (!ifs)
        {
            RED_INFO(true, "open " << batt_table_path << " failed.\n");
            exit(-1);
        }
        std::string line;
        while (std::getline(ifs, line))
        {
            float v;
            float p;
            std::stringstream ss(line);
            ss >> p >> v;
            vTable.push_back({v, p});
        }
        batt_raw_pub = n.advertise<std_msgs::Int32>(batt_raw_name, 1, true);
        batt_per_pub = n.advertise<std_msgs::Float32>(batt_percent_name, 1, true);
        sp_ptr->setCallback(serial_protocol::CMD::set_batt,
                            std::bind(&driver_batt::callback, this, std::placeholders::_1));
        runHandle = std::thread(std::bind(&driver_batt::run, this, rate));
        ifs.close();
    }
    driver_batt::~driver_batt()
    {
        runHandle.join();
    }
    float driver_batt::getPercent(const float &v)
    {
        std::pair<float, float> lower;
        std::pair<float, float> upper;
        for (int i = 0; i < vTable.size(); i++)
        {
            if (vTable[i].first >= v)
            {
                if (i == 0)
                    return 0;
                lower = vTable[i - 1];
                upper = vTable[i];
                break;
            }
            if (i == vTable.size() - 1) //超过最高电压了
                return 100;
        }
        //线性插值
        float deltaP = (v - lower.first) / (upper.first - lower.first) * (upper.second - lower.second);
        return lower.second + deltaP;
    }
    void driver_batt::run(float rate)
    {
        ros::Rate r(rate);
        while (ros::ok())
        {
            auto f = serial_protocol::get_set_batt_frame();
            sp_ptr->set_oneFrame(f);
            DEBUG_YELLOW_INFO(false, "get batt\n");
            r.sleep();
        }
    }
    void driver_batt::callback(const serial_protocol::frame_data &f)
    {
        uint16_t rawV;
        serial_protocol::get_batt(f, rawV);
        std_msgs::Int32 raw_msg;
        raw_msg.data = rawV;
        std_msgs::Float32 per_msg;
        float tmp = rawV / 1000.0 / 3.0;
        per_msg.data = getPercent(tmp);
        batt_raw_pub.publish(raw_msg);
        batt_per_pub.publish(per_msg);
    }

}