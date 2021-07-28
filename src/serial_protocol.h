#pragma once
#include <boost/asio.hpp>

namespace smallBot
{
    class serial_protocol
    {
    public:
        struct CMD
        {
            //发送到的包用的
            inline static const uint8_t set_speed = 0x10;
            inline static const uint8_t set_encode = 0x20;
            inline static const uint8_t set_pid = 0x21;
            inline static const uint8_t set_save = 0x2e;
            inline static const uint8_t set_ingnore = 0x2f;
            //接收到的包用的
            inline static const uint8_t get_ack = 0x00;
            inline static const uint8_t get_nack = 0x01;
            inline static const uint8_t get_invalid_args = 0x02;
            inline static const uint8_t get_odom = 0x08;

            //包头包尾巴
            inline static const uint8_t head = 0x55;
            inline static const uint8_t tail = 0xAA;
        };
        struct speed
        {
            inline static const int16_t nothing = UINT16_MAX;
            inline static const int16_t stop = 0x00;
        };
        /**
         * @brief Construct a new serial protocol object
         * 
         * @param port 串口
         * @param baud_rate 波特率 
         */
        serial_protocol(const std::string &port, const uint &baud_rate);

        /**
         * @brief 发送一帧数据，核心实现，所有和发送相关的数据最终都会通过这个实现
         * 
         * @param cmd 数据类型，请使用smallBot::serial_protocol::CMD
         *            这里不使用enum的原因是因为不想再查一次表（enum是int）
         * @param len 数据长度
         * @param data 数据地址
         * @return true 发送成功
         * @return false 发送失败，失败并不保证说一定不向串口写入东西，可能写一半啥的
         */
        bool write_oneFrame(const uint8_t &cmd,
                            const uint8_t &len,
                            const uint8_t *data);

        /**
         * @brief 接收一帧数据，核心实现，所有和接收相关的数据最后都会通过这个实现
         *        注意，是完整的一帧数据，包括包头和包尾
         * @param buffer 缓冲区，尽可能大一点，保证不溢出,内部不再申请
         * @param len 接收到的数据的总长度
         * @return true 接收成功
         * @return false 接收失败，可能是丢包之类啥的
         */
        bool read_oneFrame(uint8_t *buffer, std::size_t &len);

        /**
         * @brief 发送数据
         * 
         * @param sp1 轮子1速度 rpm
         * @param sp2 轮子2速度 rpm
         * @param sp3 轮子3速度 rpm
         * @param sp4 轮子4速度 rpm
         * @return true 成功
         * @return false 帧错误
         */
        bool send_speed(const int16_t &sp1 = speed::nothing,
                        const int16_t &sp2 = speed::nothing,
                        const int16_t &sp3 = speed::nothing,
                        const int16_t &sp4 = speed::nothing);

        /**
         * @brief 接收编码器数据
         * 
         * @param o1 encode1 tick
         * @param o2 encode2 tick
         * @param o3 encode3 tick
         * @param o4 encode4 tick
         * @return true 成功
         * @return false 帧错误
         */
        bool recive_encodes(int32_t &o1, int32_t &o2,
                            int32_t &o3, int32_t &o4);

    private:
        boost::asio::io_service ioserv; //这个要先初始化，写在前面
        boost::asio::serial_port serial;
        int16_t lsp1, lsp2, lsp3, lsp4;
    };
}
