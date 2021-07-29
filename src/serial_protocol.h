#pragma once
#include <boost/asio.hpp>

namespace smallBot
{
    /**
     * @brief Get the bit object i从0开始，越小越低位，一次取八位
     * 
     * @tparam i 
     * @tparam T 
     * @param s 
     * @return uint8_t 
     */
    template <int i, typename T>
    static uint8_t get_bit(T s)
    {
        return ((*(int *)(&s)) & (0xff << i * 8)) >> (i * 8);
    }
    template <int i, typename T>
    static void set_bit(uint8_t ch, T &dst)
    {
        dst |= (0xff << i * 8);
        dst &= (*(int *)(&ch)) << i * 8;
    };
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
            inline static const uint8_t set_ignore = 0x2f;
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
         * @param data 数据地址
         * @param len 数据长度
         * @return true 发送成功
         * @return false 发送失败，失败并不保证说一定不向串口写入东西，可能写一半啥的
         */
        bool write_oneFrame(const uint8_t &cmd,
                            const uint8_t *data,
                            const uint8_t &len);

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
        bool send_speed(int16_t sp1 = speed::nothing,
                        int16_t sp2 = speed::nothing,
                        int16_t sp3 = speed::nothing,
                        int16_t sp4 = speed::nothing);
        bool send_encoder_tick(uint16_t tick);
        bool send_pid(float p, float i, float d);
        bool send_save();
        bool send_ignore();

        void analysis_encodes(uint8_t *buffer, int32_t &o1, int32_t &o2,
                              int32_t &o3, int32_t &o4);

        uint8_t recive_type(uint8_t *buffer);

        uint16_t get_crc(const uint8_t &cmd, const uint8_t &len, const uint8_t *data);

    private:
        boost::asio::io_service ioserv; //这个要先初始化，写在前面
        boost::asio::serial_port serial;
        int16_t lsp1, lsp2, lsp3, lsp4;
    };
}
