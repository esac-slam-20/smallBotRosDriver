#pragma once
#include <atomic>
#include <boost/asio.hpp>
#include <condition_variable>

#include <memory>
#include <mutex>
#include <queue>
#include <thread>
namespace smallBot
{
    class serial_protocol
    {
    public:
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
            inline static const uint8_t get_odom = 0x80;

            //包头包尾巴
            inline static const uint8_t head = 0x55;
            inline static const uint8_t tail = 0xAA;
        };
        struct speed
        {
            inline static const int16_t nothing = UINT16_MAX;
            inline static const int16_t stop = 0x00;
        };
        struct frame_data
        {
            std::shared_ptr<uint8_t[]> ptr;
            std::size_t len;
            uint64_t frame_id;
            frame_data(std::shared_ptr<uint8_t[]> &_ptr,
                       std::size_t _len, uint64_t _frame_id) : ptr(_ptr), len(_len), frame_id(_frame_id)
            {
            }
            frame_data() : ptr(nullptr)
            {
            }
        };

#ifdef DEBUG
        serial_protocol()
        {
        }
#else
        serial_protocol(const std::string &port,
                        const uint &baud_rate,
                        const uint32_t &timeout_millseconds);
#endif
        ~serial_protocol();

        //成功有内容，失败没有内容
        frame_data get_oneFrame();
        void set_oneFrame(const frame_data &frame);

        frame_data get_set_speed_frame(int16_t sp1 = speed::nothing,
                                       int16_t sp2 = speed::nothing,
                                       int16_t sp3 = speed::nothing,
                                       int16_t sp4 = speed::nothing);
        frame_data get_send_encoder_tick_frame(uint16_t tick);
        frame_data get_send_pid_frame(float p, float i, float d);
        frame_data get_send_save_frame();
        frame_data get_send_ignore_frame();

        uint8_t judge_frame_type(const frame_data &f);
        void get_odom(const frame_data &f, int32_t &o1, int32_t &o2,
                      int32_t &o3, int32_t &o4);

    private:
#ifndef DEBUG
        boost::asio::io_service ioserv; //这个要先初始化，写在前面
        boost::asio::serial_port serial;
#endif
        int16_t lsp1, lsp2, lsp3, lsp4;
        uint32_t timeout_millseconds;

        uint64_t frame_id; //接收到的帧的id,64位必不可能溢出,错误的帧不包括

        std::mutex receive_qLock; //接收队列锁
        std::mutex send_qLock;    //发送队列锁
        std::mutex timeout_Lock;  //超时锁

        std::queue<frame_data> receive_q; //接收到的数据
        std::queue<frame_data> send_q;    //存储待发送的数据

        std::atomic_bool quitFlag;
        std::thread receive_thread_handle;
        std::thread sends_thread_handle;
        std::thread call_me_thread_handle;

        std::condition_variable send_cv;    //这个是队列空和不空的时候用的
        std::condition_variable timeout_cv; //超时用的
        std::atomic_uint64_t lastest_ack_id;
        void receive_thread();
        void send_thread();
        void call_me_thread();
    };
}
