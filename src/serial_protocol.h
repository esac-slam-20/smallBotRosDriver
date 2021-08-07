/**
 * @file serial_protocol.h
 * @author littledang (857551494@qq.com)
 * @brief 上位机与小车的通信协议的底层实现，差不多是一个工具类
 * @version 0.1
 * @date 2021-08-03
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#pragma once
#include <atomic>
#include <boost/asio.hpp>
#include <condition_variable>

#include <memory>
#include <mutex>
#include <queue>
#include <thread>

#ifdef TIME
#define TIME_BLOCK(...) __VA_ARGS__
#else
#define TIME_BLOCK(...)
#endif

#ifdef DEBUG
#define DEBUG_YELLOW_INFO(B, ...) YELLOW_INFO(B, __VA_ARGS__)
#define DEBUG_BLOCK(...) __VA_ARGS__
#else
#define DEBUG_YELLOW_INFO(B, ...)
#define DEBUG_BLOCK(...)
#endif
namespace smallBot
{
    class serial_protocol
    {
    public:
        /**
     * @brief 用来获取对应位置的bit，借助这个东西主要原因是为了兼容大小端机
     * 
     * @tparam i 0开始，越小表示越低，一个表示8位
     * @tparam T 自动推导即可
     * @param s 输入
     * @return uint8_t 输出
     */
        template <int i, typename T>
        static uint8_t get_bit(T s)
        {
            return ((*(int *)(&s)) & (0xff << i * 8)) >> (i * 8);
        }
        /**
         * @brief 用来设置对应位置的bit，借助这个东西主要原因是为了兼容大小端机
         * 
         * @tparam i 0开始，越小表示越低，一个表示8位
         * @tparam T 自动推导即可
         * @param ch 输入
         * @param dst 输出
         */

        template <int i, typename T>
        static void set_bit(uint8_t ch, T &dst)
        {
            dst |= (0xff << i * 8);
            dst &= (T(ch) << i * 8) | (~(0xff << i * 8));
        };
        /**
         * @brief 一堆协议的常量
         * 
         */
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
        /**
         * @brief 速度相关的一些东西
         * 
         */
        struct speed
        {
            inline static const int16_t nothing = UINT16_MAX;
            inline static const int16_t stop = 0x00;
        };
        /**
         * @brief 自己定义的帧，为了方便使用
         * 
         */
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
        /**
         * @brief Construct a new serial protocol object
         * 
         * @param port 端口
         * @param baud_rate 比特率
         * @param timeout_millseconds 最大等待时间
         * @param qs 最大队列长度
         */
        serial_protocol(
            const std::string &port,
            const uint &baud_rate,
            const uint32_t &timeout_millseconds,
            const std::size_t &rqs,
            const std::size_t &sqs);

        /**
         * @brief 设置回调函数，接收到一帧数据的时候会自动执行一次，目前没有用队列来实现，意味着只能设置一个，新的来了会替代掉旧的，之后有需要再拓展
         * 
         * @param cb 
         */
        void setCallback(std::function<void(const frame_data &)> cb);

        ~serial_protocol();

        /**
         * @brief Get the oneFrame object
         * 
         * @return frame_data 如果失败的话，fram里面的ptr是空的
         */
        frame_data get_oneFrame();
        /**
         * @brief 将一帧数据送到写队列，不一定会立刻发出去
         * 
         * @param frame 
         */
        void set_oneFrame(const frame_data &frame);

        /**
         * @brief Get the set speed frame object
         * 
         * @param sp1 
         * @param sp2 
         * @param sp3 
         * @param sp4 
         * @return frame_data 
         */
        frame_data get_set_speed_frame(int16_t sp1 = speed::nothing,
                                       int16_t sp2 = speed::nothing,
                                       int16_t sp3 = speed::nothing,
                                       int16_t sp4 = speed::nothing);
        /**
         * @brief Get the send encoder tick frame object
         * 
         * @param tick 
         * @return frame_data 
         */
        frame_data get_send_encoder_tick_frame(uint16_t tick);
        /**
         * @brief Get the send pid frame object
         * 
         * @param p 
         * @param i 
         * @param d 
         * @return frame_data 
         */
        frame_data get_send_pid_frame(float p, float i, float d);
        /**
         * @brief Get the send save frame object
         * 
         * @return frame_data 
         */
        frame_data get_send_save_frame();
        /**
         * @brief Get the send ignore frame object
         * 
         * @return frame_data 
         */
        frame_data get_send_ignore_frame();

        /**
         * @brief 判断fram的类型
         * 
         * @param f 
         * @return uint8_t serial_protocol::CMD::
         */
        uint8_t judge_frame_type(const frame_data &f);

        /**
         * @brief 将frame解析成odom 的raw数据
         * 
         * @param f 
         * @param o1 
         * @param o2 
         * @param o3 
         * @param o4 
         */
        void get_odom(const frame_data &f, int32_t &o1, int32_t &o2,
                      int32_t &o3, int32_t &o4);

    private:
        boost::asio::io_service ioserv; //这个要先初始化，写在前面
        boost::asio::serial_port serial;
        int16_t lsp1, lsp2, lsp3, lsp4;
        uint32_t timeout_millseconds;

        uint64_t frame_id; //接收到的帧的id,64位必不可能溢出,错误的帧不包括

        std::mutex receive_qLock; //接收队列锁
        std::mutex send_qLock;    //发送队列锁
        std::mutex timeout_Lock;  //超时锁
        //std::mutex async_read_lock; //异步读锁

        std::queue<frame_data> receive_q; //接收到的数据
        std::queue<frame_data> send_q;    //存储待发送的数据

        std::atomic_bool quitFlag;
        std::thread receive_thread_handle;
        std::thread sends_thread_handle;
        std::thread call_me_thread_handle;

        std::condition_variable send_cv; //这个是队列空和不空的时候用的
        //std::condition_variable read_cv;    //异步读条件变量
        std::condition_variable timeout_cv; //超时用的
        std::atomic_uint64_t lastest_ack_id;

        std::size_t r_q_size;
        std::size_t s_q_size;
        void receive_thread();
        void send_thread();
        void call_me_thread();

        //回调函数
        bool hasCallback;
        std::function<void(const frame_data &)> receive_callback;
    };
}
