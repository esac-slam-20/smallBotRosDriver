#include "serial_protocol.h"
#include <boost/bind.hpp>
#ifdef DEBUG
#include "timerAndColor/color.h"
#endif

#ifdef DEBUG
std::mutex d_info_mutex;
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
    const size_t BUFFER_UPPER = 512;
    DEBUG_BLOCK(
        void printHex(const serial_protocol::frame_data &f)
        {
            std::lock_guard<std::mutex> lg(d_info_mutex);
            for (int i = 0; i < f.len; ++i)
                DEBUG_YELLOW_INFO(false, std::hex << (uint32_t)f.ptr[i] << " ");
            std::cout << std::dec << std::endl;
        });
    serial_protocol::serial_protocol(const std::string &port,
                                     const uint &baud_rate, const uint32_t &timeout_millseconds,
                                     const std::size_t &qs)
        : ioserv(), serial(ioserv, port),
          lsp1(serial_protocol::speed::stop), lsp2(serial_protocol::speed::stop),
          lsp3(serial_protocol::speed::stop), lsp4(serial_protocol::speed::stop),
          timeout_millseconds(timeout_millseconds),
          frame_id(0), quitFlag(false), lastest_ack_id(0), r_q_size(qs)
    {
        serial.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port::flow_control());
        serial.set_option(boost::asio::serial_port::parity());
        serial.set_option(boost::asio::serial_port::stop_bits());
        serial.set_option(boost::asio::serial_port::character_size(8));

        //开启两个线程，一个用来发送，一个用来接收
        receive_thread_handle = std::thread(std::bind(&serial_protocol::receive_thread, this));
        sends_thread_handle = std::thread(std::bind(&serial_protocol::send_thread, this));
    }

    uint16_t get_crc(uint8_t buffer[BUFFER_UPPER], int len)
    {
        //TODO
        return 0;
    }

    bool check_frame_complete(uint8_t buffer[BUFFER_UPPER], size_t len)
    {
        //如果len<6,直接不完整
        if (len < 6)
            return false;
        uint8_t frame_len = buffer[2];
        //长度不够一帧，不完整
        if (frame_len + 6 != len)
            return false;

        uint16_t crc = get_crc(buffer, frame_len + 3);

        if (buffer[0] != serial_protocol::CMD::head ||
            buffer[frame_len + 5] != serial_protocol::CMD::tail ||
            serial_protocol::get_bit<0>(crc) != buffer[frame_len + 3] ||
            serial_protocol::get_bit<1>(crc) != buffer[frame_len + 4])
            return false;
        return true;
    }

    void async_read_handler(
        std::mutex &m,
        std::condition_variable &cv,
        std::size_t &len,
        const boost::system::error_code &error, // Result of operation.
        std::size_t bytes_transferred           // Number of bytes read.
    )
    {
        DEBUG_YELLOW_INFO(false, "read something" << len << std::endl);
        std::lock_guard<std::mutex> lg(m);
        len += bytes_transferred;
        cv.notify_one(); //唤醒罢了
        DEBUG_YELLOW_INFO(false, "read something:now len=" << len << std::endl);
    }
    //接收线程的使命，每次接收一帧，然后存入队列
    //如果帧不完整，则一直丢掉数据
    void serial_protocol::receive_thread()
    {
        //这里的逻辑是
        //如果读到了head符号，则检测一下完整性，如果完整即存入队列
        //如果head符号不出现，就一直丢弃数据
        //这里一次读一个好处理一点，不然很麻烦
        DEBUG_BLOCK(
            {
                std::lock_guard<std::mutex> lg(d_info_mutex);
                DEBUG_YELLOW_INFO(true, "in receive_thread...\n");
            });
        while (!quitFlag)
        {
            std::shared_ptr<uint8_t[]> buffer(new uint8_t[BUFFER_UPPER]);
            std::size_t len = 0;
            {
                std::unique_lock<std::mutex> ul(async_read_lock);
                serial.async_read_some(boost::asio::buffer(buffer.get() + len, 1),
                                       std::bind(async_read_handler, std::ref(async_read_lock),
                                                 std::ref(read_cv),
                                                 std::ref(len), std::placeholders::_1, std::placeholders::_2));
                DEBUG_YELLOW_INFO(false, "head read_cv wait:now len = " << len << std::endl);
                read_cv.wait(ul);
                DEBUG_YELLOW_INFO(false, "head read_cv wake up:now len = " << len << std::endl);
            }
            if (quitFlag)
                break;
            //len += serial.read_some(boost::asio::buffer(buffer.get() + len, 1)); //1个1个读，好处理一点
            if (buffer[0] != serial_protocol::CMD::head) //第一个字节就错了，直接过了它
                continue;
            bool complete_flag = false;
            do
            {
                {
                    std::unique_lock<std::mutex> ul(async_read_lock);
                    serial.async_read_some(boost::asio::buffer(buffer.get() + len, 1),
                                           boost::bind(async_read_handler, boost::ref(async_read_lock),
                                                       boost::ref(read_cv),
                                                       boost::ref(len),
                                                       boost::asio::placeholders::error,
                                                       boost::asio::placeholders::bytes_transferred));
                    DEBUG_YELLOW_INFO(false, "body read_cv wait:now len=" << len << std::endl);
                    read_cv.wait(ul);
                    DEBUG_YELLOW_INFO(false, "body read_cv wake up:now len=" << len << std::endl);
                }
                if (quitFlag)
                    break;

                //len += serial.read_some(boost::asio::buffer(buffer.get() + len, 1));
                if (len == BUFFER_UPPER || quitFlag)
                    break;
            } while (!(complete_flag = check_frame_complete(buffer.get(), len)));
            if (!complete_flag) //不完整，丢弃这些数据
            {
                DEBUG_YELLOW_INFO(false, "check_frame_complete failed" << std::endl);
                continue;
            }
            DEBUG_YELLOW_INFO(true, "check_frame_complete successfully" << std::endl);

            //数据完整，存起来
            {
                std::lock_guard<std::mutex> lg(receive_qLock);
                if (buffer[1] == CMD::get_ack)
                {
                    lastest_ack_id = frame_id + 1;
                    std::lock_guard<std::mutex> tL(timeout_Lock);
                    timeout_cv.notify_one(); //唤醒
                }
                receive_q.push(frame_data(buffer, len, frame_id++));
                if (receive_q.size() == r_q_size)
                    receive_q.pop();
            }
        }
        DEBUG_BLOCK(
            {
                std::lock_guard<std::mutex> lg(d_info_mutex);
                DEBUG_YELLOW_INFO(true, "quit receive_thread\n");
            });
    }
    serial_protocol::frame_data serial_protocol::get_oneFrame()
    {
        std::lock_guard<std::mutex> lg(receive_qLock);
        if (receive_q.empty())
            return frame_data();
        frame_data f = receive_q.front();
        receive_q.pop();

        return f;
    }

    void serial_protocol::send_thread()
    {
        DEBUG_BLOCK(
            {
                std::lock_guard<std::mutex> lg(d_info_mutex);
                DEBUG_YELLOW_INFO(true, "in send_thread...\n");
            });
        while (!quitFlag)
        {
            frame_data f;
            {
                std::unique_lock<std::mutex> ul(send_qLock);
                if (send_q.empty())
                    send_cv.wait(ul); //队列为空，则挂起
                if (quitFlag)
                    break;
                f = send_q.front();
                send_q.pop();
            }
            uint64_t id = lastest_ack_id;
            DEBUG_BLOCK(
                int retry_times = 0;);
            do
            {

                boost::asio::write(serial, boost::asio::buffer(f.ptr.get(),
                                                               f.len));
                std::unique_lock<std::mutex> tL(timeout_Lock);
                call_me_thread_handle = std::thread(std::bind(&serial_protocol::call_me_thread, this));
                call_me_thread_handle.detach();
                timeout_cv.wait(tL);
                //被唤醒之后，判断一下根据id判断是否收到了ack，收到就ok，没收到就重发
                DEBUG_BLOCK(
                    {
                        std::lock_guard<std::mutex> lg(d_info_mutex);
                        if (lastest_ack_id > id)
                            DEBUG_YELLOW_INFO(false, "receive ACK\n");
                        else
                            RED_INFO(false, "wait ACK timeout " << ++retry_times << " times,retry\n");
                    });
            } while (lastest_ack_id <= id && !quitFlag);
        }
        DEBUG_YELLOW_INFO(true, "quit send_thread\n");
    }
    void serial_protocol::set_oneFrame(const frame_data &frame)
    {
        std::lock_guard<std::mutex> lg(send_qLock);
        send_q.push(frame);
        send_cv.notify_one();
    }

    void serial_protocol::call_me_thread()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(timeout_millseconds));
        //时间到了唤醒它
        std::lock_guard<std::mutex> tL(timeout_Lock);

        timeout_cv.notify_one();
    }
    serial_protocol::~serial_protocol()
    {
        quitFlag = true;
        send_cv.notify_one();
        serial.cancel();
        read_cv.notify_one();
        if (call_me_thread_handle.joinable())
            call_me_thread_handle.join();
        if (sends_thread_handle.joinable())
            sends_thread_handle.join();
        if (receive_thread_handle.joinable())
            receive_thread_handle.join();
        ioserv.run();
    }

    serial_protocol::frame_data serial_protocol::get_set_speed_frame(int16_t sp1,
                                                                     int16_t sp2,
                                                                     int16_t sp3,
                                                                     int16_t sp4)
    {
        if (sp1 == speed::nothing)
            sp1 = lsp1;
        if (sp2 == speed::nothing)
            sp2 = lsp2;
        if (sp3 == speed::nothing)
            sp3 = lsp3;
        if (sp4 == speed::nothing)
            sp4 = lsp4;
        std::shared_ptr<uint8_t[]> buffer(new uint8_t[14]);
        buffer[0] = CMD::head;
        buffer[1] = CMD::set_speed;
        buffer[2] = 8;

        buffer[3] = get_bit<0>(sp1);
        buffer[4] = get_bit<1>(sp1);

        buffer[5] = get_bit<0>(sp2);
        buffer[6] = get_bit<1>(sp2);

        buffer[7] = get_bit<0>(sp3);
        buffer[8] = get_bit<1>(sp3);

        buffer[9] = get_bit<0>(sp4);
        buffer[10] = get_bit<1>(sp4);

        uint16_t crc = get_crc(buffer.get(), 11);
        buffer[11] = get_bit<0>(crc);
        buffer[12] = get_bit<1>(crc);
        buffer[13] = CMD::tail;

        lsp1 = sp1;
        lsp2 = sp2;
        lsp3 = sp3;
        lsp4 = sp4;
        return frame_data(buffer, 14, 0);
    }
    serial_protocol::frame_data serial_protocol::get_send_encoder_tick_frame(uint16_t tick)
    {
        std::shared_ptr<uint8_t[]> buffer(new uint8_t[8]);
        buffer[0] = CMD::head;
        buffer[1] = CMD::set_encode;
        buffer[2] = 2;

        buffer[3] = get_bit<0>(tick);
        buffer[4] = get_bit<1>(tick);

        uint16_t crc = get_crc(buffer.get(), 5);
        buffer[5] = get_bit<0>(crc);
        buffer[6] = get_bit<1>(crc);
        buffer[7] = CMD::tail;
        return frame_data(buffer, 8, 0);
    }
    serial_protocol::frame_data serial_protocol::get_send_pid_frame(
        float p, float i, float d)
    {
        std::shared_ptr<uint8_t[]> buffer(new uint8_t[18]);

        buffer[0] = CMD::head;
        buffer[1] = CMD::set_pid;
        buffer[2] = 12;

        buffer[3] = get_bit<0>(p);
        buffer[4] = get_bit<1>(p);
        buffer[5] = get_bit<2>(p);
        buffer[6] = get_bit<3>(p);

        buffer[7] = get_bit<0>(i);
        buffer[8] = get_bit<1>(i);
        buffer[9] = get_bit<2>(i);
        buffer[10] = get_bit<3>(i);

        buffer[11] = get_bit<0>(d);
        buffer[12] = get_bit<1>(d);
        buffer[13] = get_bit<2>(d);
        buffer[14] = get_bit<3>(d);

        uint16_t crc = get_crc(buffer.get(), 15);
        buffer[15] = get_bit<0>(crc);
        buffer[16] = get_bit<1>(crc);
        buffer[17] = CMD::tail;
        return frame_data(buffer, 18, 0);
    }
    serial_protocol::frame_data serial_protocol::get_send_save_frame()
    {
        std::shared_ptr<uint8_t[]> buffer(new uint8_t[6]);

        buffer[0] = CMD::head;
        buffer[1] = CMD::set_save;
        buffer[2] = 0;

        uint16_t crc = get_crc(buffer.get(), 3);
        buffer[3] = get_bit<0>(crc);
        buffer[4] = get_bit<1>(crc);
        buffer[5] = CMD::tail;
        return frame_data(buffer, 6, 0);
    }
    serial_protocol::frame_data serial_protocol::get_send_ignore_frame()
    {
        std::shared_ptr<uint8_t[]> buffer(new uint8_t[6]);

        buffer[0] = CMD::head;
        buffer[1] = CMD::set_ignore;
        buffer[2] = 0;

        uint16_t crc = get_crc(buffer.get(), 3);
        buffer[3] = get_bit<0>(crc);
        buffer[4] = get_bit<1>(crc);
        buffer[5] = CMD::tail;
        return frame_data(buffer, 6, 0);
    }
    uint8_t serial_protocol::judge_frame_type(const frame_data &f)
    {
        return f.ptr[1];
    }
    void serial_protocol::get_odom(const frame_data &f, int32_t &o1, int32_t &o2,
                                   int32_t &o3, int32_t &o4)
    {
        set_bit<0>(f.ptr[3 + 0], o1);
        set_bit<1>(f.ptr[3 + 1], o1);
        set_bit<2>(f.ptr[3 + 2], o1);
        set_bit<3>(f.ptr[3 + 3], o1);

        set_bit<0>(f.ptr[3 + 4], o2);
        set_bit<1>(f.ptr[3 + 5], o2);
        set_bit<2>(f.ptr[3 + 6], o2);
        set_bit<3>(f.ptr[3 + 7], o2);

        set_bit<0>(f.ptr[3 + 8], o3);
        set_bit<1>(f.ptr[3 + 9], o3);
        set_bit<2>(f.ptr[3 + 10], o3);
        set_bit<3>(f.ptr[3 + 11], o3);

        set_bit<0>(f.ptr[3 + 12], o4);
        set_bit<1>(f.ptr[3 + 13], o4);
        set_bit<2>(f.ptr[3 + 14], o4);
        set_bit<3>(f.ptr[3 + 15], o4);
    }
}
