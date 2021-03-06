#include "serial_protocol.h"
//#include <boost/bind.hpp>

#ifdef DEBUG
std::mutex d_info_mutex;
#endif

namespace smallBot
{
    const size_t BUFFER_UPPER = 512;
    DEBUG_BLOCK(
        void printHex(const serial_protocol::frame_data &f) {
            std::lock_guard<std::mutex> lg(d_info_mutex);
            for (int i = 0; i < f.len; ++i)
                DEBUG_YELLOW_INFO(false, std::hex << (uint32_t)f.ptr[i] << " ");
            std::cout << std::dec << std::endl;
        } void printHex(uint8_t ptr[512], size_t len) {
            std::lock_guard<std::mutex> lg(d_info_mutex);
            for (int i = 0; i < len; ++i)
                DEBUG_YELLOW_INFO(false, std::hex << (uint32_t)ptr[i] << " ");
            std::cout << std::dec << std::endl;
        });
    serial_protocol::serial_protocol(const std::string &port,
                                     const uint &baud_rate, const uint32_t &timeout_millseconds,
                                     const std::size_t &sqs)
        : ioserv(), serial(ioserv, port),
          timeout_millseconds(timeout_millseconds),
          quitFlag(false), hasCallback(false), s_q_size(sqs),
          countTime(""), cancelFlag(false), uniqueId(0)
    {
        serial.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port::flow_control());
        serial.set_option(boost::asio::serial_port::parity());
        serial.set_option(boost::asio::serial_port::stop_bits());
        serial.set_option(boost::asio::serial_port::character_size(8));

        //开启两个线程，一个用来发送，一个用来接收
        sends_thread_handle = std::thread(std::bind(&serial_protocol::send_thread, this));
    }

    void serial_protocol::setCallback(uint8_t cmdType, std::function<void(const frame_data &)> cb)
    {
        std::lock_guard<std::mutex> lg(send_qLock);
        hasCallback = true;
        receive_callback[cmdType] = cb;
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
        DEBUG_BLOCK(
            printHex(buffer, len);)
        if (buffer[0] != serial_protocol::CMD::head ||
            buffer[frame_len + 5] != serial_protocol::CMD::tail ||
            serial_protocol::get_bit<0>(crc) != buffer[frame_len + 3] ||
            serial_protocol::get_bit<1>(crc) != buffer[frame_len + 4])
            return false;

        return true;
    }

    void async_read_handler(
        std::size_t &len,
        const boost::system::error_code &error, // Result of operation.
        std::size_t bytes_transferred           // Number of bytes read.
    )
    {
        len += bytes_transferred;
    }

    serial_protocol::frame_data serial_protocol::get_oneFrame()
    {

        std::shared_ptr<uint8_t[]> buffer(new uint8_t[BUFFER_UPPER]);
        buffer[0] = 0;
        std::size_t len = 0;
        bool complete_flag = false;

        do
        {
            DEBUG_YELLOW_INFO(true, len);
            serial.async_read_some(boost::asio::buffer(buffer.get() + len, 1),
                                   std::bind(async_read_handler,
                                             std::ref(len),
                                             std::placeholders::_1, std::placeholders::_2));
            ioserv.reset();
            ioserv.run();
            if (quitFlag)
                break;
            DEBUG_YELLOW_INFO(false, "i got something** " << len << " " << int(buffer[0]) << std::endl);
            if (buffer[0] != serial_protocol::CMD::head) //第一个字节就错了，直接过了它
                break;
            do
            {
                serial.async_read_some(boost::asio::buffer(buffer.get() + len, 1),
                                       std::bind(async_read_handler,
                                                 std::ref(len),
                                                 std::placeholders::_1, std::placeholders::_2));
                ioserv.reset();
                ioserv.run();

                if (quitFlag)
                    break;
                if (len == BUFFER_UPPER || quitFlag)
                    break;
                if (cancelFlag)
                    break;
            } while (!(complete_flag = check_frame_complete(buffer.get(), len)));

            if (!complete_flag) //不完整，丢弃这些数据
            {
                DEBUG_YELLOW_INFO(false, "check_frame_complete failed" << std::endl);
                break;
            }
            DEBUG_YELLOW_INFO(true, "check_frame_complete successfully" << std::endl);

            auto f = frame_data(buffer, len, countTime.end("", false, false) / 1000.0);
            return f;

        } while (0);
        //到这里就说明数据有问题了，886

        DEBUG_YELLOW_INFO(true, "FIND SOMETHING WRONG\n");
        return frame_data();
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
            DEBUG_BLOCK(
                int retry_times = 0;);

            TIME_BLOCK(
                lmicroTimer("send one frame and wait ack");)
            frame_data ack;
            do
            {
                DEBUG_BLOCK(
                    {
                        if (retry_times == 0)
                            GREEN_INFO(false, "wait ACK timeout " << retry_times++ << " times,retry\n");
                        else
                            RED_INFO(true, "wait ACK timeout " << retry_times++ << " times,retry\n");
                    })

                {
                    TIME_BLOCK(
                        lmicroTimer("write one frame");)
                    boost::asio::write(serial, boost::asio::buffer(f.ptr.get(),
                                                                   f.len));
                }
                cancelFlag = false;
                uniqueId++;
                auto cancelThread = std::thread([this]() {
                    int64_t x = uniqueId;
                    std::this_thread::sleep_for(std::chrono::milliseconds(timeout_millseconds));

                    if (uniqueId == x)
                    {
                        cancelFlag = true;
                        serial.cancel();
                    }
                });
                //std::this_thread::sleep_for(std::chrono::milliseconds(timeout_millseconds - 2));
                ack = get_oneFrame();

                if (!ack.ptr)
                {
                    cancelThread.join();
                    continue;
                }
                if (judge_frame_type(f) == CMD::set_odom)
                {
                    if (judge_frame_type(ack) == CMD::get_odom)
                    {
                        cancelThread.detach();
                        break;
                    }
                }
                else if (judge_frame_type(f) == CMD::set_batt)
                {
                    if (judge_frame_type(ack) == CMD::get_batt)
                    {
                        cancelThread.detach();
                        break;
                    }
                }
                else
                {
                    if (judge_frame_type(ack) == CMD::get_ack)
                    {
                        cancelThread.detach();
                        break;
                    }
                }
                cancelThread.join();

            } while (!quitFlag);
            if (!quitFlag)
                if (receive_callback.find(judge_frame_type(f)) != receive_callback.end())
                    receive_callback[judge_frame_type(f)](ack);
        }
        DEBUG_YELLOW_INFO(true, "quit send_thread\n");
    }
    void serial_protocol::set_oneFrame(const frame_data &frame)
    {
        std::lock_guard<std::mutex> lg(send_qLock);
        while (send_q.size() >= s_q_size)
            send_q.pop();
        send_q.push(frame);
        send_cv.notify_one();
    }

    serial_protocol::~serial_protocol()
    {
        quitFlag = true;
        send_cv.notify_one();
        serial.cancel();
        if (sends_thread_handle.joinable())
            sends_thread_handle.join();
        ioserv.run();
    }

    serial_protocol::frame_data serial_protocol::get_set_speed_frame(int16_t sp1,
                                                                     int16_t sp2,
                                                                     int16_t sp3,
                                                                     int16_t sp4)
    {
        static int16_t lsp1 = serial_protocol::speed::stop,
                       lsp2 = serial_protocol::speed::stop,
                       lsp3 = serial_protocol::speed::stop,
                       lsp4 = serial_protocol::speed::stop;

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
    serial_protocol::frame_data serial_protocol::get_set_odom_frame()
    {
        std::shared_ptr<uint8_t[]> buffer(new uint8_t[6]);
        buffer[0] = CMD::head;
        buffer[1] = CMD::set_odom;
        buffer[2] = 0;

        uint16_t crc = get_crc(buffer.get(), 3);
        buffer[3] = get_bit<0>(crc);
        buffer[4] = get_bit<1>(crc);
        buffer[5] = CMD::tail;
        return frame_data(buffer, 6, 0);
    }
    serial_protocol::frame_data serial_protocol::get_set_batt_frame()
    {
        std::shared_ptr<uint8_t[]> buffer(new uint8_t[6]);
        buffer[0] = CMD::head;
        buffer[1] = CMD::set_batt;
        buffer[2] = 0;

        uint16_t crc = get_crc(buffer.get(), 3);
        buffer[3] = get_bit<0>(crc);
        buffer[4] = get_bit<1>(crc);
        buffer[5] = CMD::tail;
        return frame_data(buffer, 6, 0);
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
    void serial_protocol::get_batt(const frame_data &f, uint16_t &batt)
    {
        set_bit<0>(f.ptr[3 + 0], batt);
        set_bit<1>(f.ptr[3 + 1], batt);
    }
}
