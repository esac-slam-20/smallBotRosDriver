#include "serial_protocol2.h"
namespace smallBot
{
    const size_t BUFFER_UPPER = 512;

    serial_protocol::serial_protocol(const std::string &port,
                                     const uint &baud_rate, const uint32_t &timeout_millseconds)
        : ioserv(), serial(ioserv, port),
          lsp1(serial_protocol::speed::stop), lsp2(serial_protocol::speed::stop),
          lsp3(serial_protocol::speed::stop), lsp4(serial_protocol::speed::stop),
          timeout_millseconds(timeout_millseconds),
          frame_id(0), quitFlag(false), lastest_ack_id(0)
    {
        serial.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port::flow_control());
        serial.set_option(boost::asio::serial_port::parity());
        serial.set_option(boost::asio::serial_port::stop_bits());
        serial.set_option(boost::asio::serial_port::character_size(8));

        receive_thread_handle = std::thread(std::bind(receive_thread, this));
        sends_thread_handle = std::thread(std::bind(send_thread, this));
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
        uint8_t frame_len = buffer[1];
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
    void serial_protocol::receive_thread()
    {
        //这里的逻辑是
        //如果读到了head符号，则检测一下完整性，如果完整即存入队列
        //如果head符号不出现，就一直丢弃数据
        //这里一次读一个好处理一点，不然很麻烦
        while (!quitFlag)
        {
            std::shared_ptr<uint8_t[]> buffer(new uint8_t[BUFFER_UPPER]);
            std::size_t len = 0;
            len += serial.read_some(boost::asio::buffer(buffer.get() + len, 1)); //1个1个读，好处理一点
            if (buffer[0] != serial_protocol::CMD::head)                         //第一个字节就错了，直接过了它
                continue;
            bool complete_flag = false;
            do
            {
                len += serial.read_some(boost::asio::buffer(buffer.get() + len, 1));
                if (len == BUFFER_UPPER)
                    break;
            } while (!(complete_flag = check_frame_complete(buffer.get(), len)));
            if (!complete_flag) //不完整，丢弃这些数据
                continue;
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
            }
        }
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
        while (!quitFlag)
        {
            frame_data f;
            {
                std::unique_lock<std::mutex> ul(send_qLock);
                if (send_q.empty())
                    send_cv.wait(ul); //队列为空，则挂起
                f = send_q.front();
                send_q.pop();
            }
            uint64_t id = lastest_ack_id;
            do
            {

                boost::asio::write(serial, boost::asio::buffer(f.ptr.get(),
                                                               f.len));
                std::unique_lock<std::mutex> tL(timeout_Lock);
                call_me_thread_handle = std::thread(std::bind(call_me_thread, this));
                timeout_cv.wait(tL);
                //被唤醒之后，判断一下根据id判断是否收到了ack，收到就ok，没收到就重发
            } while (lastest_ack_id <= id);
        }
    }
    void serial_protocol::set_oneFrame(const frame_data &frame)
    {
        std::lock_guard<std::mutex> lg(send_qLock);
        send_q.push(frame);
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
        call_me_thread_handle.join();
        sends_thread_handle.join();
        receive_thread_handle.join();

        ioserv.run();
    }

}
