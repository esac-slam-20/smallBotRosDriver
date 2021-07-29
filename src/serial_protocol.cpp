#include "serial_protocol.h"
namespace smallBot
{
    serial_protocol::serial_protocol(const std::string &port, const uint &baud_rate)
        : ioserv(), serial(ioserv, port),
          lsp1(serial_protocol::speed::stop), lsp2(serial_protocol::speed::stop),
          lsp3(serial_protocol::speed::stop), lsp4(serial_protocol::speed::stop)
    {
        serial.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port::flow_control());
        serial.set_option(boost::asio::serial_port::parity());
        serial.set_option(boost::asio::serial_port::stop_bits());
        serial.set_option(boost::asio::serial_port::character_size(8));
    }
    uint16_t serial_protocol::get_crc(const uint8_t &cmd, const uint8_t &len, const uint8_t *data)
    {
        //TODO
        return 0;
    }
    bool serial_protocol::write_oneFrame(const uint8_t &cmd,
                                         const uint8_t *data,
                                         const uint8_t &len)
    {
        uint8_t head[3] = {serial_protocol::CMD::head, cmd, len};
        uint16_t crc = get_crc(cmd, len, data);
        uint8_t tail[3] = {get_bit<0>(crc), get_bit<1>(crc),
                           serial_protocol::CMD::tail};
        return boost::asio::write(serial, boost::asio::buffer(head, 3)) &&
               boost::asio::write(serial, boost::asio::buffer(data, len)) &&
               boost::asio::write(serial, boost::asio::buffer(tail, 3));
    }

    bool serial_protocol::read_oneFrame(uint8_t *buffer, std::size_t &len)
    {
        std::size_t receiveDataLen = 0;
        //头
        do
        {
            std::size_t subLen = serial.read_some(
                boost::asio::buffer(buffer + receiveDataLen,
                                    2 - receiveDataLen));
            receiveDataLen += subLen;
            if (receiveDataLen > 2)
                return false;
        } while (receiveDataLen < 2);
        //data+尾
        std::uint16_t dataLen = buffer[1] + 3; //这里用u16是因为data最大长度255 +3可能溢出
        receiveDataLen = 0;
        do
        {
            std::size_t subLen = serial.read_some(
                boost::asio::buffer(2 + buffer + receiveDataLen,
                                    dataLen - receiveDataLen));
            receiveDataLen += subLen;
            if (receiveDataLen > dataLen)
                return false;
        } while (receiveDataLen < dataLen);
        uint16_t crc;
        set_bit<0>(buffer[3 + buffer[2]], crc);
        set_bit<1>(buffer[3 + buffer[2] + 1], crc);
        len = buffer[2] + 6;
        if (get_crc(buffer[1], buffer[2], buffer + 3) != crc)
            return false;
        return true;
    }

    bool serial_protocol::send_speed(int16_t sp1,
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
        uint8_t data[8];
        data[0] = get_bit<0>(sp1);
        data[1] = get_bit<1>(sp1);

        data[2] = get_bit<0>(sp2);
        data[3] = get_bit<1>(sp2);

        data[4] = get_bit<0>(sp3);
        data[5] = get_bit<1>(sp3);

        data[6] = get_bit<0>(sp4);
        data[7] = get_bit<1>(sp4);
        lsp1 = sp1;
        lsp2 = sp2;
        lsp3 = sp3;
        lsp4 = sp4;
        return write_oneFrame(CMD::set_speed, data, 8);
    }

    void serial_protocol::analysis_encodes(uint8_t *data, int32_t &o1, int32_t &o2,
                                           int32_t &o3, int32_t &o4)
    {

        set_bit<0>(data[3 + 0], o1);
        set_bit<1>(data[3 + 1], o1);
        set_bit<2>(data[3 + 2], o1);
        set_bit<3>(data[3 + 3], o1);

        set_bit<0>(data[3 + 4], o2);
        set_bit<1>(data[3 + 5], o2);
        set_bit<2>(data[3 + 6], o2);
        set_bit<3>(data[3 + 7], o2);

        set_bit<0>(data[3 + 8], o3);
        set_bit<1>(data[3 + 9], o3);
        set_bit<2>(data[3 + 10], o3);
        set_bit<3>(data[3 + 11], o3);

        set_bit<0>(data[3 + 12], o4);
        set_bit<1>(data[3 + 13], o4);
        set_bit<2>(data[3 + 14], o4);
        set_bit<3>(data[3 + 15], o4);
    }

    bool serial_protocol::send_encoder_tick(uint16_t tick)
    {
        uint8_t data[2];
        data[0] = get_bit<0>(tick);
        data[1] = get_bit<1>(tick);
        return write_oneFrame(CMD::set_encode, data, 2);
    }
    bool serial_protocol::send_pid(float p, float i, float d)
    {
        uint8_t data[12];
        data[0] = get_bit<0>(p);
        data[1] = get_bit<1>(p);
        data[2] = get_bit<2>(p);
        data[3] = get_bit<3>(p);

        data[4] = get_bit<0>(i);
        data[5] = get_bit<1>(i);
        data[6] = get_bit<2>(i);
        data[7] = get_bit<3>(i);

        data[8] = get_bit<0>(d);
        data[9] = get_bit<1>(d);
        data[10] = get_bit<2>(d);
        data[11] = get_bit<3>(d);
        return write_oneFrame(CMD::set_pid, data, 12);
    }
    bool serial_protocol::send_save()
    {
        return write_oneFrame(CMD::set_save, NULL, 0);
    }
    bool serial_protocol::send_ignore()
    {
        return write_oneFrame(CMD::set_ignore, NULL, 0);
    }

    uint8_t serial_protocol::recive_type(uint8_t *buffer)
    {
        return buffer[0];
    }
}
