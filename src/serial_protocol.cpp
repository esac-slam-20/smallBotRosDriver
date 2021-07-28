#include "serial_protocol.h"
namespace smallBot
{
    serial_protocol::serial_protocol(const std::string &port, const uint &baud_rate)
        : ioserv(), serial(ioserv, port)
    {
        serial.set_option(boost::asio::serial_port::baud_rate(baud_rate));
        serial.set_option(boost::asio::serial_port::flow_control());
        serial.set_option(boost::asio::serial_port::parity());
        serial.set_option(boost::asio::serial_port::stop_bits());
        serial.set_option(boost::asio::serial_port::character_size(8));
    }
}
