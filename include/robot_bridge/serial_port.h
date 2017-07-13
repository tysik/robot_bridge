#pragma once

#include <cstdint>
#include <boost/asio/serial_port.hpp>

using namespace boost::asio;

namespace robot_bridge {

class SerialPort {
public:
  SerialPort() : io_service_(), serial_port_(io_service_) {}
  SerialPort(const std::string& port_name) : io_service_(), serial_port_(io_service_, port_name) {}
  ~SerialPort() { if (isOpen()) close(); }

  void open(const std::string& port_name) {
    if (port_name != std::string(""))
      serial_port_.open(port_name);
    else
      throw "Incorrect port name provided";
  }

  void close() {
    serial_port_.cancel();
    serial_port_.close();
  }

  bool isOpen() {
    return serial_port_.is_open();
  }

  void setBaudRate(const int baudrate) {
    if (baudrate > 0)
      serial_port_.set_option(serial_port_base::baud_rate(baudrate));
    else
      throw "Incorrect baud rate provided";
  }

  void setFlowControl(const int flow_control) {
    switch (flow_control) {
    case 0:
      serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::type::none));
      break;
    case 1:
      serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::type::hardware));
      break;
    case 2:
      serial_port_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::type::software));
      break;
    default:
      throw "Incorrect flow control provided";
    }
  }

  void setParity(const int parity) {
    switch (parity) {
    case 0:
      serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::type::none));
      break;
    case 1:
      serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::type::odd));
      break;
    case 2:
      serial_port_.set_option(serial_port_base::parity(serial_port_base::parity::type::even));
      break;
    default:
      throw "Incorrect parity provided";
    }
  }

  void setStopBits(const int stop_bits) {
    switch (stop_bits) {
    case 1:
      serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::type::one));
      break;
    case 2:
      serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::type::two));
      break;
    case 3:
      serial_port_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::type::onepointfive));
      break;
    default:
      throw "Incorrect stop bits provided";
    }
  }

  void setCharSize(const int char_size) {
    if (char_size > 4 && char_size < 9)
      serial_port_.set_option(serial_port_base::character_size(char_size));
    else
      throw "Incorrect character size";
  }

  void writeChar(const char c) {
    serial_port_.write_some(boost::asio::buffer(&c, 1));
  }

  char readChar() {
    char c;
    serial_port_.read_some(boost::asio::buffer(&c, 1));
    return c;
  }

  void write(const char* data, size_t size) {
    serial_port_.write_some(boost::asio::buffer(data, size));
  }

  void read(char* data, size_t size) {
    serial_port_.read_some(boost::asio::buffer(data, size));
  }

private:
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
};

}
