#include "diffdrive_arduino/arduino_comms.h"

// Optional ROS2 or console logging
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <cstdlib>
#include <iostream>

// POSIX headers for serial
#include <unistd.h>     // close(), read(), write()
#include <fcntl.h>      // open(), O_RDWR, etc.
#include <termios.h>    // termios, tcsetattr, etc.
#include <cerrno>       // errno
#include <cstring>      // strerror

/***************************************************************
 * Constructors / Destructor
 ***************************************************************/
ArduinoComms::ArduinoComms()
  : fd_(-1)
  , is_connected_(false)
  , timeout_ms_(1000) // default 1s
{
}

ArduinoComms::ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  : fd_(-1)
  , is_connected_(false)
  , timeout_ms_(timeout_ms)
{
  setup(serial_device, baud_rate, timeout_ms);
}

/***************************************************************
 * setup(...) : Open and configure the serial port
 ***************************************************************/
void ArduinoComms::setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
{
  timeout_ms_ = timeout_ms;
  if (!openPort(serial_device, baud_rate))
  {
    // If openPort() fails, is_connected_ remains false
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "Failed to open %s at %d baud",
                 serial_device.c_str(), baud_rate);
    return;
  }
  is_connected_ = true;
  RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Serial connected: %s, baud: %d, timeout_ms: %d",
              serial_device.c_str(), baud_rate, timeout_ms);
}

/***************************************************************
 * sendEmptyMsg() : Same logic as original
 ***************************************************************/
void ArduinoComms::sendEmptyMsg()
{
  // Just send "\r" and read one line of response
  std::string response = sendMsg("\r");
  (void)response; // ignore or log if desired
}

/***************************************************************
 * readEncoderValues(...) : Query "e\r", parse response
 ***************************************************************/
void ArduinoComms::readEncoderValues(int &motorA_enc, int &motorB_enc,
                                     int &motorC_enc, int &motorD_enc)
{
  std::string response = sendMsg("e\r");
  // Original parsing logic
  std::istringstream iss(response);
  RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Response: %s", response.c_str());
  char colon;
  iss >> colon >> motorA_enc >> motorB_enc >> motorC_enc >> motorD_enc;
}

/***************************************************************
 * setMotorValues(...) : Send "m A B C D\r"
 ***************************************************************/
void ArduinoComms::setMotorValues(int motorA, int motorB, int motorC, int motorD)
{
  std::stringstream ss;
  RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"), "Setting motor values: %d %d %d %d", motorA, motorB, motorC, motorD);
  ss << "m " << motorA << " " << motorB << " " << motorC << " " << motorD << "\r";
  sendMsg(ss.str(), false);
}

/***************************************************************
 * setPidValues(...) : Send "u Kp:Kd:Ki:Ko\r"
 ***************************************************************/
void ArduinoComms::setPidValues(float k_p, float k_d, float k_i, float k_o)
{
  std::stringstream ss;
  ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
  sendMsg(ss.str());
}

/***************************************************************
 * connected() : Return whether the port is open
 ***************************************************************/
bool ArduinoComms::connected() const
{
  return is_connected_;
}

/***************************************************************
 * sendMsg(...) : Write a string, then read one line response
 ***************************************************************/
std::string ArduinoComms::sendMsg(const std::string &msg_to_send, bool print_output)
{
  if (!is_connected_ || fd_ < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("ArduinoComms"), "sendMsg() called but port not open");
    return "";
  }

  // 1) Write the string
  ssize_t bytes_written = write(fd_, msg_to_send.c_str(), msg_to_send.size());
  if (bytes_written < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "Write error: %s", strerror(errno));
    return "";
  }

  // 2) Read one line (blocking until newline or timeout)
  std::string response = readLinePOSIX();

  if (print_output)
  {
    RCLCPP_INFO(rclcpp::get_logger("ArduinoComms"),
                "Sent: %s | Received: %s", msg_to_send.c_str(), response.c_str());
  }

  return response;
}

/***************************************************************
 * openPort(...) : Internal helper to open & configure the port
 ***************************************************************/
bool ArduinoComms::openPort(const std::string &device, int32_t baud_rate)
{
  // O_RDWR = read/write, O_NOCTTY = no terminal control, O_NDELAY = non-blocking
  fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "open() failed: %s", strerror(errno));
    return false;
  }

  // Make the fd blocking again
  int flags = fcntl(fd_, F_GETFL, 0);
  fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

  // Convert baud_rate to a termios speed_t
  speed_t speed;
  switch (baud_rate)
  {
    case 9600:    speed = B9600; break;
    case 19200:   speed = B19200; break;
    case 38400:   speed = B38400; break;
    case 57600:   speed = B57600; break;
    case 115200:  speed = B115200; break;
    default:
      RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "Unsupported baud rate: %d", baud_rate);
      closePort();
      return false;
  }

  // Configure termios
  struct termios options;
  memset(&options, 0, sizeof(options));

  if (tcgetattr(fd_, &options) < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "tcgetattr failed: %s", strerror(errno));
    closePort();
    return false;
  }

  // c_cflag: 8N1, local mode, enable receiver
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;

  // Input flags - no parity checking, no flow control
  options.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR |
                       IGNCR  | ICRNL  | IXON   | IXOFF  | IXANY);

  // Output flags - raw
  options.c_oflag &= ~OPOST;

  // Local flags - no echo, canonical off
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  // Set baud rates
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);

  // Set VMIN, VTIME for blocking readLinePOSIX
  // VTIME is in deci-seconds; so 100ms => 1, 1s => 10, etc.
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = timeout_ms_ / 100; // e.g. 1000 ms => 10 deci-seconds

  if (tcsetattr(fd_, TCSANOW, &options) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "tcsetattr failed: %s", strerror(errno));
    closePort();
    return false;
  }

  tcflush(fd_, TCIOFLUSH);

  RCLCPP_DEBUG(rclcpp::get_logger("ArduinoComms"), "openPort() success: %s", device.c_str());
  return true;
}

/***************************************************************
 * closePort() : Internal helper to close fd
 ***************************************************************/
void ArduinoComms::closePort()
{
  if (fd_ >= 0)
  {
    close(fd_);
    fd_ = -1;
  }
  is_connected_ = false;
}

/***************************************************************
 * readLinePOSIX() : Read until newline or timeout
 ***************************************************************/
std::string ArduinoComms::readLinePOSIX()
{
  std::string line;
  char c;
  while (true)
  {
    ssize_t n = read(fd_, &c, 1);
    if (n < 0)
    {
      // Error (except EAGAIN if no data)
      if (errno == EAGAIN)
      {
        // If non-blocking, we can keep trying or break
        // but we made the port blocking above, so EAGAIN
        // shouldn't happen unless timed out
        break;
      }
      RCLCPP_ERROR(rclcpp::get_logger("ArduinoComms"), "read() error: %s", strerror(errno));
      return "";
    }
    else if (n == 0)
    {
      // No data => likely a timeout
      break;
    }
    else
    {
      // We got a character
      if (c == '\r')
      {
        // Some devices use \r\n or just \r
        // We'll ignore \r, wait for \n
        continue;
      }
      else if (c == '\n')
      {
        // End of line
        break;
      }
      else
      {
        line.push_back(c);
      }
    }
  }
  return line;
}
