#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H

#include <string>
#include <cstring>

/**
 * @brief A drop-in replacement for the original ArduinoComms class
 *        using POSIX instead of the external serial library.
 */
class ArduinoComms
{
public:
  // Constructors
  ArduinoComms();
  ArduinoComms(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);

  // Same function signatures as original
  void setup(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms);
  void sendEmptyMsg();
  void readEncoderValues(int &motorA_enc, int &motorB_enc, int &motorC_enc, int &motorD_enc);
  void setMotorValues(int motorA, int motorB, int motorC, int motorD);
  void setPidValues(float k_p, float k_d, float k_i, float k_o);

  bool connected() const;

  std::string sendMsg(const std::string &msg_to_send, bool print_output = false);

private:
  /**
   * @brief Open and configure the serial port at the given baud rate.
   */
  bool openPort(const std::string &device, int32_t baud_rate);

  /**
   * @brief Close the file descriptor if open.
   */
  void closePort();

  /**
   * @brief Read a single line (until '\n' or end) from the serial port.
   *        Strips out '\r'. Blocks until line is complete or times out.
   */
  std::string readLinePOSIX();

private:
  int fd_;             ///< File descriptor for the opened serial port
  bool is_connected_;  ///< Indicates if the port is open and configured
  int timeout_ms_;     ///< Timeout in milliseconds for reads (via termios VTIME)
};

#endif // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_H
