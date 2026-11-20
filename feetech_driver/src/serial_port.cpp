#include <spdlog/spdlog.h>
#include <fcntl.h>
#include <limits>
#include <poll.h>
#include <termios.h>
#include <sys/file.h>
#include <unistd.h>

#include <feetech_driver/serial_port.hpp>

namespace feetech_driver {

Expected<LibSerial::BaudRate> to_baudrate(const std::size_t baud) noexcept {
  using LibSerial::BaudRate;
  switch (baud) {
    case 50:
      return BaudRate::BAUD_50;
    case 75:
      return BaudRate::BAUD_75;
    case 110:
      return BaudRate::BAUD_110;
    case 134:
      return BaudRate::BAUD_134;
    case 150:
      return BaudRate::BAUD_150;
    case 200:
      return BaudRate::BAUD_200;
    case 300:
      return BaudRate::BAUD_300;
    case 600:
      return BaudRate::BAUD_600;
    case 1'200:
      return BaudRate::BAUD_1200;
    case 1'800:
      return BaudRate::BAUD_1800;
    case 2'400:
      return BaudRate::BAUD_2400;
    case 4'800:
      return BaudRate::BAUD_4800;
    case 9'600:
      return BaudRate::BAUD_9600;
    case 19'200:
      return BaudRate::BAUD_19200;
    case 38'400:
      return BaudRate::BAUD_38400;
    case 57'600:
      return BaudRate::BAUD_57600;
    case 115'200:
      return BaudRate::BAUD_115200;
    case 230'400:
      return BaudRate::BAUD_230400;
#ifdef __linux__
    case 460'800:
      return BaudRate::BAUD_460800;
    case 500'000:
      return BaudRate::BAUD_500000;
    case 576'000:
      return BaudRate::BAUD_576000;
    case 921'600:
      return BaudRate::BAUD_921600;
    case 1'000'000:
      return BaudRate::BAUD_1000000;
    case 1'152'000:
      return BaudRate::BAUD_1152000;
    case 1'500'000:
      return BaudRate::BAUD_1500000;
#if __MAX_BAUD > B2000000
    case 2'000'000:
      return BaudRate::BAUD_2000000;
    case 2'500'000:
      return BaudRate::BAUD_2500000;
    case 3'000'000:
      return BaudRate::BAUD_3000000;
    case 3'500'000:
      return BaudRate::BAUD_3500000;
    case 4'000'000:
      return BaudRate::BAUD_4000000;
#endif /* __MAX_BAUD */
#endif /* __linux__ */
  }

  return tl::make_unexpected(fmt::format("Invalid baud rate: [{}]", baud));
}

SerialPort::SerialPort(const std::string& dev) : dev_(dev) { spdlog::info("Connecting to port: {}", dev); }

SerialPort::~SerialPort() {
  if (packet_fd_ >= 0 || port_.IsOpen()) (void)close();
}

Result SerialPort::configure(const LibSerial::BaudRate baud_rate) {
  if (auto result = open(); !result) {
    return result;
  }

  try {
    port_.SetBaudRate(baud_rate);
  } catch (const std::runtime_error& e) {
    return tl::make_unexpected(fmt::format("Configuring the serial port failed: [{}]", e.what()));
  }
  return {};
}

Result SerialPort::configure_transactions(LibSerial::BaudRate baud, std::chrono::steady_clock::duration timeout) {
  if (packet_fd_ >= 0 || port_.IsOpen()) return tl::make_unexpected("Port already open");
  if (timeout <= std::chrono::steady_clock::duration::zero() ||
      timeout > std::chrono::milliseconds(std::numeric_limits<int>::max()))
    return tl::make_unexpected("Invalid packet timeout");
  const int fd = ::open(dev_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd < 0) return tl::make_unexpected(fmt::format("Open {}: {}", dev_, std::strerror(errno)));
  termios settings{};
  auto configure_fd = [&]() {
    if (::flock(fd, LOCK_EX | LOCK_NB) < 0 || ::tcgetattr(fd, &settings) < 0) return false;
    ::cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~(CSTOPB | CRTSCTS);
    return ::cfsetspeed(&settings, static_cast<speed_t>(baud)) == 0 &&
           ::tcsetattr(fd, TCSANOW, &settings) == 0 && ::tcflush(fd, TCIOFLUSH) == 0;
  };
  if (!configure_fd()) {
    const auto error = fmt::format("Configure {}: {}", dev_, std::strerror(errno));
    ::close(fd);
    return tl::make_unexpected(error);
  }
  packet_fd_ = fd;
  packet_timeout_ = timeout;
  begin_packet();
  return {};
}

void SerialPort::begin_packet() {
  if (packet_fd_ >= 0) deadline_ = std::chrono::steady_clock::now() + packet_timeout_;
}

Result SerialPort::transfer(uint8_t* data, size_t size, bool writing) {
  pollfd descriptor{packet_fd_, static_cast<short>(writing ? POLLOUT : POLLIN), 0};
  while (size) {
    const auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline_ - std::chrono::steady_clock::now());
    if (remaining.count() <= 0) return tl::make_unexpected("Serial packet timeout");
    const int ready = ::poll(&descriptor, 1, static_cast<int>(remaining.count()));
    if (ready < 0 && errno == EINTR) continue;
    if (ready == 0) return tl::make_unexpected("Serial packet timeout");
    if (ready < 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
      return tl::make_unexpected("Serial poll failed or device disconnected");
    const auto n = writing ? ::write(descriptor.fd, data, size) : ::read(descriptor.fd, data, size);
    if (n < 0 && (errno == EINTR || errno == EAGAIN)) continue;
    if (n <= 0) return tl::make_unexpected("Serial transfer failed");
    data += n;
    size -= static_cast<size_t>(n);
  }
  return {};
}

Result SerialPort::open() {
  if (packet_fd_ >= 0) return {};
  try {
    if (!port_.IsOpen()) {
      port_.Open(dev_);
    }
  } catch (const LibSerial::OpenFailed& e) {
    return tl::make_unexpected(fmt::format("Open [{}]: {}", dev_.c_str(), e.what()));
  }

  return {};
}

Result SerialPort::close() {
  if (packet_fd_ >= 0) {
    ::close(packet_fd_);
    packet_fd_ = -1;
    return {};
  }
  try {
    port_.Close();
  } catch (const LibSerial::AlreadyOpen& e) {
    return tl::make_unexpected(fmt::format("close [{}]: {}", dev_.c_str(), e.what()));
  } catch (const std::runtime_error& e) {
    return tl::make_unexpected(fmt::format("close [{}]: {}", dev_.c_str(), e.what()));
  }
  return {};
}

Result SerialPort::check_port() const noexcept {
  if (packet_fd_ < 0 && !port_.IsOpen()) {
    return tl::make_unexpected(fmt::format("Port [{}] is not open", dev_));
  }

  return {};
}

Result SerialPort::flashInputBuffer() noexcept {
  if (packet_fd_ >= 0) {
    if (::tcflush(packet_fd_, TCIFLUSH) < 0) return tl::make_unexpected(std::string(std::strerror(errno)));
    return {};
  }
  if (auto result = check_port(); !result) {
    return result;
  }

  try {
    port_.FlushInputBuffer();
  } catch (const std::runtime_error& e) {
    return tl::make_unexpected(e.what());
  }

  return {};
}

Result SerialPort::flashOutputBuffer() noexcept {
  if (packet_fd_ >= 0) {
    if (::tcflush(packet_fd_, TCOFLUSH) < 0) return tl::make_unexpected(std::string(std::strerror(errno)));
    return {};
  }
  if (auto result = check_port(); !result) {
    return result;
  }

  try {
    port_.FlushOutputBuffer();
  } catch (const std::runtime_error& e) {
    return tl::make_unexpected(e.what());
  }

  return {};
}
}  // namespace feetech_driver
