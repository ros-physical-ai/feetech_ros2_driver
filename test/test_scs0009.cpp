// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>

#include <feetech_driver/communication_protocol.hpp>
#include <feetech_driver/scs0009.hpp>
#include <thread>

namespace scs = feetech_driver::scs0009;
TEST(Scs0009, EncoderAndSpeed) {
  EXPECT_EQ(scs::word(511), (scs::Word{1, 255}));
  EXPECT_EQ(scs::position_radians(scs::word(511)), 0);
  EXPECT_EQ(scs::position_raw(std::numbers::pi / 2), 818);
  EXPECT_EQ(scs::speed_raw(3), 0x024A);
  EXPECT_EQ(scs::speed_raw(-3), 0x824A);
  EXPECT_THROW(scs::speed_raw(-32768 * scs::step), std::out_of_range);
  EXPECT_THROW(scs::position_radians(scs::word(1024)), std::out_of_range);
  EXPECT_THROW(scs::speed_raw(0), std::out_of_range);
}

TEST(Scs0009, RejectsInvalidStatusAndBoundsPacketTime) {
  int master, slave;
  char name[128];
  ASSERT_EQ(openpty(&master, &slave, name, nullptr, nullptr), 0);
  auto port = std::make_unique<feetech_driver::SerialPort>(name);
  ASSERT_TRUE(port->configure_transactions(LibSerial::BaudRate::BAUD_1000000, std::chrono::milliseconds(100)));
  feetech_driver::SerialPort contender(name);
  EXPECT_FALSE(contender.configure_transactions(LibSerial::BaudRate::BAUD_1000000, std::chrono::milliseconds(100)));
  feetech_driver::CommunicationProtocol bus(std::move(port));
  for (const auto status : {std::array<uint8_t, 3>{2, 4, 0}, {1, 3, 0}, {1, 4, 1}, {1, 4, 0}}) {
    std::thread servo([&] {
      std::array<uint8_t, 8> request{};
      size_t done = 0;
      while (done < request.size()) {
        const auto n = ::read(master, request.data() + done, request.size() - done);
        if (n <= 0) return;
        done += static_cast<size_t>(n);
      }
      EXPECT_EQ(request, (std::array<uint8_t, 8>{255, 255, 1, 4, 2, 56, 2, 190}));
      const std::array<uint8_t, 8> reply{255,
                                         255,
                                         status[0],
                                         status[1],
                                         status[2],
                                         1,
                                         255,
                                         static_cast<uint8_t>(~(status[0] + status[1] + status[2] + 1 + 255))};
      (void)::write(master, reply.data(), reply.size());
    });
    scs::Word word{};
    const auto result = bus.read(1, scs::present_position, &word, true);
    servo.join();
    EXPECT_EQ(result.has_value(), (status == std::array<uint8_t, 3>{1, 4, 0}));
  }
  scs::Word word{};
  std::thread slow_reply([&] {
    uint8_t request[8];
    size_t done = 0;
    while (done < sizeof(request)) {
      const auto n = ::read(master, request + done, sizeof(request) - done);
      if (n <= 0) return;
      done += static_cast<size_t>(n);
    }
    for (uint8_t byte : std::array<uint8_t, 8>{255, 255, 1, 4, 0, 1, 255, 250}) {
      (void)::write(master, &byte, 1);
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });
  const auto timeout = bus.read(1, scs::present_position, &word, true);
  slow_reply.join();
  ASSERT_FALSE(timeout);
  EXPECT_NE(timeout.error().find("timeout"), std::string::npos);
  ::close(master);
  ::close(slave);
}
