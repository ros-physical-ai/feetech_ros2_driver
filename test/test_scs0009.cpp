// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <pty.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <feetech_ros2_driver/scs0009.hpp>
#include <limits>
#include <thread>

namespace ah = feetech_ros2_driver;
TEST(Protocol, KnownPackets) {
  EXPECT_EQ((ah::Bytes{255, 255, 1, 4, 2, 56, 2, 190}), ah::packet(1, 2, {56, 2}));
  EXPECT_EQ((ah::Bytes{255, 255, 254, 10, 131, 30, 2, 11, 0, 0, 12, 10, 20, 31}),
            ah::packet(254, 131, {30, 2, 11, 0, 0, 12, 10, 20}));
  EXPECT_EQ((ah::Bytes{1, 255}), ah::word(511));
}
TEST(Protocol, RejectsBadReplies) {
  const ah::Bytes good = ah::packet(1, 0, {1, 255});
  EXPECT_EQ((ah::Bytes{1, 255}), ah::parseStatus(good, 1, 2));
  auto bad = good;
  bad.back() ^= 1;
  EXPECT_THROW(ah::parseStatus(bad, 1, 2), std::runtime_error);
  EXPECT_THROW(ah::parseStatus(good, 2, 2), std::runtime_error);
  EXPECT_THROW(ah::parseStatus(good, 1, 1), std::runtime_error);
  EXPECT_THROW(ah::parseStatus(ah::packet(1, 4, {1, 255}), 1, 2), std::runtime_error);
  EXPECT_THROW(ah::parseStatus({}, 1, 2), std::runtime_error);
}
TEST(Conversion, RustypotReference) {
  EXPECT_EQ(511, ah::positionRaw(0));
  EXPECT_DOUBLE_EQ(0, ah::positionRadians(511));
  EXPECT_EQ(818, ah::positionRaw(90 * ah::kPi / 180));
  EXPECT_EQ(203, ah::positionRaw(-90 * ah::kPi / 180));
  EXPECT_EQ(586, ah::speedRaw(3));
  EXPECT_EQ(1368, ah::speedRaw(7));
  for (int degrees = -140; degrees <= 140; ++degrees)
    EXPECT_NEAR(degrees * ah::kPi / 180, ah::positionRadians(ah::positionRaw(degrees * ah::kPi / 180)), ah::kStep);
  EXPECT_THROW(ah::positionRaw(3), std::invalid_argument);
  EXPECT_THROW(ah::positionRaw(std::numeric_limits<double>::quiet_NaN()), std::invalid_argument);
  EXPECT_THROW(ah::speedRaw(0), std::invalid_argument);
  EXPECT_THROW(ah::positionRadians(1024), std::runtime_error);
}
TEST(Serial, PseudoTerminalReadWriteAndTimeout) {
  int master, slave;
  char name[128];
  ASSERT_EQ(0, openpty(&master, &slave, name, nullptr, nullptr));
  {
    ah::Scs0009 bus(name, 1000000, 0.1);
    ah::Bytes received;
    std::thread servo([&] {
      uint8_t data[8];
      std::size_t count = 0;
      while (count < sizeof(data)) {
        auto n = ::read(master, data + count, sizeof(data) - count);
        if (n <= 0) return;
        count += static_cast<std::size_t>(n);
      }
      received.assign(data, data + count);
      const auto reply = ah::packet(15, 0, {1, 255});
      // Fragmented status packets must also work.
      for (auto byte : reply) {
        (void)::write(master, &byte, 1);
        usleep(100);
      }
    });
    ah::Bytes result;
    try {
      result = bus.readRegister(15, 56, 2);
    } catch (...) {
      servo.join();
      throw;
    }
    servo.join();
    EXPECT_EQ(ah::packet(15, 2, {56, 2}), received);
    EXPECT_EQ((ah::Bytes{1, 255}), result);
    bus.syncWrite({15, 16}, 42, {ah::word(511), ah::word(818)});
    ah::Bytes written(14);
    std::size_t count = 0;
    while (count < written.size()) {
      auto n = ::read(master, written.data() + count, written.size() - count);
      ASSERT_GT(n, 0);
      count += static_cast<std::size_t>(n);
    }
    EXPECT_EQ(ah::packet(254, 131, {42, 2, 15, 1, 255, 16, 3, 50}), written);
    try {
      bus.readRegister(15, 56, 2);
      FAIL() << "Expected a timeout";
    } catch (const std::runtime_error& e) {
      EXPECT_NE(std::string::npos, std::string(e.what()).find("READ id=15 address=56 size=2"));
      EXPECT_NE(std::string::npos, std::string(e.what()).find("timeout"));
    }
  }
  ::close(master);
  ::close(slave);
}
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
