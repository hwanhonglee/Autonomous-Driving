// Copyright 2026
// Licensed under the Apache License, Version 2.0.

#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/hesai_hw_interface.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <vector>

namespace nebula::drivers
{
namespace
{

// HH_260811 - Bind only loopback UDP so the shutdown regression never contacts LiDAR hardware.
std::shared_ptr<HesaiSensorConfiguration> make_loopback_config()
{
  auto config = std::make_shared<HesaiSensorConfiguration>();
  config->host_ip = "127.0.0.1";
  config->data_port = 0;
  return config;
}

TEST(HesaiHwInterfaceShutdown, DestructorQuiescesPendingUdpReceive)
{
  for (std::size_t iteration = 0; iteration < 64; ++iteration) {
    auto interface = std::make_unique<HesaiHwInterface>();
    ASSERT_EQ(interface->SetSensorConfiguration(make_loopback_config()), Status::OK);
    ASSERT_EQ(interface->RegisterScanCallback([](std::vector<uint8_t> &) {}), Status::OK);
    ASSERT_EQ(interface->SensorInterfaceStart(), Status::OK);
  }
}

TEST(HesaiHwInterfaceShutdown, StopIsIdempotent)
{
  HesaiHwInterface interface;
  ASSERT_EQ(interface.SetSensorConfiguration(make_loopback_config()), Status::OK);
  ASSERT_EQ(interface.RegisterScanCallback([](std::vector<uint8_t> &) {}), Status::OK);
  ASSERT_EQ(interface.SensorInterfaceStart(), Status::OK);
  EXPECT_EQ(interface.SensorInterfaceStop(), Status::OK);
  EXPECT_EQ(interface.SensorInterfaceStop(), Status::OK);
}

}  // namespace
}  // namespace nebula::drivers
