// Copyright 2026 Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// HH_260811 - Verify that the memory monitor forces C locale for its `free` child process.

#include "system_monitor/mem_monitor/mem_monitor.hpp"

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <string>

namespace
{
class ScopedEnvironmentVariable
{
public:
  ScopedEnvironmentVariable(const char * name, const std::string & value) : name_(name)
  {
    const char * original = std::getenv(name);
    if (original != nullptr) {
      had_original_ = true;
      original_ = original;
    }
    EXPECT_EQ(setenv(name, value.c_str(), 1), 0);
  }

  ~ScopedEnvironmentVariable()
  {
    if (had_original_) {
      setenv(name_.c_str(), original_.c_str(), 1);
    } else {
      unsetenv(name_.c_str());
    }
  }

private:
  std::string name_;
  std::string original_;
  bool had_original_{false};
};

class TestableMemMonitor : public MemMonitor
{
public:
  explicit TestableMemMonitor(const rclcpp::NodeOptions & options) : MemMonitor(options) {}

  void checkUsageForTest(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    checkUsage(status);
  }
};

class MemMonitorLocaleTest : public testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};
}  // namespace

TEST_F(MemMonitorLocaleTest, forcesEnglishFreeOutputForLocalizedParent)
{
  const char * original_path = std::getenv("PATH");
  ASSERT_NE(original_path, nullptr);

  const ScopedEnvironmentVariable path(
    "PATH", std::string(TEST_FREE_DIRECTORY) + ":" + original_path);
  const ScopedEnvironmentVariable locale("LC_ALL", "ko_KR.UTF-8");

  TestableMemMonitor monitor(rclcpp::NodeOptions{});
  diagnostic_updater::DiagnosticStatusWrapper status;
  monitor.checkUsageForTest(status);

  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.message, "OK");

  bool found_english_total = false;
  for (const auto & value : status.values) {
    if (value.key == "Mem: total") {
      found_english_total = true;
      break;
    }
  }
  EXPECT_TRUE(found_english_total);
}
