/*
* Copyright 2022 LeoDrive.ai, Inc. All rights reserved.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
 */

#ifndef BUILD_ARENA_CAMERA_H
#define BUILD_ARENA_CAMERA_H

#include "Arena/ArenaApi.h"
#include "arena_camera/camera_settings.h"

#include <opencv2/opencv.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <condition_variable>
#include <cstddef>
#include <exception>
#include <future>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

class ArenaCamera : public Arena::IImageCallback
{
public:
  explicit ArenaCamera(Arena::IDevice * device, CameraSetting & camera_setting);

  ArenaCamera(
    Arena::IDevice * device, std::string & camera_name, std::string & frame_id,
    std::string & pixel_format, uint32_t serial_no, uint32_t fps, uint32_t horizontal_binning,
    uint32_t vertical_binning);

  ArenaCamera();

  ~ArenaCamera();

  std::thread start_stream();

  void stop_stream();

  void destroy_device(Arena::ISystem * system);

  void acquisition();

  cv::Mat convert_to_image(Arena::IImage * pImage, const std::string & frame_id);

  using ImageCallbackFunction = std::function<void(std::uint32_t, cv::Mat)>;

  ImageCallbackFunction m_signal_publish_image{};

  void set_on_image_callback(ImageCallbackFunction callback);

  // HH_260810 - Prevent new node callbacks and wait for an in-flight callback before StopStream.
  void stop_accepting_image_callbacks()
  {
    std::unique_lock<std::mutex> lock(m_image_callback_mutex);
    m_accept_image_callbacks = false;
    m_image_callback_idle.wait(lock, [this]() { return m_image_callbacks_in_flight == 0; });
  }

  void OnImage(Arena::IImage * pImage)
  {
    {
      std::lock_guard<std::mutex> lock(m_image_callback_mutex);
      if (!m_accept_image_callbacks) {
        return;
      }
      ++m_image_callbacks_in_flight;
    }

    try {
      m_signal_publish_image(m_cam_idx, convert_to_image(pImage, m_frame_id));
    } catch (const std::exception & exception) {
      // HH_260810 - Exceptions must not cross the Arena SDK callback boundary.
      std::cerr << "Image callback failed: " << exception.what() << std::endl;
    } catch (...) {
      std::cerr << "Image callback failed with unknown exception." << std::endl;
    }

    {
      std::lock_guard<std::mutex> lock(m_image_callback_mutex);
      --m_image_callbacks_in_flight;
    }
    m_image_callback_idle.notify_all();
  }

private:
  Arena::IDevice * m_device{nullptr};

  std::string m_camera_name;

  std::string m_frame_id;

  std::string m_pixel_format;

  uint32_t m_serial_no{0};

  uint32_t m_fps{0};

  uint32_t m_cam_idx{0};

  uint32_t m_horizontal_binning{1};

  uint32_t m_vertical_binning{1};

  uint32_t m_reached_horizontal_binning{1};

  uint32_t m_reached_vertical_binning{1};

  std::shared_future<void> future_;

  Arena::IImage * pImage = nullptr;

  // HH_260810 - Guard callback lifetime independently of the Arena stream worker lifecycle.
  std::mutex m_image_callback_mutex;
  std::condition_variable m_image_callback_idle;
  std::size_t m_image_callbacks_in_flight{0};
  bool m_accept_image_callbacks{true};
};

#endif  // BUILD_ARENA_CAMERA_H
