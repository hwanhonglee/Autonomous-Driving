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


#include "arena_camera/arena_camera.h"
#include <rclcpp/rclcpp.hpp>

ArenaCamera::ArenaCamera(Arena::IDevice * device, CameraSetting & camera_setting)
: m_device(device),
  m_camera_name(camera_setting.get_camera_name()),
  m_frame_id(camera_setting.get_frame_id()),
  m_pixel_format(camera_setting.get_pixel_format()),
  m_serial_no(camera_setting.get_serial_no()),
  m_fps(camera_setting.get_fps()),
  m_horizontal_binning(camera_setting.get_horizontal_binning()),
  m_vertical_binning(camera_setting.get_vertical_binning())
{
  std::cout << "Camera:" << m_cam_idx << " is created." << std::endl;
}

ArenaCamera::ArenaCamera(
  Arena::IDevice * device, std::string & camera_name, std::string & frame_id,
  std::string & pixel_format, uint32_t serial_no, uint32_t fps, uint32_t horizontal_binning,
  uint32_t vertical_binning)
: m_device(device),
  m_camera_name(camera_name),
  m_frame_id(frame_id),
  m_pixel_format(pixel_format),
  m_serial_no(serial_no),
  m_fps(fps),
  m_horizontal_binning(horizontal_binning),
  m_vertical_binning(vertical_binning)
{
  std::cout << "Camera:" << m_cam_idx << " is created." << std::endl;
}

std::thread ArenaCamera::start_stream()
{
  return std::thread([=] { this->acquisition(); });
}

void ArenaCamera::acquisition()
{
  std::cout << "Camera idx:" << m_cam_idx << " acquisition thread." << std::endl;

  try {
    std::cout << "[LUCID DEBUG] AcquisitionMode start" << std::endl;
    Arena::SetNodeValue<GenICam::gcstring>(
      m_device->GetNodeMap(), "AcquisitionMode", "Continuous");
    std::cout << "[LUCID DEBUG] AcquisitionMode done" << std::endl;
  } catch (const GenICam::GenericException & e) {
    std::cerr << "[LUCID WARN] AcquisitionMode failed: "
              << e.GetDescription() << std::endl;
  } catch (...) {
    std::cerr << "[LUCID WARN] AcquisitionMode failed unknown." << std::endl;
  }

  try {
    std::cout << "[LUCID DEBUG] StreamAutoNegotiatePacketSize start" << std::endl;
    Arena::SetNodeValue<bool>(
      m_device->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
    std::cout << "[LUCID DEBUG] StreamAutoNegotiatePacketSize done" << std::endl;
  } catch (const GenICam::GenericException & e) {
    std::cerr << "[LUCID WARN] StreamAutoNegotiatePacketSize failed: "
              << e.GetDescription() << std::endl;
  } catch (...) {
    std::cerr << "[LUCID WARN] StreamAutoNegotiatePacketSize failed unknown." << std::endl;
  }

  try {
    std::cout << "[LUCID DEBUG] StreamPacketResendEnable start" << std::endl;
    Arena::SetNodeValue<bool>(
      m_device->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
    std::cout << "[LUCID DEBUG] StreamPacketResendEnable done" << std::endl;
  } catch (const GenICam::GenericException & e) {
    std::cerr << "[LUCID WARN] StreamPacketResendEnable failed: "
              << e.GetDescription() << std::endl;
  } catch (...) {
    std::cerr << "[LUCID WARN] StreamPacketResendEnable failed unknown." << std::endl;
  }

  try {
    std::cout << "[LUCID DEBUG] StartStream start" << std::endl;
    m_device->StartStream();
    std::cout << "[LUCID DEBUG] StartStream done" << std::endl;
  } catch (const GenICam::GenericException & e) {
    std::cerr << "[LUCID ERROR] StartStream failed: "
              << e.GetDescription() << std::endl;
    throw;
  } catch (const std::exception & e) {
    std::cerr << "[LUCID ERROR] StartStream std exception: "
              << e.what() << std::endl;
    throw;
  } catch (...) {
    std::cerr << "[LUCID ERROR] StartStream unknown exception." << std::endl;
    throw;
  }
}

void ArenaCamera::stop_stream() { m_device->StopStream(); }

void ArenaCamera::destroy_device(Arena::ISystem * system)
{
  if (m_device != nullptr) {
    system->DestroyDevice(m_device);
  }
}

void ArenaCamera::set_on_image_callback(ImageCallbackFunction callback)
{
  m_signal_publish_image = std::move(callback);
}

cv::Mat ArenaCamera::convert_to_image(Arena::IImage * pImage, const std::string & frame_id)
{
  cv::Mat image_cv =
    cv::Mat(pImage->GetHeight(), pImage->GetWidth(), CV_8UC1, (uint8_t *)pImage->GetData());

  cv::Mat image_bgr(image_cv.rows, image_cv.cols, CV_8UC3);
  cvtColor(image_cv, image_bgr, cv::COLOR_BayerBG2BGR);

  if (
    m_vertical_binning / m_reached_vertical_binning != 1 ||
    m_horizontal_binning / m_reached_horizontal_binning != 1) {
    int ext_vertical_binning = m_vertical_binning / m_reached_vertical_binning;
    int ext_horizontal_binning = m_horizontal_binning / m_reached_horizontal_binning;

    cv::resize(
      image_bgr, image_bgr,
      cv::Size(image_bgr.cols / ext_horizontal_binning, image_bgr.rows / ext_vertical_binning));
  }

  return image_bgr;
}

ArenaCamera::~ArenaCamera()
{
  std::cout << "Camera:" << m_cam_idx << " ~ArenaCamera()" << std::endl;
  //stop_stream(); 260604 KYS CAMERA BUG
}
