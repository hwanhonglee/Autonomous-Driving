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

#include "arena_camera/arena_cameras_handler.h"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>

ArenaCamerasHandler::ArenaCamerasHandler()
{
  m_p_system = Arena::OpenSystem();
  m_p_system->UpdateDevices(2000);
}
void ArenaCamerasHandler::create_camera_from_settings(CameraSetting & camera_settings)
{
  std::vector<Arena::DeviceInfo> devicesInfos = m_p_system->GetDevices();

  std::cout << "[LUCID DEBUG] devices count: " << devicesInfos.size() << std::endl;

  for (auto & d_info : devicesInfos) {
    std::cout << "[LUCID DEBUG] device serial: " << d_info.SerialNumber()
              << " model: " << d_info.ModelName()
              << " ip: " << d_info.IpAddressStr()
              << " mac: " << d_info.MacAddressStr()
              << std::endl;
  }

  if (devicesInfos.size() == 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "There is no connected devices. Please connect a device and try again.");
    throw std::runtime_error("arena_camera: There is no connected devices.");
  }

  auto it = std::find_if(devicesInfos.begin(), devicesInfos.end(), [&](Arena::DeviceInfo & d_info) {
    return std::to_string(camera_settings.get_serial_no()) == d_info.SerialNumber().c_str();
  });

  if (it == devicesInfos.end()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Wrong device serial no in parameters file. Please check the serial no and try again.");
    throw std::runtime_error("arena_camera: Wrong device serial no in parameters file.");
  }

  try {
    std::cout << "[LUCID DEBUG] CreateDevice start" << std::endl;
    m_device = m_p_system->CreateDevice(*it);
    std::cout << "[LUCID DEBUG] CreateDevice done" << std::endl;
  } catch (const GenICam::GenericException & e) {
    std::cerr << "[LUCID ERROR] CreateDevice GenICam exception: "
              << e.GetDescription() << std::endl;
    throw;
  } catch (const std::exception & e) {
    std::cerr << "[LUCID ERROR] CreateDevice std exception: "
              << e.what() << std::endl;
    throw;
  } catch (...) {
    std::cerr << "[LUCID ERROR] CreateDevice unknown exception." << std::endl;
    throw;
  }

  m_enable_rectifying = camera_settings.get_enable_rectifying();
  m_enable_compressing = camera_settings.get_enable_compressing();
  m_use_default_device_settings = camera_settings.get_use_default_device_settings();

  std::cout << "[LUCID DEBUG] use_default_device_settings: "
            << std::boolalpha << m_use_default_device_settings << std::endl;

  m_cameras = new ArenaCamera(m_device, camera_settings);
  m_device->RegisterImageCallback(m_cameras);

  if (!m_use_default_device_settings) {
    try {
      std::cout << "[LUCID DEBUG] set_fps start" << std::endl;
      this->set_fps(camera_settings.get_fps());
      std::cout << "[LUCID DEBUG] set_fps done" << std::endl;
    } catch (const GenICam::GenericException & e) {
      std::cerr << "[LUCID WARN] set_fps failed: " << e.GetDescription() << std::endl;
    } catch (const std::exception & e) {
      std::cerr << "[LUCID WARN] set_fps failed: " << e.what() << std::endl;
    } catch (...) {
      std::cerr << "[LUCID WARN] set_fps failed with unknown exception." << std::endl;
    }

    try {
      std::cout << "[LUCID DEBUG] set_auto_exposure start" << std::endl;
      this->set_auto_exposure(camera_settings.get_enable_exposure_auto());
      std::cout << "[LUCID DEBUG] set_auto_exposure done" << std::endl;
    } catch (const GenICam::GenericException & e) {
      std::cerr << "[LUCID WARN] set_auto_exposure failed: " << e.GetDescription() << std::endl;
    } catch (...) {
      std::cerr << "[LUCID WARN] set_auto_exposure failed." << std::endl;
    }

    try {
      std::cout << "[LUCID DEBUG] set_exposure_value start" << std::endl;
      this->set_exposure_value(camera_settings.get_auto_exposure_value());
      std::cout << "[LUCID DEBUG] set_exposure_value done" << std::endl;
    } catch (const GenICam::GenericException & e) {
      std::cerr << "[LUCID WARN] set_exposure_value failed: " << e.GetDescription() << std::endl;
    } catch (...) {
      std::cerr << "[LUCID WARN] set_exposure_value failed." << std::endl;
    }

    try {
      std::cout << "[LUCID DEBUG] set_auto_gain start" << std::endl;
      this->set_auto_gain(camera_settings.get_enable_gain_auto());
      std::cout << "[LUCID DEBUG] set_auto_gain done" << std::endl;
    } catch (const GenICam::GenericException & e) {
      std::cerr << "[LUCID WARN] set_auto_gain failed: " << e.GetDescription() << std::endl;
    } catch (...) {
      std::cerr << "[LUCID WARN] set_auto_gain failed." << std::endl;
    }

    try {
      std::cout << "[LUCID DEBUG] set_gain_value start" << std::endl;
      this->set_gain_value(camera_settings.get_auto_gain_value());
      std::cout << "[LUCID DEBUG] set_gain_value done" << std::endl;
    } catch (const GenICam::GenericException & e) {
      std::cerr << "[LUCID WARN] set_gain_value failed: " << e.GetDescription() << std::endl;
    } catch (...) {
      std::cerr << "[LUCID WARN] set_gain_value failed." << std::endl;
    }

    try {
      std::cout << "[LUCID DEBUG] set_gamma_value start" << std::endl;
      this->set_gamma_value(camera_settings.get_gamma_value());
      std::cout << "[LUCID DEBUG] set_gamma_value done" << std::endl;
    } catch (const GenICam::GenericException & e) {
      std::cerr << "[LUCID WARN] set_gamma_value failed: " << e.GetDescription() << std::endl;
    } catch (...) {
      std::cerr << "[LUCID WARN] set_gamma_value failed." << std::endl;
    }

    try {
      std::cout << "[LUCID DEBUG] set_reverse_image_y start" << std::endl;
      this->set_reverse_image_y(camera_settings.get_image_horizontal_flip());
      std::cout << "[LUCID DEBUG] set_reverse_image_y done" << std::endl;
    } catch (const GenICam::GenericException & e) {
      std::cerr << "[LUCID WARN] set_reverse_image_y failed: " << e.GetDescription() << std::endl;
    } catch (...) {
      std::cerr << "[LUCID WARN] set_reverse_image_y failed." << std::endl;
    }

    try {
      std::cout << "[LUCID DEBUG] set_reverse_image_x start" << std::endl;
      this->set_reverse_image_x(camera_settings.get_image_vertical_flip());
      std::cout << "[LUCID DEBUG] set_reverse_image_x done" << std::endl;
    } catch (const GenICam::GenericException & e) {
      std::cerr << "[LUCID WARN] set_reverse_image_x failed: " << e.GetDescription() << std::endl;
    } catch (...) {
      std::cerr << "[LUCID WARN] set_reverse_image_x failed." << std::endl;
    }
  } else {
    std::cout << "[LUCID DEBUG] Skip camera register settings because use_default_device_settings=true" << std::endl;
  }
}

void ArenaCamerasHandler::set_image_callback(ArenaCamera::ImageCallbackFunction callback)
{
  this->m_cameras->set_on_image_callback(callback);
}

//void ArenaCamerasHandler::start_stream() { this->m_cameras->acquisition(); }

void ArenaCamerasHandler::start_stream() // 260604 KYS CAMERA BUG
{
  // HH_260810 - A concurrent pre-shutdown callback must finish StartStream before it gates/stops it.
  std::lock_guard<std::mutex> shutdown_lock(m_shutdown_mutex);

  if (!m_cameras) {
    throw std::runtime_error("ArenaCamera is not created.");
  }
  if (m_callbacks_blocked) {
    throw std::runtime_error("ArenaCamera shutdown already started.");
  }

  m_cameras->acquisition();
  m_streaming = true;
}


//void ArenaCamerasHandler::stop_stream() { this->m_cameras->stop_stream(); }

void ArenaCamerasHandler::set_fps(uint32_t fps)
{
  try {
    auto node_map = m_device->GetNodeMap();

    GenApi::CBooleanPtr pFrameRateEnable = node_map->GetNode("AcquisitionFrameRateEnable");
    if (pFrameRateEnable && GenApi::IsWritable(pFrameRateEnable)) {
      pFrameRateEnable->SetValue(true);
    } else {
      std::cerr << "[LUCID WARN] AcquisitionFrameRateEnable is not writable." << std::endl;
    }

    GenApi::CFloatPtr pFrameRate = node_map->GetNode("AcquisitionFrameRate");
    if (!pFrameRate || !GenApi::IsWritable(pFrameRate)) {
      std::cerr << "[LUCID WARN] AcquisitionFrameRate is not writable. Skip set_fps." << std::endl;
      return;
    }

    double fps_to_set = static_cast<double>(fps);
    if (fps_to_set > pFrameRate->GetMax()) {
      fps_to_set = pFrameRate->GetMax();
    }
    if (fps_to_set < pFrameRate->GetMin()) {
      fps_to_set = pFrameRate->GetMin();
    }

    pFrameRate->SetValue(fps_to_set);
    std::cout << "[LUCID DEBUG] FPS set to " << fps_to_set << std::endl;
  } catch (const GenICam::GenericException & e) {
    std::cerr << "[LUCID WARN] set_fps GenICam exception: " << e.GetDescription() << std::endl;
  } catch (const std::exception & e) {
    std::cerr << "[LUCID WARN] set_fps std exception: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "[LUCID WARN] set_fps unknown exception." << std::endl;
  }
}

//ArenaCamerasHandler::~ArenaCamerasHandler()
//{
//  std::cout << " ~ArenaCamerasHandler()" << std::endl;

//  this->stop_stream();
//  m_device->DeregisterImageCallback(m_cameras);
//  this->m_cameras->destroy_device(m_p_system);
//  CloseSystem(m_p_system);

//  delete m_cameras;
//  delete m_p_system;
//  delete m_device;
//}

void ArenaCamerasHandler::stop_stream() noexcept // 260604 KYS CAMERA BUG
{
  try {
    std::lock_guard<std::mutex> shutdown_lock(m_shutdown_mutex);

    if (m_cameras && !m_callbacks_blocked) {
      // HH_260810 - Drain node work before rcl_shutdown can invalidate its publishers.
      std::cout << "[LUCID SHUTDOWN] callback gate begin" << std::endl;
      m_cameras->stop_accepting_image_callbacks();
      m_callbacks_blocked = true;
      std::cout << "[LUCID SHUTDOWN] callback gate done" << std::endl;
    }

    if (m_device && m_cameras && m_streaming) {
      std::cout << "[LUCID SHUTDOWN] StopStream begin" << std::endl;
      m_cameras->stop_stream();
      m_streaming = false;
      std::cout << "[LUCID SHUTDOWN] StopStream done" << std::endl;
    }
  } catch (const std::exception & e) {
    std::cerr << "[LUCID SHUTDOWN] callback gate/StopStream failed: " << e.what() << std::endl;
  } catch (...) {
    std::cerr << "[LUCID SHUTDOWN] callback gate/StopStream failed with unknown exception."
              << std::endl;
  }
}

ArenaCamerasHandler::~ArenaCamerasHandler() // 260604 KYS CAMERA BUG
{
  std::cout << " ~ArenaCamerasHandler()" << std::endl;

  // HH_260810 - Idempotent when the rclcpp pre-shutdown callback already stopped the stream.
  stop_stream();

  bool callback_can_be_deleted = true;
  if (m_device && m_cameras) {
    // HH_260810 - Follow the SDK order: StopStream, then deregister the callback.
    try {
      std::cout << "[LUCID SHUTDOWN] DeregisterImageCallback begin" << std::endl;
      const bool was_registered = m_device->DeregisterImageCallback(m_cameras);
      std::cout << "[LUCID SHUTDOWN] DeregisterImageCallback done (registered="
                << std::boolalpha << was_registered << ")" << std::endl;
    } catch (...) {
      callback_can_be_deleted = false;
      std::cerr << "[LUCID SHUTDOWN] DeregisterImageCallback failed; callback object retained."
                << std::endl;
    }
  }

  if (m_cameras && callback_can_be_deleted) {
    // HH_260810 - Delete only after successful SDK callback deregistration.
    std::cout << "[LUCID SHUTDOWN] callback delete begin" << std::endl;
    delete m_cameras;
    m_cameras = nullptr;
    std::cout << "[LUCID SHUTDOWN] callback delete done" << std::endl;
  }

  if (m_p_system && m_device) {
    // HH_260810 - Destroy the device only after the callback object is no longer SDK-visible.
    try {
      std::cout << "[LUCID SHUTDOWN] DestroyDevice begin" << std::endl;
      m_p_system->DestroyDevice(m_device);
      std::cout << "[LUCID SHUTDOWN] DestroyDevice done" << std::endl;
    } catch (...) {
      std::cerr << "[LUCID SHUTDOWN] DestroyDevice failed." << std::endl;
    }
    m_device = nullptr;
  }

  if (m_p_system) {
    // HH_260810 - Close Arena only after every device teardown attempt has completed.
    try {
      std::cout << "[LUCID SHUTDOWN] CloseSystem begin" << std::endl;
      Arena::CloseSystem(m_p_system);
      std::cout << "[LUCID SHUTDOWN] CloseSystem done" << std::endl;
    } catch (...) {
      std::cerr << "[LUCID SHUTDOWN] CloseSystem failed." << std::endl;
    }
    m_p_system = nullptr;
  }

}

GenICam_3_3_LUCID::gcstring ArenaCamerasHandler::get_auto_exposure()
{
  return Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "ExposureAuto");
}

void ArenaCamerasHandler::set_auto_exposure(bool auto_exposure)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set auto exposure. Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring exposure_auto = auto_exposure ? "Continuous" : "Off";
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "ExposureAuto", exposure_auto);
}

void ArenaCamerasHandler::set_exposure_value(float exposure_value)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure value. Using default device settings.");
    return;
  }

  const auto auto_exposure = this->get_auto_exposure();
  if (auto_exposure == "Off") {
    GenApi::CFloatPtr pExposureTime = m_device->GetNodeMap()->GetNode("ExposureTime");
    try {
      if (exposure_value < pExposureTime->GetMin()) {
        exposure_value = pExposureTime->GetMin();

      } else if (exposure_value > pExposureTime->GetMax()) {
        exposure_value = pExposureTime->GetMax();
      }
      pExposureTime->SetValue(exposure_value);
    } catch (const GenICam::GenericException & e) {
      std::cerr << "Exception occurred during exposure value handling: " << e.GetDescription()
                << std::endl;
    }
  }else{
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set exposure value when auto exposure is enabled.");
  }
}
GenICam_3_3_LUCID::gcstring ArenaCamerasHandler::get_auto_gain()
{
  return Arena::GetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "GainAuto");
}

void ArenaCamerasHandler::set_auto_gain(bool auto_gain)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set auto gain. Using default device settings.");
    return;
  }
  GenICam_3_3_LUCID::gcstring gain_auto = auto_gain ? "Continuous" : "Off";
  Arena::SetNodeValue<GenICam::gcstring>(m_device->GetNodeMap(), "GainAuto", gain_auto);
}

void ArenaCamerasHandler::set_gain_value(float gain_value)
{
  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value. Using default device settings.");
    return;
  }
  const auto auto_gain = this->get_auto_gain();
  if (auto_gain == "Off") {
    GenApi::CFloatPtr pGain = m_device->GetNodeMap()->GetNode("Gain");
    try {
      if (gain_value < pGain->GetMin()) {
        gain_value = pGain->GetMin();
      } else if (gain_value > pGain->GetMax()) {
        gain_value = pGain->GetMax();
      }
      pGain->SetValue(gain_value);
    } catch (const GenICam::GenericException & e) {
      std::cerr << "Exception occurred during gain value handling: " << e.GetDescription()
                << std::endl;
    }
  }else{
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value when auto gain is enabled.");
  }
}
void ArenaCamerasHandler::set_gamma_value(float gamma_value)
{

  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set gain value. Using default device settings.");
    return;
  }

  try {
    GenApi::CBooleanPtr pGammaEnable = m_device->GetNodeMap()->GetNode("GammaEnable");
    if (GenApi::IsWritable(pGammaEnable)) {
      pGammaEnable->SetValue(true);
    }
    GenApi::CFloatPtr pGamma = m_device->GetNodeMap()->GetNode("Gamma");
    if (pGamma && GenApi::IsWritable(pGamma)) {
      if (pGamma->GetMin() > gamma_value) {
        gamma_value = pGamma->GetMin();
      } else if (pGamma->GetMax() < gamma_value) {
        gamma_value = pGamma->GetMax();
      }
      pGamma->SetValue(gamma_value);
    }
  } catch (const GenICam::GenericException & e) {
    std::cerr << "Exception occurred during gamma value handling: " << e.GetDescription()
              << std::endl;
  }
}
void ArenaCamerasHandler::set_enable_rectifying(bool enable_rectifying)
{
  this->m_enable_rectifying = enable_rectifying;
}
bool ArenaCamerasHandler::get_enable_rectifying()
{
  return m_enable_rectifying;
}
void ArenaCamerasHandler::set_enable_compressing(bool enable_compressing)
{
  this->m_enable_compressing = enable_compressing;
}
bool ArenaCamerasHandler::get_enable_compressing()
{
  return m_enable_compressing;
}
void ArenaCamerasHandler::set_use_default_device_settings(bool use_default_device_settings)
{
  this->m_use_default_device_settings = use_default_device_settings;
}
bool ArenaCamerasHandler::get_use_default_device_settings()
{
  return m_use_default_device_settings;
}
void ArenaCamerasHandler::set_reverse_image_y(bool image_horizontal_flip)
{

  if(m_use_default_device_settings){
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set horizontal flip. Using default device settings.");
    return;
  }

  try {
    GenApi::CBooleanPtr pHorizontalFlip = m_device->GetNodeMap()->GetNode("ReverseY");
    if (GenApi::IsWritable(pHorizontalFlip)) {
      pHorizontalFlip->SetValue(image_horizontal_flip);
    }
  } catch (const GenICam::GenericException & e) {
    std::cerr << "Exception occurred during ReverseY value handling: " << e.GetDescription()
              << std::endl;
  }
}

void ArenaCamerasHandler::set_reverse_image_x(bool image_vertical_flip)
{
  if (m_use_default_device_settings) {
    RCLCPP_WARN(
      rclcpp::get_logger("ARENA_CAMERA_HANDLER"),
      "Not possible to set vertical flip. Using default device settings.");
    return;
  }
  try {
    GenApi::CBooleanPtr pVerticalFlip = m_device->GetNodeMap()->GetNode("ReverseX");
    if (GenApi::IsWritable(pVerticalFlip)) {
      pVerticalFlip->SetValue(image_vertical_flip);
    }
  } catch (const GenICam::GenericException & e) {
    std::cerr << "Exception occurred during ReverseX value handling: " << e.GetDescription()
              << std::endl;
  }
}
