//
// Created by jvaccaro on 5/10/19.
//
/**
* Copyright 2019 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include "lib_kongsberg_em/kongsberg_em2040.h"

#include <ds_core_msgs/msg/clock_offset.hpp>
#include <ds_kongsberg_msgs/msg/kongsberg_kssis.hpp>
#include <ds_kongsberg_msgs/msg/kongsberg_mrz.hpp>
#include <ds_kongsberg_msgs/msg/kongsberg_mrz_full.hpp>

#include <fstream>
#include <mutex>
#include <sstream>

namespace kongsberg_em{
struct KongsbergEM2040Private{
  rclcpp::Node::SharedPtr node_;

  // Cached so the logging macros don't have to reach through node_ everywhere
  rclcpp::Logger logger_ = rclcpp::get_logger("kongsberg_em2040");

  // The socket callbacks are invoked from the boost::asio thread while the service and
  // timer callbacks are invoked from the rclcpp executor thread, so all of the state
  // below is guarded. Recursive because the locking entry points (_on_*_data, service
  // and timer callbacks) call into public methods that lock as well.
  std::recursive_mutex mutex_;

  // Callback given to the ROS node to send data using the KController
  // UDP connection
  SendKCtrlDataFunc send_kctrl_data_;

  rclcpp::Service<ds_kongsberg_msgs::srv::PingCmd>::SharedPtr ping_srv_;
  rclcpp::Service<ds_kongsberg_msgs::srv::PowerCmd>::SharedPtr power_srv_;
  rclcpp::Service<ds_kongsberg_msgs::srv::SettingsCmd>::SharedPtr settings_srv_;
  rclcpp::Service<ds_kongsberg_msgs::srv::MultiSettingsCmd>::SharedPtr multi_settings_srv_;
  rclcpp::Service<ds_kongsberg_msgs::srv::SettingsCmd>::SharedPtr read_settings_srv_;
  rclcpp::Service<ds_kongsberg_msgs::srv::BistCmd>::SharedPtr bist_srv_;
  rclcpp::Service<ds_kongsberg_msgs::srv::LoadXmlCmd>::SharedPtr xml_srv_;

  rclcpp::Publisher<ds_multibeam_msgs::msg::MultibeamRaw>::SharedPtr mbraw_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<ds_core_msgs::msg::ClockOffset>::SharedPtr offset_pub_;
  rclcpp::Publisher<ds_kongsberg_msgs::msg::KongsbergMRZ>::SharedPtr mrz_pub_;
  rclcpp::Publisher<ds_kongsberg_msgs::msg::KongsbergMRZFull>::SharedPtr mrz_full_pub_;

  rclcpp::Publisher<ds_kongsberg_msgs::msg::KongsbergKSSIS>::SharedPtr kssis_pub_;
  rclcpp::Publisher<ds_kongsberg_msgs::msg::KongsbergStatus>::SharedPtr kmstatus_pub_;
  rclcpp::Publisher<ds_kongsberg_msgs::msg::KongsbergKMAllRecord>::SharedPtr kmall_record_pub_;
  rclcpp::Publisher<ds_kongsberg_msgs::msg::KongsbergRuntimeParams>::SharedPtr rt_params_pub_;

  // The MRZ data from the KMALL format are provided wrt
  // "Vessel Coordinate System" as configured in installation parameters
  std::string mrz_frame_id_;

  // KCtrl startup info
  ds_kongsberg_msgs::msg::KongsbergStatus m_status;
  ds_kongsberg_msgs::msg::KongsbergRuntimeParams m_rtParams;

  rclcpp::TimerBase::SharedPtr kctrl_timer;
  rclcpp::TimerBase::SharedPtr kmall_timer;
  rclcpp::TimerBase::SharedPtr pu_powered_timer;
  rclcpp::TimerBase::SharedPtr pu_connected_timer;
  rclcpp::TimerBase::SharedPtr pinging_timer;
  std::ofstream kmall_stream;
  uint16_t pck_cnt = 0;

  // Diagnostics
  bool m_decodingFailed = false;

  // If there's a larger kmall datagram that gets partitioned, we need to fix it
  ds_core_msgs::msg::RawData kmall_partitioned;
  uint16_t kmall_dgmNum = 0;
  uint16_t kmall_numOfDgms = 0;

  // Data logging
  bool save_kmall_files;

  // All size values in bytes
  long int kmall_buffer_size, kmall_max_buffer_size, kmall_file_size, kmall_max_file_size;

  //BIST
  std::vector<std::string> bist_tests;
  std::stringstream bist_summary_stream;

};
}
