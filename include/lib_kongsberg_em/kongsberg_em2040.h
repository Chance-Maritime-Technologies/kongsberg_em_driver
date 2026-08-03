//
// Created by jvaccaro on 4/11/19.
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

#include <ds_core_msgs/msg/raw_data.hpp>
#include "../../src/em_driver_library/EM_datagrams/EMdgmFormat.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ds_kongsberg_msgs/srv/ping_cmd.hpp"
#include "ds_kongsberg_msgs/srv/power_cmd.hpp"
#include "ds_kongsberg_msgs/srv/settings_cmd.hpp"
#include "ds_kongsberg_msgs/srv/multi_settings_cmd.hpp"
#include "ds_kongsberg_msgs/srv/bist_cmd.hpp"
#include "ds_kongsberg_msgs/srv/load_xml_cmd.hpp"
#include "ds_kongsberg_msgs/msg/kongsberg_status.hpp"
#include "ds_kongsberg_msgs/msg/kongsberg_km_all_record.hpp"
#include "ds_kongsberg_msgs/msg/kongsberg_setting.hpp"
#include "ds_kongsberg_msgs/msg/kongsberg_runtime_params.hpp"

#include <ds_multibeam_msgs/msg/multibeam_raw.hpp>
#include <boost/utility.hpp>

#include <functional>
#include <memory>

namespace kongsberg_em{

struct KongsbergEM2040Private;
using SendKCtrlDataFunc = std::function<void(const std::string &)>;
using KSetting = ds_kongsberg_msgs::msg::KongsbergSetting;

/// \brief Driver for the Kongsberg EM2040 multibeam.
///
/// Threading: the UDP sockets feeding _on_kmall_data()/_on_kctrl_data() are driven by a
/// boost::asio io_context running on its own thread, while the service and timer callbacks
/// run on the rclcpp executor thread. Every entry point from either thread therefore takes
/// the recursive mutex held in KongsbergEM2040Private before touching driver state.
class KongsbergEM2040  : boost::noncopyable
{

 public:
  KongsbergEM2040(rclcpp::Node::SharedPtr node, SendKCtrlDataFunc send_kctrl_data);
  ~KongsbergEM2040();

  bool parse_data(const ds_core_msgs::msg::RawData &raw);
  bool parse_message(const ds_core_msgs::msg::RawData &raw);
  bool read_kmall_dgm_from_kctrl(int type, const ds_core_msgs::msg::RawData &raw);
  bool parse_ipu(std::vector<std::string> fields);

  /**
   * @brief check_and_append_mpartition
   * @return Returns a bool and a RawData msg.
   * If the datagram is not partitioned, then it returns the datagram.
   * If the datagram is partitioned, then
   *  True means that the datagram is complete (SHOULD BE LOGGED AND PARSED)
   *  False means that the datagram is incomplete (NO LOGGING)
   *
   * This function expects partitions to arrive sequentially for one given message, which is the
   * case when receiving data from the real hardware.
   */
  std::pair<bool, ds_core_msgs::msg::RawData> check_and_append_mpartition(const ds_core_msgs::msg::RawData & );
  bool read_bist_result(ds_core_msgs::msg::RawData& raw);
  uint8_t read_good_bad_missing(std::string);

  void mbraw_to_kmstatus(ds_multibeam_msgs::msg::MultibeamRaw raw);

  // Calls all the ROS setup functions at once(publishers, services, parameters, timers)
  void setupAll();

  void setupServices();
  void setupPublishers();
  void setupParameters();
  void setupTimers();

  void _on_kmall_data(ds_core_msgs::msg::RawData raw);
  void _on_kctrl_data(ds_core_msgs::msg::RawData raw);

private:

  void _ping_cmd(const std::shared_ptr<ds_kongsberg_msgs::srv::PingCmd::Request> req,
                 std::shared_ptr<ds_kongsberg_msgs::srv::PingCmd::Response> res);
  void _power_cmd(const std::shared_ptr<ds_kongsberg_msgs::srv::PowerCmd::Request> req,
                  std::shared_ptr<ds_kongsberg_msgs::srv::PowerCmd::Response> res);
  void _settings_cmd(const std::shared_ptr<ds_kongsberg_msgs::srv::SettingsCmd::Request> req,
                     std::shared_ptr<ds_kongsberg_msgs::srv::SettingsCmd::Response> res);
  void _multisettings_cmd(const std::shared_ptr<ds_kongsberg_msgs::srv::MultiSettingsCmd::Request> req,
                          std::shared_ptr<ds_kongsberg_msgs::srv::MultiSettingsCmd::Response> res);
  void _read_settings_cmd(const std::shared_ptr<ds_kongsberg_msgs::srv::SettingsCmd::Request> req,
                          std::shared_ptr<ds_kongsberg_msgs::srv::SettingsCmd::Response> res);
  void _bist_cmd(const std::shared_ptr<ds_kongsberg_msgs::srv::BistCmd::Request> req,
                 std::shared_ptr<ds_kongsberg_msgs::srv::BistCmd::Response> res);
  void _load_xml_cmd(const std::shared_ptr<ds_kongsberg_msgs::srv::LoadXmlCmd::Request> req,
                     std::shared_ptr<ds_kongsberg_msgs::srv::LoadXmlCmd::Response> res);

  std::string _send_kctrl_command(int cmd);
  template <class T1>
  std::string _send_kctrl_param(std::string param, T1 param_val);
  template <class T1>
  std::string _send_kctrl_param(std::vector<std::string> params, std::vector<T1> vals);
  std::string _send_kctrl_param(std::vector<KSetting> params);
  void _read_kctrl_params();

  template <class EMdgmMRZ_S>
  void _read_and_publish_mrz(const ds_kongsberg_msgs::msg::KongsbergKMAllRecord &r,
                             std::vector<uint8_t> &data);

  void _startup_sequence();
  void _print_bist(std::string name, std::string status, std::string msg);
  void _run_next_bist();
  void _new_kmall_file();
  void _write_kmall_data(const std::vector<uint8_t> &data);
  void _write_kctrl_xml(std::vector<uint8_t>& data);
  std::string _read_kctrl_xml(std::string filename);
  void _on_kctrl_timeout();
  void _on_kmall_timeout();
  void _on_pu_powered_timeout();
  void _on_pu_connected_timeout();
  void _on_pinging_timeout();

  void _reset_rt_params();
  void _reset_sensor_input_statuses();

  static constexpr auto UNKNOWN_STATUS = "UNKNOWN";

  // Needs the node clock, so unlike the ROS 1 version this is not static
  double _timeToLastPartition(const EMdgm_f::EMdgmHeader *hdr);

  // All object members are hidden inside a pointer to preserve binary compatiblity
  // See https://wiki.qt.io/D-Pointer for explanation
  std::unique_ptr<KongsbergEM2040Private> d;


};

}
