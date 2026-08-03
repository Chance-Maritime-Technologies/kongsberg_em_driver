  //
// Created by jvaccaro on 7/17/19.
//

#pragma once

#define M_PI_RAD 3.14159
#define M_PI_DEG 180.0

#include "boost/date_time/posix_time/posix_time_io.hpp"
#include "boost/date_time/posix_time/posix_time_types.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ds_multibeam_msgs/msg/multibeam_raw.hpp>
#include <ds_kongsberg_msgs/msg/kongsberg_mrz.hpp>
// KongsbergMRZFull.msg pulls in KongsbergMRZtxSectorInfo and KongsbergMRZsounding
#include <ds_kongsberg_msgs/msg/kongsberg_mrz_full.hpp>

namespace kongsberg_em {

/// \brief Build a ROS time from the seconds/nanoseconds pair in a KMALL datagram header.
///
/// The ROS 1 version went through ros::Time::fromSec(sec + nsec/1e9), which loses sub-microsecond
/// precision; composing the nanoseconds directly avoids that while producing the same instant.
inline rclcpp::Time kmall_stamp(uint32_t time_sec, uint32_t time_nanosec)
{
  return rclcpp::Time(static_cast<int64_t>(time_sec) * 1000000000LL + time_nanosec, RCL_ROS_TIME);
}

inline std::string filename(std::string directory, int count, std::string base, std::string extension)
{
  std::stringstream filename_ss;
  auto facet = new boost::posix_time::time_facet("%Y%m%d_%H%M");
  filename_ss.imbue(std::locale(filename_ss.getloc(), facet));
  filename_ss << directory;
  filename_ss << std::setfill('0') << std::setw(4) << count;
  filename_ss << "_" << boost::posix_time::second_clock::universal_time();
  filename_ss << "_" << base;
  filename_ss << extension;
  return filename_ss.str();
}

inline double deg_to_rad(double deg){
  return deg*M_PI_RAD/M_PI_DEG;
}

inline double rad_to_deg(double rad){
  return rad*M_PI_DEG/M_PI_RAD;
}

inline std::pair<std::vector<std::string>, std::vector<std::string>>
string_split_out_xml_params(std::string data, std::string token_param="ID", std::string token_value="VALUE", std::string token_delim="TOKEN")
{
  std::vector<std::string> params, values;
  std::stringstream ss(data);
  std::string l;
  std::string param_start = "<" + token_param + ">";
  std::string param_end = "</" + token_param + ">";
  std::string value_start = "<" + token_value + ">";
  std::string value_end = "</" + token_value + ">";
  std::string delim_start = "<" + token_delim + ">";
  std::string delim_end = "</" + token_delim + ">";
  std::string param{};
  std::string value{};
  while(std::getline(ss >> std::ws, l,'\n')){
    if (!l.find(param_start)){
      param = l.substr(param_start.length(), l.length() - param_start.size() - param_end.length());
    } else if (!l.find(value_start)) {
      value = l.substr(value_start.length(), l.length() - value_start.size() - value_end.length());
    } else if (!l.find(delim_start)){
      param = "";
      value = "";
    } else if (!l.find(delim_end)){
      if (!param.empty() && !value.empty()){
        params.push_back(param);
        values.push_back(value);
      }
    }
  }
  return {params, values};
}

inline std::pair<std::vector<std::string>, std::vector<std::string>>
file_split_out_xml_params(std::string filename, std::string token_param="ID", std::string token_value="VALUE", std::string token_delim="TOKEN")
{
  std::vector<std::string> params, values;
  std::string param_start = "<" + token_param + ">";
  std::string param_end = "</" + token_param + ">";
  std::string value_start = "<" + token_value + ">";
  std::string value_end = "</" + token_value + ">";
  std::string delim_start = "<" + token_delim + ">";
  std::string delim_end = "</" + token_delim + ">";
  std::string param{};
  std::string value{};
  std::string l;
  std::ifstream in(filename, std::ios::in | std::ios::binary);
  while(in >> l){
    if (!l.find(param_start)){
      param = l.substr(param_start.length(), l.length() - param_start.size() - param_end.length());
    } else if (!l.find(value_start)) {
      value = l.substr(value_start.length(), l.length() - value_start.size() - value_end.length());
    } else if (!l.find(delim_start)){
      param = "";
      value = "";
    } else if (!l.find(delim_end)){
      if (!param.empty() && !value.empty()){
        params.push_back(param);
        values.push_back(value);
      }
    }
  }
  return {params, values};
}

template <typename EMdgmMRZ_S>
static ds_kongsberg_msgs::msg::KongsbergMRZ mrz_to_msg(const EMdgmMRZ_S& msg)
{
  ds_kongsberg_msgs::msg::KongsbergMRZ m;
  m.header.stamp = kmall_stamp(msg.header.time_sec, msg.header.time_nanosec);
  m.latitude_deg = msg.pingInfo.latitude_deg;
  m.longitude_deg  = msg.pingInfo.longitude_deg;
  m.ellipsoid_height_re_ref_point_m  = msg.pingInfo.ellipsoidHeightReRefPoint_m;
  m.heading_vessel_deg  = msg.pingInfo.headingVessel_deg;
  m.z_water_level_re_ref_point_m  = msg.pingInfo.z_waterLevelReRefPoint_m;
  return m;
}

template <typename EMdgmMRZ_S>
static ds_kongsberg_msgs::msg::KongsbergMRZFull mrz_to_msg_full(const EMdgmMRZ_S& msg)
{
  ds_kongsberg_msgs::msg::KongsbergMRZFull m;
  m.header.stamp = kmall_stamp(msg.header.time_sec, msg.header.time_nanosec);
  m.dgm_version = msg.header.dgmVersion;
  m.system_id = msg.header.systemID;
  m.echo_sounder_id = msg.header.echoSounderID;

  m.num_of_dgms = msg.partition.numOfDgms;
  m.dgm_num = msg.partition.dgmNum;

  m.ping_cnt = msg.cmnPart.pingCnt;
  m.rx_fans_per_ping = msg.cmnPart.rxFansPerPing;
  m.rx_fan_index = msg.cmnPart.rxFanIndex;
  m.swaths_per_ping = msg.cmnPart.swathsPerPing;
  m.swath_along_position = msg.cmnPart.swathAlongPosition;
  m.tx_transducer_ind = msg.cmnPart.txTransducerInd;
  m.rx_transducer_ind = msg.cmnPart.rxTransducerInd;
  m.num_rx_transducers = msg.cmnPart.numRxTransducers;
  m.algorithm_type = msg.cmnPart.algorithmType;

  m.ping_rate_hz = msg.pingInfo.pingRate_Hz;
  m.beam_spacing = msg.pingInfo.beamSpacing;
  m.depth_mode = msg.pingInfo.depthMode;
  m.sub_depth_mode = msg.pingInfo.subDepthMode;
  m.distance_btw_swath = msg.pingInfo.distanceBtwSwath;
  m.detection_mode = msg.pingInfo.detectionMode;
  m.pulse_form = msg.pingInfo.pulseForm;

  m.frequency_mode_hz = msg.pingInfo.frequencyMode_Hz;
  m.freq_range_low_lim_hz = msg.pingInfo.freqRangeLowLim_Hz;
  m.freq_range_high_lim_hz = msg.pingInfo.freqRangeHighLim_Hz;
  m.max_total_tx_pulse_length_sec = msg.pingInfo.maxTotalTxPulseLength_sec;
  m.max_eff_tx_pulse_length_sec = msg.pingInfo.maxEffTxPulseLength_sec;
  m.max_eff_tx_band_width_hz = msg.pingInfo.maxEffTxBandWidth_Hz;
  m.abs_coeff_db_per_km = msg.pingInfo.absCoeff_dBPerkm;
  m.port_sector_edge_deg = msg.pingInfo.portSectorEdge_deg;
  m.starb_sector_edge_deg = msg.pingInfo.starbSectorEdge_deg;
  m.port_mean_cov_deg = msg.pingInfo.portMeanCov_deg;
  m.starb_mean_cov_deg = msg.pingInfo.starbMeanCov_deg;
  m.port_mean_cov_m = msg.pingInfo.portMeanCov_m;
  m.starb_mean_cov_m = msg.pingInfo.starbMeanCov_m;
  m.mode_and_stabilisation = msg.pingInfo.modeAndStabilisation;
  m.runtime_filter1 = msg.pingInfo.runtimeFilter1;
  m.runtime_filter2 = msg.pingInfo.runtimeFilter2;
  m.pipe_tracking_status = msg.pingInfo.pipeTrackingStatus;
  m.transmit_array_size_used_deg = msg.pingInfo.transmitArraySizeUsed_deg;
  m.receive_array_size_used_deg = msg.pingInfo.receiveArraySizeUsed_deg;
  m.transmit_power_db = msg.pingInfo.transmitPower_dB;
  m.sl_ramp_up_time_remaining = msg.pingInfo.SLrampUpTimeRemaining;

  m.yaw_angle_deg = msg.pingInfo.yawAngle_deg;
  m.num_tx_sectors = msg.pingInfo.numTxSectors;
  m.num_bytes_per_tx_sector = msg.pingInfo.numBytesPerTxSector;
  m.heading_vessel_deg = msg.pingInfo.headingVessel_deg;
  m.sound_speed_at_tx_depth_m_per_sec = msg.pingInfo.soundSpeedAtTxDepth_mPerSec;
  m.tx_transducer_depth_m = msg.pingInfo.txTransducerDepth_m;
  m.z_water_level_re_ref_point_m = msg.pingInfo.z_waterLevelReRefPoint_m;
  m.x_kmall_to_all_m = msg.pingInfo.x_kmallToall_m;
  m.y_kmall_to_all_m = msg.pingInfo.y_kmallToall_m;
  m.lat_long_info = msg.pingInfo.latLongInfo;
  m.pos_sensor_status = msg.pingInfo.posSensorStatus;
  m.attitude_sensor_status = msg.pingInfo.attitudeSensorStatus;
  m.latitude_deg = msg.pingInfo.latitude_deg;
  m.longitude_deg = msg.pingInfo.longitude_deg;
  m.ellipsoid_height_re_ref_point_m = msg.pingInfo.ellipsoidHeightReRefPoint_m;

  for(int i = 0; i < msg.pingInfo.numTxSectors; i++)
  {
    ds_kongsberg_msgs::msg::KongsbergMRZtxSectorInfo sectorInfo;
    sectorInfo.tx_sector_numb = msg.sectorInfo[i].txSectorNumb;
    sectorInfo.tx_arr_number = msg.sectorInfo[i].txArrNumber;
    sectorInfo.tx_sub_array = msg.sectorInfo[i].txSubArray;
    sectorInfo.sector_transmit_delay_sec = msg.sectorInfo[i].sectorTransmitDelay_sec;
    sectorInfo.tilt_angle_re_tx_deg = msg.sectorInfo[i].tiltAngleReTx_deg;
    sectorInfo.tx_nominal_source_level_db = msg.sectorInfo[i].txNominalSourceLevel_dB;
    sectorInfo.tx_focus_range_m = msg.sectorInfo[i].txFocusRange_m;
    sectorInfo.centre_freq_hz = msg.sectorInfo[i].centreFreq_Hz;
    sectorInfo.signal_band_width_hz = msg.sectorInfo[i].signalBandWidth_Hz;
    sectorInfo.total_signal_length_sec = msg.sectorInfo[i].totalSignalLength_sec;
    sectorInfo.pulse_shading = msg.sectorInfo[i].pulseShading;
    sectorInfo.signal_wave_form = msg.sectorInfo[i].signalWaveForm;
    m.sector_info.push_back(sectorInfo);
  }

  m.num_bytes_rx_info = msg.rxInfo.numBytesRxInfo;
  m.num_soundings_max_main = msg.rxInfo.numSoundingsMaxMain;
  m.num_soundings_valid_main = msg.rxInfo.numSoundingsValidMain;
  m.num_bytes_per_sounding = msg.rxInfo.numBytesPerSounding;
  m.wc_sample_rate = msg.rxInfo.WCSampleRate;
  m.seabed_image_sample_rate = msg.rxInfo.seabedImageSampleRate;
  m.bs_normal_db = msg.rxInfo.BSnormal_dB;
  m.bs_oblique_db = msg.rxInfo.BSoblique_dB;
  m.extra_detection_alarm_flag = msg.rxInfo.extraDetectionAlarmFlag;
  m.num_extra_detections = msg.rxInfo.numExtraDetections;
  m.num_extra_detection_classes = msg.rxInfo.numExtraDetectionClasses;
  m.num_bytes_per_class = msg.rxInfo.numBytesPerClass;

  std::size_t number_of_ss_samples = 0;
  for(int i = 0; i < msg.rxInfo.numSoundingsMaxMain + msg.rxInfo.numExtraDetections; i++)
  {
    ds_kongsberg_msgs::msg::KongsbergMRZsounding sounding;
    sounding.sounding_index = msg.sounding[i].soundingIndex;
    sounding.tx_sector_numb = msg.sounding[i].txSectorNumb;
    sounding.detection_type = msg.sounding[i].detectionType;
    sounding.detection_method = msg.sounding[i].detectionMethod;
    sounding.rejection_info1 = msg.sounding[i].rejectionInfo1;
    sounding.rejection_info2 = msg.sounding[i].rejectionInfo2;
    sounding.post_processing_info = msg.sounding[i].postProcessingInfo;
    sounding.detection_class = msg.sounding[i].detectionClass;
    sounding.detection_confidence_level = msg.sounding[i].detectionConfidenceLevel;

    sounding.range_factor = msg.sounding[i].rangeFactor;
    sounding.quality_factor = msg.sounding[i].qualityFactor;
    sounding.detection_uncertainty_ver_m = msg.sounding[i].detectionUncertaintyVer_m;
    sounding.detection_uncertainty_hor_m = msg.sounding[i].detectionUncertaintyHor_m;
    sounding.detection_window_length_sec = msg.sounding[i].detectionWindowLength_sec;
    sounding.echo_length_sec = msg.sounding[i].echoLength_sec;
    sounding.wc_beam_numb = msg.sounding[i].WCBeamNumb;
    sounding.wc_range_samples = msg.sounding[i].WCrange_samples;
    sounding.wc_nom_beam_angle_across_deg = msg.sounding[i].WCNomBeamAngleAcross_deg;
    sounding.mean_abs_coeff_db_per_km = msg.sounding[i].meanAbsCoeff_dBPerkm;
    sounding.reflectivity1_db = msg.sounding[i].reflectivity1_dB;
    sounding.reflectivity2_db = msg.sounding[i].reflectivity2_dB;
    sounding.receiver_sensitivity_applied_db = msg.sounding[i].receiverSensitivityApplied_dB;
    sounding.source_level_applied_db = msg.sounding[i].sourceLevelApplied_dB;
    sounding.bs_calibration_db = msg.sounding[i].BScalibration_dB;
    sounding.tvg_db = msg.sounding[i].TVG_dB;
    sounding.beam_angle_re_rx_deg = msg.sounding[i].beamAngleReRx_deg;
    sounding.beam_angle_correction_deg = msg.sounding[i].beamAngleCorrection_deg;
    sounding.two_way_travel_time_sec = msg.sounding[i].twoWayTravelTime_sec;
    sounding.two_way_travel_time_correction_sec = msg.sounding[i].twoWayTravelTimeCorrection_sec;
    sounding.delta_latitude_deg = msg.sounding[i].deltaLatitude_deg;
    sounding.delta_longitude_deg = msg.sounding[i].deltaLongitude_deg;
    sounding.z_re_ref_point_m = msg.sounding[i].z_reRefPoint_m;
    sounding.y_re_ref_point_m = msg.sounding[i].y_reRefPoint_m;
    sounding.x_re_ref_point_m = msg.sounding[i].x_reRefPoint_m;
    sounding.beam_inc_angle_adj_deg = msg.sounding[i].beamIncAngleAdj_deg;
    sounding.real_time_clean_info = msg.sounding[i].realTimeCleanInfo;
    sounding.si_start_range_samples = msg.sounding[i].SIstartRange_samples;
    sounding.si_centre_sample = msg.sounding[i].SIcentreSample;
    sounding.si_num_samples = msg.sounding[i].SInumSamples;
    m.soundings.push_back(sounding);
    number_of_ss_samples += sounding.si_num_samples;
  }

  for(std::size_t i = 0; i < number_of_ss_samples; i++)
  {
    m.si_sample_desi_db.push_back(msg.SIsample_desidB[i]);
  }

  return m;
}


template <typename EMdgmMRZ_S>
static ds_multibeam_msgs::msg::MultibeamRaw mrz_to_mb_raw(EMdgmMRZ_S* msg){
  ds_multibeam_msgs::msg::MultibeamRaw mb{};
  mb.header.stamp = kmall_stamp(msg->header.time_sec, msg->header.time_nanosec);

  // Only take bottom soundings: extra detections are from the water column and shall be ignored
  int num_soundings = msg->rxInfo.numSoundingsMaxMain;
  mb.beamflag.resize(num_soundings);
  mb.twoway_travel_time.resize(num_soundings);
  mb.tx_delay.resize(num_soundings);
  mb.intensity.resize(num_soundings);
  mb.angle_along_track.resize(num_soundings);
  mb.angle_across_track.resize(num_soundings);
  mb.beamwidth_along_track.resize(num_soundings);
  mb.beamwidth_across_track.resize(num_soundings);
  for (int i = 0; i < num_soundings; i++) {
    mb.beamflag[i] = (msg->sounding[i].detectionType == 2
                          ? ds_multibeam_msgs::msg::MultibeamRaw::BEAM_BAD_SONAR
                          : ds_multibeam_msgs::msg::MultibeamRaw::BEAM_OK);
    mb.twoway_travel_time[i] = msg->sounding[i].twoWayTravelTime_sec;
    mb.tx_delay[i] = msg->sounding[i].twoWayTravelTimeCorrection_sec;
    mb.intensity[i] = msg->sounding[i].reflectivity1_dB;
    int sector = msg->sounding[i].txSectorNumb;
    if (sector < msg->pingInfo.numTxSectors){
      mb.angle_along_track[i] = deg_to_rad(msg->sectorInfo[sector].tiltAngleReTx_deg); // use sector index to get tilt angle, then convert to rad
    }
    mb.angle_across_track[i] = deg_to_rad(msg->sounding[i].beamAngleReRx_deg); // convert deg to rad
    mb.beamwidth_along_track[i] = 0;
    mb.beamwidth_across_track[i] = deg_to_rad(msg->sounding[i].WCNomBeamAngleAcross_deg);
  }

  mb.soundspeed = msg->pingInfo.soundSpeedAtTxDepth_mPerSec;
  return mb;
}

template <typename EMdgmMRZ_S>
static sensor_msgs::msg::PointCloud2 mrz_to_pointcloud2(const EMdgmMRZ_S& msg,
                                                  const std::string &frame_id){
  pcl::PointCloud<pcl::PointXYZI> pcl;

  // Only take bottom soundings: extra detections are from the water column and shall be ignored
  int num_soundings = msg.rxInfo.numSoundingsMaxMain;
  pcl::PointXYZI pt;
  for (int i = 0; i < num_soundings; i++)
  {
    // detectionType
    //   0 = normal detection
    //   1 = extra detection
    //   2 = rejected detection In case 2, the estimated range has been used to fill in
    //       amplitude samples in the seabed image datagram.
    //
    // detectionMethod
    //   0 = no valid detection
    //   1 = amplitude detection
    //   2 = phase detection
    //   3-15 for future use.
    if(msg.sounding[i].detectionType == 0 && msg.sounding[i].detectionMethod != 0)
    {
      pt.x = msg.sounding[i].x_reRefPoint_m;
      pt.y = msg.sounding[i].y_reRefPoint_m;
      pt.z = msg.sounding[i].z_reRefPoint_m - msg.pingInfo.z_waterLevelReRefPoint_m;
      pt.intensity = msg.sounding[i].reflectivity1_dB;
      pcl.push_back(pt);
    }
  }

  sensor_msgs::msg::PointCloud2 m;
  pcl::toROSMsg(pcl, m);
  m.header.stamp = kmall_stamp(msg.header.time_sec, msg.header.time_nanosec);
  m.header.frame_id = frame_id;
  return m;
}


} //namespace
