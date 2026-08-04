/**
* Copyright 2018 Woods Hole Oceanographic Institution
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
//
// Created by jvaccaro on 5/28/19.
//
#include "lib_kongsberg_em/kongsberg_em2040.h"
#include "../../src/em_driver_library/EM_datagrams/KMALL_mrz_decoder.h"

#include <list>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

// Test fixture
//
// The ROS 1 version pulled these paths from the rostest launch file. ROS 2 has no
// add_rostest_gtest equivalent, so they are plain node parameters whose defaults resolve
// against the WORKING_DIRECTORY (the package source dir) set in CMakeLists.txt.
class Em2040Test : public::testing::Test
{
 public:
  rclcpp::Node::SharedPtr node;
  std::string pass_path;
  std::string mrz_version_f_path;
  std::string mrz_version_h_path;
  std::string fail_path;

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_em2040_parse");
    pass_path = node->declare_parameter<std::string>("pass_path", "test/pass.kmall");
    mrz_version_f_path = node->declare_parameter<std::string>("mrz_f_path", "test/MRZ_versionF.kmall");
    mrz_version_h_path = node->declare_parameter<std::string>("mrz_h_path", "test/MRZ_versionH.kmall");
    fail_path = node->declare_parameter<std::string>("fail_path", "test/kmall_fail.BIN");
  }

  void TearDown() override
  {
    node.reset();
  }
};

void sendKCtrlData(const std::string &data){
  (void)data;
}

TEST_F(Em2040Test, alwaysPass)
{
  EXPECT_TRUE(true);
}

TEST_F(Em2040Test, parseMRZversionF)
{
  FILE *fp = fopen(mrz_version_f_path.c_str(), "r");

  if (fp != nullptr)
  {
    // read file size
    fseek(fp, 0L, SEEK_END);
    auto size = ftell(fp);
    rewind(fp);
    auto byte_msg = ds_core_msgs::msg::RawData{};
    byte_msg.data.resize(size);
    uint8_t foo[size];

    fread(&foo, 1, size, fp);
    fclose(fp);
    for (int i = 0; i < size; i++)
    {
      byte_msg.data[i] = foo[i];
    }
    byte_msg.ds_header.io_time = node->now();
    EMdgm_f::EMdgmMRZ mrz;
    bool ok = false;
    std::tie(ok, mrz) = kmall::read_mrz<EMdgm_f::EMdgmMRZ>(byte_msg.data.data(), byte_msg.data.size());
    EXPECT_TRUE(ok);
  }else{
    FAIL() << "Failed reading file: " << mrz_version_f_path;
  }
}

TEST_F(Em2040Test, parseMRZversionH)
{
  FILE *fp = fopen(mrz_version_h_path.c_str(), "r");

  if (fp != nullptr)
  {
    // read file size
    fseek(fp, 0L, SEEK_END);
    auto size = ftell(fp);
    rewind(fp);
    auto byte_msg = ds_core_msgs::msg::RawData{};
    byte_msg.data.resize(size);
    uint8_t foo[size];

    fread(&foo, 1, size, fp);
    fclose(fp);
    for (int i = 0; i < size; i++)
    {
      byte_msg.data[i] = foo[i];
    }
    byte_msg.ds_header.io_time = node->now();
    EMdgm_h::EMdgmMRZ mrz;
    bool ok = false;
    std::tie(ok, mrz) = kmall::read_mrz<EMdgm_h::EMdgmMRZ>(byte_msg.data.data(), byte_msg.data.size());
    EXPECT_TRUE(ok);
  }else{
    FAIL() << "Failed reading file: " << mrz_version_h_path;
  }
}

TEST_F(Em2040Test, parsePass)
{
  kongsberg_em::KongsbergEM2040 driver(node, &sendKCtrlData);
  driver.setupAll();
  long long int index = 0;
  const auto test_ls = std::list<std::pair<bool, int>>{
    {true, 1104}, // #IIP
    {true, 1308}, // #IOP
    {true, 1192}, // #SVP
    {true, 19572},// #SKM
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 55304},// #MRZ
    {true, 55236},// #MRZ
    {true, 19572},// #SKM
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 19572},// #SKM
    {true, 55398},// #MRZ
    {true, 55336},// #MRZ
    {true, 19572},// #MRZ
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156},  // #CPO
    {true, 56332},// #MRZ
    {true, 55880},// #MRZ
    {true, 19572},// #SKM
    {true, 74},   // #SCL
    {true, 156},  // #SPO
    {true, 156}  // #CPO
  };
  for (auto test_l : test_ls){
    auto pass = test_l.first;
    auto size = test_l.second;
    RCLCPP_INFO_STREAM(node->get_logger(), " >> Reading " << size << " bytes from index " << index);
    auto byte_msg = ds_core_msgs::msg::RawData{};
    bool ok = false;
    byte_msg.data.resize(size);
    uint8_t foo[size];
    FILE* file = fopen(pass_path.c_str(), "r");
    if (file!=nullptr)
    {
      fseek(file, index, SEEK_SET);
      fread(&foo, 1, size, file);
      fclose(file);
      for (int i=0; i<size; i++)
      {
        byte_msg.data[i] = foo[i];
      }
      byte_msg.ds_header.io_time = node->now();
      ok = driver.parse_data(byte_msg);
    }
    RCLCPP_ERROR_STREAM_EXPRESSION(node->get_logger(), pass != ok, "Failed on message of size " << size);
    EXPECT_EQ(pass, ok);
    index += size;
  }

}

// XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
// Run all the tests that were declared with TEST()
int main(int argc, char** argv)
{
  /* To enable DEBUG logging, either pass --ros-args --log-level debug or call:
   * rcutils_logging_set_logger_level("test_em2040_parse", RCUTILS_LOG_SEVERITY_DEBUG);
   */
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  auto ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
