#include "lib_kongsberg_em/kongsberg_em2040.h"
#include "nodeudpsocket.h"

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <thread>

int main(int argc, char* argv[])
{
  // Boost Event loop handler
#if BOOST_VERSION < 106600
    auto io_context = std::make_shared<boost::asio::io_service>();
#else
    auto io_context = std::make_shared<boost::asio::io_context>();
#endif

  // ROS setup. rclcpp installs its own SIGINT handler, which shuts down the context and so
  // drops us out of spin() below -- there is no equivalent of ROS 1's NoSigintHandler to set.
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("kongsberg_em_node");

  // UDP sockets for sonar communication
  NodeUdpSocket kctrl_socket(*io_context, "kctrl_connection", node);
  NodeUdpSocket kmall_socket(*io_context, "kmall_connection", node);

  // The driver will send commands using the kctrl socket
  auto sendKCtrlData = [&kctrl_socket](const std::string& data) { kctrl_socket.sendData(data); };
  kongsberg_em::KongsbergEM2040 driver(node, sendKCtrlData);
  driver.setupAll();
  // The driver will receive data from the sockets using these call backs
  kctrl_socket.setReceiveCb(
      [&driver](const ds_core_msgs::msg::RawData& raw) { driver._on_kctrl_data(raw); });
  kmall_socket.setReceiveCb(
      [&driver](const ds_core_msgs::msg::RawData& raw) { driver._on_kmall_data(raw); });

  // ROS 2 has no global callback queue to fold the asio event loop into, the way
  // librosasio's AsioCallbackQueue did under ROS 1. So the io_context gets a thread of its
  // own and the executor keeps this one. That makes the socket callbacks concurrent with the
  // timer and service callbacks, which is why KongsbergEM2040 locks its own state.
  std::thread io_thread([io_context]() { io_context->run(); });

  rclcpp::spin(node);

  io_context->stop();
  io_thread.join();
  rclcpp::shutdown();

  return 0;
}
