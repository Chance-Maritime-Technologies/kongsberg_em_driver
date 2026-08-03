#pragma once

#include <rclcpp/rclcpp.hpp>
#include <boost/asio/ip/udp.hpp>
#include <ds_core_msgs/msg/raw_data.hpp>
#include <boost/utility.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>

using ReceiveCallback = std::function<void(const ds_core_msgs::msg::RawData&)>;

/// \brief A boost::asio UDP socket that stamps received datagrams with ROS time.
///
/// The socket is serviced by whichever thread runs the io_context, which in this node is a
/// thread of its own rather than the rclcpp executor. The receive callback therefore fires
/// off-executor and the driver on the other end is responsible for its own locking.
class NodeUdpSocket : boost::noncopyable
{
 public:
  #if BOOST_VERSION < 106600
  NodeUdpSocket(boost::asio::io_service &io_service, const std::string &name,
                rclcpp::Node::SharedPtr node);
#else
  NodeUdpSocket(boost::asio::io_context &io_service, const std::string &name,
                rclcpp::Node::SharedPtr node);
#endif

  void sendData(const std::string &data);
  void setReceiveCb(const ReceiveCallback recv_cb)
  {
    recv_cb_ = recv_cb;
  }

 private:
  std::string name_;
  rclcpp::Node::SharedPtr node_;

  // Held so sendData() can marshal onto the io_context thread instead of touching the
  // socket from whichever thread happened to call it.
#if BOOST_VERSION < 106600
  boost::asio::io_service &io_context_;
#else
  boost::asio::io_context &io_context_;
#endif

  void handle_receive(const boost::system::error_code& error, std::size_t bytes_transferred);
  void receive();
  void handle_send(const boost::system::error_code &error);

  std::unique_ptr<boost::asio::ip::udp::socket> socket_;
  boost::asio::ip::udp::endpoint remote_endpoint_;

  std::vector<char> recv_buffer_;
  ReceiveCallback recv_cb_;
};
