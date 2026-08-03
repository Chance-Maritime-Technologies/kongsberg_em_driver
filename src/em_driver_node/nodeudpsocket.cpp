#include "nodeudpsocket.h"
#include <memory>
#include <boost/asio/ip/multicast.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/asio/post.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/bind/bind.hpp>

using namespace boost::asio::ip;

namespace
{
/// Declare-on-first-use parameter read.
///
/// ROS 1 checked nh.hasParam() for each of the three socket settings and logged FATAL if any
/// were absent, then went ahead with the default anyway. ROS 2 gives a declared parameter its
/// default immediately, so "absent" is no longer observable without the Galactic+
/// declare_parameter(name, type) overload. The resolved value is logged instead, which
/// surfaces a misconfigured socket just as effectively.
template <typename T>
T socket_param(const rclcpp::Node::SharedPtr &node, const std::string &name, const T &fallback)
{
  const T value = node->has_parameter(name) ? node->get_parameter(name).get_value<T>()
                                            : node->declare_parameter<T>(name, fallback);
  RCLCPP_INFO_STREAM(node->get_logger(), "  " << name << " = " << value);
  return value;
}
}  // namespace

  #if BOOST_VERSION < 106600
  NodeUdpSocket::NodeUdpSocket(boost::asio::io_service& io_service, const std::string& name,
                               rclcpp::Node::SharedPtr node)
#else
  NodeUdpSocket::NodeUdpSocket(boost::asio::io_context& io_service, const std::string& name,
                               rclcpp::Node::SharedPtr node)
#endif
: name_(name), node_(node), io_context_(io_service)
{
  recv_buffer_.resize(65536);
  // ROS 2 parameter names nest with '.' rather than '/'
  const auto PARAM_PREFIX = name + ".";

  RCLCPP_INFO_STREAM(node_->get_logger(), "UDP socket " << name << " configuration:");
  int udp_rx_port = socket_param<int>(node_, PARAM_PREFIX + "udp_rx_port", 4444);
  int udp_tx_port = socket_param<int>(node_, PARAM_PREFIX + "udp_tx_port", 5555);
  std::string udp_address = socket_param<std::string>(node_, PARAM_PREFIX + "udp_address", "127.0.0.1");

  // Create the socket and endpoint
  auto udp_ip = address::from_string(udp_address);
  if (!udp_ip.is_multicast()){
    // Create the socket
    socket_ = std::make_unique<udp::socket>(io_service, udp::endpoint(udp::v4(), udp_rx_port));;
    remote_endpoint_ = udp::endpoint(udp_ip, udp_tx_port);
  } else {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Multicast address "<<udp_address<<" receive only");
    // Create the socket
    socket_ = std::unique_ptr<udp::socket>(new udp::socket(io_service));
    auto listen_address = address::from_string("0.0.0.0");
    udp::endpoint listen_endpoint(listen_address, udp_rx_port);
    socket_->open(listen_endpoint.protocol());
//    socket_->set_option(udp::socket::reuse_address(true));
    socket_->bind(listen_endpoint);

    socket_->set_option(multicast::join_group(udp_ip));

    remote_endpoint_ = udp::endpoint();
    remote_endpoint_.port(udp_tx_port);
  }
  // Check for broadcast addresses
  std::vector<std::string> split_add;
  boost::algorithm::split(split_add, udp_address, boost::is_any_of("."));
  if (split_add.size() > 0)
  {
    if (!split_add.back().compare("255"))
    {
      RCLCPP_INFO_STREAM(node_->get_logger(), "Setting socket broadcast option");
      boost::asio::socket_base::broadcast option(true);
      socket_->set_option(option);
    }
  }

  receive();
}



void NodeUdpSocket::sendData(const std::string &data)
{
  // Called from the rclcpp executor thread, whereas the receive chain runs on the io_context
  // thread. A boost::asio socket may not be used concurrently from two threads, so hop onto
  // the io_context before touching it. Under ROS 1 both lived on the same thread (courtesy of
  // AsioCallbackQueue) and this could go straight to async_send_to.
  //
  // The payload is also copied into a shared_ptr the completion handler keeps alive: the
  // caller's string dies at the end of the calling statement, but async_send_to needs the
  // buffer to stay valid until the write completes.
  auto payload = std::make_shared<std::string>(data);
  auto do_send = [this, payload]() {
    socket_->async_send_to(boost::asio::buffer(*payload), remote_endpoint_,
                           [this, payload](const boost::system::error_code &error, std::size_t) {
                             handle_send(error);
                           });
  };
#if BOOST_VERSION < 106600
  io_context_.post(do_send);
#else
  boost::asio::post(io_context_, do_send);
#endif
}

void NodeUdpSocket::receive(void)
{
  recv_buffer_.assign(recv_buffer_.size(), 0);
  socket_->async_receive(boost::asio::buffer(recv_buffer_), 0,
                         boost::bind(&NodeUdpSocket::handle_receive, this, boost::asio::placeholders::error,
                                     boost::asio::placeholders::bytes_transferred));
}

void NodeUdpSocket::handle_receive(const boost::system::error_code &error, std::size_t bytes_transferred)
{
  if (!error || error == boost::asio::error::message_size)
  {
    // Store timestamp as soon as received
    ds_core_msgs::msg::RawData m;
    m.ds_header.io_time = node_->now();
    m.data = std::vector<unsigned char>(recv_buffer_.begin(), recv_buffer_.begin() + bytes_transferred);
    m.data_direction = ds_core_msgs::msg::RawData::DATA_IN;

    if (recv_cb_)
    {
      recv_cb_(m);
    }

    receive();
    if (error == boost::asio::error::message_size)
    {
      RCLCPP_ERROR_STREAM(node_->get_logger(), "UDP Socket message size error.Bytes received" << bytes_transferred);
    }
  }
}

void NodeUdpSocket::handle_send(const boost::system::error_code &error)
{
  if (error.value() != boost::system::errc::success)
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Failed sending udp data (" << name_ << "):" << error.message());
  }
}


