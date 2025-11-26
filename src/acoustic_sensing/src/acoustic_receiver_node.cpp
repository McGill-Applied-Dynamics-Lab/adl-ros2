#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include "acoustic_sensing/msg/acoustic_packet.hpp"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <atomic>
#include <vector>

using acoustic_sensing::msg::AcousticPacket;

class AcousticReceiverNode : public rclcpp::Node
{
public:
  AcousticReceiverNode()
  : Node("acoustic_receiver_node"),
    server_fd_(-1),
    client_fd_(-1),
    running_(true)
  {
    declare_parameter<int>("port", 1234);
    declare_parameter<bool>("two_rf_mode", false);
    declare_parameter<int>("buf_size", 4000);
    declare_parameter<std::string>("frame_id", "beaglebone_acoustic");

    port_ = get_parameter("port").as_int();
    two_rf_mode_ = get_parameter("two_rf_mode").as_bool();
    buf_size_ = get_parameter("buf_size").as_int();
    frame_id_ = get_parameter("frame_id").as_string();

    publisher_ = create_publisher<AcousticPacket>(
      "/acoustic/raw",
      rclcpp::SensorDataQoS().keep_all().reliable()
    );

    if (!setup_server())
      throw std::runtime_error("Failed to open server socket.");

    receiver_thread_ = std::thread(&AcousticReceiverNode::receive_loop, this);
  }

  ~AcousticReceiverNode()
  {
    running_ = false;

    if (client_fd_ >= 0) {
      shutdown(client_fd_, SHUT_RDWR);
      close(client_fd_);
    }
    if (server_fd_ >= 0) {
      shutdown(server_fd_, SHUT_RDWR);
      close(server_fd_);
    }

    if (receiver_thread_.joinable())
      receiver_thread_.join();
  }

private:
  bool setup_server()
  {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0)
      return false;

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);

    if (bind(server_fd_, (sockaddr*)&addr, sizeof(addr)) < 0)
      return false;

    if (listen(server_fd_, 1) < 0)
      return false;

    RCLCPP_INFO(get_logger(), "Listening on port %d...", port_);
    return true;
  }

  int accept_connection()
  {
    sockaddr_in caddr{};
    socklen_t clen = sizeof(caddr);

    int fd = accept(server_fd_, (sockaddr*)&caddr, &clen);
    if (fd < 0)
      return -1;

    char ip[32];
    inet_ntop(AF_INET, &caddr.sin_addr, ip, sizeof(ip));
    RCLCPP_INFO(get_logger(), "BeagleBone connected from %s", ip);

    return fd;
  }

  bool recv_all(int fd, void* buf, size_t len)
  {
    size_t r = 0;
    uint8_t* p = (uint8_t*)buf;

    while (r < len && running_) {
      ssize_t ret = recv(fd, p + r, len - r, 0);
      if (ret <= 0)
        return false;
      r += ret;
    }
    return true;
  }

  void recv_single_rf(std::vector<uint16_t>& samples)
  {
    uint16_t tdiff;
    if (!recv_all(client_fd_, &tdiff, sizeof(tdiff))) {
      handle_disconnect();
      return;
    }

    if (!recv_all(client_fd_, samples.data(), samples.size() * sizeof(uint16_t))) {
      handle_disconnect();
      return;
    }

    AcousticPacket msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_id_;
    msg.rf_id = 1;
    msg.time_offset_ms = tdiff;
    msg.samples.assign(samples.begin(), samples.end());

    publisher_->publish(msg);
  }

  void recv_dual_rf(std::vector<uint16_t>& samples)
  {
    uint8_t rf;
    uint16_t tdiff;

    // --- RF1 ---
    if (!recv_all(client_fd_, &rf, sizeof(rf)) ||
        !recv_all(client_fd_, &tdiff, sizeof(tdiff)) ||
        !recv_all(client_fd_, samples.data(), samples.size() * sizeof(uint16_t))) {
      handle_disconnect();
      return;
    }

    {
      AcousticPacket msg;
      msg.header.stamp = now();
      msg.header.frame_id = frame_id_;
      msg.rf_id = rf;
      msg.time_offset_ms = tdiff;
      msg.samples.assign(samples.begin(), samples.end());
      publisher_->publish(msg);
    }

    // --- RF2 ---
    if (!recv_all(client_fd_, &rf, sizeof(rf)) ||
        !recv_all(client_fd_, &tdiff, sizeof(tdiff)) ||
        !recv_all(client_fd_, samples.data(), samples.size() * sizeof(uint16_t))) {
      handle_disconnect();
      return;
    }

    {
      AcousticPacket msg;
      msg.header.stamp = now();
      msg.header.frame_id = frame_id_;
      msg.rf_id = rf;
      msg.time_offset_ms = tdiff;
      msg.samples.assign(samples.begin(), samples.end());
      publisher_->publish(msg);
    }
  }

  void receive_loop()
  {
    std::vector<uint16_t> samples(buf_size_);

    while (running_) {
      if (client_fd_ < 0) {
        client_fd_ = accept_connection();
        if (client_fd_ < 0)
          continue;
      }

      if (!two_rf_mode_)
        recv_single_rf(samples);
      else
        recv_dual_rf(samples);
    }
  }

  void handle_disconnect()
  {
    if (client_fd_ >= 0) {
      RCLCPP_WARN(get_logger(), "BeagleBone disconnected.");
      shutdown(client_fd_, SHUT_RDWR);
      close(client_fd_);
      client_fd_ = -1;
    }
  }

  int port_;
  bool two_rf_mode_;
  int buf_size_;
  std::string frame_id_;
  int server_fd_;
  int client_fd_;
  std::atomic<bool> running_;
  std::thread receiver_thread_;
  rclcpp::Publisher<AcousticPacket>::SharedPtr publisher_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AcousticReceiverNode>());
  rclcpp::shutdown();
  return 0;
}
