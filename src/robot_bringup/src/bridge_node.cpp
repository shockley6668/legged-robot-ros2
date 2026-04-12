#include <chrono>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
#include <cmath>

using namespace std::chrono_literals;

#define RAD_TO_DEG 57.295779513082320876798154814105f

// --- MAVLink v2 Lite Implementation ---
class MAVLinkV2 {
public:
  static constexpr uint8_t STX = 0xFD;

  static uint16_t crc_accumulate(uint8_t data, uint16_t crc) {
    uint8_t tmp = data ^ (uint8_t)(crc & 0xFF);
    tmp ^= (tmp << 4);
    return (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
  }

  // Appends a packet to the provided buffer
  static void pack_append(uint32_t msgid, const std::vector<uint8_t> &payload,
                          uint8_t seq, std::vector<uint8_t> &buffer) {
    uint8_t len = static_cast<uint8_t>(payload.size());
    buffer.reserve(buffer.size() + 12 + len);

    buffer.push_back(STX);
    buffer.push_back(len);
    buffer.push_back(0); // incompat
    buffer.push_back(0); // compat
    buffer.push_back(seq);
    buffer.push_back(1); // sysid
    buffer.push_back(1); // compid
    buffer.push_back(msgid & 0xFF);
    buffer.push_back((msgid >> 8) & 0xFF);
    buffer.push_back((msgid >> 16) & 0xFF);

    buffer.insert(buffer.end(), payload.begin(), payload.end());

    uint16_t crc = 0xFFFF;
    // Calculate CRC starting from LEN byte (index 1 relative to start of
    // packet) The last inserted byte is at buffer.size() - 1 The packet started
    // at (buffer.size() - (10 + len)) ? No. Length added so far: 10 bytes
    // (header) + len bytes (payload) = 10 + len Packet start index in buffer:
    // buffer.size() - (10 + len)
    size_t packet_start = buffer.size() - (10 + len);

    // Checksum covers from LEN to end of payload
    // i.e., buffer[packet_start + 1] to buffer[packet_start + 10 + len - 1]
    for (size_t i = packet_start + 1; i < buffer.size(); ++i) {
      crc = crc_accumulate(buffer[i], crc);
    }

    buffer.push_back(crc & 0xFF);
    buffer.push_back((crc >> 8) & 0xFF);
  }
};

// --- Serial Port Helper (Termios) ---
class SerialPort {
public:
  SerialPort() : fd_(-1) {}
  ~SerialPort() { close_port(); }

  bool open_port(const std::string &port, int baud) {
    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1)
      return false;

    struct termios options;
    tcgetattr(fd_, &options);

    speed_t speed;
    switch (baud) {
    case 9600:
      speed = B9600;
      break;
    case 115200:
      speed = B115200;
      break;
    case 921600:
      speed = B921600;
      break;
    default:
      speed = B921600; // Use high speed default
    }

    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // CRITICAL FIX: Disable special character translation for binary communication!
    // Without ICRNL/INLCR/IGNCR it converts 0x0D (CR) to 0x0A (LF), corrupting payload and CRC!
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR | ISTRIP | BRKINT);
    
    options.c_oflag &= ~OPOST;

    tcsetattr(fd_, TCSANOW, &options);

    // Flush existing data
    tcflush(fd_, TCIOFLUSH);

    return true;
  }

  void close_port() {
    if (fd_ != -1) {
      close(fd_);
      fd_ = -1;
    }
  }

  int read_data(uint8_t *buf, int max_len) {
    if (fd_ == -1)
      return -1;
    int n = read(fd_, buf, max_len);
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
      return -2;
    return n;
  }

  bool write_data(const std::vector<uint8_t> &data) {
    if (fd_ == -1)
      return false;
    ssize_t written = 0;
    size_t total = data.size();
    while (written < (ssize_t)total) {
      ssize_t n = write(fd_, data.data() + written, total - written);
      if (n < 0) {
        if (errno == EAGAIN)
          continue;
        close_port();
        return false;
      }
      written += n;
    }
    return true;
  }

  bool is_open() const { return fd_ != -1; }

private:
  int fd_;
};

// --- Motor Utilities ---
struct MotorParams {
  float p_min, p_max;
  float v_min, v_max;
  float kp_min, kp_max;
  float kd_min, kd_max;
  float t_min, t_max;
};

class DM_Utils {
public:
  static uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    if (x < x_min)
      x = x_min;
    if (x > x_max)
      x = x_max;
    return (uint16_t)((x - x_min) * ((1 << bits) - 1) / span);
  }

  static float uint_to_float(uint16_t x_int, float x_min, float x_max,
                             int bits) {
    float span = x_max - x_min;
    return (float)x_int * span / ((1 << bits) - 1) + x_min;
  }
};

// --- ROS 2 Node ---
class LegMavlinkBridge : public rclcpp::Node {
public:
  LegMavlinkBridge() : Node("leg_mavlink_cpp_bridge"), seq_(0), rx_mask_(0) {
    this->declare_parameter("port", "/dev/ttyACM0");
    this->declare_parameter("baud", 921600);

    port_ = this->get_parameter("port").as_string();
    baud_ = this->get_parameter("baud").as_int();

    try_open();

    // Define Motor Specs
    // DM6006 (0, 4, 5, 9)
    params_6006_ = {-12.5f, 12.5f, -45.0f, 45.0f,  0.0f,
                    500.0f, 0.0f,  5.0f,   -12.0f, 12.0f};
    // DM8006 (Others)
    params_8006_ = {-12.5f, 12.5f, -45.0f, 45.0f,  0.0f,
                    500.0f, 0.0f,  5.0f,   -20.0f, 20.0f};

    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
    imu_status_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
        "imu/status", 10);

    cmd_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "motor_cmds", 1,
        std::bind(&LegMavlinkBridge::cmd_callback, this,
                  std::placeholders::_1));

    // Increased timer frequency to process buffer frequently (2000Hz)
    timer_ = this->create_wall_timer(
        500us, std::bind(&LegMavlinkBridge::receive_loop, this));

    joint_msg_.name = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4",
                       "joint_5", "joint_6", "joint_7", "joint_8", "joint_9"};
    joint_msg_.position.resize(10, 0.0);
    joint_msg_.velocity.resize(10, 0.0);
    joint_msg_.position.resize(10, 0.0);
    joint_msg_.velocity.resize(10, 0.0);
    // joint_msg_.effort.resize(10, 0.0); // Effort removed to save bandwidth

    rx_buffer_.reserve(2048);

    // Service to save zero position
    save_zero_srv_ = this->create_service<std_srvs::srv::Empty>(
        "set_zero_position",
        std::bind(&LegMavlinkBridge::save_zero_callback, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Topic to save zero position for specific motor (or all if -1)
    set_zero_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "set_zero_cmd", 1,
        std::bind(&LegMavlinkBridge::set_zero_cmd_callback, this,
                  std::placeholders::_1));
  }

private:
  void save_zero_callback(
      const std::shared_ptr<std_srvs::srv::Empty::Request> request,
      std::shared_ptr<std_srvs::srv::Empty::Response> response) {
    (void)request;
    (void)response;
    RCLCPP_INFO(
        this->get_logger(),
        "Service called: Sending Save Zero Position command to ALL motors.");

    // Payload: 'S' 'T' 0xFF (All)
    std::vector<uint8_t> payload = {0x53, 0x54, 0xFF};
    std::vector<uint8_t> buffer;
    MAVLinkV2::pack_append(0xFF, payload, seq_++, buffer);
    serial_.write_data(buffer);
  }

  void set_zero_cmd_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    uint8_t target_id = 0xFF;
    if (msg->data >= 0 && msg->data <= 9) {
      target_id = static_cast<uint8_t>(msg->data);
      RCLCPP_INFO(this->get_logger(), "Received set_zero_cmd for Motor %d",
                  target_id);
    } else if (msg->data == -1) {
      target_id = 0xFF; // All
      RCLCPP_INFO(this->get_logger(), "Received set_zero_cmd for ALL Motors");
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid motor ID for set_zero_cmd: %d",
                  msg->data);
      return;
    }

    // Payload: 'S' 'T' [ID]
    std::vector<uint8_t> payload = {0x53, 0x54, target_id};
    std::vector<uint8_t> buffer;
    MAVLinkV2::pack_append(0xFF, payload, seq_++, buffer);
    serial_.write_data(buffer);
  }
  // Batch Command: Send only Positions for 10 motors (MsgID 0x02)
  // Payload: 10 motors * 2 bytes = 20 bytes.
  // Kp=15, Kd=0.5, V=0, T=0 are fixed on MCU side.
  // Batch Command: Send only Positions for 10 motors (MsgID 0x02)
  // Payload: 10 motors * 2 bytes = 20 bytes.
  // Kp=15, Kd=0.5, V=0, T=0 are fixed on MCU side.
  void cmd_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() != 10)
      return;

    std::vector<uint8_t> payload(20);

    for (int i = 0; i < 10; ++i) {
      // Input: [p0, p1, ..., p9]
      float p = msg->data[i];

      // Reverse direction for all motors EXCEPT 2 and 7 (match feedback logic)
      if (i != 2 && i != 7) {
        p = -p;
      }

      const MotorParams &pms = is_6006(i) ? params_6006_ : params_8006_;
      uint16_t p_int = DM_Utils::float_to_uint(p, pms.p_min, pms.p_max, 16);

      payload[i * 2 + 0] = (p_int >> 8) & 0xFF;
      payload[i * 2 + 1] = p_int & 0xFF;
    }

    // Send single packet
    std::vector<uint8_t> buffer;
    MAVLinkV2::pack_append(0x02, payload, seq_++, buffer);
    serial_.write_data(buffer);
  }

  void try_open() {
    if (serial_.open_port(port_, baud_)) {
      RCLCPP_INFO(this->get_logger(), "Serial port opened: %s", port_.c_str());
    }
  }

  void receive_loop() {
    if (!serial_.is_open()) {
      static auto last_try = std::chrono::steady_clock::now();
      auto now = std::chrono::steady_clock::now();
      if (now - last_try > 1s) {
        try_open();
        last_try = now;
      }
      return;
    }

    // Non-blocking bulk read
    uint8_t buf[1024];
    int n = serial_.read_data(buf, sizeof(buf));
    if (n > 0) {
      rx_buffer_.insert(rx_buffer_.end(), buf, buf + n);
      process_buffer();
    }
  }

  void process_buffer() {
    // Sliding window approach: search_idx keeps track of where we are evaluating
    size_t search_idx = 0;

    while (search_idx + 12 <= rx_buffer_.size()) { // Min Frame Size: 10 header + 0 payload + 2 CRC = 12
      // Look for STX
      if (rx_buffer_[search_idx] != MAVLinkV2::STX) {
        search_idx++;
        continue;
      }

      uint8_t len = rx_buffer_[search_idx + 1];
      uint32_t total_len = 12 + len;

      if (search_idx + total_len > rx_buffer_.size()) {
        // Not enough data for the full frame yet, stop processing and wait for more
        break;
      }

      // Check CRC
      uint16_t crc = 0xFFFF;
      for (size_t i = 1; i < total_len - 2; ++i) {
        crc = MAVLinkV2::crc_accumulate(rx_buffer_[search_idx + i], crc);
      }
      uint8_t crc_low = rx_buffer_[search_idx + total_len - 2];
      uint8_t crc_high = rx_buffer_[search_idx + total_len - 1];

      if ((crc & 0xFF) == crc_low && ((crc >> 8) & 0xFF) == crc_high) {
        // Valid Frame, Extract ID and Payload
        uint32_t msgid = rx_buffer_[search_idx + 7] | 
                         (rx_buffer_[search_idx + 8] << 8) | 
                         (rx_buffer_[search_idx + 9] << 16);

        std::vector<uint8_t> payload(rx_buffer_.begin() + search_idx + 10,
                                     rx_buffer_.begin() + search_idx + 10 + len);

        if (msgid == 0x02) {
          parse_feedback(payload);
        } else if (msgid == 0x03) {
          // Single batch packet (Motors 0-9) -> Update and Publish immediately
          parse_batch_feedback(payload, true);
        } else if (msgid == 0x04) {
          parse_imu(payload);
        }

        // Successfully parsed, skip over this entire message
        search_idx += total_len;
      } else {
        // Invalid CRC. We just skip the STX byte to re-sync
        static int crc_err_cnt = 0;
        if (++crc_err_cnt % 100 == 0) {
          RCLCPP_WARN(this->get_logger(), "CRC Fail! Total: %d. Found len: %d", crc_err_cnt, len);
        }
        search_idx++;
      }
    }

    // Clean up processed bytes from the front of the vector in O(N) operations once
    if (search_idx > 0) {
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + search_idx);
    }
  }

  // Parse batch feedback (MsgID 0x03): New Optimized 40-byte protocol
  // Payload: 10 motors * 4 bytes/motor (2 bytes P, 2 bytes V)
  void parse_batch_feedback(const std::vector<uint8_t> &payload,
                            bool publish_now) {
    // New protocol expects at least 4 bytes per motor
    if (payload.size() < 4)
      return;

    // Detect payload format by stride
    // 40 bytes = 10 motors * 4 bytes
    size_t num_motors = payload.size() / 4;

    for (size_t k = 0; k < num_motors; k++) {
      uint8_t offset = k * 4;
      uint8_t idx = k; // Motor ID is implied 0..9

      if (idx >= 10)
        continue;

      uint16_t p_int = (payload[offset + 0] << 8) | payload[offset + 1];
      uint16_t v_int = (payload[offset + 2] << 8) | payload[offset + 3];

      const MotorParams &pms = is_6006(idx) ? params_6006_ : params_8006_;
      float pos = DM_Utils::uint_to_float(p_int, pms.p_min, pms.p_max, 16);

      // Velocity mapped as 12-bit resolution by driver usually, but sent as 2
      // bytes. Assuming straightforward map.
      float vel = DM_Utils::uint_to_float(v_int, pms.v_min, pms.v_max, 12);

      float tor = 0.0f; // Torque data optimized out

      // Reverse direction except 2 and 7
      if (idx != 2 && idx != 7) {
        pos = -pos;
        vel = -vel;
      }

      joint_msg_.position[idx] = pos;
      joint_msg_.velocity[idx] = vel;
      // joint_msg_.effort[idx] = tor; // Removed
    }

    if (publish_now) {
      joint_msg_.header.stamp = this->get_clock()->now();
      joint_pub_->publish(joint_msg_);
    }

    // Debug: Log all motors every second
    // static auto last_log = std::chrono::steady_clock::now();
    // auto now = std::chrono::steady_clock::now();
    // if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log)
    //         .count() >= 1000) {
    //   RCLCPP_INFO(this->get_logger(),
    //               "Motors [0-9]: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, "
    //               "%.2f, %.2f, %.2f",
    //               joint_msg_.position[0], joint_msg_.position[1],
    //               joint_msg_.position[2], joint_msg_.position[3],
    //               joint_msg_.position[4], joint_msg_.position[5],
    //               joint_msg_.position[6], joint_msg_.position[7],
    //               joint_msg_.position[8], joint_msg_.position[9]);
    //   last_log = now;
    // }
  }

  // Legacy: Parse single motor feedback (msgid 0x02) - kept for compatibility
  void parse_feedback(const std::vector<uint8_t> &payload) {
    if (payload.size() < 6)
      return;
    uint8_t idx = payload[0];
    if (idx >= 10)
      return;

    // Decode Data
    uint16_t p_int = (payload[1] << 8) | payload[2];
    uint16_t v_int = (payload[3] << 4) | (payload[4] >> 4);
    uint16_t t_int = ((payload[4] & 0x0F) << 8) | payload[5];

    const MotorParams &pms = is_6006(idx) ? params_6006_ : params_8006_;
    joint_msg_.position[idx] =
        DM_Utils::uint_to_float(p_int, pms.p_min, pms.p_max, 16);
    joint_msg_.velocity[idx] =
        DM_Utils::uint_to_float(v_int, pms.v_min, pms.v_max, 12);
    joint_msg_.effort[idx] =
        DM_Utils::uint_to_float(t_int, pms.t_min, pms.t_max, 12);

    // Throttled publishing: max 100Hz
    static auto last_pub = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();

    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pub)
            .count() >= 10) {
      joint_msg_.header.stamp = this->get_clock()->now();
      joint_pub_->publish(joint_msg_);
      last_pub = now;
    }
  }

  void parse_imu(const std::vector<uint8_t> &payload) {
    if (payload.size() < 40)
      return;
    auto imu_msg = sensor_msgs::msg::Imu();
    imu_msg.header.stamp = this->get_clock()->now();
    imu_msg.header.frame_id = "imu_link";

    float q[4], gyro[3], accel[3];
    std::memcpy(q, payload.data(), 16);
    std::memcpy(gyro, payload.data() + 16, 12);
    std::memcpy(accel, payload.data() + 28, 12);

    imu_msg.orientation.w = q[0];
    imu_msg.orientation.x = q[1];
    imu_msg.orientation.y = q[2];
    imu_msg.orientation.z = q[3];

    imu_msg.angular_velocity.x = gyro[0];
    imu_msg.angular_velocity.y = gyro[1];
    imu_msg.angular_velocity.z = gyro[2];

    imu_msg.linear_acceleration.x = accel[0];
    imu_msg.linear_acceleration.y = accel[1];
    imu_msg.linear_acceleration.z = accel[2];

    imu_pub_->publish(imu_msg);
  }

  bool is_6006(int idx) {
    return (idx == 0 || idx == 4 || idx == 5 || idx == 9);
  }

  SerialPort serial_;
  std::string port_;
  int baud_;
  uint8_t seq_;
  uint16_t rx_mask_; // Bitmap for received motors in current round

  MotorParams params_6006_, params_8006_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      imu_status_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState joint_msg_;

  std::vector<uint8_t> rx_buffer_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_zero_srv_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr set_zero_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LegMavlinkBridge>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
