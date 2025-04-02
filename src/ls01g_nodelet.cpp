#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "ls01g_driver.hpp"
#include <iostream>
#include <std_msgs/msg/int32.hpp>
using namespace std;

bool is_scan_stop = false;
bool is_motor_stop = false;
bool zero_as_max = true;
bool min_as_zero = true;
bool inverted = true;
string laser_link = "laser_link";
double angle_disable_min = -1;
double angle_disable_max = -1;
io_driver driver;

class LS01GNode : public rclcpp::Node {
public:
  // 公共访问方法
  bool is_initialized() const {
    return initialization_successful;
  }
  
  LS01GNode() : Node("ls01g"), initialization_successful(false), port_opened(false) {
    // 声明和获取参数
    this->declare_parameter("scan_topic", "scan");
    this->declare_parameter("laser_link", "laser_link");
    this->declare_parameter("serial_port", "/dev/ttyUSB0");
    this->declare_parameter("angle_disable_min", -1.0);
    this->declare_parameter("angle_disable_max", -1.0);
    this->declare_parameter("zero_as_max", true);
    this->declare_parameter("min_as_zero", true);
    this->declare_parameter("inverted", true);

    // 获取参数值
    scan_topic = this->get_parameter("scan_topic").as_string();
    laser_link = this->get_parameter("laser_link").as_string();
    port = this->get_parameter("serial_port").as_string();
    angle_disable_min = this->get_parameter("angle_disable_min").as_double();
    angle_disable_max = this->get_parameter("angle_disable_max").as_double();
    zero_as_max = this->get_parameter("zero_as_max").as_bool();
    min_as_zero = this->get_parameter("min_as_zero").as_bool();
    inverted = this->get_parameter("inverted").as_bool();

    // 创建发布者和订阅者
    scan_pub =
        this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);
    stop_sub = this->create_subscription<std_msgs::msg::Int32>(
        "startOrStop", 10,
        std::bind(&LS01GNode::startStopCB, this, std::placeholders::_1));

    // 初始化雷达
    if (!init_lidar()) {
      RCLCPP_ERROR(this->get_logger(), "Lidar initialization failed");
      return; // 不要创建定时器
    }

    initialization_successful = true;

    // 创建定时器用于发布激光数据
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     std::bind(&LS01GNode::publish_data, this));
  }

  ~LS01GNode() {
    // 只在端口成功打开时尝试关闭
    if (port_opened) {
      driver.StopScan(STOP_DATA);
      driver.StopScan(STOP_MOTOR);
      driver.CloseSerial();
      RCLCPP_INFO(this->get_logger(), "Shutting down, ls01g stop!");
    }
  }

private:
  bool init_lidar() {
    int ret = driver.OpenSerial(port.c_str(), B230400);
    if (ret < 0) {
      RCLCPP_ERROR(this->get_logger(), "Could not open port: %s", port.c_str());
      return false; // 返回失败而不是调用 rclcpp::shutdown()
    }
    port_opened = true; // 标记端口已打开
    RCLCPP_INFO(this->get_logger(), "Open port: %s", port.c_str());

    if (inverted) {
      RCLCPP_INFO(
          this->get_logger(),
          "This laser is inverted, zero degree direction is align with line");
    }

    driver.StartScan();
    RCLCPP_INFO(this->get_logger(), "Send start command successfully");
    return true; // 返回成功
  }

  void publish_data() {
    if (is_scan_stop)
      return;

    memset(data, 0, sizeof(data));
    int ret = driver.GetScanData(angle, distance, PACKLEN, &speed);
    for (int i = 0; i < ret; i++) {
      data[i] = distance[i];
      data_intensity[i] = angle[i];
    }

    static int log_count = 0;
    if (++log_count >= 300) { // 大约相当于ROS_INFO_THROTTLE(30, ...)
      RCLCPP_INFO(this->get_logger(), "ls01g works fine!");
      log_count = 0;
    }

    auto ends = this->now();
    double scan_duration = (ends - starts).seconds() * 1e-3;
    publish_scan(data, data_intensity, ret, starts, scan_duration);
    starts = ends;
  }

  void startStopCB(const std_msgs::msg::Int32::SharedPtr msg) {
    Command cmd = (Command)msg->data;
    switch (cmd) {
    case STOP_DATA:
      if (!is_scan_stop) {
        driver.StopScan(STOP_DATA);
        is_scan_stop = true;
        RCLCPP_INFO(this->get_logger(), "stop scan");
      }
      break;
    case STOP_MOTOR:
      if (!is_scan_stop) {
        driver.StopScan(STOP_DATA);
        is_scan_stop = true;
        RCLCPP_INFO(this->get_logger(), "stop scan");
      }
      if (!is_motor_stop) {
        driver.StopScan(STOP_MOTOR);
        is_motor_stop = true;
        RCLCPP_INFO(this->get_logger(), "stop motor");
      }
      break;
    case START_MOTOR_AND_SCAN:
      if (is_scan_stop) {
        RCLCPP_INFO(this->get_logger(), "start scan");
        int res = driver.StartScan();
        RCLCPP_INFO(this->get_logger(), "start: %d", res);
        is_scan_stop = false;
        is_motor_stop = false;
      }
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown command: %d ", cmd);
      break;
    }
  }

  void publish_scan(double *dist, double *intensities, int count,
                    rclcpp::Time start, double scan_time) {
    static int scan_count = 0;
    auto scan_msg = sensor_msgs::msg::LaserScan();
    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = laser_link;
    scan_count++;
    scan_msg.angle_min = 0.0;
    scan_msg.angle_max = 2 * M_PI;
    scan_msg.angle_increment =
        (scan_msg.angle_max - scan_msg.angle_min) / (double)(count - 1);
    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(count - 1);

    scan_msg.range_min = 0.1;
    scan_msg.range_max = 10.0;

    scan_msg.intensities.resize(count);
    scan_msg.ranges.resize(count);

    if (!inverted) {
      for (int i = count - 1; i >= 0; i--) {
        if (dist[count - i - 1] == 0.0 && zero_as_max)
          scan_msg.ranges[i] = scan_msg.range_max - 0.2;
        else if (dist[count - i - 1] == 0.0)
          if (min_as_zero)
            scan_msg.ranges[i] = 0.0;
          else
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        else
          scan_msg.ranges[i] = dist[count - i - 1] / 1000.0;
        scan_msg.intensities[i] = floor(intensities[count - i - 1]);
      }
    } else {
      for (int i = 0; i <= 179; i++) {
        if (dist[179 - i] == 0.0 && zero_as_max)
          scan_msg.ranges[i] = scan_msg.range_max - 0.2;
        else if (dist[179 - i] == 0.0)
          if (min_as_zero)
            scan_msg.ranges[i] = 0.0;
          else
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        else
          scan_msg.ranges[i] = dist[179 - i] / 1000.0;
        scan_msg.intensities[i] = floor(intensities[179 - i]);
      }
      for (int i = 180; i < 360; i++) {
        if (dist[540 - i] == 0.0 && zero_as_max)
          scan_msg.ranges[i] = scan_msg.range_max - 0.2;
        else if (dist[540 - i] == 0.0)
          if (min_as_zero)
            scan_msg.ranges[i] = 0.0;
          else
            scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
        else
          scan_msg.ranges[i] = dist[540 - i] / 1000.0;
        scan_msg.intensities[i] = floor(intensities[540 - i]);
      }
    }
    for (int i = 0; i < 360; i++) {
      if ((i >= angle_disable_min) &&
          (i < angle_disable_max)) // disable angle part
      {
        if (min_as_zero)
          scan_msg.ranges[i] = 0.0;
        else
          scan_msg.ranges[i] = std::numeric_limits<float>::infinity();
      }
    }

    scan_pub->publish(scan_msg);
  }

  // 成员变量
  bool initialization_successful;
  bool port_opened;
  std::string scan_topic;
  std::string port;
  double angle[PACKLEN + 10];
  double distance[PACKLEN + 10];
  double data[PACKLEN + 10];
  double data_intensity[PACKLEN + 10];
  double speed;
  rclcpp::Time starts = this->now();
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr stop_sub;
  rclcpp::TimerBase::SharedPtr timer_;
};

// 主函数，使用公共方法访问初始化状态
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LS01GNode>();

  // 检查初始化是否成功
  if (node->is_initialized()) {
    rclcpp::spin(node);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ls01g"),
                 "Node initialization failed, not spinning");
  }

  rclcpp::shutdown();
  return 0;
}