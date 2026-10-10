// 影子模式的高频输入打包:/IMU_data(~100 Hz)和 /encoder(~50 Hz)逐条带接收时刻攒起来,每 50 ms 发一批到
// /shadow_feed(std_msgs/Float64MultiArray)。Python 逐条收一条消息就要 ~1 ms CPU,这样它每秒只收 20 条,
// 样本一条不丢,回放顺序和逐条收一样。
// data = [n_imu, n_enc, (t wx wy wz ax ay az) * n_imu, (t v) * n_enc],t = 节点时钟秒(和 rclpy 的 now() 同一个钟)。
#include <chrono>
#include <vector>
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class Feed : public rclcpp::Node {
 public:
  Feed() : Node("shadow_feed") {
    pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/shadow_feed", 5);
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>("/IMU_data", 50, [this](sensor_msgs::msg::Imu::ConstSharedPtr m) {
      const auto &w = m->angular_velocity, &a = m->linear_acceleration;
      imu_.insert(imu_.end(), {now_s(), w.x, w.y, w.z, a.x, a.y, a.z});
    });
    enc_sub_ = create_subscription<nav_msgs::msg::Odometry>("/encoder", 50, [this](nav_msgs::msg::Odometry::ConstSharedPtr m) {
      enc_.insert(enc_.end(), {now_s(), m->twist.twist.linear.x});
    });
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this] {
      if (imu_.empty() && enc_.empty()) return;
      std_msgs::msg::Float64MultiArray out;
      out.data.reserve(2 + imu_.size() + enc_.size());
      out.data.push_back(imu_.size() / 7);
      out.data.push_back(enc_.size() / 2);
      out.data.insert(out.data.end(), imu_.begin(), imu_.end());
      out.data.insert(out.data.end(), enc_.begin(), enc_.end());
      imu_.clear();
      enc_.clear();
      pub_->publish(out);
    });
  }

 private:
  double now_s() { return now().nanoseconds() * 1e-9; }
  std::vector<double> imu_, enc_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr enc_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Feed>());
  rclcpp::shutdown();
}
