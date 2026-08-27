#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>

namespace
{

using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;
using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

const PointField * find_field(const PointCloud2 & cloud, const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const PointField & field) {return field.name == name;});
  return it == cloud.fields.end() ? nullptr : &(*it);
}

template<typename T>
T read_value(const std::uint8_t * point, const std::uint32_t offset)
{
  T value{};
  std::memcpy(&value, point + offset, sizeof(T));
  return value;
}

std::uint8_t to_reflectivity(const float intensity)
{
  if (!std::isfinite(intensity)) {
    return 0U;
  }
  return static_cast<std::uint8_t>(std::clamp(std::lround(intensity), 0L, 255L));
}

std::uint32_t to_offset_time(const double timestamp_ns, const double timebase_ns)
{
  if (!std::isfinite(timestamp_ns) || !std::isfinite(timebase_ns)) {
    return 0U;
  }
  // PointCloud2中的Livox timestamp是每个点的绝对纳秒时间；CustomMsg要求
  // offset_time保存相对于timebase（首点/消息头时间）的纳秒偏移。
  const double relative_ns = timestamp_ns - timebase_ns;
  if (relative_ns <= 0.0) {
    return 0U;
  }
  constexpr double max_offset = static_cast<double>(std::numeric_limits<std::uint32_t>::max());
  return static_cast<std::uint32_t>(std::min(std::round(relative_ns), max_offset));
}

}  // namespace

class PointCloud2ToCustomNode : public rclcpp::Node
{
public:
  PointCloud2ToCustomNode()
  : Node("pointcloud2_to_custom")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/lidar_custom");
    const auto lidar_id = declare_parameter<int>("lidar_id", 0);
    lidar_id_ = static_cast<std::uint8_t>(std::clamp<long>(lidar_id, 0L, 255L));

    auto qos = rclcpp::SensorDataQoS();
    qos.keep_last(10);
    publisher_ = create_publisher<CustomMsg>(output_topic_, qos);
    subscription_ = create_subscription<PointCloud2>(
      input_topic_, qos,
      std::bind(&PointCloud2ToCustomNode::convert, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Converting Livox PointCloud2 '%s' -> CustomMsg '%s'",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  bool validate(
    const PointCloud2 & cloud, std::array<const PointField *, 7> & fields)
  {
    static const std::array<std::string, 7> names{
      "x", "y", "z", "intensity", "tag", "line", "timestamp"};
    static const std::array<std::uint8_t, 7> types{
      PointField::FLOAT32, PointField::FLOAT32, PointField::FLOAT32,
      PointField::FLOAT32, PointField::UINT8, PointField::UINT8, PointField::FLOAT64};
    static const std::array<std::uint32_t, 7> sizes{4U, 4U, 4U, 4U, 1U, 1U, 8U};

    for (std::size_t i = 0; i < names.size(); ++i) {
      fields[i] = find_field(cloud, names[i]);
      if (
        fields[i] == nullptr || fields[i]->datatype != types[i] ||
        fields[i]->count != 1U || fields[i]->offset + sizes[i] > cloud.point_step)
      {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Invalid Livox PointCloud2: field '%s' is missing or has an unexpected layout",
          names[i].c_str());
        return false;
      }
    }

    if (cloud.is_bigendian) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Big-endian PointCloud2 is not supported");
      return false;
    }

    const std::uint64_t required_size = cloud.height == 0U ? 0U :
      static_cast<std::uint64_t>(cloud.height - 1U) * cloud.row_step +
      static_cast<std::uint64_t>(cloud.width) * cloud.point_step;
    if (required_size > cloud.data.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "PointCloud2 data is truncated: need %llu bytes, got %zu",
        static_cast<unsigned long long>(required_size), cloud.data.size());
      return false;
    }
    return true;
  }

  void convert(const PointCloud2::ConstSharedPtr cloud)
  {
    std::array<const PointField *, 7> fields{};
    if (!validate(*cloud, fields)) {
      return;
    }

    CustomMsg output;
    output.header = cloud->header;
    output.timebase =
      static_cast<std::uint64_t>(rclcpp::Time(cloud->header.stamp).nanoseconds());
    const double timebase_ns = static_cast<double>(output.timebase);
    output.lidar_id = lidar_id_;
    output.rsvd = {0U, 0U, 0U};
    output.points.reserve(static_cast<std::size_t>(cloud->width) * cloud->height);

    for (std::uint32_t row = 0; row < cloud->height; ++row) {
      const auto * input_row =
        cloud->data.data() + static_cast<std::size_t>(row) * cloud->row_step;
      for (std::uint32_t col = 0; col < cloud->width; ++col) {
        const auto * point = input_row + static_cast<std::size_t>(col) * cloud->point_step;
        CustomPoint custom_point;
        custom_point.x = read_value<float>(point, fields[0]->offset);
        custom_point.y = read_value<float>(point, fields[1]->offset);
        custom_point.z = read_value<float>(point, fields[2]->offset);
        custom_point.reflectivity =
          to_reflectivity(read_value<float>(point, fields[3]->offset));
        custom_point.tag = read_value<std::uint8_t>(point, fields[4]->offset);
        custom_point.line = read_value<std::uint8_t>(point, fields[5]->offset);
        custom_point.offset_time =
          to_offset_time(read_value<double>(point, fields[6]->offset), timebase_ns);
        output.points.push_back(custom_point);
      }
    }

    output.point_num = static_cast<std::uint32_t>(output.points.size());
    publisher_->publish(std::move(output));
  }

  std::string input_topic_;
  std::string output_topic_;
  std::uint8_t lidar_id_{};
  rclcpp::Publisher<CustomMsg>::SharedPtr publisher_;
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloud2ToCustomNode>());
  rclcpp::shutdown();
  return 0;
}
