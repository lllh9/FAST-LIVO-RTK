#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

namespace
{

using PointCloud2 = sensor_msgs::msg::PointCloud2;
using PointField = sensor_msgs::msg::PointField;

constexpr std::uint32_t kOutputPointStep = 24U;

const PointField * find_field(const PointCloud2 & cloud, const std::string & name)
{
  const auto it = std::find_if(
    cloud.fields.begin(), cloud.fields.end(),
    [&name](const PointField & field) {return field.name == name;});
  return it == cloud.fields.end() ? nullptr : &(*it);
}

template<typename T>
T read_value(const std::uint8_t * point, std::uint32_t offset)
{
  T value{};
  std::memcpy(&value, point + offset, sizeof(T));
  return value;
}

template<typename T>
void write_value(std::uint8_t * point, std::uint32_t offset, const T & value)
{
  std::memcpy(point + offset, &value, sizeof(T));
}

PointField make_field(
  const std::string & name, std::uint32_t offset, std::uint8_t datatype)
{
  PointField field;
  field.name = name;
  field.offset = offset;
  field.datatype = datatype;
  field.count = 1U;
  return field;
}

}  // namespace

class LivoxPointcloudConverter : public rclcpp::Node
{
public:
  LivoxPointcloudConverter()
  : Node("livox_pointcloud_converter")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/livox/points");

    publisher_ = create_publisher<PointCloud2>(output_topic_, rclcpp::QoS(10));
    subscription_ = create_subscription<PointCloud2>(
      input_topic_, rclcpp::QoS(10),
      std::bind(&LivoxPointcloudConverter::convert, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Converting Livox PointXYZRTL '%s' -> PointXYZIT '%s'; time is relative seconds",
      input_topic_.c_str(), output_topic_.c_str());
  }

private:
  bool validate_input(
    const PointCloud2 & cloud,
    const PointField *& x,
    const PointField *& y,
    const PointField *& z,
    const PointField *& intensity,
    const PointField *& timestamp)
  {
    x = find_field(cloud, "x");
    y = find_field(cloud, "y");
    z = find_field(cloud, "z");
    intensity = find_field(cloud, "intensity");
    timestamp = find_field(cloud, "timestamp");

    if (!x || !y || !z || !intensity || !timestamp) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input cloud must contain x, y, z, intensity and timestamp fields");
      return false;
    }

    const bool xyz_i_are_float32 =
      x->datatype == PointField::FLOAT32 &&
      y->datatype == PointField::FLOAT32 &&
      z->datatype == PointField::FLOAT32 &&
      intensity->datatype == PointField::FLOAT32;
    if (!xyz_i_are_float32 || timestamp->datatype != PointField::FLOAT64) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unexpected Livox field types: x/y/z/intensity must be FLOAT32 and timestamp FLOAT64");
      return false;
    }

    const std::array<const PointField *, 5> fields{x, y, z, intensity, timestamp};
    const std::array<std::uint32_t, 5> sizes{4U, 4U, 4U, 4U, 8U};
    for (std::size_t i = 0; i < fields.size(); ++i) {
      if (fields[i]->count != 1U || fields[i]->offset + sizes[i] > cloud.point_step) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Invalid input PointCloud2 field layout (point_step=%u)", cloud.point_step);
        return false;
      }
    }

    const std::uint64_t required_size = cloud.height == 0U ? 0U :
      static_cast<std::uint64_t>(cloud.height - 1U) * cloud.row_step +
      static_cast<std::uint64_t>(cloud.width) * cloud.point_step;
    if (required_size > cloud.data.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Input PointCloud2 data is truncated: need %llu bytes, got %zu",
        static_cast<unsigned long long>(required_size), cloud.data.size());
      return false;
    }

    if (cloud.is_bigendian) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Big-endian PointCloud2 input is not supported");
      return false;
    }
    return true;
  }

  void convert(const PointCloud2::ConstSharedPtr cloud)
  {
    const PointField * x = nullptr;
    const PointField * y = nullptr;
    const PointField * z = nullptr;
    const PointField * intensity = nullptr;
    const PointField * timestamp = nullptr;
    if (!validate_input(*cloud, x, y, z, intensity, timestamp)) {
      return;
    }

    PointCloud2 output;
    output.header = cloud->header;
    output.height = cloud->height;
    output.width = cloud->width;
    output.fields = {
      make_field("x", 0U, PointField::FLOAT32),
      make_field("y", 4U, PointField::FLOAT32),
      make_field("z", 8U, PointField::FLOAT32),
      make_field("intensity", 12U, PointField::FLOAT32),
      make_field("time", 16U, PointField::FLOAT64)};
    output.is_bigendian = false;
    output.point_step = kOutputPointStep;
    output.row_step = output.width * output.point_step;
    output.is_dense = cloud->is_dense;
    output.data.resize(static_cast<std::size_t>(output.row_step) * output.height);

    const auto frame_stamp_ns = static_cast<double>(rclcpp::Time(cloud->header.stamp).nanoseconds());
    bool invalid_time_found = false;

    for (std::uint32_t row = 0; row < cloud->height; ++row) {
      const std::uint8_t * input_row = cloud->data.data() +
        static_cast<std::size_t>(row) * cloud->row_step;
      std::uint8_t * output_row = output.data.data() +
        static_cast<std::size_t>(row) * output.row_step;

      for (std::uint32_t col = 0; col < cloud->width; ++col) {
        const std::uint8_t * input_point = input_row +
          static_cast<std::size_t>(col) * cloud->point_step;
        std::uint8_t * output_point = output_row +
          static_cast<std::size_t>(col) * output.point_step;

        write_value(output_point, 0U, read_value<float>(input_point, x->offset));
        write_value(output_point, 4U, read_value<float>(input_point, y->offset));
        write_value(output_point, 8U, read_value<float>(input_point, z->offset));
        write_value(output_point, 12U, read_value<float>(input_point, intensity->offset));

        const double timestamp_ns = read_value<double>(input_point, timestamp->offset);
        double relative_time_sec = (timestamp_ns - frame_stamp_ns) * 1.0e-9;
        if (!std::isfinite(relative_time_sec)) {
          relative_time_sec = 0.0;
          invalid_time_found = true;
        }
        write_value(output_point, 16U, relative_time_sec);
      }
    }

    if (invalid_time_found) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Non-finite Livox timestamps were replaced with zero");
    }
    publisher_->publish(std::move(output));
  }

  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Publisher<PointCloud2>::SharedPtr publisher_;
  rclcpp::Subscription<PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LivoxPointcloudConverter>());
  rclcpp::shutdown();
  return 0;
}
