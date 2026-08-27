#include "MvCameraControl.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <algorithm>
#include <cerrno>
#include <cinttypes>
#include <cstdlib>
#include <cstring>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <pwd.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>
#include <thread>

using namespace std;

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt = nullptr;
int64_t last_shared_sequence = 0;
bool has_matched_shared_timestamp = false; // 首帧用于建立同步基准，避免把相机配置期间的雷达更新误报为丢帧。

enum PixelFormat : unsigned int {
  RGB8 = 0x02180014,
  BayerRG8 = 0x01080009,
  BayerRG12Packed = 0x010C002B,
  BayerGB12Packed = 0x010C002C,
  BayerGB8 = 0x0108000A
};
// unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
volatile sig_atomic_t exit_flag = 0;
int width, height;
image_transport::Publisher pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
sensor_msgs::msg::CameraInfo camera_info_template;
std::string camera_frame_id;
std::vector<PixelFormat> PIXEL_FORMAT = { RGB8, BayerRG8, BayerRG12Packed, BayerGB12Packed, BayerGB8 };
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
float image_scale = 0.0;
int trigger_enable = 1;
int timestamp_wait_timeout_ms = 300; // 10Hz雷达周期约100ms，默认等待300ms以覆盖调度、网络和写盘抖动。
int timestamp_remap_after_timeouts = 3; // 连续超时3次后重建共享文件映射，自动恢复雷达驱动重启或文件被替换的情况。

rclcpp::Node::SharedPtr g_node;
rclcpp::Logger g_logger = rclcpp::get_logger("mvs_trigger");
std::chrono::steady_clock::time_point next_timestamp_share_retry;

bool InitializeTimestampShare()
{
  std::string shared_file_name;
  const char *configured_path = std::getenv("LIVOX_TIMESHARE_PATH");
  if (configured_path != nullptr && configured_path[0] != '\0')
  {
    shared_file_name = configured_path;
  }
  else
  {
    const char *home_dir = std::getenv("HOME");
    if (home_dir == nullptr || home_dir[0] == '\0')
    {
      const struct passwd *user_info = getpwuid(geteuid());
      if (user_info != nullptr)
      {
        home_dir = user_info->pw_dir;
      }
    }

    if (home_dir == nullptr || home_dir[0] == '\0')
    {
      RCLCPP_WARN(g_logger,
                  "Timestamp sharing disabled: unable to determine the user home directory. "
                  "Camera frames will use the ROS clock.");
      return false;
    }
    shared_file_name = std::string(home_dir) + "/timeshare";
  }

  const int fd = open(shared_file_name.c_str(), O_RDONLY);
  if (fd < 0)
  {
    RCLCPP_WARN(g_logger,
                "Timestamp sharing disabled: failed to open '%s': %s. "
                "Camera frames will use the ROS clock.",
                shared_file_name.c_str(), std::strerror(errno));
    return false;
  }

  struct stat file_status;
  if (fstat(fd, &file_status) != 0)
  {
    const int error_number = errno;
    close(fd);
    RCLCPP_WARN(g_logger,
                "Timestamp sharing disabled: failed to inspect '%s': %s. "
                "Camera frames will use the ROS clock.",
                shared_file_name.c_str(), std::strerror(error_number));
    return false;
  }

  if (file_status.st_size < static_cast<off_t>(sizeof(time_stamp)))
  {
    close(fd);
    RCLCPP_WARN(g_logger,
                "Timestamp sharing disabled: '%s' is too small (%jd bytes, need at least %zu). "
                "Camera frames will use the ROS clock.",
                shared_file_name.c_str(), static_cast<intmax_t>(file_status.st_size),
                sizeof(time_stamp));
    return false;
  }

  void *mapped_address = mmap(nullptr, sizeof(time_stamp), PROT_READ, MAP_SHARED, fd, 0);
  const int error_number = errno;
  close(fd);
  if (mapped_address == MAP_FAILED)
  {
    RCLCPP_WARN(g_logger,
                "Timestamp sharing disabled: failed to map '%s': %s. "
                "Camera frames will use the ROS clock.",
                shared_file_name.c_str(), std::strerror(error_number));
    return false;
  }

  pointt = static_cast<time_stamp *>(mapped_address);
  last_shared_sequence = __atomic_load_n(&pointt->high, __ATOMIC_ACQUIRE);
  if (last_shared_sequence & 1) --last_shared_sequence;
  RCLCPP_INFO(g_logger, "Using shared timestamps from '%s'.", shared_file_name.c_str());
  return true;
}

void TryInitializeTimestampShare()
{
  if (pointt != nullptr)
  {
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if (now < next_timestamp_share_retry)
  {
    return;
  }

  next_timestamp_share_retry = now + std::chrono::seconds(5);
  InitializeTimestampShare();
}

void CleanupTimestampShare()
{
  if (pointt != nullptr)
  {
    munmap(pointt, sizeof(time_stamp));
    pointt = nullptr;
  }
  last_shared_sequence = 0; // 清除旧文件的序列号，防止重新映射后把新驱动的小序列号与旧状态混用。
  has_matched_shared_timestamp = false; // 重映射后重新建立同步基准，不统计断开期间的历史更新。
}

bool ReadSharedTimestamp(int64_t &sequence, int64_t &timestamp)
{
  if (pointt == nullptr) return false;

  for (int attempt = 0; attempt < 8; ++attempt)
  {
    const int64_t sequence_before = __atomic_load_n(&pointt->high, __ATOMIC_ACQUIRE);
    if (sequence_before == 0 || (sequence_before & 1)) continue;
    const int64_t value = __atomic_load_n(&pointt->low, __ATOMIC_ACQUIRE);
    const int64_t sequence_after = __atomic_load_n(&pointt->high, __ATOMIC_ACQUIRE);
    if (sequence_before == sequence_after && !(sequence_after & 1) && value > 0)
    {
      sequence = sequence_after;
      timestamp = value;
      return true;
    }
  }
  return false;
}

bool WaitForNextSharedTimestamp(int64_t &timestamp)
{
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(timestamp_wait_timeout_ms); // 使用可配置超时，避免10Hz雷达轻微抖动就误丢图像。
  while (std::chrono::steady_clock::now() < deadline && rclcpp::ok() && !exit_flag)
  {
    int64_t sequence = 0;
    int64_t value = 0;
    if (ReadSharedTimestamp(sequence, value) && sequence != last_shared_sequence)
    {
      if (has_matched_shared_timestamp &&
          last_shared_sequence != 0 &&
          sequence > last_shared_sequence + 2)
      {
        RCLCPP_WARN(g_logger,
                    "Camera missed %lld lidar timestamp update(s).",
                    static_cast<long long>((sequence - last_shared_sequence) / 2 - 1));
      }
      last_shared_sequence = sequence;
      has_matched_shared_timestamp = true; // 从首次成功匹配后再统计运行期间真正缺失的更新。
      timestamp = value;
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

bool ReadDoubleSequence(const cv::FileNode &node, size_t expected_size,
                        std::vector<double> &values)
{
  values.clear();
  if (node.empty() || !node.isSeq())
  {
    return false;
  }

  for (auto it = node.begin(); it != node.end(); ++it)
  {
    values.push_back(static_cast<double>(*it));
  }
  return expected_size == 0 ? !values.empty() : values.size() == expected_size;
}

std::string DefaultCameraInfoTopic(const std::string &image_topic)
{
  std::string topic = image_topic;
  while (topic.size() > 1 && topic.back() == '/')
  {
    topic.pop_back();
  }

  const size_t separator = topic.find_last_of('/');
  const std::string leaf = separator == std::string::npos
                               ? topic
                               : topic.substr(separator + 1);
  if (leaf == "image" || leaf == "image_raw")
  {
    return separator == std::string::npos
               ? "camera_info"
               : topic.substr(0, separator + 1) + "camera_info";
  }
  return topic + "/camera_info";
}

bool LoadCameraInfo(const cv::FileStorage &params)
{
  const int calibration_width = static_cast<int>(params["Width"]);
  const int calibration_height = static_cast<int>(params["Height"]);
  if (calibration_width <= 0 || calibration_height <= 0)
  {
    RCLCPP_ERROR(g_logger, "Camera calibration Width and Height must be positive.");
    return false;
  }

  std::vector<double> values;
  if (!ReadDoubleSequence(params["D"], 0, values))
  {
    RCLCPP_ERROR(g_logger, "Camera calibration D must be a non-empty sequence.");
    return false;
  }

  sensor_msgs::msg::CameraInfo info;
  info.width = static_cast<uint32_t>(calibration_width);
  info.height = static_cast<uint32_t>(calibration_height);
  info.d = values;

  if (!ReadDoubleSequence(params["K"], 9, values))
  {
    RCLCPP_ERROR(g_logger, "Camera calibration K must contain exactly 9 values.");
    return false;
  }
  std::copy(values.begin(), values.end(), info.k.begin());

  const cv::FileNode r_node = params["R"];
  if (r_node.empty())
  {
    info.r.fill(0.0);
    info.r[0] = 1.0;
    info.r[4] = 1.0;
    info.r[8] = 1.0;
  }
  else
  {
    if (!ReadDoubleSequence(r_node, 9, values))
    {
      RCLCPP_ERROR(g_logger, "Camera calibration R must contain exactly 9 values.");
      return false;
    }
    std::copy(values.begin(), values.end(), info.r.begin());
  }

  const cv::FileNode p_node = params["P"];
  if (p_node.empty())
  {
    info.p.fill(0.0);
    info.p[0] = info.k[0];
    info.p[1] = info.k[1];
    info.p[2] = info.k[2];
    info.p[4] = info.k[3];
    info.p[5] = info.k[4];
    info.p[6] = info.k[5];
    info.p[8] = info.k[6];
    info.p[9] = info.k[7];
    info.p[10] = info.k[8];
  }
  else
  {
    if (!ReadDoubleSequence(p_node, 12, values))
    {
      RCLCPP_ERROR(g_logger, "Camera calibration P must contain exactly 12 values.");
      return false;
    }
    std::copy(values.begin(), values.end(), info.p.begin());
  }

  info.distortion_model = "plumb_bob";
  const cv::FileNode distortion_model_node = params["DistortionModel"];
  if (!distortion_model_node.empty())
  {
    info.distortion_model = static_cast<std::string>(distortion_model_node);
  }

  const cv::FileNode frame_id_node = params["FrameId"];
  if (!frame_id_node.empty())
  {
    camera_frame_id = static_cast<std::string>(frame_id_node);
  }
  else
  {
    camera_frame_id = std::string(g_node->get_name()) + "_optical_frame";
  }

  camera_info_template = info;
  RCLCPP_INFO(g_logger,
              "Loaded camera calibration: %ux%u, D=%zu values, frame_id='%s'.",
              info.width, info.height, info.d.size(), camera_frame_id.c_str());
  return true;
}

sensor_msgs::msg::CameraInfo BuildCameraInfo(uint32_t image_width,
                                             uint32_t image_height)
{
  sensor_msgs::msg::CameraInfo info = camera_info_template;
  if (info.width == 0 || info.height == 0)
  {
    return info;
  }

  const double scale_x = static_cast<double>(image_width) / info.width;
  const double scale_y = static_cast<double>(image_height) / info.height;
  info.width = image_width;
  info.height = image_height;

  info.k[0] *= scale_x;
  info.k[1] *= scale_x;
  info.k[2] *= scale_x;
  info.k[3] *= scale_y;
  info.k[4] *= scale_y;
  info.k[5] *= scale_y;

  for (size_t i = 0; i < 4; ++i)
  {
    info.p[i] *= scale_x;
    info.p[4 + i] *= scale_y;
  }
  return info;
}


bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
  if (NULL == pstMVDevInfo)
  {
    printf("The Pointer of pstMVDevInfo is NULL!\n");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
  {
    int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    printf("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
    printf("SerialNumber: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
  }
  else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
  {
    printf("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    printf("SerialNumber: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
  }
  else
  {
    printf("Not support.\n");
  }
  return true;
}

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    RCLCPP_ERROR_STREAM(g_logger, msg);
    exit(-1);
  }
  image_scale = Params["image_scale"];   
  if (image_scale < 0.1) image_scale = 1;
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int ExposureTime = Params["ExposureTime"];
  int ExposureAutoMode = Params["ExposureAutoMode"];
  int GainAuto = Params["GainAuto"];
  float Gain = Params["Gain"];
  float Gamma = Params["Gamma"];
  int GammaSlector = Params["GammaSelector"];
  int nRet;

  // 设置曝光模式
  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);
      std::string msg = "Set ExposureAutoMode: " + ExposureAutoStr[ExposureAutoMode];

  if (MV_OK == nRet) {
    RCLCPP_INFO_STREAM(g_logger, msg);
  } else {
    if(ExposureAutoMode == 2) {
      RCLCPP_WARN_STREAM(g_logger, "Fail to set Exposure Auto Mode to Continues");
    }
    else {
      RCLCPP_INFO_STREAM(g_logger, msg);
    }
  }

  // 如果是自动曝光
  if (ExposureAutoMode == 2) {
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) +
          "us";
      RCLCPP_INFO_STREAM(g_logger, msg);
    } else {
      RCLCPP_ERROR_STREAM(g_logger, "Fail to set Exposure Time Lower");
    }
    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) +
          "us";
      RCLCPP_INFO_STREAM(g_logger, msg);
    } else {
      RCLCPP_ERROR_STREAM(g_logger, "Fail to set Exposure Time Upper");
    }
  }

  // 如果是固定曝光
  if (ExposureAutoMode == 0) {
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time: " + std::to_string(ExposureTime) + "us";
      RCLCPP_INFO_STREAM(g_logger, msg);
    } else {
      RCLCPP_ERROR_STREAM(g_logger, "Fail to set Exposure Time");
    }
  }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);

  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    RCLCPP_INFO_STREAM(g_logger, msg);
  } else {
    RCLCPP_ERROR_STREAM(g_logger, "Fail to set Gain auto mode");
  }

  if (GainAuto == 0) {
    nRet = MV_CC_SetGain(handle, Gain);
    if (MV_OK == nRet) {
      std::string msg = "Set Gain: " + std::to_string(Gain);
      RCLCPP_INFO_STREAM(g_logger, msg);
    } else {
      RCLCPP_ERROR_STREAM(g_logger, "Fail to set Gain");
    }
  }

  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
    RCLCPP_INFO_STREAM(g_logger, msg);
  } else {
    RCLCPP_ERROR_STREAM(g_logger, "Fail to set GammaSlector");
  }

  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    std::string msg = "Set Gamma: " + std::to_string(Gamma);
    RCLCPP_INFO_STREAM(g_logger, msg);
  } else {
    RCLCPP_ERROR_STREAM(g_logger, "Fail to set Gamma");
  }
}

void SignalHandler(int signal) {
  if (signal == SIGINT) {  // 捕捉 Ctrl + C 触发的 SIGINT 信号
    exit_flag = 1;    // 设置退出标志
  }
}

void SetupSignalHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler; // 设置处理函数
  sigemptyset(&sigIntHandler.sa_mask);      // 清空信号屏蔽集
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

static void *WorkThread(void *pUser) {
  int nRet = MV_OK;
  int consecutive_timestamp_timeouts = 0; // 统计连续失败次数，仅在共享通道持续异常时触发重映射。
  uint32_t last_camera_frame_number = 0; // 记录海康相机SDK上一帧编号，用于判断相机或传输链路是否真实丢帧。
  uint64_t received_camera_frame_count = 0; // 统计SDK成功取到的图像总数，不受后续雷达时间戳匹配结果影响。
  constexpr uint32_t kCameraFrameNumberMax = 65535; // 当前相机帧号按1~65535循环，65535之后回到1。

  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
    return NULL;
  }

  MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
  
  unsigned char* pData = (unsigned char *)malloc(sizeof(unsigned char) * stParam.nCurValue * 3);
  unsigned char* pDataForBGR = (unsigned char*)malloc(sizeof(unsigned char) * stParam.nCurValue * 3);

  if (pData == nullptr || pDataForBGR == nullptr) {
    printf("Memory allocation failed!\n");
    if (pData) free(pData);
    if (pDataForBGR) free(pDataForBGR);
    return nullptr;
  }

  while (!exit_flag && rclcpp::ok()) {

    nRet = MV_CC_GetOneFrameTimeout(pUser, pData, stParam.nCurValue * 3, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      // 必须在等待雷达共享时间戳之前记录帧号。这样可以区分：
      // 1. nFrameNum不连续：相机未响应触发或相机到主机的传输链路丢帧；
      // 2. nFrameNum连续但图像随后被丢弃：问题发生在雷达时间戳匹配环节。
      ++received_camera_frame_count;
      const uint32_t current_camera_frame_number = stImageInfo.nFrameNum;
      if (last_camera_frame_number != 0)
      {
        // 海康SDK字段类型虽然是unsigned int，但当前相机实际使用16位非零帧号：
        // 1, 2, ..., 65535, 1。必须显式处理65535到1的正常回绕，
        // 否则会误报丢失4294901761帧。
        const uint32_t expected_camera_frame_number =
            last_camera_frame_number == kCameraFrameNumberMax
                ? 1
                : last_camera_frame_number + 1;
        if (current_camera_frame_number != expected_camera_frame_number)
        {
          const uint32_t missed_camera_frames =
              current_camera_frame_number > last_camera_frame_number
                  ? current_camera_frame_number - last_camera_frame_number - 1
                  : (kCameraFrameNumberMax - last_camera_frame_number) +
                        (current_camera_frame_number - 1);
          RCLCPP_WARN(
              g_logger,
              "Camera SDK frame discontinuity: previous=%u, current=%u, "
              "missed=%u, received_total=%llu.",
              last_camera_frame_number, current_camera_frame_number,
              missed_camera_frames,
              static_cast<unsigned long long>(received_camera_frame_count));
        }
      }
      last_camera_frame_number = current_camera_frame_number;

      // 调试时可取消下面的注释，每5秒输出一次当前SDK帧号和累计接收帧数。
      // 正常运行保持关闭以避免无异常时持续输出日志；帧号不连续的WARN仍然保留。
      // RCLCPP_INFO_THROTTLE(
      //     g_logger, *g_node->get_clock(), 5000,
      //     "Camera SDK status: nFrameNum=%u, received_total=%llu.",
      //     current_camera_frame_number,
      //     static_cast<unsigned long long>(received_camera_frame_count));
      
      rclcpp::Time rcv_time;
      int64_t shared_timestamp = 0;
      if (trigger_enable)
      {
        TryInitializeTimestampShare();
        if (!WaitForNextSharedTimestamp(shared_timestamp))
        {
          ++consecutive_timestamp_timeouts;
          RCLCPP_WARN_THROTTLE(g_logger, *g_node->get_clock(), 2000,
                               "Dropping triggered image: no new matching lidar timestamp within %d ms "
                               "(consecutive timeouts: %d)",
                               timestamp_wait_timeout_ms, consecutive_timestamp_timeouts);
          if (consecutive_timestamp_timeouts >= timestamp_remap_after_timeouts)
          {
            RCLCPP_WARN(g_logger,
                        "Reopening LiDAR timestamp sharing after %d consecutive timeouts.",
                        consecutive_timestamp_timeouts);
            CleanupTimestampShare(); // 解除可能指向旧inode的mmap，下一帧会按相同路径重新打开当前共享文件。
            consecutive_timestamp_timeouts = 0;
          }
          continue;
        }
        consecutive_timestamp_timeouts = 0; // 成功取得新雷达时间戳后清零，偶发抖动不会触发不必要的重映射。
      }

      if (shared_timestamp != 0)
      {
        // Keep the integer nanoseconds intact. Converting through double loses
        // precision at Unix-epoch-sized timestamps.
        rcv_time = rclcpp::Time(shared_timestamp);
      }
      else
      {
        if (g_node) {
          rcv_time = g_node->get_clock()->now();
        } else {
          rcv_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
        }
      }
     
      // std::string debug_msg;
      // debug_msg = "GetOneFrame,nFrameNum[" +
      //             std::to_string(stImageInfo.nFrameNum) + "], FrameTime:" +
      //             std::to_string(rcv_time.toSec());
      // ROS_INFO_STREAM(debug_msg.c_str());

      stConvertParam.nWidth = stImageInfo.nWidth;
      stConvertParam.nHeight = stImageInfo.nHeight;
      stConvertParam.pSrcData = pData;
      stConvertParam.nSrcDataLen = stParam.nCurValue * 3; 
      stConvertParam.enSrcPixelType = stImageInfo.enPixelType;
      stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
      stConvertParam.pDstBuffer = pDataForBGR;
      stConvertParam.nDstBufferSize = stParam.nCurValue * 3;
      nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
      if (MV_OK != nRet)
      {
        printf("MV_CC_ConvertPixelType failed! nRet [%x], skipping this frame\n", nRet);
        continue;
      }
      cv::Mat srcImage;
      srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pDataForBGR);

      // cv::Mat srcImage;
      // srcImage = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      if (image_scale > 0.0) {
        cv::resize(srcImage, srcImage, cv::Size(srcImage.cols * image_scale, srcImage.rows * image_scale), cv::INTER_LINEAR);
      } else {
        printf("Invalid image_scale: %f. Skipping resize.\n", image_scale);
      }
      sensor_msgs::msg::Image::SharedPtr msg =
          cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", srcImage).toImageMsg();
      builtin_interfaces::msg::Time stamp;
      int64_t ns = rcv_time.nanoseconds();
      if (ns < 0) {
        ns = 0;
      }
      stamp.sec = static_cast<int32_t>(ns / 1000000000LL);
      stamp.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
      msg->header.stamp = stamp;
      msg->header.frame_id = camera_frame_id;

      sensor_msgs::msg::CameraInfo camera_info =
          BuildCameraInfo(msg->width, msg->height);
      camera_info.header = msg->header;
      pub.publish(msg);
      camera_info_pub->publish(camera_info);
    }
  }

  if (pData) {
    free(pData);
    pData = nullptr;
  }

  if (pDataForBGR)
  {
    free(pDataForBGR);
    pDataForBGR = nullptr;
  }

  return 0;
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  g_node = rclcpp::Node::make_shared("mvs_trigger");
  g_logger = g_node->get_logger();
  if (argc < 2) {
    RCLCPP_ERROR(g_logger, "Usage: mvs_camera_node <params_file>");
    rclcpp::shutdown();
    return -1;
  }
  std::string params_file = std::string(argv[1]);
  image_transport::ImageTransport it(g_node);
  int nRet = MV_OK;
  void *handle = NULL;
  rclcpp::Rate loop_rate(10.0);
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    RCLCPP_ERROR_STREAM(g_logger, msg);
    exit(-1);
  }
  trigger_enable = Params["TriggerEnable"];
  const cv::FileNode timestamp_wait_node = Params["TimestampWaitTimeoutMs"];
  if (!timestamp_wait_node.empty())
  {
    timestamp_wait_timeout_ms =
        std::clamp(static_cast<int>(timestamp_wait_node), 100, 2000); // 限制配置范围，防止过小误丢帧或过大阻塞相机取流线程。
  }
  const cv::FileNode timestamp_remap_node = Params["TimestampRemapAfterTimeouts"];
  if (!timestamp_remap_node.empty())
  {
    timestamp_remap_after_timeouts =
        std::clamp(static_cast<int>(timestamp_remap_node), 1, 100); // 确保重映射阈值有效且不会因错误配置永久不恢复。
  }
  RCLCPP_INFO(g_logger,
              "Timestamp synchronization: wait timeout=%d ms, remap after=%d consecutive timeouts.",
              timestamp_wait_timeout_ms, timestamp_remap_after_timeouts); // 启动时打印最终参数，便于现场日志确认配置是否生效。
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];
  int PixelFormat = Params["PixelFormat"];

  if (!LoadCameraInfo(Params)) {
    RCLCPP_ERROR(g_logger, "Failed to load camera calibration from '%s'.",
                 params_file.c_str());
    rclcpp::shutdown();
    return -1;
  }

  pub = it.advertise(pub_topic, 1);
  std::string camera_info_topic;
  const cv::FileNode camera_info_topic_node = Params["CameraInfoTopic"];
  if (!camera_info_topic_node.empty()) {
    camera_info_topic = static_cast<std::string>(camera_info_topic_node);
  }
  if (camera_info_topic.empty()) {
    camera_info_topic = DefaultCameraInfoTopic(pub_topic);
  }
  camera_info_pub =
      g_node->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10);
  RCLCPP_INFO(g_logger, "Publishing image on '%s' and CameraInfo on '%s'.",
              pub_topic.c_str(), camera_info_topic.c_str());
  
  if (trigger_enable)
  {
    TryInitializeTimestampShare();
  }

  SetupSignalHandler();
 
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    return -1;
  }

  if (stDeviceList.nDeviceNum > 0)
  {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++)
    {
      printf("[device %d]:\n", i);
      MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (pDeviceInfo == NULL)
      {
        printf("Device info is NULL for device %d\n", i);
        return -1;
      } 
      PrintDeviceInfo(pDeviceInfo);            
    }  
  } 
  else
  {
    printf("Find No Devices!\n");
    return -1;
  }

  bool find_expect_camera = false;
  unsigned int nIndex = 0;

  if (stDeviceList.nDeviceNum > 1) 
  {
    if (expect_serial_number.empty()) 
    {
      RCLCPP_ERROR(g_logger, "Expected serial number is empty!");
      return -1;
    }
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) 
    {
      if (stDeviceList.pDeviceInfo[i] == NULL) 
      {
        printf("Device info is NULL for device %d\n", i);
        continue;
      }
      
      std::string serial_number;
      if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE)
      {
        serial_number = 
            std::string((char *)stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber);
      }
      else if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE)
      {
        serial_number = 
            std::string((char *)stDeviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber);
      }
      else
      {
        printf("Unknown device type!\n");
        continue;
      }
      if (serial_number.empty()) 
      {
        printf("Serial number is empty for device %d\n", i);
        continue;
      }
      if (expect_serial_number == serial_number) 
      {
        find_expect_camera = true;
        nIndex = i;
        break;
      }
    }
    if (!find_expect_camera) 
    {
      std::string msg =
          "Can not find the camera with serial number " + expect_serial_number;
      RCLCPP_ERROR_STREAM(g_logger, msg);
      return -1;
    }
  }
  else
  {
    nIndex = 0;
  }
  
  // select device and create handle
  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
  if (MV_OK != nRet)
  {
    printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
    return -1;
  }

  // open device
  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet)
  {
    printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
  if (MV_OK != nRet) {
    printf("set AcquisitionFrameRateEnable fail! nRet [%x]\n", nRet);
    return -1;
  }

  // MVCC_INTVALUE stParam;
  // memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  // nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
  // if (MV_OK != nRet) {
  //   printf("Get PayloadSize fail\n");
  //   return -1;
  // }
  // g_nPayloadSize = stParam.nCurValue * 3;

  nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PIXEL_FORMAT[PixelFormat]); // BayerRG8 0x01080009 RGB8 0x02180014 BayerRG12Packed 0x010C002B
  if (nRet != MV_OK) {
    printf("Pixel setting can't work.");
    return -1;
  }

  setParams(handle, params_file);

  // set trigger mode as on
  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  if (MV_OK != nRet) {
    printf("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
    return -1;
  }

  // set trigger source
  nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
  if (MV_OK != nRet) {
    printf("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
    return -1;
  }

  RCLCPP_INFO(g_logger, "Finish all params set! Start grabbing...");
  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) {
    printf("Start Grabbing fail.\n");
    return -1;
  }

  pthread_t nThreadID;
  nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
  if (nRet != 0) {
    printf("thread create failed.ret = %d\n", nRet);
    return -1;
  }

  while (!exit_flag && rclcpp::ok()) {
    rclcpp::spin_some(g_node);
    loop_rate.sleep();
  }
  
  if (nThreadID) {
    pthread_join(nThreadID, NULL);
    RCLCPP_INFO_STREAM(g_logger, "Worker thread joined.");
  }

  nRet = MV_CC_StopGrabbing(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_CloseDevice(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
    return -1;
  }

  nRet = MV_CC_DestroyHandle(handle);
  if (MV_OK != nRet) {
    printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
    return -1;
  }

  CleanupTimestampShare();

  // These are global objects, so relying on static destruction would destroy
  // the node before its publishers. Release ROS entities explicitly while the
  // context is still valid.
  pub.shutdown();
  camera_info_pub.reset();
  g_node.reset();
  rclcpp::shutdown();

  return 0;
}
