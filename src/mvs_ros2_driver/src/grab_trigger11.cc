#include "MvCameraControl.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <sys/mman.h>
#include <unistd.h>
#include <vector>

// 日志输出 port to ROS2
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)

using namespace std;

struct time_stamp {
  int64_t high;
  int64_t low;
};

static time_stamp *pointt = (time_stamp *)MAP_FAILED;
static int g_shm_fd = -1;

enum PixelFormat : unsigned int {
  RGB8 = 0x02180014,
  BayerRG8 = 0x01080009,
  BayerRG12Packed = 0x010C002B,
  BayerGB12Packed = 0x010C002C,
  BayerGB8 = 0x0108000A
};

static bool exit_flag = false;
static rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
static std::vector<PixelFormat> PIXEL_FORMAT = {
    RGB8, BayerRG8, BayerRG12Packed, BayerGB12Packed, BayerGB8};

static std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
static std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
static std::string GainAutoStr[3] = {"Off", "Once", "Continues"};

static float image_scale = 1.0f;
static int trigger_enable = 1;

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (pstMVDevInfo == NULL) {
    ROS_ERROR("The Pointer of pstMVDevInfo is NULL!");
    return false;
  }

  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
    int nIp1 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    ROS_INFO("[GigE] device");
    ROS_INFO("Device Model Name: %s",
             pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    ROS_INFO("CurrentIp: %d.%d.%d.%d", nIp1, nIp2, nIp3, nIp4);
    ROS_INFO("SerialNumber: %s",
             pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
  } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    ROS_INFO("[USB] device");
    ROS_INFO("Device Model Name: %s",
             pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    ROS_INFO("SerialNumber: %s",
             pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
  } else {
    ROS_WARN("Not support.");
  }
  return true;
}

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    ROS_ERROR("Failed to open settings file at: %s", params_file.c_str());
    exit(-1);
  }

  image_scale = (float)Params["image_scale"];
  if (image_scale < 0.1f) {
    image_scale = 1.0f;
  }

  int ExposureTimeLower = (int)Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = (int)Params["AutoExposureTimeUpper"];
  int ExposureTime = (int)Params["ExposureTime"];
  int ExposureAutoMode = (int)Params["ExposureAutoMode"];
  int GainAuto = (int)Params["GainAuto"];
  float Gain = (float)Params["Gain"];
  float Gamma = (float)Params["Gamma"];
  int GammaSlector = (int)Params["GammaSelector"];

  int nRet = MV_OK;

  if (ExposureAutoMode < 0 || ExposureAutoMode > 2) {
    ROS_WARN("Invalid ExposureAutoMode: %d, force to Off(0)", ExposureAutoMode);
    ExposureAutoMode = 0;
  }
  if (GainAuto < 0 || GainAuto > 2) {
    ROS_WARN("Invalid GainAuto: %d, force to Off(0)", GainAuto);
    GainAuto = 0;
  }
  if (GammaSlector < 0 || GammaSlector > 2) {
    ROS_WARN("Invalid GammaSelector: %d, force to User(0)", GammaSlector);
    GammaSlector = 0;
  }

  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);
  if (MV_OK == nRet) {
    ROS_INFO("Set ExposureAutoMode: %s", ExposureAutoStr[ExposureAutoMode].c_str());
  } else {
    ROS_WARN("Fail to set Exposure Auto Mode, nRet[0x%x]", nRet);
  }

  if (ExposureAutoMode == 2) {
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      ROS_INFO("Set Exposure Time Lower: %dus", ExposureTimeLower);
    } else {
      ROS_ERROR("Fail to set Exposure Time Lower, nRet[0x%x]", nRet);
    }

    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      ROS_INFO("Set Exposure Time Upper: %dus", ExposureTimeUpper);
    } else {
      ROS_ERROR("Fail to set Exposure Time Upper, nRet[0x%x]", nRet);
    }
  }

  if (ExposureAutoMode == 0) {
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      ROS_INFO("Set Exposure Time: %dus", ExposureTime);
    } else {
      ROS_ERROR("Fail to set Exposure Time, nRet[0x%x]", nRet);
    }
  }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);
  if (MV_OK == nRet) {
    ROS_INFO("Set Gain Auto: %s", GainAutoStr[GainAuto].c_str());
  } else {
    ROS_ERROR("Fail to set Gain auto mode, nRet[0x%x]", nRet);
  }

  if (GainAuto == 0) {
    nRet = MV_CC_SetGain(handle, Gain);
    if (MV_OK == nRet) {
      ROS_INFO("Set Gain: %.3f", Gain);
    } else {
      ROS_ERROR("Fail to set Gain, nRet[0x%x]", nRet);
    }
  }

  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    ROS_INFO("Set GammaSlector: %s", GammaSlectorStr[GammaSlector].c_str());
  } else {
    ROS_ERROR("Fail to set GammaSlector, nRet[0x%x]", nRet);
  }

  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    ROS_INFO("Set Gamma: %.3f", Gamma);
  } else {
    ROS_ERROR("Fail to set Gamma, nRet[0x%x]", nRet);
  }
}

void SignalHandler(int signal) {
  if (signal == SIGINT) {
    ROS_WARN("Received Ctrl+C, exiting...");
    exit_flag = true;
  }
}

void SetupSignalHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

static void *WorkThread(void *pUser) {
  int nRet = MV_OK;

  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(stParam));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    ROS_ERROR("Get PayloadSize fail! nRet [0x%x]", nRet);
    return NULL;
  }
  ROS_INFO("Get PayloadSize success! val [%u]", stParam.nCurValue);

  MV_CC_PIXEL_CONVERT_PARAM stConvertParam;
  memset(&stConvertParam, 0, sizeof(stConvertParam));

  MV_FRAME_OUT stImageInfo;
  memset(&stImageInfo, 0, sizeof(stImageInfo));

  // 复用缓冲区，避免每帧malloc导致泄漏/碎片
  std::vector<unsigned char> rgb_buffer;
  size_t rgb_capacity = 0;

  ROS_INFO("Capture loop start.");
  while (!exit_flag && rclcpp::ok()) {
    memset(&stImageInfo, 0, sizeof(stImageInfo));
    nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 10000);
    if (nRet != MV_OK) {
      ROS_WARN("Capture timeout, retrying... nRet [0x%x]", nRet);
      continue;
    }

    // 兜底：本轮必须释放SDK buffer
    bool need_free = true;

    rclcpp::Time rcv_time;
    if (trigger_enable && pointt != MAP_FAILED && pointt != nullptr &&
        pointt->low != 0) {
      // pointt->low 假设就是纳秒时间戳
      rcv_time = rclcpp::Time(pointt->low);
    } else {
      rcv_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    }

    const uint32_t w = stImageInfo.stFrameInfo.nExtendWidth;
    const uint32_t h = stImageInfo.stFrameInfo.nExtendHeight;

    // RGB8: 3 bytes/pixel
    size_t need_bytes = static_cast<size_t>(w) * static_cast<size_t>(h) * 3;
    if (need_bytes == 0) {
      ROS_WARN("Invalid frame size: %ux%u", w, h);
      if (need_free) {
        MV_CC_FreeImageBuffer(pUser, &stImageInfo);
      }
      continue;
    }

    if (need_bytes > rgb_capacity) {
      rgb_buffer.resize(need_bytes);
      rgb_capacity = need_bytes;
    }

    memset(&stConvertParam, 0, sizeof(stConvertParam));
    stConvertParam.nWidth = w;
    stConvertParam.nHeight = h;
    stConvertParam.pSrcData = stImageInfo.pBufAddr;
    stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLenEx;
    stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
    stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
    stConvertParam.pDstBuffer = rgb_buffer.data();
    stConvertParam.nDstBufferSize = static_cast<unsigned int>(rgb_capacity);

    nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
    if (MV_OK != nRet) {
      ROS_WARN("MV_CC_ConvertPixelType failed! nRet [0x%x], skipping frame", nRet);
      if (need_free) {
        MV_CC_FreeImageBuffer(pUser, &stImageInfo);
      }
      continue;
    }

    cv::Mat srcImage((int)h, (int)w, CV_8UC3, rgb_buffer.data());

    // 降低日志级别，避免大量I/O拖垮系统
    ROS_DEBUG("GetOneFrame, Width[%u], Height[%u], nFrameNum[%u]",
              w, h, stImageInfo.stFrameInfo.nFrameNum);

    cv::Mat outImage;
    if (image_scale > 0.0f && image_scale != 1.0f) {
      cv::resize(srcImage, outImage,
                 cv::Size((int)(srcImage.cols * image_scale),
                          (int)(srcImage.rows * image_scale)),
                 0, 0, cv::INTER_LINEAR);
    } else {
      outImage = srcImage; // 共享同一块数据，仅用于立刻拷贝到msg
    }

    sensor_msgs::msg::Image msg;
    msg.header.stamp = rcv_time;
    msg.height = (uint32_t)outImage.rows;
    msg.width = (uint32_t)outImage.cols;
    msg.encoding = "rgb8";
    msg.is_bigendian = false;
    msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(outImage.step);
    msg.data.assign(outImage.data,
                    outImage.data + outImage.total() * outImage.elemSize());

    pub->publish(msg);

    if (need_free) {
      int fr = MV_CC_FreeImageBuffer(pUser, &stImageInfo);
      if (fr != MV_OK) {
        ROS_WARN("MV_CC_FreeImageBuffer failed! nRet [0x%x]", fr);
      }
    }
  }

  return nullptr;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // ===== 提前声明，避免 goto 跨初始化 =====
  int nRet = MV_OK;
  void *handle = NULL;
  pthread_t nThreadID = 0;
  int tret = 0;
  bool thread_created = false;
  bool grabbing_started = false;
  bool device_opened = false;
  bool handle_created = false;

  bool find_expect_camera = false;
  unsigned int nIndex = 0;

  std::string params_file;
  cv::FileStorage Params;
  std::string expect_serial_number;
  std::string pub_topic;
  int pixelFormatIndex = 0;

  auto node = rclcpp::Node::make_shared("mvs_camera_trigger");

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(stDeviceList));
  // ===== 声明结束 =====

  if (argc < 2) {
    ROS_ERROR("Usage: mvs_camera_node <params_yaml_path>");
    goto EXIT_WITH_CLEANUP;
  }

  params_file = std::string(argv[1]);
  Params.open(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    ROS_ERROR("Failed to open settings file at: %s", params_file.c_str());
    goto EXIT_WITH_CLEANUP;
  }

  trigger_enable = (int)Params["TriggerEnable"];
  expect_serial_number = (std::string)Params["SerialNumber"];
  pub_topic = (std::string)Params["TopicName"];
  pixelFormatIndex = (int)Params["PixelFormat"];

  if (pixelFormatIndex < 0 || pixelFormatIndex >= (int)PIXEL_FORMAT.size()) {
    ROS_WARN("Invalid PixelFormat index: %d, fallback to 0", pixelFormatIndex);
    pixelFormatIndex = 0;
  }

  pub = node->create_publisher<sensor_msgs::msg::Image>(pub_topic, 10);

  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet || stDeviceList.nDeviceNum == 0) {
    ROS_ERROR("MV_CC_EnumDevices failed or no device. nRet [0x%x]", nRet);
    goto EXIT_WITH_CLEANUP;
  }

  for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
    if (stDeviceList.pDeviceInfo[i]) PrintDeviceInfo(stDeviceList.pDeviceInfo[i]);
  }

  if (stDeviceList.nDeviceNum > 1) {
    if (expect_serial_number.empty()) {
      ROS_ERROR("Expected serial number is empty!");
      goto EXIT_WITH_CLEANUP;
    }
    for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++) {
      if (!stDeviceList.pDeviceInfo[i]) continue;
      std::string sn;
      if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE) {
        sn = (char*)stDeviceList.pDeviceInfo[i]->SpecialInfo.stUsb3VInfo.chSerialNumber;
      } else if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE) {
        sn = (char*)stDeviceList.pDeviceInfo[i]->SpecialInfo.stGigEInfo.chSerialNumber;
      }
      if (sn == expect_serial_number) {
        nIndex = i;
        find_expect_camera = true;
        break;
      }
    }
    if (!find_expect_camera) {
      ROS_ERROR("Cannot find camera serial: %s", expect_serial_number.c_str());
      goto EXIT_WITH_CLEANUP;
    }
  } else {
    nIndex = 0;
  }

  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_CreateHandle fail! nRet [0x%x]", nRet);
    goto EXIT_WITH_CLEANUP;
  }
  handle_created = true;

  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_OpenDevice fail! nRet [0x%x]", nRet);
    goto EXIT_HANDLE;
  }
  device_opened = true;

  nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
  if (MV_OK != nRet) goto EXIT_DEVICE;

  nRet = MV_CC_SetEnumValue(handle, "PixelFormat", PIXEL_FORMAT[pixelFormatIndex]);
  if (MV_OK != nRet) goto EXIT_DEVICE;

  setParams(handle, params_file);

  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  if (MV_OK != nRet) goto EXIT_DEVICE;

  if (trigger_enable != 0) {
    nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
    if (MV_OK != nRet) goto EXIT_DEVICE;
  }

  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) goto EXIT_DEVICE;
  grabbing_started = true;

  tret = pthread_create(&nThreadID, NULL, WorkThread, handle);
  if (tret != 0) {
    ROS_ERROR("thread create failed. ret=%d", tret);
    goto EXIT_STOP_GRAB;
  }
  thread_created = true;

  while (!exit_flag && rclcpp::ok()) {
    rclcpp::spin_some(node);
    usleep(100000);
  }

EXIT_STOP_GRAB:
  if (thread_created) {
    pthread_join(nThreadID, NULL);
    thread_created = false;
  }
  if (grabbing_started && handle) {
    MV_CC_StopGrabbing(handle);
  }

EXIT_DEVICE:
  if (device_opened && handle) {
    MV_CC_CloseDevice(handle);
  }

EXIT_HANDLE:
  if (handle_created && handle) {
    MV_CC_DestroyHandle(handle);
    handle = NULL;
  }

EXIT_WITH_CLEANUP:
  if (pointt != MAP_FAILED && pointt != nullptr) {
    munmap(pointt, sizeof(time_stamp));
    pointt = (time_stamp *)MAP_FAILED;
  }
  if (g_shm_fd >= 0) {
    close(g_shm_fd);
    g_shm_fd = -1;
  }

  rclcpp::shutdown();
  return 0;
}