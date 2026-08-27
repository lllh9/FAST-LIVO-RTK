#include "MvCameraControl.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

// 日志输出 port to ROS2
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_ERROR(...)                                                         \
  RCLCPP_ERROR(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_DEBUG(...)                                                         \
  RCLCPP_DEBUG(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)

using namespace std;

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt;

enum PixelFormat : unsigned int {
  RGB8 = 0x02180014,
  BayerRG8 = 0x01080009,
  BayerRG12Packed = 0x010C002B,
  BayerGB12Packed = 0x010C002C,
  BayerGB8 = 0x0108000A
};

// unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
bool exit_flag = false;
int width, height;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub;
sensor_msgs::msg::CameraInfo camera_info_msg_;
std::vector<PixelFormat> PIXEL_FORMAT = {RGB8, BayerRG8, BayerRG12Packed,
                                         BayerGB12Packed, BayerGB8};
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
float image_scale = 0.0;
int trigger_enable = 1;
std::string frame_id = "camera";

// 互斥锁保护发布器
pthread_mutex_t publisher_mutex = PTHREAD_MUTEX_INITIALIZER;

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (NULL == pstMVDevInfo) {
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
    ROS_INFO("Sgrab_triggererialNumber: %s",
             pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
  } else {
    ROS_WARN("Not support.");
  }
  return true;
}

// 初始化 camera_info
void initCameraInfo(int w, int h)
{
  camera_info_msg_.width = w;
  camera_info_msg_.height = h;
  camera_info_msg_.distortion_model = "plumb_bob";

  // D - 5个畸变系数
  camera_info_msg_.d.clear();
  camera_info_msg_.d = {
      -0.062221,
      0.013896,
      -0.000052,
      -0.000191,
      0.000000};

  // K - 3x3 内参矩阵
  camera_info_msg_.k.fill(0.0);
  camera_info_msg_.k[0] = 1320.300279;  // fx
  camera_info_msg_.k[2] = 653.660791;    // cx
  camera_info_msg_.k[4] = 1321.153958;  // fy
  camera_info_msg_.k[5] = 475.605485;   // cy
  camera_info_msg_.k[8] = 1.0;

  // R - 3x3 旋转矩阵
  camera_info_msg_.r.fill(0.0);
  camera_info_msg_.r[0] = 1.0;
  camera_info_msg_.r[4] = 1.0;
  camera_info_msg_.r[8] = 1.0;

  // P - 3x4 投影矩阵
  camera_info_msg_.p.fill(0.0);
  camera_info_msg_.p[0] = 1300.422119;
  camera_info_msg_.p[2] = 653.343830;
  camera_info_msg_.p[5] = 1307.535889;
  camera_info_msg_.p[6] = 474.449932;
  camera_info_msg_.p[10] = 1.0;
}

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR(msg.c_str());
    exit(-1);
  }
  image_scale = Params["image_scale"];
  if (image_scale < 0.1)
    image_scale = 1;
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
  std::string msg =
      "Set ExposureAutoMode: " + ExposureAutoStr[ExposureAutoMode];

  if (MV_OK == nRet) {
    ROS_INFO(msg.c_str());
  } else {
    if (ExposureAutoMode == 2) {
      ROS_WARN("Fail to set Exposure Auto Mode to Continues");
    } else {
      ROS_INFO(msg.c_str());
    }
  }

  // 如果是自动曝光
  if (ExposureAutoMode == 2) {
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) +
          "us";
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Exposure Time Lower");
    }
    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) +
          "us";
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Exposure Time Upper");
    }
  }

  // 如果是固定曝光
  if (ExposureAutoMode == 0) {
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time: " + std::to_string(ExposureTime) + "us";
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Exposure Time");
    }
  }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);

  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Gain auto mode");
  }

  if (GainAuto == 0) {
    nRet = MV_CC_SetGain(handle, Gain);
    if (MV_OK == nRet) {
      std::string msg = "Set Gain: " + std::to_string(Gain);
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Gain");
    }
  }

  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set GammaSlector");
  }

  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    std::string msg = "Set Gamma: " + std::to_string(Gamma);
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Gamma");
  }
}

void SignalHandler(int signal) {
  if (signal == SIGINT) { // 捕捉 Ctrl + C 触发的 SIGINT 信号
    fprintf(stderr, "\nReceived Ctrl+C, exiting...\n");
    exit_flag = true; // 设置退出标志
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

  unsigned char *pDataForRGB = NULL;
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    ROS_ERROR("Get PayloadSize fail! nRet [0x%x]", nRet);
    return NULL;
  }
  ROS_INFO("Get PayloadSize success! val [%d]", stParam.nCurValue);

  // MV_FRAME_OUT_INFO_EX stImageInfo = {0};
  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
  MV_FRAME_OUT stImageInfo = {0};
  MV_CC_IMAGE stImage = {0};

  ROS_INFO("Capture loop start.");
  int frame_count = 0;
  
  while (!exit_flag && rclcpp::ok()) {

    nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 10000);

    if (nRet == MV_OK) {

      rclcpp::Time rcv_time;
      if (trigger_enable && pointt != MAP_FAILED && pointt->low != 0) {
        int64_t b = pointt->low;
        double time_pc = b / 1000000000.0;
        rcv_time =
            rclcpp::Time(static_cast<int64_t>(time_pc * 1e9)); // 转换为纳
      } else {
        rcv_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
      }

      std::string debug_msg;
      debug_msg = "GetOneFrame,nFrameNum[" +
                  std::to_string(stImageInfo.stFrameInfo.nFrameNum) +
                  "], FrameTime:" + std::to_string(rcv_time.seconds());
      ROS_DEBUG(debug_msg.c_str());

      pDataForRGB = (unsigned char *)malloc(
          stImageInfo.stFrameInfo.nExtendWidth *
              stImageInfo.stFrameInfo.nExtendHeight * 4 +
          2048);
      if (NULL == pDataForRGB) {
        ROS_ERROR("pDataForRGB is null");
        break;
      }

      stConvertParam.nWidth = stImageInfo.stFrameInfo.nExtendWidth;
      stConvertParam.nHeight = stImageInfo.stFrameInfo.nExtendHeight;
      stConvertParam.pSrcData = stImageInfo.pBufAddr;
      stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLenEx;
      stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
      stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
      stConvertParam.pDstBuffer = pDataForRGB;
      stConvertParam.nDstBufferSize =
          stImageInfo.stFrameInfo.nExtendWidth *
              stImageInfo.stFrameInfo.nExtendHeight * 4 +
          2048;

      nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);

      if (MV_OK != nRet) {
        ROS_WARN(
            "MV_CC_ConvertPixelType failed! nRet [%x], skipping this frame",
            nRet);
        free(pDataForRGB);                // 修改：失败分支释放
        pDataForRGB = nullptr;            // 修改：置空防悬挂
        MV_CC_FreeImageBuffer(pUser, &stImageInfo);
        continue;
      }

      cv::Mat srcImage;
      srcImage = cv::Mat(stImageInfo.stFrameInfo.nHeight,
                         stImageInfo.stFrameInfo.nWidth, CV_8UC3, pDataForRGB);

      ROS_INFO("GetOneFrame, Width[%d], Height[%d], nFrameNum[%d]",
               stImageInfo.stFrameInfo.nExtendWidth,
               stImageInfo.stFrameInfo.nExtendHeight,
               stImageInfo.stFrameInfo.nFrameNum);
      
      MV_CC_FreeImageBuffer(pUser, &stImageInfo);

      // 如果图像尺寸���化，更新 camera_info
      if (width != (int)stImageInfo.stFrameInfo.nExtendWidth || 
          height != (int)stImageInfo.stFrameInfo.nExtendHeight) {
        width = stImageInfo.stFrameInfo.nExtendWidth;
        height = stImageInfo.stFrameInfo.nExtendHeight;
        initCameraInfo(width, height);
        ROS_INFO("Camera info updated: %dx%d", width, height);
      }

      stImage.nWidth = stImageInfo.stFrameInfo.nExtendWidth;
      stImage.nHeight = stImageInfo.stFrameInfo.nExtendHeight;
      stImage.enPixelType = stImageInfo.stFrameInfo.enPixelType;
      stImage.pImageBuf = stImageInfo.pBufAddr;
      stImage.nImageLen = stImageInfo.stFrameInfo.nFrameLenEx;

      if (image_scale > 0.0) {
        cv::resize(
            srcImage, srcImage,
            cv::Size(srcImage.cols * image_scale, srcImage.rows * image_scale),
            cv::INTER_LINEAR);
      } else {
        ROS_WARN("Invalid image_scale: %f. Skipping resize.", image_scale);
      }

      // ==================== 同时发布 Image 和 CameraInfo ====================
      pthread_mutex_lock(&publisher_mutex);
      
      if (pub && camera_info_pub) {
        try {
          // 构建并发布 Image 消息
          sensor_msgs::msg::Image msg;
          msg.header.stamp = rcv_time;
          msg.header.frame_id = frame_id;
          msg.height = srcImage.rows;
          msg.width = srcImage.cols;
          msg.encoding = "rgb8";
          msg.is_bigendian = false;
          msg.step = srcImage.step;
          msg.data.assign(srcImage.data,
                          srcImage.data + srcImage.total() * srcImage.elemSize());

          pub->publish(msg);
          
          // 构建并发布 CameraInfo 消息
          sensor_msgs::msg::CameraInfo info_msg = camera_info_msg_;
          info_msg.header.stamp = rcv_time;
          info_msg.header.frame_id = frame_id;
          
          camera_info_pub->publish(info_msg);
          
          frame_count++;
          if (frame_count % 30 == 0) {
            ROS_INFO("Published %d frames with camera_info", frame_count);
          }
        }
        catch (const std::exception &e) {
          ROS_ERROR("Error publishing message: %s", e.what());
        }
      } else {
        ROS_ERROR("Publisher not initialized!");
      }
      
      pthread_mutex_unlock(&publisher_mutex);

      free(pDataForRGB);                  // 修改：成功路径释放
      pDataForRGB = nullptr;              // 修改：置空
    } else {
      ROS_WARN("Capture timeout, retrying...");
    }
  }

  // 修改：线程结束前兜底释放（防止 break 等路径遗漏）
  if (pDataForRGB) {
    free(pDataForRGB);
    pDataForRGB = nullptr;
  }

  ROS_INFO("Capture thread exiting. Published %d frames", frame_count);
  return 0;
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  std::string params_file = std::string(argv[1]);

  int nRet = MV_OK;
  void *handle = NULL;
  rclcpp::Rate loop_rate(10);
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR(msg.c_str());
    exit(-1);
  }
  ROS_INFO("Load settings from file: %s", params_file.c_str());
  trigger_enable = Params["TriggerEnable"];
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];
  int PixelFormat = Params["PixelFormat"];

  auto node = rclcpp::Node::make_shared("mvs_trigger");
  pub = node->create_publisher<sensor_msgs::msg::Image>(pub_topic, 10);
  ROS_INFO("Created image publisher on topic: %s", pub_topic.c_str());
  
  // 创建 camera_info 发布器
  std::string camera_info_topic = pub_topic + "/camera_info";
  camera_info_pub = node->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic, 10);
  ROS_INFO("Created camera_info publisher on topic: %s", camera_info_topic.c_str());

  const char *user_name = getlogin();
  std::string path_for_time_stamp =
      "/home/" + std::string(user_name) + "/timeshare";
  const char *shared_file_name = path_for_time_stamp.c_str();
  int fd = open(shared_file_name, O_RDWR);
  pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
                              MAP_SHARED, fd, 0);

  SetupSignalHandler();

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_EnumDevices fail! nRet [%x]", nRet);
    return -1;
  }

  if (stDeviceList.nDeviceNum > 0) {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      ROS_INFO("[device %d]:", i);
      MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (pDeviceInfo == NULL) {
        ROS_ERROR("Device info is NULL for device %d", i);
        return -1;
      }
      PrintDeviceInfo(pDeviceInfo);
    }
  } else {
    ROS_ERROR("Find No Devices!");
    return -1;
  }

  bool find_expect_camera = false;
  unsigned int nIndex = 0;

  if (stDeviceList.nDeviceNum > 1) {
    if (expect_serial_number.empty()) {
      ROS_ERROR("Expected serial number is empty!");
      return -1;
    }
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      if (stDeviceList.pDeviceInfo[i] == NULL) {
        ROS_ERROR("Device info is NULL for device %d", i);
        continue;
      }

      std::string serial_number;
      if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE) {
        serial_number =
            std::string((char *)stDeviceList.pDeviceInfo[i]
                            ->SpecialInfo.stUsb3VInfo.chSerialNumber);
      } else if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE) {
        serial_number =
            std::string((char *)stDeviceList.pDeviceInfo[i]
                            ->SpecialInfo.stGigEInfo.chSerialNumber);
      } else {
        ROS_ERROR("Unknown device type!");
        continue;
      }
      if (serial_number.empty()) {
        ROS_ERROR("Serial number is empty for device %d", i);
        continue;
      }
      if (expect_serial_number == serial_number) {
        find_expect_camera = true;
        nIndex = i;
        break;
      }
    }
    if (!find_expect_camera) {
      std::string msg =
          "Can not find the camera with serial number " + expect_serial_number;
      ROS_ERROR(msg.c_str());
      return -1;
    }
  } else {
    nIndex = 0;
  }

  // select device and create handle
  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_CreateHandle fail! nRet [%x]", nRet);
    return -1;
  }

  // open device
  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_OpenDevice fail! nRet [%x]", nRet);
    return -1;
  }

  nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
  if (MV_OK != nRet) {
    ROS_ERROR("set AcquisitionFrameRateEnable fail! nRet [%x]", nRet);
    return -1;
  }

  nRet = MV_CC_SetEnumValue(
      handle, "PixelFormat",
      PIXEL_FORMAT[PixelFormat]); // BayerRG8 0x01080009 RGB8 0x02180014
                                  // BayerRG12Packed 0x010C002B
  if (nRet != MV_OK) {
    ROS_ERROR("Pixel setting can't work.");
    return -1;
  }

  setParams(handle, params_file);

  // set trigger mode as on
  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_SetTriggerMode fail! nRet [%x]", nRet);
    return -1;
  } else {
    ROS_INFO("Set TriggerMode [%s]", trigger_enable == 0 ? "OFF" : "ON");
  }

  // set trigger source
  nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_SetTriggerSource fail! nRet [%x]", nRet);
    return -1;
  }

  ROS_INFO("Finish all params set! Start grabbing...");
  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("Start Grabbing fail.");
    return -1;
  }
  ROS_INFO("Start Grabbing Success.");

  pthread_t nThreadID;
  nRet = pthread_create(&nThreadID, NULL, WorkThread, handle);
  if (nRet != 0) {
    ROS_ERROR("thread create failed.ret = %d", nRet);
    return -1;
  }
  ROS_INFO("Start Grabbing thread Success, pid %ld", nThreadID);

  while (!exit_flag && rclcpp::ok()) {
    rclcpp::spin_some(node);
    usleep(100000);
  }

  if (nThreadID) {
    pthread_join(nThreadID, NULL);
    ROS_INFO("Worker thread joined.");
  }

  nRet = MV_CC_StopGrabbing(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_StopGrabbing fail! nRet [%x]", nRet);
    return -1;
  }
  ROS_INFO("MV_CC_StopGrabbing success!");

  nRet = MV_CC_CloseDevice(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_CloseDevice fail! nRet [%x]", nRet);
    return -1;
  }
  ROS_INFO("MV_CC_CloseDevice success!");

  nRet = MV_CC_DestroyHandle(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_DestroyHandle fail! nRet [%x]", nRet);
    return -1;
  }
  ROS_INFO("MV_CC_DestroyHandle success!");

  munmap(pointt, sizeof(time_stamp));
  pthread_mutex_destroy(&publisher_mutex);

  return 0;
}