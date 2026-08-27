#ifndef __HC_CAN_COMMON_HPP_
#define __HC_CAN_COMMON_HPP_

#include "device_connector.hpp"

#include <cstring>
#include <iostream>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <linux/ioctl.h>

#define LATITUDE_LONGITUDE_ID (804u)

class can_common : public hc__device_connector
{
private:
    int status = -1;                            //  连接状态。 -1 未连接，1 已连接
    int socketfd;                               //  socket fd
    std::string dev;                            //  CAN设备名称
    int can_rate = 0;                      //  CAN速率
    std::string data_format;                    //  数据格式

    int time_id;                                //  时间 ID
    int angrate_rawIMU_id = 0;                  //  IMU角速度原始值 ID
    int accel_rawIMU_id = 0;                    //  IMU加速度原始值 ID
    int sys_status_id = 0;                      //  INS定位状态 ID
    int altitude_id = 0;                        //  海拔高度 ID
    int pos_sigma_id = 0;                       //  位置西格玛 ID
    int velocity_level_id = 0;                  //  大地坐标系速度 ID
    int velocity_level_sigma_id = 0;            //  大地坐标系速度西格玛 ID
    int accel_vehicle_id = 0;                   //  车辆坐标系加速度 ID
    int heading_pitch_roll_id = 0;              //  姿态角 ID
    int heading_pitch_roll_sigma_id = 0;        //  姿态角西格玛 ID
    int angrate_vehicle_id = 0;                 //  车辆坐标系角速度 ID  
    int longitude_id = 0;                       //  定位经度 ID
    int latitude_id = 0;                        //  定位纬度 ID

    uint16_t gps_week = 0;
    float gps_time = 0;

    double angRate_raw_x = 0;
    double angRate_raw_y = 0;
    double angRate_raw_z = 0;

    double accel_raw_x = 0;
    double accel_raw_y = 0;
    double accel_raw_z = 0;

    uint8_t state_ins = 0;
    uint8_t num_sats_used = 0;
    uint8_t status_gnss = 0;
    uint8_t num_sats2_used = 0;
    float age = 0;                     
    uint8_t num_sats = 0;
    uint8_t num_sats2 = 0;

    double pos_alt = 0;

    double pos_e_sigma = 0;
    double pos_n_sigma = 0;
    double pos_u_sigma = 0;

    float vel_e = 0;
    float vel_n = 0;
    float vel_u = 0;
    float vel = 0;

    float vel_e_sigma = 0;
    float vel_n_sigma = 0;
    float vel_u_sigma = 0;
    float vel_sigma = 0;

    double accel_x = 0;
    double accel_y = 0;
    double accel_z = 0;

    float angle_yaw = 0;
    float angle_pitch = 0;
    float angle_roll = 0;
    float angle_heading = 0;

    double angle_yaw_sigma = 0;
    double angle_pitch_sigma = 0;
    double angle_roll_sigma = 0;

    double angRate_x = 0;
    double angRate_y = 0;
    double angRate_z = 0;

    double pos_lon2 = 0;

    double pos_lat2 = 0;

    double pos_lon = 0;
    double pos_lat = 0;

	struct ifreq ifr = {0};
	struct sockaddr_can can_addr = {0};
	struct can_frame frame = {0};

public:
    can_common(std::string dev, int can_rate, std::string data_format, int time_id, int angrate_rawIMU_id, int accel_rawIMU_id,\
            int sys_status_id,int altitude_id, int pos_sigma_id, int velocity_level_id, int velocity_level_sigma_id, int accel_vehicle_id,\
            int heading_pitch_roll_id,int heading_pitch_roll_sigma_id, int angrate_vehicle_id, int longitude_id, int latitude_id) : hc__device_connector()
    {
        this->dev = dev;
        this->can_rate = can_rate;
        this->data_format = data_format;  
        this->time_id = time_id;
        this->angrate_rawIMU_id = angrate_rawIMU_id;
        this->accel_rawIMU_id = accel_rawIMU_id;
        this->sys_status_id = sys_status_id;
        this->altitude_id = altitude_id;
        this->pos_sigma_id = pos_sigma_id;
        this->velocity_level_id = velocity_level_id;
        this->velocity_level_sigma_id = velocity_level_sigma_id;
        this->accel_vehicle_id = accel_vehicle_id;
        this->heading_pitch_roll_id = heading_pitch_roll_id;
        this->heading_pitch_roll_sigma_id = heading_pitch_roll_sigma_id;
        this->angrate_vehicle_id = angrate_vehicle_id;
        this->longitude_id = longitude_id;
        this->latitude_id = latitude_id;        
    }

    int connect(void)
    {
        if (this->status != -1)
            this->disconnect();

        // 建立套接字
        this->socketfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	    if(0 > this->socketfd) {
		    perror("socket error");
		    return -1;
	    }
	    /* 指定can0设备 */
	    strcpy(this->ifr.ifr_name, this->dev.c_str());
	    ioctl(this->socketfd, SIOCGIFINDEX, &this->ifr);
	    this->can_addr.can_family = AF_CAN;
	    this->can_addr.can_ifindex = this->ifr.ifr_ifindex;

        // 设置CAN速率，例如500 kbps
        //ioctl(this->socketfd, SIOCDEVPRIVATE, &this->can_rate);
        //::write(this->socketfd, can_rate, 6);
        if (bind(this->socketfd, (struct sockaddr *)&this->can_addr, sizeof(this->can_addr)) == -1)
        {
            this->status = -1;
            return -1;
        }
        else
        {
            this->status = 1;
            return 0;
        }
    }

    int write(char *data, unsigned int len)
    {
        while (this->status != 1)
        {
            fprintf(stderr, "CAN socket not created! recreate now\n");
            this->connect();
            sleep(1);
        }

        int write_bytes = ::write(this->socketfd, data, len);
        if (write_bytes == -1)
        {
            printf("socker [%d] send failed\n", this->socketfd);
            this->disconnect();
            return -1;
        }

        return write_bytes;
    }

    int read(char *data, unsigned int maxsize)
    {
        while (this->status != 1)
        {
            fprintf(stderr, "can socket not created! recreate now\n");
            this->connect();
            sleep(1);
        }

        int read_bytes = ::read(this->socketfd, &this->frame, sizeof(struct can_frame));
        if (read_bytes <= 0)
        {
            if (errno == EWOULDBLOCK) // nothing sended by client sock
            {
                errno = 0;
            }
            else
            {
                printf("socker [%d] read failed\n", this->socketfd);
                this->disconnect();
                return -1;
            }
        }
		/* 校验是否接收到错误帧 */
		if (this->frame.can_id & CAN_ERR_FLAG) {
			printf("Error frame!\n");
			return -1;
		}

		/* 校验帧类型：数据帧还是远程帧 */
		if (this->frame.can_id & CAN_RTR_FLAG) {
			printf("remote request\n");
			return 0;
		}

		/* 校验帧格式 */
		if (this->frame.can_id & CAN_EFF_FLAG)	//扩展帧
			sprintf(data, "%s扩展帧 <0x%08x> ", data, this->frame.can_id & CAN_EFF_MASK);
		else		//标准帧
			sprintf(data, "%s标准帧 <0x%03x> ", data, this->frame.can_id & CAN_SFF_MASK);

		/* 打印数据长度 */
		sprintf(data, "%s[%d] ", data, this->frame.can_dlc);

		/* 打印数据 */
		for (int i = 0; i < this->frame.can_dlc; i++)
			sprintf(data, "%s%02x ", data, this->frame.data[i]);
		//sprintf(data, "\n");
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), data);

        if( data_format.compare("motorola") == 0 )
        {
            decode_motorola();
        }
        else if( data_format.compare("intel") == 0 )
        {
            decode_intel();
        }

        return read_bytes;
    }

    int decode_motorola(void)
    {
        if(this->frame.can_id == this->time_id)
        {
            this->gps_week = (this->frame.data[0] << 8u) | this->frame.data[1];
            this->gps_time = ((this->frame.data[2] << 24u) | (this->frame.data[3] << 16u) | (this->frame.data[4] << 8u) | this->frame.data[5]) * 0.001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gps_week = %u, gps_time = %f", this->gps_week, this->gps_time);
        }
        else if(this->frame.can_id == this->angrate_rawIMU_id)
        {
            int32_t temp_x = 0;
            int32_t temp_y = 0;
            int32_t temp_z = 0;

            temp_x = (this->frame.data[0] << 12u) | (this->frame.data[1] << 4u) | (this->frame.data[2] >> 4u);
            temp_y = ((this->frame.data[2] & 0x0F) << 16u) | (this->frame.data[3] << 8u) | this->frame.data[4];
            temp_z = (this->frame.data[5] << 12u) | (this->frame.data[6] << 4u) | (this->frame.data[7] >> 4u);

            // X,Y,Z数据的最高位是符号位，且只有20位，因此用32位数据保存时需要使用符号位来填充前12位数据，以保证符号不会改变
            if((this->frame.data[0] & 0x80) != 0)
            {
                temp_x |= 0xFFF00000;
            }
            else
            {
                temp_x &= 0x000FFFFF;               
            }

            if((this->frame.data[2] & 0x08) != 0)
            {
                temp_y |= 0xFFF00000;
            }
            else
            {
                temp_y &= 0x000FFFFF;               
            }

            if((this->frame.data[5] & 0x80) != 0)
            {
                temp_z |= 0xFFF00000;
            }
            else
            {
                temp_z &= 0x000FFFFF;               
            }

            this->angRate_raw_x = temp_x * 0.01;
            this->angRate_raw_y = temp_y * 0.01;
            this->angRate_raw_z = temp_z * 0.01;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angRate_raw_x = %f, angRate_raw_y = %f, angRate_raw_z = %f", this->angRate_raw_x, this->angRate_raw_y, this->angRate_raw_z);
        }
        else if(this->frame.can_id == this->accel_rawIMU_id)
        {
            int32_t temp_x = 0;
            int32_t temp_y = 0;
            int32_t temp_z = 0;

            temp_x = (this->frame.data[0] << 12u) | (this->frame.data[1] << 4u) | (this->frame.data[2] >> 4u);
            temp_y = ((this->frame.data[2] & 0x0F) << 16u) | (this->frame.data[3] << 8u) | this->frame.data[4];
            temp_z = (this->frame.data[5] << 12u) | (this->frame.data[6] << 4u) | (this->frame.data[7] >> 4u);

            // X,Y,Z数据的最高位是符号位，且只有20位，因此用32位数据保存时需要使用符号位来填充前12位数据，以保证符号不会改变
            if((this->frame.data[0] & 0x80) != 0)
            {
                temp_x |= 0xFFF00000;
            }
            else
            {
                temp_x &= 0x000FFFFF;               
            }

            if((this->frame.data[2] & 0x08) != 0)
            {
                temp_y |= 0xFFF00000;
            }
            else
            {
                temp_y &= 0x000FFFFF;               
            }

            if((this->frame.data[5] & 0x80) != 0)
            {
                temp_z |= 0xFFF00000;
            }
            else
            {
                temp_z &= 0x000FFFFF;               
            }

            this->accel_raw_x = temp_x * 0.0001;
            this->accel_raw_y = temp_y * 0.0001;
            this->accel_raw_z = temp_z * 0.0001;            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "accel_raw_x = %f, accel_raw_y = %f, accel_raw_z = %f", accel_raw_x, accel_raw_y, accel_raw_z);
        }
        else if(this->frame.can_id == this->sys_status_id)
        {
            this->state_ins = this->frame.data[0];
            this->num_sats_used = this->frame.data[1];
            this->status_gnss = this->frame.data[2];
            this->num_sats2_used = this->frame.data[3];
            this->age = ((this->frame.data[4] << 8u) | this->frame.data[5]) * 0.01;
            this->num_sats = this->frame.data[6];
            this->num_sats2 = this->frame.data[7];

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "state_ins = %u, num_sats_used = %u, status_gnss = %u, num_sats2_used = %u, age = %f, num_sats = %u, num_sats2 = %u", \
            this->state_ins, this->num_sats_used, this->status_gnss, this->num_sats2_used, this->age, this->num_sats, this->num_sats2);
        }    
        else if(this->frame.can_id == this->altitude_id)
        {
            this->pos_alt = ((this->frame.data[0] << 24u) | (this->frame.data[1] << 16u) | (this->frame.data[2] << 8u) | this->frame.data[3]) * 0.001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pos_alt = %f", this->pos_alt);
        }
        else if(this->frame.can_id == this->pos_sigma_id)
        {
            this->pos_e_sigma = ((this->frame.data[0] << 12u) | (this->frame.data[1] << 4u) | (this->frame.data[2] >> 4u)) * 0.0001;
            this->pos_n_sigma = (((this->frame.data[2] & 0x0F) << 16u) | (this->frame.data[3] << 8u) | this->frame.data[4]) * 0.0001;
            this->pos_u_sigma = ((this->frame.data[5] << 12u) | (this->frame.data[6] << 4u) | (this->frame.data[7] >> 4u)) * 0.0001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pos_e_sigma = %f, pos_n_sigma = %f, pos_u_sigma = %f", this->pos_e_sigma, this->pos_n_sigma, this->pos_u_sigma);
        }
        else if(this->frame.can_id == this->velocity_level_id)
        {
            this->vel_e = (int16_t)((this->frame.data[0] << 8u) | this->frame.data[1]) * 0.01;
            this->vel_n = (int16_t)((this->frame.data[2] << 8u) | this->frame.data[3]) * 0.01;
            this->vel_u = (int16_t)((this->frame.data[4] << 8u) | this->frame.data[5]) * 0.01;
            this->vel = (int16_t)((this->frame.data[6] << 8u) | this->frame.data[7]) * 0.01;         
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel_e = %f, vel_n = %f, vel_u = %f, vel = %f", this->vel_e, this->vel_n, this->vel_u, this->vel);
        }
        else if(this->frame.can_id == this->velocity_level_sigma_id)
        {
            this->vel_e_sigma = ((this->frame.data[0] << 8u) | this->frame.data[1]) * 0.001;
            this->vel_n_sigma = ((this->frame.data[2] << 8u) | this->frame.data[3]) * 0.001;
            this->vel_u_sigma = ((this->frame.data[4] << 8u) | this->frame.data[5]) * 0.001;
            this->vel_sigma = ((this->frame.data[6] << 8u) | this->frame.data[7]) * 0.001;         
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel_e_sigma = %f, vel_n_sigma = %f, vel_u_sigma = %f, vel_sigma = %f", this->vel_e_sigma, this->vel_n_sigma, this->vel_u_sigma, this->vel_sigma);
        }
        else if(this->frame.can_id == this->accel_vehicle_id)
        {
            int32_t temp_x = 0;
            int32_t temp_y = 0;
            int32_t temp_z = 0;

            temp_x = (this->frame.data[0] << 12u) | (this->frame.data[1] << 4u) | (this->frame.data[2] >> 4u);
            temp_y = ((this->frame.data[2] & 0x0F) << 16u) | (this->frame.data[3] << 8u) | this->frame.data[4];
            temp_z = (this->frame.data[5] << 12u) | (this->frame.data[6] << 4u) | (this->frame.data[7] >> 4u);

            // X,Y,Z数据的最高位是符号位，且只有20位，因此用32位数据保存时需要使用符号位来填充前12位数据，以保证符号不会改变
            if((this->frame.data[0] & 0x80) != 0)
            {
                temp_x |= 0xFFF00000;
            }
            else
            {
                temp_x &= 0x000FFFFF;               
            }

            if((this->frame.data[2] & 0x08) != 0)
            {
                temp_y |= 0xFFF00000;
            }
            else
            {
                temp_y &= 0x000FFFFF;               
            }

            if((this->frame.data[5] & 0x80) != 0)
            {
                temp_z |= 0xFFF00000;
            }
            else
            {
                temp_z &= 0x000FFFFF;               
            }

            this->accel_x = temp_x * 0.0001;
            this->accel_y = temp_y * 0.0001;
            this->accel_z = temp_z * 0.0001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "accel_x = %f, accel_y = %f, accel_z = %f", accel_x, accel_y, accel_z);
        }
        else if(this->frame.can_id == this->heading_pitch_roll_id)
        {
            this->angle_yaw = ((this->frame.data[0] << 8u) | this->frame.data[1]) * 0.01;
            this->angle_pitch = (int16_t)((this->frame.data[2] << 8u) | this->frame.data[3]) * 0.01;
            this->angle_roll = (int16_t)((this->frame.data[4] << 8u) | this->frame.data[5]) * 0.01;
            this->angle_heading = ((this->frame.data[6] << 8u) | this->frame.data[7]) * 0.01;         
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angle_yaw = %f, angle_pitch = %f, angle_roll = %f, angle_heading = %f", this->angle_yaw, this->angle_pitch, this->angle_roll, this->angle_heading);
        }
        else if(this->frame.can_id == this->heading_pitch_roll_sigma_id)
        {
            this->angle_yaw_sigma = ((this->frame.data[0] << 12u) | (this->frame.data[1] << 4u) | (this->frame.data[2] >> 4u)) * 0.0001;
            this->angle_pitch_sigma = (((this->frame.data[2] & 0x0F) << 16u) | (this->frame.data[3] << 8u) | this->frame.data[4]) * 0.0001;
            this->angle_roll_sigma = ((this->frame.data[5] << 12u) | (this->frame.data[6] << 4u) | (this->frame.data[7] >> 4u)) * 0.0001;       
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angle_yaw_sigma = %f, angle_pitch_sigma = %f, angle_roll_sigma = %f", this->angle_yaw_sigma, this->angle_pitch_sigma, this->angle_roll_sigma);
        }
        else if(this->frame.can_id == this->angrate_vehicle_id)
        {
            int32_t temp_x = 0;
            int32_t temp_y = 0;
            int32_t temp_z = 0;

            temp_x = (this->frame.data[0] << 12u) | (this->frame.data[1] << 4u) | (this->frame.data[2] >> 4u);
            temp_y = ((this->frame.data[2] & 0x0F) << 16u) | (this->frame.data[3] << 8u) | this->frame.data[4];
            temp_z = (this->frame.data[5] << 12u) | (this->frame.data[6] << 4u) | (this->frame.data[7] >> 4u);

            // X,Y,Z数据的最高位是符号位，且只有20位，因此用32位数据保存时需要使用符号位来填充前12位数据，以保证符号不会改变
            if((this->frame.data[0] & 0x80) != 0)
            {
                temp_x |= 0xFFF00000;
            }
            else
            {
                temp_x &= 0x000FFFFF;               
            }

            if((this->frame.data[2] & 0x08) != 0)
            {
                temp_y |= 0xFFF00000;
            }
            else
            {
                temp_y &= 0x000FFFFF;               
            }

            if((this->frame.data[5] & 0x80) != 0)
            {
                temp_z |= 0xFFF00000;
            }
            else
            {
                temp_z &= 0x000FFFFF;               
            }

            this->angRate_x = temp_x * 0.01;
            this->angRate_y = temp_y * 0.01;
            this->angRate_z = temp_z * 0.01;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angRate_x = %f, angRate_y = %f, angRate_z = %f", angRate_x, angRate_y, angRate_z);
        }
        else if(this->frame.can_id == this->longitude_id)
        {
            this->pos_lon2 = (((int64_t)this->frame.data[0] << 56u) | ((int64_t)this->frame.data[1] << 48u) | ((int64_t)this->frame.data[2] << 40u) | ((int64_t)this->frame.data[3] << 32u) | ((int64_t)this->frame.data[4] << 24u) | ((int64_t)this->frame.data[5] << 16u) | ((int64_t)this->frame.data[6] << 8u) | (int64_t)this->frame.data[7]) * 0.00000001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pos_lon2 = %f", this->pos_lon2);
        }
        else if(this->frame.can_id == this->latitude_id)
        {
            this->pos_lat2 = (((int64_t)this->frame.data[0] << 56u) | ((int64_t)this->frame.data[1] << 48u) | ((int64_t)this->frame.data[2] << 40u) | ((int64_t)this->frame.data[3] << 32u) | ((int64_t)this->frame.data[4] << 24u) | ((int64_t)this->frame.data[5] << 16u) | ((int64_t)this->frame.data[6] << 8u) | (int64_t)this->frame.data[7]) * 0.00000001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pos_lat2 = %f", this->pos_lat2);
        }
        else if(this->frame.can_id == LATITUDE_LONGITUDE_ID)
        {
            this->pos_lat = ((this->frame.data[0] << 24u) | (this->frame.data[1] << 16u) | (this->frame.data[2] << 8u) | this->frame.data[3]) * 0.0000001;
            this->pos_lon = ((this->frame.data[4] << 24u) | (this->frame.data[5] << 16u) | (this->frame.data[6] << 8u) | this->frame.data[7]) * 0.0000001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pos_lat = %f, pos_lon = %f", this->pos_lat, this->pos_lon);
        }
        else 
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "frame.can_id(%u) unkonw", this->frame.can_id);
        }

        return 0;
    }

    int decode_intel(void)
    {
        if(this->frame.can_id == this->time_id)
        {
            this->gps_week = (this->frame.data[1] << 8u) | this->frame.data[0];
            this->gps_time = ((this->frame.data[5] << 24u) | (this->frame.data[4] << 16u) | (this->frame.data[3] << 8u) | this->frame.data[2]) * 0.001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "gps_week = %u, gps_time = %f", this->gps_week, this->gps_time);
        }
        else if(this->frame.can_id == this->angrate_rawIMU_id)
        {
            int32_t temp_x = 0;
            int32_t temp_y = 0;
            int32_t temp_z = 0;

            temp_x = (this->frame.data[2] << 16u) | (this->frame.data[1] << 8u) | this->frame.data[0];
            temp_y = (this->frame.data[4] << 12u) | (this->frame.data[3] << 4u) | (this->frame.data[2] >> 4u);
            temp_z = (this->frame.data[7] << 16u) | (this->frame.data[6] << 8u) | this->frame.data[5];

            // X,Y,Z数据的最高位是符号位，且只有20位，因此用32位数据保存时需要使用符号位来填充前12位数据，以保证符号不会改变
            if((this->frame.data[2] & 0x08) != 0)
            {
                temp_x |= 0xFFF00000;
            }
            else
            {
                temp_x &= 0x000FFFFF;               
            }

            if((this->frame.data[4] & 0x80) != 0)
            {
                temp_y |= 0xFFF00000;
            }
            else
            {
                temp_y &= 0x000FFFFF;               
            }

            if((this->frame.data[7] & 0x08) != 0)
            {
                temp_z |= 0xFFF00000;
            }
            else
            {
                temp_z &= 0x000FFFFF;               
            }

            this->angRate_raw_x = temp_x * 0.01;
            this->angRate_raw_y = temp_y * 0.01;
            this->angRate_raw_z = temp_z * 0.01;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angRate_raw_x = %f, angRate_raw_y = %f, angRate_raw_z = %f", this->angRate_raw_x, this->angRate_raw_y, this->angRate_raw_z);
        }
        else if(this->frame.can_id == this->accel_rawIMU_id)
        {
            int32_t temp_x = 0;
            int32_t temp_y = 0;
            int32_t temp_z = 0;

            temp_x = (this->frame.data[2] << 16u) | (this->frame.data[1] << 8u) | this->frame.data[0];
            temp_y = (this->frame.data[4] << 12u) | (this->frame.data[3] << 4u) | (this->frame.data[2] >> 4u);
            temp_z = (this->frame.data[7] << 16u) | (this->frame.data[6] << 8u) | this->frame.data[5];

            // X,Y,Z数据的最高位是符号位，且只有20位，因此用32位数据保存时需要使用符号位来填充前12位数据，以保证符号不会改变
            if((this->frame.data[2] & 0x08) != 0)
            {
                temp_x |= 0xFFF00000;
            }
            else
            {
                temp_x &= 0x000FFFFF;               
            }

            if((this->frame.data[4] & 0x80) != 0)
            {
                temp_y |= 0xFFF00000;
            }
            else
            {
                temp_y &= 0x000FFFFF;               
            }

            if((this->frame.data[7] & 0x08) != 0)
            {
                temp_z |= 0xFFF00000;
            }
            else
            {
                temp_z &= 0x000FFFFF;               
            }

            this->accel_raw_x = temp_x * 0.0001;
            this->accel_raw_y = temp_y * 0.0001;
            this->accel_raw_z = temp_z * 0.0001;            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "accel_raw_x = %f, accel_raw_y = %f, accel_raw_z = %f", accel_raw_x, accel_raw_y, accel_raw_z);
        }
        else if(this->frame.can_id == this->sys_status_id)
        {
            this->state_ins = this->frame.data[0];
            this->num_sats_used = this->frame.data[1];
            this->status_gnss = this->frame.data[2];
            this->num_sats2_used = this->frame.data[3];
            this->age = ((this->frame.data[5] << 8u) | this->frame.data[4]) * 0.01;
            this->num_sats = this->frame.data[6];
            this->num_sats2 = this->frame.data[7];

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "state_ins = %u, num_sats_used = %u, status_gnss = %u, num_sats2_used = %u, age = %f, num_sats = %u, num_sats2 = %u", \
            this->state_ins, this->num_sats_used, this->status_gnss, this->num_sats2_used, this->age, this->num_sats, this->num_sats2);
        }    
        else if(this->frame.can_id == this->altitude_id)
        {
            this->pos_alt = ((this->frame.data[3] << 24u) | (this->frame.data[2] << 16u) | (this->frame.data[1] << 8u) | this->frame.data[0]) * 0.001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pos_alt = %f", this->pos_alt);
        }
        else if(this->frame.can_id == this->pos_sigma_id)
        {
            this->pos_e_sigma = (((this->frame.data[2] & 0x0F) << 16u) | (this->frame.data[1] << 8u) | this->frame.data[0]) * 0.0001;
            this->pos_n_sigma = ((this->frame.data[4] << 12u) | (this->frame.data[3] << 4u) | (this->frame.data[2] >> 4u)) * 0.0001;
            this->pos_u_sigma = (((this->frame.data[7] & 0x0F) << 16u) | (this->frame.data[6] << 8u) | this->frame.data[5]) * 0.0001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pos_e_sigma = %f, pos_n_sigma = %f, pos_u_sigma = %f", this->pos_e_sigma, this->pos_n_sigma, this->pos_u_sigma);
        }
        else if(this->frame.can_id == this->velocity_level_id)
        {
            this->vel_e = (int16_t)((this->frame.data[1] << 8u) | this->frame.data[0]) * 0.01;
            this->vel_n = (int16_t)((this->frame.data[3] << 8u) | this->frame.data[2]) * 0.01;
            this->vel_u = (int16_t)((this->frame.data[5] << 8u) | this->frame.data[4]) * 0.01;
            this->vel = (int16_t)((this->frame.data[7] << 8u) | this->frame.data[6]) * 0.01;         
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel_e = %f, vel_n = %f, vel_u = %f, vel = %f", this->vel_e, this->vel_n, this->vel_u, this->vel);
        }
        else if(this->frame.can_id == this->velocity_level_sigma_id)
        {
            this->vel_e_sigma = ((this->frame.data[1] << 8u) | this->frame.data[0]) * 0.001;
            this->vel_n_sigma = ((this->frame.data[3] << 8u) | this->frame.data[2]) * 0.001;
            this->vel_u_sigma = ((this->frame.data[5] << 8u) | this->frame.data[4]) * 0.001;
            this->vel_sigma = ((this->frame.data[7] << 8u) | this->frame.data[6]) * 0.001;         
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "vel_e_sigma = %f, vel_n_sigma = %f, vel_u_sigma = %f, vel_sigma = %f", this->vel_e_sigma, this->vel_n_sigma, this->vel_u_sigma, this->vel_sigma);
        }
        else if(this->frame.can_id == this->accel_vehicle_id)
        {
            int32_t temp_x = 0;
            int32_t temp_y = 0;
            int32_t temp_z = 0;

            temp_x = (this->frame.data[2] << 16u) | (this->frame.data[1] << 8u) | this->frame.data[0];
            temp_y = (this->frame.data[4] << 12u) | (this->frame.data[3] << 4u) | (this->frame.data[2] >> 4u);
            temp_z = (this->frame.data[7] << 16u) | (this->frame.data[6] << 8u) | this->frame.data[5];

            // X,Y,Z数据的最高位是符号位，且只有20位，因此用32位数据保存时需要使用符号位来填充前12位数据，以保证符号不会改变
            if((this->frame.data[2] & 0x08) != 0)
            {
                temp_x |= 0xFFF00000;
            }
            else
            {
                temp_x &= 0x000FFFFF;               
            }

            if((this->frame.data[4] & 0x80) != 0)
            {
                temp_y |= 0xFFF00000;
            }
            else
            {
                temp_y &= 0x000FFFFF;               
            }

            if((this->frame.data[7] & 0x08) != 0)
            {
                temp_z |= 0xFFF00000;
            }
            else
            {
                temp_z &= 0x000FFFFF;               
            }

            this->accel_x = temp_x * 0.0001;
            this->accel_y = temp_y * 0.0001;
            this->accel_z = temp_z * 0.0001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "accel_x = %f, accel_y = %f, accel_z = %f", accel_x, accel_y, accel_z);
        }
        else if(this->frame.can_id == this->heading_pitch_roll_id)
        {
            this->angle_yaw = ((this->frame.data[1] << 8u) | this->frame.data[0]) * 0.01;
            this->angle_pitch = (int16_t)((this->frame.data[3] << 8u) | this->frame.data[2]) * 0.01;
            this->angle_roll = (int16_t)((this->frame.data[5] << 8u) | this->frame.data[4]) * 0.01;
            this->angle_heading = ((this->frame.data[7] << 8u) | this->frame.data[6]) * 0.01;         
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angle_yaw = %f, angle_pitch = %f, angle_roll = %f, angle_heading = %f", this->angle_yaw, this->angle_pitch, this->angle_roll, this->angle_heading);
        }
        else if(this->frame.can_id == this->heading_pitch_roll_sigma_id)
        {
            this->angle_yaw_sigma = (((this->frame.data[2] & 0x0F) << 16u) | (this->frame.data[1] << 8u) | this->frame.data[0]) * 0.0001;
            this->angle_pitch_sigma = ((this->frame.data[4] << 12u) | (this->frame.data[3] << 4u) | (this->frame.data[2] >> 4u)) * 0.0001;
            this->angle_roll_sigma = (((this->frame.data[7] & 0x0F) << 16u) | (this->frame.data[6] << 8u) | this->frame.data[5]) * 0.0001;       
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angle_yaw_sigma = %f, angle_pitch_sigma = %f, angle_roll_sigma = %f", this->angle_yaw_sigma, this->angle_pitch_sigma, this->angle_roll_sigma);
        }
        else if(this->frame.can_id == this->angrate_vehicle_id)
        {
            int32_t temp_x = 0;
            int32_t temp_y = 0;
            int32_t temp_z = 0;

            temp_x = (this->frame.data[2] << 16u) | (this->frame.data[1] << 8u) | this->frame.data[0];
            temp_y = (this->frame.data[4] << 12u) | (this->frame.data[3] << 4u) | (this->frame.data[2] >> 4u);
            temp_z = (this->frame.data[7] << 16u) | (this->frame.data[6] << 8u) | this->frame.data[5];

            // X,Y,Z数据的最高位是符号位，且只有20位，因此用32位数据保存时需要使用符号位来填充前12位数据，以保证符号不会改变
            if((this->frame.data[2] & 0x08) != 0)
            {
                temp_x |= 0xFFF00000;
            }
            else
            {
                temp_x &= 0x000FFFFF;               
            }

            if((this->frame.data[4] & 0x80) != 0)
            {
                temp_y |= 0xFFF00000;
            }
            else
            {
                temp_y &= 0x000FFFFF;               
            }

            if((this->frame.data[7] & 0x08) != 0)
            {
                temp_z |= 0xFFF00000;
            }
            else
            {
                temp_z &= 0x000FFFFF;               
            }

            this->angRate_x = temp_x * 0.01;
            this->angRate_y = temp_y * 0.01;
            this->angRate_z = temp_z * 0.01;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "angRate_x = %f, angRate_y = %f, angRate_z = %f", angRate_x, angRate_y, angRate_z);
        }
        else if(this->frame.can_id == this->longitude_id)
        {
            this->pos_lon2 = (((int64_t)this->frame.data[7] << 56u) | ((int64_t)this->frame.data[6] << 48u) | ((int64_t)this->frame.data[5] << 40u) | ((int64_t)this->frame.data[4] << 32u) | ((int64_t)this->frame.data[3] << 24u) | ((int64_t)this->frame.data[2] << 16u) | ((int64_t)this->frame.data[1] << 8u) | (int64_t)this->frame.data[0]) * 0.00000001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pos_lon2 = %f", this->pos_lon2);
        }
        else if(this->frame.can_id == this->latitude_id)
        {
            this->pos_lat2 = (((int64_t)this->frame.data[7] << 56u) | ((int64_t)this->frame.data[6] << 48u) | ((int64_t)this->frame.data[5] << 40u) | ((int64_t)this->frame.data[4] << 32u) | ((int64_t)this->frame.data[3] << 24u) | ((int64_t)this->frame.data[2] << 16u) | ((int64_t)this->frame.data[1] << 8u) | (int64_t)this->frame.data[0]) * 0.00000001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pos_lat2 = %f", this->pos_lat2);
        }
        else if(this->frame.can_id == LATITUDE_LONGITUDE_ID)
        {
            this->pos_lat = ((this->frame.data[3] << 24u) | (this->frame.data[2] << 16u) | (this->frame.data[1] << 8u) | this->frame.data[0]) * 0.0000001;
            this->pos_lon = ((this->frame.data[7] << 24u) | (this->frame.data[6] << 16u) | (this->frame.data[5] << 8u) | this->frame.data[4]) * 0.0000001;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pos_lat = %f, pos_lon = %f", this->pos_lat, this->pos_lon);
        }
        else 
        {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "frame.can_id(%u) unkonw", this->frame.can_id);
        }

        return 0;
    }

    int disconnect(void)
    {
        close(socketfd);
        this->status = -1;

        return 0;
    }
};

#endif