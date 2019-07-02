 /* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <stdio.h>
#include <time.h>
#include <iostream>
#include <string>
#include "rfans_driver/RfansCommand.h"
#include "bufferDecode.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <pthread.h>
#include <dynamic_reconfigure/server.h>
#include "rfans_driver/FilterParamsConfig.h"
//#include <pcl_conversions/pcl_conversions.h>


static const int RFANS_POINT_CLOUD_NUM = 1024 ;

static std::vector<SCDRFANS_BLOCK_S> outBlocks ;
static std::vector<RFANS_XYZ_S> outXyzBlocks ;
static sensor_msgs::PointCloud2 outCloud ;

static ros::Publisher  s_output;
static ros::Subscriber s_sub ;
static int scanSpeed;
extern int gs_pointsPerRound;
extern bool use_gps;
//extern int data_level_;
extern int year;
extern int month;
extern int day;
extern int hour;
extern double min_range;
extern double max_range;
extern double min_angle;
extern double max_angle;
extern int ringID;
extern bool use_laserSelection_;
typedef int ( *PFUNC_THREAD)(void *);
//pcl::PointCloud<pcl::PointXYZI> cloud;
static DEVICE_TYPE_E s_deviceType = DEVICE_TYPE_NONE;
typedef struct{
    unsigned int pkgflag;
    unsigned int pkgnumber;
    unsigned int date;
    unsigned short time;
    unsigned int maca;
    unsigned short macb;
    unsigned short dataport;
    unsigned short msgport;
    unsigned char motorspd;
    unsigned int deviceType;
    unsigned short phaseAngle;
    unsigned char padding[225];
}RFANS_HEARTBEAT_S;
static tm gs_lidar_time;

static  pthread_t ssCreateThread(int pri, void * obj, PFUNC_THREAD fnth) {
    pthread_t thrd_ ;
    pthread_create(&thrd_, NULL, (void *(*)(void*))fnth, (void *)obj);
    return thrd_ ;
}
static int heartbeat_thread_run(void *para) {
    printf("heart beat thread start \n");
    // create socket, then read broadcast port2030 package
    int client_fd;
    int rtn;
    struct sockaddr_in ser_addr;
    bool optval = true;
    const int opt = -1;
    client_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (client_fd < 0) {
        printf("create socket failed!\n");
        return -1;
    }

    // memset(&ser_addr, 0, sizeof(ser_addr));
    bzero(&ser_addr,sizeof(struct sockaddr_in));
    ser_addr.sin_family = AF_INET;
    ser_addr.sin_port = htons(2030);
    ser_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    rtn =setsockopt(client_fd,SOL_SOCKET,SO_REUSEPORT,(char*)&opt,sizeof(opt));
    if(rtn<0)
    {
        printf("setsockopt failed! \n");
    }
    if (bind(client_fd,(struct sockaddr*)&ser_addr, sizeof(sockaddr_in)) < 0) {
        printf("bind server failed!\n");
        return -1;
    }
    socklen_t len;
    struct sockaddr_in src;
    printf("after heartbeat init \n");
    while(1) {
        unsigned char buff[512] = {'\0'};
        int rcv = recvfrom(client_fd, buff, sizeof(buff), 0, (struct sockaddr*)&ser_addr, &len);
        if (rcv > 0) {
            RFANS_HEARTBEAT_S hb;
            memset(&hb, '\0', sizeof(RFANS_HEARTBEAT_S));
            memcpy(&hb, buff, sizeof(RFANS_HEARTBEAT_S));
            swapchar((unsigned char*)&hb.pkgflag, sizeof(hb.pkgflag));
            swapchar((unsigned char*)&hb.pkgnumber, sizeof(hb.pkgnumber));
            swapchar((unsigned char*)&hb.date, sizeof(hb.date));
            swapchar((unsigned char*)&hb.time, sizeof(hb.time));

            if (hb.pkgflag == 0xe1e2e3e4) {//heartbeat flag
                gs_lidar_time.tm_year = ((hb.date& 0xFF000000)>>24)+2000;;
                gs_lidar_time.tm_mon =((hb.date & 0xFF0000) >> 16);
                gs_lidar_time.tm_mday = ((hb.date & 0xFF00) >> 8);;
                gs_lidar_time.tm_hour = (hb.date & 0xFF);
                year = gs_lidar_time.tm_year;
                month = gs_lidar_time.tm_mon;
                day = gs_lidar_time.tm_mday;
                hour = gs_lidar_time.tm_hour;
//                ROS_INFO(" Dong %d,%d,%d,%d",year,month,day,hour);

//                printf("flag:0x%08x, num:0x%08x, date:0x%08x, time:0x%08x\n",
//                        hb.pkgflag,hb.pkgnumber, hb.date, hb.time);
            }
        }
        usleep(500000);//500ms;
    }

    close(client_fd);

    return 0;
}

static void RFansPacketReceived(rfans_driver::RfansPacket pkt) {
  int rtn = 0 ;
  rtn =  SSBufferDec::Depacket(pkt, outCloud,s_output, s_deviceType) ;
  return ;
}

void callback(rfans_driver::FilterParamsConfig &config, uint32_t level){
    min_range = config.min_range;
    max_range = config.max_range;
    min_angle = config.min_angle;
    max_angle = config.max_angle;
    use_laserSelection_ = config.use_laserSelection;
    ringID = config.laserID;
}


int main ( int argc , char ** argv )
{
  // Initialize the ROS system

    ros::init ( argc , argv , "calculation_node") ;
    ros::NodeHandle nh ;
    ros::NodeHandle priv_nh("~");
    dynamic_reconfigure::Server<rfans_driver::FilterParamsConfig> server;
    dynamic_reconfigure::Server<rfans_driver::FilterParamsConfig>::CallbackType f;
    f = boost::bind(&callback,_1,_2);
    server.setCallback(f);
    bool use_gps_;
    bool use_double_echo_ = false;
    SSBufferDec::InitPointcloud2(outCloud) ;

    //node name
    std::string node_name = ros::this_node::getName();
    priv_nh.param("use_gps",use_gps_,false);
//    priv_nh.param<int>("data_level",data_level_,1);
//    priv_nh.param<double>("min_range",min_range,0.0);
//    priv_nh.param<double>("max_range",max_range,180.0);
//	priv_nh.param<double>("min_angle",min_angle,0.0);
//    priv_nh.param<double>("max_angle",max_angle,360.0);
//    priv_nh.param<int>("laserID",ringID,0);
//    priv_nh.param<bool>("use_laserSelection",use_laserSelection_,false);
    use_gps = use_gps_;

  //advertise name
  std::string advertise_name = "rfans_points";
  std::string advertise_path = node_name + "/advertise_name";
  ros::param::get(advertise_path,advertise_name);
  advertise_path = "rfans_driver/" + advertise_name;
  //ROS_INFO("%s : advertise name %s : %s",node_name.c_str(), advertise_name.c_str(), advertise_path.c_str() );

    //subscribe name
    std::string subscribe_name = "rfans_packets";
    std::string subscribe_path = node_name + "/subscribe_name";
    ros::param::get(subscribe_path, subscribe_name);
    subscribe_path = "rfans_driver/" + subscribe_name;
    ROS_INFO("%s : subscribe name %s : %s",node_name.c_str(), subscribe_name.c_str(), subscribe_path.c_str() );
    pthread_t s_heartbeat_worker_id = ssCreateThread(1, NULL, heartbeat_thread_run) ;
    //angle durantion
    double angle_duration = 360;
	priv_nh.param<double>("angle_duration",angle_duration,360.0);
    SSBufferDec::SetAngleDuration(angle_duration);
    ROS_INFO("%s : angle_duration : %f",node_name.c_str(), angle_duration);

    bool ok= ros::param::get("/rfans_driver/rps",scanSpeed);
    bool ok1 = ros::param::get("/rfans_driver/use_double_echo",use_double_echo_);

    if(ok && ok1){
        if(scanSpeed ==5)
        {
            if(use_double_echo_) {
                gs_pointsPerRound = 8000;
            }
            else {
                gs_pointsPerRound = 4000;
            }
        }else if(scanSpeed == 10)
        {
            if (use_double_echo_) {
                gs_pointsPerRound = 4000;
            }
            else {
                gs_pointsPerRound = 2000;
            }
        }
        else
        {
            if(use_double_echo_) {
                gs_pointsPerRound = 2000;
            }
            else {
                gs_pointsPerRound = 1000;
            }
        }
    }

    // device type
    std::string device_model;
    ros::param::get("model",device_model);
    ROS_INFO("device_model_value: %s", device_model.c_str());
    if (device_model == "C-Fans-128")
    {
        s_deviceType = DEVICE_TYPE_CFANS;
        std::string revise_angle_key = node_name + "/revise_angle_128";
        std::string revise_angle_value = "0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0,";//default
        ros::param::get(revise_angle_key, revise_angle_value);
        initCFansPara(revise_angle_value);
    }
    else if(device_model == "C-Fans-32")
    {
        s_deviceType = DEVICE_TYPE_CFANS;
        std::string revise_angle_key = node_name + "/revise_angle_32";
        std::string revise_angle_value = "0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0,";//default
        ros::param::get(revise_angle_key, revise_angle_value);
        initCFans_32(revise_angle_value);
    }
    else
    {
        s_deviceType = DEVICE_TYPE_RFANS;
    }
//    std::string device_type_key = node_name + "/device_type";
//    std::string device_type_value = "rfans";//default rfans
//    ros::param::get(device_type_key, device_type_value);
//    ROS_INFO("device_type_key: %s", device_type_key.c_str());
//    ROS_INFO("device_type_value: %s", device_type_value.c_str());
//    const char *device_type_cfans = "cfans";
//    const char *device_type_rfans = "rfans";
    //    if (0 == strcmp(device_type_value.c_str(), device_type_rfans)) {
    //      s_deviceType = DEVICE_TYPE_RFANS;
    //    } else if (0 == strcmp(device_type_value.c_str(), device_type_cfans)) {
    //      ROS_INFO("device type: cfans");
    //      s_deviceType = DEVICE_TYPE_CFANS;

    //      std::string revise_angle_key = node_name + "/revise_angle";
    //      std::string revise_angle_value = "0, 0, 0, 0, 0, 0, 0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,0, 0, 0,45,-15,45,-15,0,0,";//default
    //      ros::param::get(revise_angle_key, revise_angle_value);
    //      initCFansPara(revise_angle_value);
    //    }

    // save xyz
    std::string save_xyz_key = node_name + "/save_xyz";
    std::string save_xyz_value = "no";//default not save
    ros::param::get(save_xyz_key, save_xyz_value);
    if (0 == strcmp(save_xyz_value.c_str(), "yes")) {
        SSBufferDec::setSaveXYZ(true);
    } else {
        SSBufferDec::setSaveXYZ(false);
    }


  s_sub= nh.subscribe (subscribe_path , 30, &RFansPacketReceived ) ;
  s_output = nh.advertise<sensor_msgs::PointCloud2>(advertise_path, RFANS_POINT_CLOUD_NUM);
  ros::spin () ;

  return  0;
}
