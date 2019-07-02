/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#ifndef _RFANS_DRIVER_H_
#define _RFANS_DRIVER_H_
#include <ros/ros.h>
#include "ioapi.h"
#include <stdint.h>


namespace rfans_driver
{
class Rfans_Driver
{
public:
    Rfans_Driver(ros::NodeHandle node, ros::NodeHandle nh);
    ~Rfans_Driver();

    int spinOnce();
    int prog_Set(DEB_PROGRM_S &program);
    int datalevel_Set(DEB_PROGRM_S &program);

private:
    double calcReplayPacketRate();
    void configDeviceParams();
    void setupNodeParams(ros::NodeHandle node,ros::NodeHandle nh);

    bool worRealtime();
    int spinOnceRealtime();
    bool spinOnceSimu();

private:
    struct
    {
        std::string command_path;
        std::string advertise_path;
        std::string device_ip;
        std::string device_name;
        std::string simu_filepath;
        int dataport;
        int scnSpeed;
        int data_level;
        bool dual_echo;
    } config_;
    rfans_driver::IOAPI *m_devapi;
    ros::Publisher m_output;
    ros::ServiceServer server_ ;
    rfans_driver::RfansPacket tmpPacket;
    rfans_driver::InputPCAP *input_;
};

}

#endif //_RFANS_DRIVER_H_
