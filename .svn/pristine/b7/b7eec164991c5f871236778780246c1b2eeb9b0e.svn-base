/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */
#include <unistd.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>

#include "rfans_driver.h"
#include "rfans_driver/RfansCommand.h"
#include "rfans_driver/RfansScan.h"

namespace rfans_driver {

static const size_t packet_size = sizeof(rfans_driver::RfansPacket().data);
static const int RFANS_PACKET_NUM = 1024 ;
size_t packet_size_pcap = 1206;

static Rfans_Driver *s_this = NULL;


/** @brief Rfans Command Handle */
bool CommandHandle(rfans_driver::RfansCommand::Request  &req,
                   rfans_driver::RfansCommand::Response &res)
{
    res.status = 1;

    ROS_INFO("request: cmd= %d , speed = %d Hz", (int)req.cmd, (int)req.speed);
    ROS_INFO("sending back response: [%d]", (int)res.status);

    DEB_PROGRM_S tmpProg ;
    tmpProg.cmdstat = (DEB_CMD_E)req.cmd;
    tmpProg.dataFormat = eFormatCalcData;
    tmpProg.scnSpeed = req.speed;
    if(s_this) {
        s_this->prog_Set(tmpProg);
    }
    return true;
}

Rfans_Driver::Rfans_Driver(ros::NodeHandle node, ros::NodeHandle nh)
{
    setupNodeParams(node,nh);

    //std::string node_name = ros::this_node::getName();//useless
    server_ = node.advertiseService("rfans_driver/" + config_.command_path, CommandHandle);

    //driver init
    m_output = node.advertise<rfans_driver::RfansPacket>("rfans_driver/"+config_.advertise_path, RFANS_PACKET_NUM);
    double packet_rate = calcReplayPacketRate();
    if (config_.simu_filepath != "") {
        input_ = new rfans_driver::InputPCAP(nh, config_.dataport, packet_rate,
                        config_.simu_filepath, config_.device_ip);
    } else {
        m_devapi = new rfans_driver::IOSocketAPI(config_.device_ip, config_.dataport, config_.dataport);
        configDeviceParams();
    }

    s_this = this ;
}

Rfans_Driver::~Rfans_Driver()
{
    if(m_devapi) delete m_devapi;
    if(input_)  delete input_;
}

/** @brief Rfnas Driver Core */
int Rfans_Driver::spinOnce()
{
    if (worRealtime()) {
        spinOnceRealtime();
    } else {
        spinOnceSimu();
    }
}

int Rfans_Driver::spinOnceRealtime()
{
    int rtn = 0 ;

    m_devapi->revPacket(tmpPacket);

    rtn = m_devapi->getPacket(tmpPacket);
    if(rtn > 0) {
        m_output.publish(tmpPacket) ;
    }
    return rtn ;
}

bool Rfans_Driver::spinOnceSimu(void)
{
    rfans_driver::RfansScanPtr scan(new rfans_driver::RfansScan);
    rfans_driver::RfansPacket pkt;
    scan->packets.resize(32);
    for (int i = 0; i < 32; i++) {
        while (true) {
            int rc = input_->getPacket(&scan->packets[i]);
            if(rc ==0)break;
            if(rc <0) return false;
        }
    }

    ROS_DEBUG("Publishing a full Rfans scan");
    scan->header.stamp = scan->packets.back().stamp;
    scan->header.frame_id = "world";
    pkt.data.resize(scan->packets.size()*packet_size_pcap);
    for(int j=0; j<32; j++ )
    {
        memcpy(&pkt.data[j*packet_size_pcap],&(scan->packets[j].data[0]),packet_size_pcap);
    }
    pkt.stamp = scan->packets.back().stamp;
    pkt.udpCount = scan->packets.size();
    //pkt.udpSize = sizeof(rfans_driver::Packet().data);
    pkt.udpSize = packet_size_pcap;
    m_output.publish(pkt);
    memset(&pkt,0,sizeof(pkt));
    return true;
}

/** @brief control the device
     *  @param .parameters
     */
int Rfans_Driver::prog_Set(DEB_PROGRM_S &program)
{
    unsigned int tmpData = 0;

    switch (program.dataFormat) {
    case eFormatCalcData:
        tmpData |= CMD_CALC_DATA;
        break;
    case eFormatDebugData:
        tmpData |= CMD_DEBUG_DATA;
        break;
    }
    m_devapi->HW_WRREG(0, REG_DATA_TRANS, tmpData);
    //===============================================================
    tmpData = 0;
    switch (program.scnSpeed) {
    case ANGLE_SPEED_10HZ:
        tmpData |= CMD_SCAN_ENABLE;
        tmpData |= CMD_SCAN_SPEED_10HZ;
        break;
    case ANGLE_SPEED_20HZ:
        tmpData |= CMD_SCAN_ENABLE;
        tmpData |= CMD_SCAN_SPEED_20HZ;
        break;
    case ANGLE_SPEED_5HZ:
        tmpData |= CMD_SCAN_ENABLE;
        tmpData |= CMD_SCAN_SPEED_5HZ;
        break;
    default:
        tmpData |= CMD_SCAN_ENABLE;
        tmpData |= CMD_SCAN_SPEED_5HZ;
        break;
    }

    tmpData |= CMD_LASER_ENABLE;
    switch (program.cmdstat) {
    case eDevCmdWork:
        m_devapi->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
        break;
    case eDevCmdIdle:
        tmpData = CMD_RCV_CLOSE;
        m_devapi->HW_WRREG(0, REG_DEVICE_CTRL, tmpData);
        break;
    case eDevCmdAsk:
        break;
    default:
        break;
    }

    return 0;

}

int Rfans_Driver::datalevel_Set(DEB_PROGRM_S &program)
{
    unsigned int regData =0;
    switch (program.dataFormat) {
    case eFormatCalcData:
        regData |= CMD_CALC_DATA;
        break;
    case eFormatDebugData:
        regData |= CMD_DEBUG_DATA;
        break;
    }
    m_devapi->HW_WRREG(0, REG_DATA_TRANS, regData);
    regData =0;
    switch (program.dataLevel) {
    case LEVEL0_ECHO:
        regData = CMD_LEVEL0_ECHO;
        break;
    case LEVEL0_DUAL_ECHO:
        regData= CMD_LEVLE0_DUAL_ECHO;
        break;
    case LEVEL1_ECHO:
        regData = CMD_LEVEL1_ECHO;
        break;
    case LEVEL1_DUAL_ECHO:
        regData = CMD_LEVEL1_DUAL_ECHO;
        break;
    case LEVEL2_ECHO:
        regData = CMD_LEVEL2_ECHO;
        break;
    case LEVEL2_DUAL_ECHO:
        regData = CMD_LEVEL2_DUAL_ECHO;
        break;
    case LEVEL3_ECHO:
        regData = CMD_LEVEL3_ECHO;
        break;
       case LEVEL3_DUAL_ECHO:
        regData = CMD_LEVEL3_DUAL_ECHO;
        break;
    default:
        break;
    }

    switch (program.cmdstat) {
    case eDevCmdWork:
        m_devapi->HW_WRREG(0, REG_DATA_LEVEL, regData);
        break;
    case eDevCmdAsk:
        break;
    default:
        break;
    }

    return 0;

}

void Rfans_Driver::setupNodeParams(ros::NodeHandle node,ros::NodeHandle nh)
{
    node.param<std::string>("model",config_.device_name,"R-Fans-32");
    nh.param<std::string>("advertise_name", config_.advertise_path, "rfans_packets");
    nh.param<std::string>("control_name", config_.command_path, "rfans_control");
    nh.param<int>("device_port", config_.dataport, 2014);
    nh.param<std::string>("device_ip", config_.device_ip, "192.168.0.3");
    nh.param<int>("rps", config_.scnSpeed, 10);
    nh.param<std::string>("pcap", config_.simu_filepath, "");
//    nh.param<std::string>("model", config_.device_name, "R-Fans-32");
    nh.param<bool>("use_double_echo", config_.dual_echo, "false");
    nh.param<int>("data_level", config_.data_level, 0);
}

bool Rfans_Driver::worRealtime()
{
    return ((config_.simu_filepath == "")? true : false);
}

double Rfans_Driver::calcReplayPacketRate()
{
    double rate = 0.0f;
    std::string device = config_.device_name;
    int data_level = config_.data_level;
    bool dual_echo = config_.dual_echo;
    ROS_INFO("Device: %s",device.c_str());
    //one second generate 640k points,and each packet have 32*12 points.
    if (device == "R-Fans-32") {
        if (((data_level == 0) || (data_level == 1)) && dual_echo) {
            rate = 6666.67;
        } else if (((data_level == 0)|| (data_level == 1)) && (!dual_echo)) {
            rate = 3333.33;
        } else if (data_level == 2 && dual_echo) {
            rate = 4000;
        } else if (data_level == 2 &&(!dual_echo)) {
            rate = 2000;
        } else if (data_level == 3 && dual_echo) {
            rate = 3333.33;
        } else if (data_level == 3 &&(!dual_echo)) {
            rate = 1666.67;
        } else if (data_level == 4 && dual_echo) {//FIXME: should check GM & BK format
            rate = 3333.33;
        }
        else{
            rate = 1666.67;
        }
    } else if (device == "R-Fans-16") {
        if (((data_level == 0)|| (data_level == 1)) && dual_echo) {
            rate = 3333.33;
        } else if (((data_level == 0)|| (data_level == 1)) && (!dual_echo)) {
            rate = 1666.67;
        } else if (data_level == 2 && dual_echo) {
            rate = 2000;
        } else if (data_level == 2 && (!dual_echo)) {
            rate = 1000;
        } else if (data_level == 3 && dual_echo) {
            rate = 1666.67;
        } else if (data_level == 3 && (!dual_echo)) {
            rate = 833.3;
        } else if (data_level == 4) {
            rate = 833.3;
        }
    }else if(device == "R-Fans-V6K"){
        if (data_level == 2 && dual_echo) {
            rate = 2133.33;
        } else if (data_level == 2 && (!dual_echo)) {
            rate = 1066.67;
        } else if (data_level == 3 && dual_echo) {
            rate = 1562.5;
        } else if (data_level == 3 && (!dual_echo)) {
            rate = 781.25;
        }
    }
    else if (device == "C-Fans-128") {
        rate = 1666.67;
        // TODO:
    } else if (device == "C-Fans-32") {
        // TODO:
        rate = 416.67;
    } else {
        //rate = 1666.67;
        rate = 781.25;
    }

    return rate;
}

void Rfans_Driver::configDeviceParams()
{
    int data_level = config_.data_level;
    bool dual_echo = config_.dual_echo;

    DEB_PROGRM_S params;
    params.cmdstat = eDevCmdWork;
    params.dataFormat = eFormatCalcData;

    // set start rps
    params.scnSpeed =  config_.scnSpeed;
    prog_Set(params);

    if (data_level== 0 && !dual_echo) {
        params.dataLevel = LEVEL0_ECHO;
    } else if (data_level == 0 && dual_echo) {
        params.dataLevel = LEVEL0_DUAL_ECHO;
    } else if (data_level == 1 && !dual_echo) {
        params.dataLevel = LEVEL1_ECHO;
    } else if (data_level == 1 && dual_echo) {
        params.dataLevel = LEVEL1_DUAL_ECHO;
    } else if(data_level == 2 && !dual_echo) {
        params.dataLevel = LEVEL2_ECHO;
    } else if (data_level == 2 && dual_echo) {
        params.dataLevel = LEVEL2_DUAL_ECHO;
    } else if (data_level == 3 && !dual_echo) {
        params.dataLevel = LEVEL3_ECHO;
    } else {
        params.dataLevel = LEVEL3_DUAL_ECHO;
    }

    // set data level
    datalevel_Set(params);
}


} //rfans_driver namespace
