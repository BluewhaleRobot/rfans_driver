#ifndef _BUFFER_DECODER_H_
#define _BUFFER_DECODER_H_
//#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include "rfans_driver/RfansPacket.h"
#include "ssFrameLib.h"
#include <ros/ros.h>
#include "calculation.h"
typedef enum {
    eReady,
    eSearch,
    eReadPackData,
    eReadFinish
}DEC_STSTUS_E ;

typedef enum {
    DEVICE_TYPE_NONE,
    DEVICE_TYPE_RFANS,
    DEVICE_TYPE_CFANS,
} DEVICE_TYPE_E;


typedef struct {
    double theta[32];
    double beta[32];
    double odd_even_angle[4];
    double mirror_angle[4];
    double RangeCorr[32];
    double intensityCoeff[6];
    double IntensiytCorr[255];
    double planeAngle[4];
    double planeNormal1[3];
    double planeNormal2[3];
    double planeNormal3[3];
    double planeNormal4[3];
    double convergePoint1[3];
    double convergePoint2[3];
    double planeFixPoint1[3];
    double planeFixPoint2[3];
    double planeFixPoint3[3];
    double planeFixPoint4[3];
}Calib_para;

class SSBufferDec {
public:
    SSBufferDec();
    ~SSBufferDec();
    int moveWriteIndex(int setpIndex);
    int moveReadIndex(int setpIndex);
    unsigned char * getWriteIndex();
    unsigned char * getReadIndex();

    int writeBuffer(unsigned char *data, int size);

    int readPacket(rfans_driver::RfansPacket &pkt);
    //int readPacketStream(rfans_driver::RfansPacket &pkt);
    int size();
    int freeSize();
    void reset();

    int getUdpCount() { return m_udpCount;}
    int getUdpSize() {return m_udpSize;}

    static int Depacket(rfans_driver::RfansPacket &inPack, sensor_msgs::PointCloud2 &outCloud , ros::Publisher &rosOut, DEVICE_TYPE_E deviceType);
    static void InitPointcloud2(sensor_msgs::PointCloud2 &initCloud) ;
    static void ResetPointCloud2(sensor_msgs::PointCloud2 &initCloud) ;

    static void SetAngleDuration(float value);
    static void setSaveXYZ(bool save);
    //  static int initCFansPara(std::string reviseAngle);
private:  //int readPacket(std::vector<SCDRFANS_BLOCK_S> &outBlocks);
    int readBuffer() ;
    int readBufferSteam() ;
    int ouputPacket(rfans_driver::RfansPacket &pkt);
    void bufferReverse();
private:
    UDP_DECBUFFER_S m_decBuf;
    DEC_STSTUS_E m_status ;
    rfans_driver::RfansPacket m_packet ;
    int m_packetSize ;
    int m_blockCout ;
    float s_preAngle ;
    int m_udpSize;
    int m_udpCount;
};


#endif
