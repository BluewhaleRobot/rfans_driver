/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include <ros/ros.h>
#include <stdio.h>
#include <time.h>

#include "rfans_driver/RfansCommand.h"
#include "ioapi.h"
#include "bufferDecode.h"

static rfans_driver::SSFileAPI s_fileAPI ;

static std::vector<SCDRFANS_BLOCK_S> outBlocks ;
static std::vector<RFANS_XYZ_S> outXyzBlocks ;
static int saveFile = 0 ;

static void RFansPacketReceived(rfans_driver::RfansPacket pkt) {

  SSBufferDec::Depacket(pkt, outBlocks) ;
  SSBufferDec::ConvertXyz(outBlocks, outXyzBlocks) ;
  if(saveFile)
    s_fileAPI.outputFile(outBlocks,outXyzBlocks) ;
}

static void RFansPointCloudReceived(sensor_msgs::PointCloud2 pointCloud) {
  if(saveFile)
    s_fileAPI.outputFile(pointCloud) ;
}

int main ( int argc , char ** argv ) 
{
  if(argc != 3 ) {
    std::cout<< "Usage: rfans_subscriber [cmd] [speed]\n"
             << "     Parameter:\n"
             << "       cmd,   0: stop RFans; 1:start RFans\n"
             << "     speed,   5: 5hz; 10: 10hz; 20: 20hz\n";

    return 0;
  }

  // Initialize the ROS system
  ros::init ( argc , argv , "rfans_subscribe") ;
  ros::NodeHandle nh ;

//  ros::Subscriber packetSub = nh.subscribe ("rfans_driver/rfans_packets" ,
//  1024, &RFansPacketReceived ) ;

//  ros::Subscriber pointSub = nh.subscribe ("rfans_driver/rfans_points" ,
//                                           1024, &RFansPointCloudReceived ) ;

  // Let ROS take over .
  ros::ServiceClient client = nh.serviceClient<rfans_driver::RfansCommand>("rfans_driver/rfans_control");
  rfans_driver::RfansCommand srv;
  srv.request.cmd = atoll(argv[1]);
  srv.request.speed = atoll(argv[2]);
  if (client.call(srv)) {
    ROS_INFO("status: %ld", (long int)srv.response.status);
  } else {
    ROS_ERROR("Failed to call service RfansCommand,Please");
  }
  ros::spin () ;
}
