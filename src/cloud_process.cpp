#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "ssFrameLib.h"
const int SIZE_RFANS_DATA = sizeof(RFANS_XYZ_S);
ros::Publisher pub;
sensor_msgs::PointCloud2 msg_pub;
void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    msg_pub.header = msg->header;
    msg_pub.width = msg->width;
    msg_pub.height = msg->height;

    msg_pub.point_step = 20;
    int data_size = msg_pub.width * msg_pub.point_step;
    msg_pub.data.resize(data_size);
    msg_pub.row_step = msg_pub.data.size();
    for(size_t i=0; i< msg_pub.width; i++)
    {
        memcpy(&msg_pub.data[i*msg_pub.point_step], &msg->data[i*SIZE_RFANS_DATA],msg_pub.point_step);
    }
    pub.publish(msg_pub);

//    int offset_x = -1;
//    int offset_y = -1;
//    int offset_z = -1;
//    int offset_i = -1;
//    int offset_r = -1;
//    for(size_t i=0; i<msg->fields.size();i++)
//    {
//        if (msg->fields[i].datatype == sensor_msgs::PointField::FLOAT32) {
//            if (msg->fields[i].name == "x") {
//                offset_x = msg->fields[i].offset;
//            } else if (msg->fields[i].name == "y") {
//                offset_y = msg->fields[i].offset;
//            } else if (msg->fields[i].name == "z") {
//                offset_z = msg->fields[i].offset;
//            } else if (msg->fields[i].name == "intensity") {
//                offset_i = msg->fields[i].offset;
//            }
//        } else if (msg->fields[i].datatype == sensor_msgs::PointField::INT32) {
//            if (msg->fields[i].name == "laserid") {
//                offset_r = msg->fields[i].offset;
//            }
//        }
//    }

//    if((offset_x >= 0) && (offset_y >= 0) && (offset_z >=0) && (offset_i >= 0) && (offset_r >= 0))
//    {
//       msg_pub.data.resize(msg->data.size());
//       msg_pub.fields.resize(5);
//       for(sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it)
//       {
//           const int r = *((const int*)(&it[4]));
//           const float x = it[0];
//           const float y = it[1];
//           const float z = it[2];
//           const float intensity = it[3];


//       }
//    }



}

void InitPointcloud2(sensor_msgs::PointCloud2 &initCloud) {
    static const size_t DataSize = sizeof(rfans_driver::RfansPacket().data) / sizeof(SCDRFANS_BLOCK_S ) * sizeof(RFANS_XYZ_S) *RFANS_LASER_COUNT;
    initCloud.data.clear();
    initCloud.data.resize( DataSize); //point data

    initCloud.is_bigendian = false ;//false;      //stream foramt
    initCloud.fields.resize(5);          //line format
    initCloud.is_dense = false;

    int tmpOffset = 0 ;
    for(int i=0; i < initCloud.fields.size() ;i++) {
        switch(i) { //value type
        case 0:
            initCloud.fields[i].name = "x" ;
            initCloud.fields[i].datatype = 7u;
            break;
        case 1:
            initCloud.fields[i].name = "y" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 2:
            initCloud.fields[i].name = "z" ;
            initCloud.fields[i].datatype = 7u;
            tmpOffset += 4;
            break;
        case 3:
            initCloud.fields[i].name = "intensity" ;
            initCloud.fields[i].datatype = 7u;//2u;
            tmpOffset += 4;
            break;
        case 4:
            initCloud.fields[i].name = "ring" ;
            initCloud.fields[i].datatype = 5u;
            tmpOffset += 4;
            break;
        }
        initCloud.fields[i].offset = tmpOffset ;      //value offset
        initCloud.fields[i].count = 1 ;
    }
    initCloud.height = 1;
    initCloud.point_step = 20;
    initCloud.row_step = DataSize ;
    initCloud.width = 0 ;

}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"cloud_process_node");
    ros::NodeHandle nh;
    InitPointcloud2(msg_pub);
    ros::Subscriber sub = nh.subscribe("/rfans_driver/rfans_points",1024,&cloudCallback);
    pub = nh.advertise<sensor_msgs::PointCloud2>("rfans_points",10);
    ros::spin();
    return 0;
}
