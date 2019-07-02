/* -*- mode: C++ -*-
 *  All right reserved, Sure_star Coop.
 *  @Technic Support: <sdk@isurestar.com>
 *  $Id$
 */

#include <ros/ros.h>
#include "rfans_driver.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rfans_driver");
    ros::NodeHandle node;
    ros::NodeHandle nh("~");
    rfans_driver::Rfans_Driver* driver = new rfans_driver::Rfans_Driver(node, nh);

    while( (ros::ok()) && (driver->spinOnce()))
    {
        ros::spinOnce();
    }

    return 0;
}
