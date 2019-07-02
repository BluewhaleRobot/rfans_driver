
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <ctime>

FILE *g_dump_file;

void processPoints(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    for (sensor_msgs::PointCloud2ConstIterator<float> it(*msg, "x"); it != it.end(); ++it) {
        const float x = it[0]; // x
        const float y = it[1]; // y
        const float z = it[2]; // z
        const float i = it[3]; // intensity
        const uint32_t r = *((const uint32_t*)(&it[4])); // laserid
        const double t = *((const double*)(&it[5])); // timeflag
        const float a = it[7]; // hangle
        fprintf(g_dump_file, "%u, %f, %f, %f, %f, %f, %f \r\n", r, a, x, y, z, i, t);
        fflush(g_dump_file);
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "dump_point_node");//check
    ros::NodeHandle node;
    ros::NodeHandle nh("~");

    time_t now; struct tm *time_now;
    time(&now);
    time_now = localtime(&now);
    int yy = time_now->tm_year % 100;
    int mm = time_now->tm_mon + 1;
    int dd = time_now->tm_mday;
    int hh = time_now->tm_hour;
    int mn = time_now->tm_min;
    int ss = time_now->tm_sec;
    char filename[FILENAME_MAX] = { '\0' };
    sprintf(filename, "dump-points-%02d%02d%02d-%02d%02d%02d.txt", yy, mm, dd, hh, mn, ss);

    g_dump_file = fopen(filename, "w+");
    fprintf(g_dump_file,"id, h_angle, x, y, z, intensity, timeflag \r\n");
    fflush(g_dump_file);

    ros::Subscriber sub = node.subscribe("/rfans_driver/rfans_points", 1024, &processPoints);
    ros::spin();

    return 0;
}
