#include <ros/ros.h> 
#include <pcl_ros/point_cloud.h> 
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h> 
//Header file for writing PCD file 
#include <pcl/io/pcd_io.h> 
#include <iostream>
#include <ctime>
#include <chrono>
#include <string.h>
float depthTreshold;
void cloudCB(const sensor_msgs::PointCloud2 input) 
{ 
    pcl::PointCloud<pcl::PointXYZ>::Ptr newCapture(new pcl::PointCloud<pcl::PointXYZ>); 
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalPoint(new pcl::PointCloud<pcl::PointXYZ>);

    //float depthTreshold = 1.5;
    float threshold2 = depthTreshold*depthTreshold;

    //sensor_msgs::PointCloud2 pc = input;
    pcl::fromROSMsg (input, *newCapture);
    
    /*for(int p = 0;p<newCapture->points.size(); ++p){

        float pointDepth2 = ((newCapture->points[p].x * newCapture->points[p].x) + \
            (newCapture->points[p].y * newCapture->points[p].y) + \
            (newCapture->points[p].z * newCapture->points[p].z));
        if(pointDepth2 > threshold2){
            newCapture->points[p] = newCapture->points[newCapture->points.size()-1];
            newCapture->points.resize(newCapture->points.size()-1);
            --p;
        }
    }*/
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (newCapture);
    sor.filter (*newCapture);
    
    char folder[] = "./";
    ros::Time stamp = ros::Time::now();

    int stampLength = snprintf(NULL, 0, "%d", stamp.sec);
    int stampnLength = snprintf(NULL, 0, "%d", stamp.nsec);
    int standaardLength = 5+(strlen(folder));

    char str[stampLength];
    char str2[stampnLength];
    char buf[stampLength + stampnLength+standaardLength+1];
    for(uint8_t i=0;i< sizeof(buf); i++){
        buf[i] = NULL;
    }

    snprintf(str, stampLength, "%d", stamp.sec);
    snprintf(str2, stampnLength, "%d", stamp.nsec);
    strcat(buf, folder);
    strcat(buf, str);
    strcat(buf, ".");
    strcat(buf, str2);
    strcat(buf, ".pcd");

    ROS_INFO("%s",buf);
    finalPoint->points.resize(newCapture->points.size());
    *finalPoint = *newCapture;
    
    pcl::io::savePCDFileASCII (buf, *finalPoint); 
} 
int main (int argc, char **argv) 
{ 
    ros::init (argc, argv, "pointcloud2_to_PCD"); 
    //depthTreshold = std::stof(argv[1]);
    ROS_INFO("Started PCL write node"); 
    
    ros::NodeHandle nh; 
    ros::Subscriber bat_sub = nh.subscribe("neuvition_cloud", 1000, cloudCB); 
    //ros::Subscriber 
    ros::spin(); 

    return 0; 
}
/*
rospy.Subscriber('neuvition_cloud', PointCloud2, ReadPointCloud)
    f = open(str(rospy.get_time())+".txt", "x")
    f.write(str(vision.gpsWidth)+","+str(vision.gpsLength)+","+str(vision.gpsHeight))
def ReadPointCloud(data):
    #set pointcloud to pcd
    pcd = o3d.io.read_point_cloud(data)
    name = "/home/olav/pcdFromRos/"+rospy.get_time()+".pcd"
    o3d.io.write_point_cloud(name, pcd)
    rospy.loginfo(rospy.get_caller_id() + "PointCloud2.header: %s", data.header)
    rospy.loginfo(rospy.get_caller_id() + "PointCloud2.fields: %s", data.fields)
*/