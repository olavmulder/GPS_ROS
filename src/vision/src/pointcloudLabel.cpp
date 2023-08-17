#include <ros/ros.h> 
#include <pcl_ros/point_cloud.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <sensor_msgs/NavSatFix.h> 
#include <vision/visionMessage.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h> 
#include <dirent.h>
//#include <ublox_msgs/NavPVT.h>

vision::visionMessage last_msg;
ros::Publisher pub;

int neuEnable;


void NeuvtitionCallback(const sensor_msgs::PointCloud2ConstPtr& data){
    ROS_INFO("neuvition received");
    last_msg.pc = *data;
    if(neuEnable){
        pub.publish(last_msg);
    }
}
/*void GpsCallback(gps_data::gps data){
    last_msg = data;
    ros::Time time = ros::Time::now();
    last_msg.sec = time.sec;
    last_msg.nsec = time.nsec;
    pub.publish(last_msg);
    ROS_INFO("gps received");
}*/

//void UbloxGPSCallback(ublox_msgs::NavPVT data){
void UbloxGPSCallback(sensor_msgs::NavSatFix data){
    last_msg.gpslat = data.latitude;///10000000.0f;
    last_msg.gpslon = data.longitude;///10000000.0f;
    last_msg.gpsheight = data.altitude;///1000.0f;//mm -> m 
    ROS_INFO("%.10f %.10f", last_msg.gpslat, last_msg.gpslon);
    ros::Time time = ros::Time::now();
    last_msg.sec = time.sec;
    last_msg.nsec = time.nsec;
    if(!neuEnable){
        pub.publish(last_msg);
    }
    ROS_INFO("gps ublox received");
}
int main (int argc, char** argv) {

  // Initialize ROS
    neuEnable = atoi(argv[2]);
    ROS_INFO("start");
    ROS_INFO("ARG 1 = gps topic name to subscribe to, ARG 2 = neuvition enable");
    ros::init(argc, argv, "PointCloud_GPS_Label");
    ros::NodeHandle n;
    
    ros::Subscriber subNeuvition = n.subscribe("neuvition_cloud", 1000, NeuvtitionCallback);
    //ros::Subscriber subGPS = n.subscribe("gps_data", 1000, GpsCallback);
    ros::Subscriber subGPS = n.subscribe(argv[1], 1000, UbloxGPSCallback);
    
    pub = n.advertise<vision::visionMessage>("pc_label", 1000);
    
    ros::spin();
    return 0;
} 