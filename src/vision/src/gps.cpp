#include <ros/ros.h> 
#include <pcl_ros/point_cloud.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <vision/visionMessage.h>

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <dirent.h>
#include <cmath>
#include <string>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#define PI 3.14159265359
int counter;
double lastLon = 0, lastLat = 0, lastHeight = 0;
double lastTime = 0;
double totalDistance;
/*
gps latitute, lonitute & height
for each point = row in pcd file,  in pointcloud add difference in gps data

*/
/*Get GPS data*/

/*Get pointcloud*/

ros::Publisher gps_pub;
char* input;//distance
char* input2;//duration
char* input3;
char* enableLonLatData;

double GetTime(int sec, int nsec){
    std::stringstream s;
    int amount = log10(nsec)+1;
    s << sec;
    s << ".";
    while(amount < 9){
        s << 0;
        amount++;
    }
    s << nsec;
    double duration = std::stod(s.str());
    
    return(std::stod(s.str()));
}

void GeteringData(vision::visionMessage data){
       
    int sec = data.sec;
    int nsec = data.nsec;
    //ROS_INFO("%d sec, %d nsec", sec, nsec);
    double duration;
    double time = GetTime(sec, nsec);
    if(time < lastTime){
        ROS_INFO("time < lastTime .....\ntime %lf...lasttime %lf...", time, lastTime);
    }else{
        duration = time - lastTime;
    }
        

    //pcl::fromROSMsg (pc, *newCapture);
    
    double r = 6378.137;
    double diffLon, diffLat, diffHeight;
    double lon  = data.gpslon, lat = data.gpslat, height = data.gpsheight;
    
    FILE *distanceFile, *latFile, *lonFile;
    double distance = 0;
    if(counter != 0){

       
        
        diffLon = (lon * PI / 180) - (lastLon * PI / 180);
        diffLat = (lat * PI / 180) - (lastLat * PI / 180);

        double a = sin(diffLat/2) * sin(diffLat/2) + cos(lat * PI / 180) * cos(lastLat * PI / 180) * sin(diffLon/2) * sin(diffLon/2);
        double c = 2 * atan2(sqrt(a), sqrt(1-a));
        ROS_INFO("%lf %lf", a,c);
        distance = r * c * 1000;
        ROS_INFO("duration: %lf\nlat: %.10lf\nlon: %.10lf\nheight: %.10lf\ndistance = %.5lf", duration, lat, lon, height, distance);
        
        distanceFile = fopen(input,"a");
        latFile = fopen(input2, "a");
        lonFile = fopen(input3, "a");
        
        if(distanceFile == NULL){
            perror("Error opening file");
        }
        else{
            fprintf(distanceFile, "%f\n", distance);
            fprintf(latFile, "%f\n", lat);
            fprintf(lonFile, "%f\n", lon);
        }
        fclose(distanceFile);
        fclose(latFile);
        fclose(lonFile);

    }else{
        distanceFile = fopen(input,"w");
        latFile = fopen(input2, "w");
        lonFile = fopen(input3, "w");
        
        if(distanceFile == NULL){
            perror("Error opening file");
        }
        else{
            fprintf(distanceFile, "%lf\n", distance);
            fprintf(latFile, "%f\n", lat);
            fprintf(lonFile, "%f\n", lon);

        }
        fclose(distanceFile);
        fclose(latFile);
        fclose(lonFile);
        

    }
    lastLon = lon;
    lastLat = lat;
    lastHeight = height;
    lastTime = time;
    counter++;
   
}
int main (int argc, char** argv) {

  // Initialize ROS
    ros::init(argc, argv, "pointCloudProcessing");
    ROS_INFO("arg1 = distance file, arg2 = lat file, arg3 = lon file");
    input = argv[1];
    input2 = argv[2];//lat
    input3 = argv[3];//lon
    ros::NodeHandle n;
    ros::Subscriber pcProcessing = n.subscribe("pc_label", 1000, GeteringData);
    //gps_pub = n.advertise<vision::visionMessage>("gpsProcessing", 1000);
    
    ros::spin();
    return 0;
}
 