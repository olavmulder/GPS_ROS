#include <dirent.h>
#include "merge.h"
#include "gps.h"

#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>

void SavePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
void MainPointCloudProcessing(vision::visionMessage data);

bool firstGPSdata = true;
double meanK, mulTresh;
char *filename;

int main(int argc, char** argv){
    int opt;
    while ((opt = getopt(argc, argv, "nt:")) != -1) {
        switch (opt) {
            case 'm':
                meanK = std::stof(optarg);
                break;
            case 't':
                mulTresh = std::stof(optarg);
                break;
            case 'f':
                filename = optarg;


        }
    }
    ros::init(argc, argv, "pointCloudProcessing");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("pc_label", 1000, MainPointCloudProcessing);
    ros::spin();

    return 0;
}
void MainPointCloudProcessing(vision::visionMessage data){
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalPc(new pcl::PointCloud<pcl::PointXYZ>);
    vision::visionMessage lastMsg;
    if(firstGPSdata){
        firstGPSdata = false;
        sensor_msgs::PointCloud2 pc_temp = data.pc;
        pcl::fromROSMsg (pc_temp, *finalPc);
        lastMsg = data;
        return;
    }else{
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2 pc_temp = data.pc;
        pcl::fromROSMsg (pc_temp, *pc);
        pc = RemovePoints(pc, meanK, mulTresh);

        GPSMergeWithGPS(pc, finalPc, gpsdif(lastMsg.gpslon, lastMsg.gpslat, data.gpslon, data.gpslat),\
            meanK, mulTresh);
    }
    lastMsg = data;

    SavePointCloud(finalPc);
    

}

void SavePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pc){
    pcl::PCLPointCloud2 point_cloud2;
    pcl::toPCLPointCloud2(*pc, point_cloud2);
    pcl::PLYWriter ply_w;
    
    std::string ss = *filename + ".ply";
    
    ply_w.writeASCII(ss, point_cloud2);
}