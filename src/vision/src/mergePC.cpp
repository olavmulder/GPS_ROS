#include <ros/ros.h> 
#include <pcl/io/io.h>
#include <pcl_ros/point_cloud.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <vision/visionMessage.h>
#include <string>
//#include <pcl/filter/passthrough.h>
#include <iostream>
#include <pcl/filters/voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <dirent.h>
#include <math.h>
#include <unistd.h>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <chrono>

#define PI 3.14159265359
pcl::PointCloud<pcl::PointXYZ>::Ptr finalPoint(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
int counter = 0;
double totalDistance;
double lastLon, lastLat;

double GPS(double lon, double lat, double lastLon, double lastLat){ 
    const double r = 6378.137;
    double diffLon, diffLat;
    diffLon = (lon * PI / 180) - (lastLon * PI / 180);
    diffLat = (lat * PI / 180) - (lastLat * PI / 180);
    double a = sin(diffLat/2) * sin(diffLat/2) + cos(lat * PI / 180) * cos(lastLat * PI / 180) * sin(diffLon/2) * sin(diffLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return(r * c * 1000);

}

void icpFiltering(vision::visionMessage data){

    //gps
    double distance;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 pc_temp = data.pc;
    pcl::fromROSMsg (pc_temp, *pc);


    
    
    //x(height filter/ground filter)
    for(int p = 0;p<pc->points.size(); ++p){
        if(pc->points[p].x < -1.20 ){
            pc->points[p] = pc->points[pc->points.size()-1];
            pc->points.resize(pc->points.size()-1);
            --p;
        }
    }
    float depthTreshold = 5.0;
    float threshold2 = depthTreshold*depthTreshold;

    for(int p = 0;p<pc->points.size(); ++p){
        float pointDepth2 = ((pc->points[p].x * pc->points[p].x) + \
            (pc->points[p].y * pc->points[p].y) + \
            (pc->points[p].z * pc->points[p].z));
        if(pointDepth2 > threshold2){
            pc->points[p] = pc->points[pc->points.size()-1];
            pc->points.resize(pc->points.size()-1);
            --p;
        }
    }
    pcl :: VoxelGrid<pcl::PointXYZ> vox;
    vox. setInputCloud (pc) ;
    vox. setLeafSize (0.015 f , 0.015f , 0.015f ) ; // 1.5cm
    vox. filter (*pc) ;
    pc->width = pc->points.size();

    /*Point cloud outlier removal */
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(pc);
    outrem.setRadiusSearch(0.5);
    outrem.setMinNeighborsInRadius(1);
    outrem.setKeepOrganized(true);
    outrem.filter(*pc);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (pc);
    sor.setMeanK(1); //(meanK);
    sor.setStddevMulThresh(0.005);//(mulTresh);
    sor.setNegative(false);
    sor.filter(*pc);

    pc->width = pc->points.size();
    pcl::PCLPointCloud2 point_cloud2;
	pcl::toPCLPointCloud2(*pc, point_cloud2);
	pcl::PLYWriter ply_w;
	ply_w.writeASCII("1.ply", point_cloud2);
    ROS_INFO("saved %d", counter);
    
    while(1);
    pc->width = pc->points.size();

    //icpif
    if(counter == 0){
        *merged = *pc;
        *finalPoint= *merged;
    }else{
        distance = GPS(data.gpslon, data.gpslat, lastLon, lastLat);
        
        ROS_INFO("distance %lf\ntotal distance %lf", distance, totalDistance);
        if(counter % 10 == 0){
            for(int i = 0; i < pc->points.size(); i++)
            {
                pc->points[i].y += (distance+totalDistance);                
            }
            pc->width = pc->points.size();              
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputCloud(finalPoint);
            icp.setInputTarget(pc);
            icp.setMaxCorrespondenceDistance (0.05);//0.05
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(0.05);
            icp.setEuclideanFitnessEpsilon(0.00000000008);
            icp.align(*finalPoint);
            if(icp.hasConverged() == true){
                std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                icp.getFitnessScore() << std::endl;
            }
            *finalPoint += *pc;
            finalPoint->width = finalPoint->points.size();
        }
    }
    counter ++;
    lastLon = data.gpslon;
    lastLat = data.gpslat;
    totalDistance += distance;
    
    /*pcl::PCLPointCloud2 point_cloud2;
	pcl::toPCLPointCloud2(*finalPoint, point_cloud2);
	pcl::PLYWriter ply_w;
	ply_w.writeASCII("second.ply", point_cloud2);
    ROS_INFO("saved %d", counter);*/
}

int main (int argc, char** argv) {

  // Initialize ROS
    ros::init(argc, argv, "pointCloudProcessing");
    ros::NodeHandle n;
    ros::Subscriber pcProcessing = n.subscribe("pc_label", 1000, icpFiltering);
    ros::spin();
    return 0;
}
 
