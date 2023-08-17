#ifndef MERGE
#define MERGE

#include <ros/ros.h> 
#include <pcl/io/io.h>
#include <pcl_ros/point_cloud.h> 
#include <sensor_msgs/PointCloud2.h> 
#include <vision/visionMessage.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/registration/icp.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/io/ply_io.h> 
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>


double heigthFilter =  -1.25;
double minDepth = 1.50;
double maxDepth = 4; 
pcl::PointCloud<pcl::PointXYZ>::Ptr RemovePoints(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc, \
    double meanK, double mulTresh
    ){
    //x(height), z(depth) filter
    for(int p = 0;p<pc->points.size(); ++p){
        if( pc->points[p].x < heigthFilter || \
            pc->points[p].z < minDepth || \
            pc->points[p].z > maxDepth){
            
            pc->points[p] = pc->points[pc->points.size()-1];
            pc->points.resize(pc->points.size()-1);
            --p;
        }
    }
    pc->width = pc->points.size();
    
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (pc);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (mulTresh);
    sor.setNegative (false);
    sor.filter (*pc);



    return pc;
}

void GPSMergeWithGPS(pcl::PointCloud<pcl::PointXYZ>::Ptr pc, \
 pcl::PointCloud<pcl::PointXYZ>::Ptr finalPc, double distance,\
    double meanK, double mulTresh){
	
	ROS_INFO("difference: %.10lf", distance);

	for(int i = 0; i < pc->points.size(); i++){
		pc->points[i].y -= distance;
        finalPc->points.push_back(pc->points[i]);
    }
	finalPc->width = finalPc->points.size();
}

#endif