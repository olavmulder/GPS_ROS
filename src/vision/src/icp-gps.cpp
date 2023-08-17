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
#include <pcl/io/pcd_io.h> 
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <dirent.h>
#include <math.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "merge.h"
#define PI 3.14159265

pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);

vision::visionMessage *firstmsg = new vision::visionMessage;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudxyz;



void icpmerge(pcl::PointCloud<pcl::PointXYZ>::Ptr pc);

double gpsLatToMeter(double v1, double v2)
{
        double r = 6371000; //straal aarde in meter
        //#print(v1, v2)
        return (2 * PI * r * ((v2 - v1) / 360));
}
double gpsLongToMeter(double angle, double v1, double v2)
{
        double r = 6371000; //#straal earth in meters
        //#print(angle)
        //#print(math.cos(math.radians(angle)))
	double rad_angle = angle * (180/PI);
        double a = (cos(rad_angle)) * r; //#Hight rechthoekige triangle, needed to calculate radius
        //#print(a)
        //#a = math.sqrt((r * r) - (x * x)) #radius circle at given coordinates
        double meters = 2 * PI * a * ((v2 - v1) / 360); 
        return meters;
}

double gpsdif(double lon1, double lat1, double lon2, double lat2)
{
	double r = 6371000;
	double lat1_rad = lat1 * (PI/180);
	double lat2_rad = lat2 * (PI/180);
	double lat_dif = (lat2-lat1) * (PI/180);
	double lon_dif = (lon2-lon1) * (PI/180);

	double a = sin(lat_dif/2) * sin(lat_dif/2) + cos(lat1_rad) * cos(lat2_rad) * sin(lon_dif/2) * sin(lon_dif/2);
	double c = 2 * atan2(sqrt(a), sqrt(1-a));
	double meters = r*c;
	ROS_INFO("Afstand: %f", meters);
	
	
	//cloud.push_back (pcl::PointXYZ (rand (), rand (), rand ()));

	return meters;	
}

void gpsmerge(vision::visionMessage data){
	
	//ROS_INFO("TESTT");

	pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
        sensor_msgs::PointCloud2 pc_temp = data.pc;
        pcl::fromROSMsg (pc_temp, *pc);

	//double avg_x_first = 0;
	//double avg_x_new = 0;
	
	pc = RemovePoints(pc, 0.05, 2);
	


	if(firstmsg->gpslon == 0)
	{
		ROS_INFO("hier");
		*firstmsg = data;
		*merged = *pc;

		pc->width = pc->points.size();
		pcl::PCLPointCloud2 point_cloud2;
        	pcl::toPCLPointCloud2(*pc, point_cloud2);

        	pcl::PLYWriter ply_w;
        	//ply_w.writeASCII("first.ply", point_cloud2);
		
		//delete pc;
		return;
	}
	//ROS_INFO("TEST2");
	//pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    	//sensor_msgs::PointCloud2 pc_temp = data.pc;
	//pcl::fromROSMsg (pc_temp, *pc);
	double distance = gpsdif(firstmsg->gpslon, firstmsg->gpslat, data.gpslon, data.gpslat);
	//if(distance < 0.8 && !(distance > 0.289 && distance < 0.3 )) return;
	//if(!(distance > 0.6 && distance < 0.9)) return;
	//if(distance > 0.9) return;
	//if(distance < 0.4) return;
	//double lat_dif = gpsLatToMeter(data.gpslat, firstmsg->gpslat);
	//double long_dif = gpsLongToMeter(data.gpslat, data.gpslon, firstmsg->gpslon);

	//double dif = sqrt((lat_dif * lat_dif) + (long_dif * long_dif));
	ROS_INFO("start: Long: %.10lf lat %.10lf", firstmsg->gpslon, firstmsg->gpslat);
	ROS_INFO("current: Long: %.10lf lat %.10lf", data.gpslon, data.gpslat);
	ROS_INFO("difference: %.10lf", distance);

	

	/*
	for(int i = 0; i < pc->points.size(); i++)
        {
                avg_x_new += pc->points[i].x;
        }
        avg_x_new /= pc->points.size();
	*/
	for(int i = 0; i < pc->points.size(); i++)
    {
		pc->points[i].y -= distance;
        merged->points.push_back(pc->points[i]);
    }
	merged->width = merged->points.size();

	pc->width = pc->points.size();
	pcl::PCLPointCloud2 point_cloud2;
	pcl::toPCLPointCloud2(*pc, point_cloud2);

	pcl::PLYWriter ply_w;
	ply_w.writeASCII("second.ply", point_cloud2);
	//icpmerge(pc);

	return;
}

void read_pcd(int n)
{

	pcl::PCDReader pcr;
    char s[50];
	for(int i = 0; i < n; i++)
	{
		if(i % 3 != 0) continue;
		ROS_INFO("I: %d", i);
        sprintf(s, "pc/0%d.pcd", i);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc (new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ> (s, *pc) != -1)
        {
			double distance = 0.1;
            ROS_INFO("Found PCD file");
			
			
			float depthTreshold = 5.0;
    		float threshold2 = depthTreshold*depthTreshold;
    		//pcl::PCLPointCloud2 cloudFiltered;

    	//sensor_msgs::PointCloud2 pc = data.pc;
    	//pcl::fromROSMsg (pc, *newCapture);
    
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
			


			pc->width = pc->points.size();
			pcl::PCLPointCloud2 point_cloud2;
			pcl::toPCLPointCloud2(*pc, point_cloud2);
			pcl::PLYWriter ply_w;
			sprintf(s, "pc%d.ply", i);
			ply_w.writeASCII(s, point_cloud2);

			
			if(merged->points.size() == 0)
			{
				*merged = *pc;
				continue;
			}
			for(int j = 0; j < pc->points.size(); j++)
        	{
				pc->points[j].y -= (distance * i);
        	}
			
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;//1 0.01
            sor.setInputCloud (pc);
            sor.setMeanK (20);
            sor.setStddevMulThresh (0.005);//0.5cm afstand
            sor.setNegative (false);
            sor.filter(*pc);
			ROS_INFO("sor filter completed");

            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputCloud(pc);
            icp.setInputTarget(merged);
            icp.setMaxCorrespondenceDistance (0.01);
            icp.setMaximumIterations(50);
            icp.setTransformationEpsilon (1e-8);
            icp.align(*pc);
            if(icp.hasConverged() == true){
                std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                icp.getFitnessScore() << std::endl;
            }
            *merged += *pc;
            pc->width = pc->points.size();
            merged->width = merged->points.size();
			


			
			//icpmerge(pc);

			//pcl::PointCloud<pcl::PointXYZ>::Ptr sor_pc (new pcl::PointCloud<pcl::PointXYZ>);
			/*
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  			sor.setInputCloud(pc);
  			sor.setMeanK (1);
  			sor.setStddevMulThresh (0.005);
  			sor.filter (*pc);
			

			for(int j = 0; j < pc->points.size(); j++)
        	{
				//pc->points[j].y -= (distance * i);
        		merged->points.push_back(pc->points[j]);
        	}

			
			merged->width = merged->points.size();
			ROS_INFO("Merged size: %d", merged->width);
			*/
			//icpmerge(sor_pc);
			
        }
	}
	return;


}
void icpmerge(pcl::PointCloud<pcl::PointXYZ>::Ptr pc)
{
	
	//copyPointCloud

	/*
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>::Ptr icp ( new pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> () );

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr source_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );
  	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr merged_normals ( new pcl::PointCloud<pcl::PointXYZRGBNormal> () );

	addNormal( pc, source_normals );
 	addNormal( merged, merged_normals );

	icp->setMaximumIterations(5);
	icp->setInputSource ( source_normals ); // not cloud_source, but cloud_source_trans!
  	icp->setInputTarget ( merged_normals );

	icp->align(*merged_normals);
	*/
	if(merged->points.size() == 0)
	{
		*merged = *pc;
		return;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaximumIterations(1);
	icp.setInputCloud(pc);
	icp.setInputTarget(merged);
    //pcl::PointCloud<pcl::PointXYZ> finalPoint;
    icp.align(*result);
    *merged = *result;
	
	return;
}
int main (int argc, char** argv) {
  // Initialize ROS
    firstmsg->gpslon = 0;
    ros::init(argc, argv, "pointCloudProcessing");
    ros::NodeHandle n;

	read_pcd(12);

	merged->width = merged->points.size();
	merged->height = 1;
	pcl::PCLPointCloud2 point_cloud2;
	pcl::toPCLPointCloud2(*merged, point_cloud2);
	
	pcl::PLYWriter ply_w;
	ply_w.writeASCII("gpsicpmerge.ply", point_cloud2);
	ROS_INFO("Merge finished");

    ros::Subscriber sub = n.subscribe("pc_label", 1000, gpsmerge);
    ros::spin();	
	
	
//pcl::io::savePCDFileASCII ("gpsmerge.pcd", *merged);
	//pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
   	
	//viewer.showCloud(merged);
   	//while (!viewer.wasStopped ())
   	//{
   	//}
return 0;
}

