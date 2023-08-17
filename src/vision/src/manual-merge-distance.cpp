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
#define PI 3.14159265

int GetAmountFiles(const char* path);
void GetFileNames(char const *path, int amount, char **arr, char const* curDir);
pcl::PointCloud<pcl::PointXYZ>::Ptr GpsMerge(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr pc, int j);

double distance;
bool first;
int counter;
double mulTresh, meanK;

pcl::PointCloud<pcl::PointXYZ>::Ptr merged(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);




int main(int argc, char** argv){
    auto start = std::chrono::high_resolution_clock::now();
    
    char *dirName;
    if(argc < 4){
        printf("error not enough arguments");
        return 0;
    }else{
        distance = std::stof(argv[1]);
        mulTresh = std::stof(argv[2]);
        meanK = std::stof(argv[3]);
        dirName = argv[4];

    }
    first = true;
    
    int amountFiles = GetAmountFiles(dirName);
    printf("amount = %d\n", amountFiles);
    
    char *arr[amountFiles];
    *arr = (char*)malloc(7*sizeof(char**));
    for(int i=0; i<amountFiles; i++){
        arr[i] = (char*)malloc(amountFiles);
        *arr[i] = '\0';
    }
    //get curr dir
    char cwd[PATH_MAX];
    if(getcwd(cwd, sizeof(cwd)) != NULL){
        printf("Current working dir: %s\n", cwd);
    }else {
        perror("getcwd() error");
        return 1;
    }
    GetFileNames(dirName, amountFiles, arr, cwd);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);

    
    std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > sourceClouds;
    int j=amountFiles;
    for(int i=0;i<amountFiles;i++){
        //load files
        pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZ>);
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(arr[--j],  *sourceCloud)!=0){
            return -1;
        }
        std::cout << "Loaded file " << arr[j] << " (" << sourceCloud->size() << " points)" << std::endl;
        sourceClouds.push_back(sourceCloud);
        std::cout << "Point Cloud " << i << "has got " << sourceClouds[i]->size() << " Points" << std::endl;
        final = GpsMerge(final, sourceClouds[i], i);
        //j-=5;
        //if(j<0)exit(0);
    }


    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sorF;
    sorF.setInputCloud (final);
    sorF.setMeanK (25);
    sorF.setStddevMulThresh (1);
    sorF.setNegative (false);
    sorF.filter (*final);

    pcl::io::savePLYFileBinary ("gpsmerge.ply", *final);

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << std::endl;
    return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr GpsMerge(pcl::PointCloud<pcl::PointXYZ>::Ptr final, pcl::PointCloud<pcl::PointXYZ>::Ptr pc, int j){

        /*float depthTreshold = 4;
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
        }*/
    //x(height) filter
        for(int p = 0;p<pc->points.size(); ++p){
            if(pc->points[p].x < -1.25 ){
                pc->points[p] = pc->points[pc->points.size()-1];
                pc->points.resize(pc->points.size()-1);
                --p;
            }
        }
    /*  pcl::PointIndicesPtr ground (new pcl::PointIndices);
        pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ
        > pmf;
        pcl::PointCloud<pcl::PointXYZ
        >::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ
        >);

        pmf.setInputCloud (pc);
        pmf.setMaxWindowSize (200);
        pmf.setSlope (1.0f);
        pmf.setInitialDistance (0.5f);
        pmf.setMaxDistance (3.0f);
        pmf.extract (ground->indices);

        // Filter the ground points out
        pcl::ExtractIndices<pcl::PointXYZ
        > extract;
        extract.setInputCloud (pc);
        extract.setIndices (ground);
        extract.filter (*cloud_filtered);
        pc = cloud_filtered;
        ROS_INFO("Ground points found");
    */
    //z(diepte) filter 
        for(int p = 0;p<pc->points.size(); ++p){
            if(pc->points[p].z < 1.50 || pc->points[p].z > 4){
                pc->points[p] = pc->points[pc->points.size()-1];
                pc->points.resize(pc->points.size()-1);
                --p;
            }
        } 

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(pc);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius (2);
        outrem.setKeepOrganized(true);
        outrem.filter (*pc);

        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud (pc);
        sor.setMeanK (meanK);
        sor.setStddevMulThresh (mulTresh);
        sor.setNegative (false);
        sor.filter (*pc);

        

        pc->width = pc->points.size();

        

        if(first)
        {
            *merged = *pc;
            *final=*merged;
            first=false;
        }else{
            for(int i = 0; i < pc->points.size(); i++)
            {
                pc->points[i].y -= distance*j;                
                merged->points.push_back(pc->points[i]);
            }
            
                        
            /*pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputCloud(merged);
            icp.setInputTarget(final);
            icp.setMaxCorrespondenceDistance (0.002);
            icp.setMaximumIterations(300);
            icp.setTransformationEpsilon (0.05);
            icp.setEuclideanFitnessEpsilon(1e-10);
            icp.align(*merged);
            if(icp.hasConverged() == true){
                std::cout << "has converged:" << icp.hasConverged() << " score: " <<
                icp.getFitnessScore() << std::endl;
            }
            *final += *merged;
            final->width = final->points.size();*/
            
        }
        //std::string s = std::to_string(counter);
        //std::string ss = s+"tree.ply";
        //pcl::io::savePLYFileASCII (ss, *final); 
        
        pc->width = pc->points.size();
        merged->width = merged->points.size();
        
        
        
        //pcl::io::savePCDFileASCII ("gpsmergeICP.xyz", *merged);

        //merged->height = 1;
        /*pcl::PCLPointCloud2 point_cloud2;
        pcl::toPCLPointCloud2(*merged, point_cloud2);
        pcl::PLYWriter ply_w;
        ply_w.writeASCII("gpsmerge.ply", point_cloud2);*/
        
        //counter++;
        
        //final->width = final->points.size();
	return merged;
}

int GetAmountFiles(const char* path){
    int file_count = 0;
    DIR * dirp;
    struct dirent * entry;

    dirp = opendir(path); /* There should be error handling after this */
    while ((entry = readdir(dirp)) != NULL) {
        if (entry->d_type == DT_REG) { /* If the entry is a regular file */
            file_count++;
        }
    }
    closedir(dirp);
    return file_count;
}
void GetFileNames(char const *path, int amount, char **arr, char const* curDir){

    if(strcmp(path,curDir) == 0){
        struct dirent **namelist;
        
        char a[] = ".";
        char b[] = "..";
        
        int n;

        n = scandir(path, &namelist, 0, alphasort);
        int i=0;
        if (n < 0)
            perror("scandir");
        else {
            while(n--){
                if(!((strcmp(namelist[n]->d_name, a) == 0) || (strcmp(namelist[n]->d_name, b) == 0))){
                //printf("%s\n", namelist[n]->d_name);
                //strcat(*(arr+i), path);
                *(arr+i)=  namelist[n]->d_name;
                i++;
                }    
            }
        }
    }else{
        struct dirent **namelist;
        
        char a[] = ".";
        char b[] = "..";
        
        int n;

        n = scandir(path, &namelist, 0, alphasort);
        int i=0;
        if (n < 0)
            perror("scandir");
        else {
            while(n--){
                if(!((strcmp(namelist[n]->d_name, a) == 0) || (strcmp(namelist[n]->d_name, b) == 0))){
                //printf("%s\n", namelist[n]->d_name);
                //strcat(*(arr+i), path);
                strcat(*(arr+i),path);
                strcat(*(arr+i),namelist[n]->d_name);
                i++;
                }    
            }
        }
    }
}